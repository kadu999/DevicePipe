using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace DevicePipe
{
    /// <summary>
    /// Parses a byte stream into protocol frames. Ported from UnityViewer DataAnalysis.cs.
    /// </summary>
    public class FrameDecoder
    {
        readonly ProtocolConfig _config;
        readonly RingBuffer _buffer;

        int _parsedFrames;
        int _badFrames;

        // FPS measurement
        readonly System.Diagnostics.Stopwatch _sw = System.Diagnostics.Stopwatch.StartNew();
        int _lastFpsFrames;
        float _framesPerSecond;

        // ── Pipeline latency tracking ──

        struct TimedFrame
        {
            public int[] Data;
            public long FeedTick;   // Stopwatch tick when Feed() was called
            public long ParseTick;  // Stopwatch tick when TryDecode finished parsing
        }

        /// <summary>Parsed frames waiting to be consumed by main thread.</summary>
        readonly System.Collections.Concurrent.ConcurrentQueue<TimedFrame> _frameQueue = new();

        long _lastFeedTick;       // most recent Feed() call tick

        long _lastFeedTick_us;    // most recent completed frame: Feed→Parse µs
        long _lastQueueWait_us;   // most recent completed frame: Parse→Dequeue µs
        long _lastDispatch_us;    // most recent completed frame: Dequeue→OnFrameDone µs (includes GPU upload)
        long _lastTotal_us;       // most recent completed frame: Feed→OnFrameDone µs

        /// <summary>Last frame pipeline latency breakdown (microseconds). (parse, queue, dispatch, total).</summary>
        /// <remarks>
        /// parse    = Feed → frame decoded (background thread)
        /// queue    = frame decoded → main thread picks it up
        /// dispatch = main thread dequeue → OnFrame callback + GPU upload completes
        /// total    = Feed → OnFrame done (end-to-end software latency)
        /// </remarks>
        public (long parseUs, long queueUs, long dispatchUs, long totalUs) LastFrameLatency =>
            (_lastFeedTick_us, _lastQueueWait_us, _lastDispatch_us, _lastTotal_us);

        public System.Action<int[]> OnFrame;

        public int ParsedFrameCount => _parsedFrames;
        public int BadFrameCount => _badFrames;
        public int BufferedByteCount => _buffer.Count;
        public int QueuedFrameCount => _frameQueue.Count;

        /// <summary>Estimated frames per second, updated every second.</summary>
        public float FramesPerSecond
        {
            get
            {
                long elapsed = _sw.ElapsedMilliseconds;
                if (elapsed >= 1000)
                {
                    float delta = elapsed / 1000f;
                    _framesPerSecond = (_parsedFrames - _lastFpsFrames) / delta;
                    _lastFpsFrames = _parsedFrames;
                    _sw.Restart();
                }
                return _framesPerSecond;
            }
        }

        public event Action<string> OnError;

        public FrameDecoder(ProtocolConfig config)
        {
            _config = config ?? throw new ArgumentNullException(nameof(config));
            _buffer = new RingBuffer(230400);
            FrameDecoderRunner.Add(this);
        }

        public void Dispose()
        {
            FrameDecoderRunner.Remove(this);
            OnFrame = null;
        }

        public void Feed(byte[] data) => Feed(data, 0, data.Length);

        public void Feed(byte[] data, int offset, int count)
        {
            if (count <= 0) return;
            _lastFeedTick = System.Diagnostics.Stopwatch.GetTimestamp();
            _buffer.Write(data, offset, count);
            TryDecode();
        }

        public void Reset()
        {
            _buffer.Clear();
            _parsedFrames = 0;
            _badFrames = 0;
        }

        // ── Scratch buffer for frame operations ──
        // Sized for a full 100x100@16bit frame: ~20KB + head + checksum
        byte[] _readBuf = new byte[23040];
        // Smaller buffer for header scanning — 4KB on stack avoids heap alloc
        byte[] _scanBuf = new byte[4096];

        void TryDecode()
        {
            // Row*Col threshold (single Count access — cheap, one lock)
            if (_buffer.Count < _config.RowCount * _config.ColCount) return;

            // Find header by scanning a batched copy (single lock) instead of
            // per-byte indexer calls (each with its own lock).
            int searchLen = Math.Min(_buffer.Count, _scanBuf.Length);
            _buffer.ReadBatch(_scanBuf, 0, searchLen);
            int headBytePos = FindHeaderInBuffer(_scanBuf, searchLen);
            if (headBytePos < 0)
            {
                if (_buffer.Count > _readBuf.Length / 2)
                    _buffer.Discard(_buffer.Count / 4);
                return;
            }
            if (headBytePos > 0) { _buffer.Discard(headBytePos); return; }

            // Verify header bytes (re-read after discard — head is now at position 0)
            if (_buffer.Count < 3) return;
            _buffer.ReadBatch(_scanBuf, 0, 3);
            if (_scanBuf[0] != 0xA5 || _scanBuf[1] != 0x5A || _scanBuf[2] != 0x01) { _buffer.Discard(1); return; }

            // Read length field (bytes 3-4, little-endian)
            if (_buffer.Count < 5) return;
            _buffer.ReadBatch(_scanBuf, 0, 5);
            int rawLen = _scanBuf[3] + _scanBuf[4] * 256;
            int dataBytes = rawLen - _config.HeadLen;
            if (dataBytes <= 0) { _buffer.Discard(1); return; }

            // Full frame = head(3) + len(2) + gap(1) + data + checksum
            int frameByteLen = _config.HeadLen + dataBytes + ChecksumLen();
            if (_buffer.Count < frameByteLen) return;

            // Batch-copy the entire frame in one lock
            _buffer.ReadBatch(_readBuf, 0, frameByteLen);

            bool checksumOk = SumCheck(_readBuf, 0, frameByteLen);
            long parseTick = System.Diagnostics.Stopwatch.GetTimestamp();
            if (!checksumOk) { _badFrames++; DumpChecksumFail(0, frameByteLen * 2); }
            else _parsedFrames++;

            CommandAnalysis(frameByteLen, checksumOk, _lastFeedTick, parseTick);
            _buffer.Discard(frameByteLen);
        }

        int FindHeaderInBuffer(byte[] buf, int len)
        {
            for (int i = 0; i < len - 2; i++)
                if (buf[i] == 0xA5 && buf[i + 1] == 0x5A && buf[i + 2] == 0x01)
                    return i;
            return -1;
        }

        // ── CRC32 Mpeg2 lookup table (precomputed once, O(1)/byte) ──
        static readonly uint[] _crc32Table = BuildCrc32Table();

        static uint[] BuildCrc32Table()
        {
            var table = new uint[256];
            for (int i = 0; i < 256; i++)
            {
                uint crc = (uint)i << 24;
                for (int j = 0; j < 8; j++)
                    crc = (crc & 0x80000000) != 0 ? (crc << 1) ^ 0x04C11DB7 : crc << 1;
                table[i] = crc;
            }
            return table;
        }

        // ── CRC16 Modbus lookup table (precomputed once, O(1)/byte) ──
        static readonly ushort[] _crc16Table = BuildCrc16Table();

        static ushort[] BuildCrc16Table()
        {
            var table = new ushort[256];
            for (int i = 0; i < 256; i++)
            {
                ushort crc = (ushort)i;
                for (int j = 0; j < 8; j++)
                    crc = (crc & 1) != 0 ? (ushort)((crc >> 1) ^ 0xA001) : (ushort)(crc >> 1);
                table[i] = crc;
            }
            return table;
        }

        // ── Ported from DataAnalysis.SumCheck ──

        bool SumCheck(byte[] cmd, int start, int len)
        {
            return _config.Checksum switch
            {
                ChecksumType.Sum16        => SumCheck_Sum16(cmd, start, len),
                ChecksumType.CRC16_Modbus => SumCheck_CRC16(cmd, start, len),
                ChecksumType.CRC32_Mpeg2  => SumCheck_CRC32(cmd, start, len),
                _ => false
            };
        }

        bool SumCheck_Sum16(byte[] cmd, int start, int len)
        {
            short temp = 0;
            for (int i = start; i < len + start - 2; i++)
                temp += (short)cmd[i];

            return cmd[start + len - 2] == (byte)(temp & 0xFF)
                && cmd[start + len - 1] == (byte)(temp >> 8 & 0xFF);
        }

        bool SumCheck_CRC16(byte[] cmd, int start, int len)
        {
            ushort crc = 0xFFFF;
            int end = len + start - 2;
            for (int i = start; i < end; i++)
                crc = (ushort)((crc >> 8) ^ _crc16Table[(crc ^ cmd[i]) & 0xFF]);

            var bytes = BitConverter.GetBytes(crc);
            return bytes[0] == cmd[start + len - 2] && bytes[1] == cmd[start + len - 1];
        }

        bool SumCheck_CRC32(byte[] cmd, int start, int len)
        {
            uint crc = 0xFFFFFFFF;
            int end = len + start - 4;
            for (int j = start; j < end; j++)
                crc = (crc << 8) ^ _crc32Table[(byte)(crc >> 24) ^ cmd[j]];

            byte crc3 = (byte)((crc >> 24) & 0xFF);
            byte crc2 = (byte)((crc >> 16) & 0xFF);
            byte crc1 = (byte)((crc >> 8) & 0xFF);
            byte crc0 = (byte)(crc & 0xFF);

            return cmd[start + len - 4] == crc3
                && cmd[start + len - 3] == crc2
                && cmd[start + len - 2] == crc1
                && cmd[start + len - 1] == crc0;
        }

        void DumpChecksumFail(int headIndex, int len)
        {
            if (_badFrames > 5) return;
            int s = headIndex / 2;
            int l = len / 2;
            var sb = new System.Text.StringBuilder();
            sb.AppendLine($"校验失败 #{_badFrames}: chk={_config.Checksum} bits={_config.BitsPerSample} len={l}");

            if (_config.Checksum == ChecksumType.CRC16_Modbus)
            {
                ushort crc = 0xFFFF;
                for (int i = s; i < l + s - 2; i++)
                    crc = (ushort)((crc >> 8) ^ _crc16Table[(crc ^ _readBuf[i]) & 0xFF]);
                sb.AppendLine($"  Computed: {crc:X4}  Expected: {_readBuf[s + l - 2]:X2}{_readBuf[s + l - 1]:X2}");
            }
            else if (_config.Checksum == ChecksumType.Sum16)
            {
                short t = 0; for (int i = s; i < l + s - 2; i++) t += (short)_readBuf[i];
                sb.AppendLine($"  Computed: {(byte)(t & 0xFF):X2}{(byte)(t >> 8):X2}  Expected: {_readBuf[s + l - 2]:X2}{_readBuf[s + l - 1]:X2}");
            }

            sb.Append("  Hex: ").AppendLine(BitConverter.ToString(_readBuf, s, Math.Min(l, 64)).Replace("-", " "));
            OnError?.Invoke(sb.ToString());
        }

        // ── Ported from DataAnalysis.CommandAnalysis ──

        void CommandAnalysis(int frameByteLen, bool checksumOk, long feedTick, long parseTick)
        {
            int total = _config.RowCount * _config.ColCount;
            int bitsPerSample = _config.BitsPerSample;
            int dataBytes = total * (bitsPerSample / 8);

            if (frameByteLen < _config.HeadLen + dataBytes + ChecksumLen()) return;

            int[] result = new int[total];
            int dataStart = _config.HeadLen;

            if (bitsPerSample == 8)
            {
                for (int i = 0; i < total; i++)
                    result[i] = _readBuf[dataStart + i];
            }
            else
            {
                for (int i = 0; i < total; i++)
                {
                    int off = dataStart + i * 2;
                    result[i] = (int)((_readBuf[off] + _readBuf[off + 1] * 256) / 4096f * 255f);
                }
            }

            if (checksumOk)
            {
                _frameQueue.Enqueue(new TimedFrame { Data = result, FeedTick = feedTick, ParseTick = parseTick });
            }
        }

        int ChecksumLen()
        {
            return _config.Checksum switch
            {
                ChecksumType.CRC32_Mpeg2 => 4,
                _ => 2
            };
        }

        public void Update()
        {
            long freq = System.Diagnostics.Stopwatch.Frequency;
            while (_frameQueue.TryDequeue(out var tf))
            {
                long dequeueTick = System.Diagnostics.Stopwatch.GetTimestamp();
                OnFrame?.Invoke(tf.Data);
                long doneTick = System.Diagnostics.Stopwatch.GetTimestamp();

                _lastFeedTick_us  = (tf.ParseTick - tf.FeedTick) * 1_000_000 / freq;
                _lastQueueWait_us = (dequeueTick - tf.ParseTick) * 1_000_000 / freq;
                _lastDispatch_us  = (doneTick - dequeueTick) * 1_000_000 / freq;
                _lastTotal_us     = (doneTick - tf.FeedTick) * 1_000_000 / freq;
            }
        }
    }

    public class FrameDecoderRunner : MonoBehaviour
    {
        private static FrameDecoderRunner _instance;
        private static FrameDecoderRunner Instance
        {
            get
            {
                if (_instance == null)
                {
                    var obj = new GameObject("FrameDecoderRunner");
                    DontDestroyOnLoad(obj);
                    _instance = obj.AddComponent<FrameDecoderRunner>();
                }
                return _instance;
            }
        }

        private List<FrameDecoder> _list = new List<FrameDecoder>();


        private void Update()
        {
            foreach (var node in _list)
            {
                node.Update();
            }
        }

        public static void Add(FrameDecoder decoder)
        {
            Instance._list.Add(decoder);
        }

        public static void Remove(FrameDecoder decoder)
        {
            Instance._list.Remove(decoder);
        }
    }
}

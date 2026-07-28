using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace DevicePipe
{
    /// <summary>
    /// Parses a byte stream into protocol frames.
    /// Multi-frame batch decode — one Feed() call processes ALL complete frames in the buffer.
    /// </summary>
    public class FrameDecoder
    {
        readonly ProtocolConfig _config;
        readonly RingBuffer _buffer;

        int _parsedFrames;
        int _badFrames;
        int _droppedFrames; // frames evicted from queue (main thread too slow)

        // FPS measurement (EMA updated continuously)
        readonly System.Diagnostics.Stopwatch _sw = System.Diagnostics.Stopwatch.StartNew();
        int _lastFpsFrames;
        float _emaFps;

        // ── Pipeline latency tracking ──

        struct TimedFrame
        {
            public int[] Data;
            public long FeedTick;   // Stopwatch tick when Feed() was called
            public long ParseTick;  // Stopwatch tick when TryDecode finished parsing
        }

        /// <summary>Parsed frames waiting to be consumed by main thread.</summary>
        readonly System.Collections.Concurrent.ConcurrentQueue<TimedFrame> _frameQueue = new();

        /// <summary>Max queued frames before oldest is evicted (backpressure).</summary>
        const int MaxQueueSize = 4;

        long _lastFeedTick;
        long _lastFeedTick_us;
        long _lastQueueWait_us;
        long _lastDispatch_us;
        long _lastTotal_us;

        /// <summary>Last frame pipeline latency breakdown (microseconds). (parse, queue, dispatch, total).</summary>
        public (long parseUs, long queueUs, long dispatchUs, long totalUs) LastFrameLatency =>
            (_lastFeedTick_us, _lastQueueWait_us, _lastDispatch_us, _lastTotal_us);

        public System.Action<int[]> OnFrame;

        public int ParsedFrameCount => _parsedFrames;
        public int BadFrameCount => _badFrames;

        /// <summary>Frames evicted from the output queue (main thread not keeping up).</summary>
        public int DroppedFrameCount => _droppedFrames;

        public int BufferedByteCount => _buffer.Count;
        public int QueuedFrameCount => _frameQueue.Count;

        /// <summary>Estimated frames per second (EMA smoothed, updates continuously).</summary>
        public float FramesPerSecond
        {
            get
            {
                long elapsed = _sw.ElapsedMilliseconds;
                if (elapsed > 50) // don't update too aggressively, avoid jitter
                {
                    float instant = (_parsedFrames - _lastFpsFrames) / (elapsed / 1000f);
                    float alpha = 0.2f; // EMA smoothing — fast response, low noise
                    if (_emaFps <= 0) _emaFps = instant;
                    else _emaFps += (instant - _emaFps) * alpha;
                }

                if (elapsed >= 1000)
                {
                    _lastFpsFrames = _parsedFrames;
                    _sw.Restart();
                }

                return _emaFps;
            }
        }

        public event Action<string> OnError;

        public FrameDecoder(ProtocolConfig config)
        {
            _config = config ?? throw new ArgumentNullException(nameof(config));
            _buffer = new RingBuffer(230400);
            _buffer.OnOverflow += () => OnError?.Invoke(
                $"[RingBuffer] 溢出! 容量={_buffer.Capacity} 溢出次数={_buffer.OverflowCount} 丢弃字节={_buffer.OverflowBytes}");
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
            _droppedFrames = 0;
        }

        // ── Scratch buffers ──
        // _readBuf: sized for a full 100×100@16bit frame ~20KB + head + checksum
        byte[] _readBuf = new byte[23040];
        // _scanBuf: smaller buffer for header scanning — avoids per-byte lock calls
        byte[] _scanBuf = new byte[4096];

        // ── Frame format constants (A55A01 protocol) ──
        // Byte layout: [A5][5A][01] [len_lo][len_hi] [gap] [data...] [checksum...]
        // header=3, length_field=2, gap=1 → HeadLen=6
        const int HEADER_A5 = 0;
        const int HEADER_5A = 1;
        const int HEADER_01 = 2;
        const int LEN_LO    = 3;
        const int LEN_HI    = 4;

        /// <summary>
        /// Decode ALL complete frames currently in the ring buffer.
        /// Loops until no more full frames are available, minimizing per-frame lock overhead.
        /// </summary>
        void TryDecode()
        {
            int checksumLen = ChecksumLen();
            int minDataBytes = _config.RowCount * _config.ColCount * (_config.BitsPerSample / 8);
            int minTotalBytes = _config.HeadLen + minDataBytes + checksumLen;

            while (_buffer.Count >= minTotalBytes)
            {
                // ── Phase 1: Peek header region ──
                int peekLen = Math.Min(_buffer.Count, _scanBuf.Length);
                _buffer.ReadBatch(_scanBuf, 0, peekLen);

                // ── Phase 2: Find A5 5A 01 header ──
                int headerPos = -1;
                int scanEnd = peekLen - 2;
                for (int i = 0; i < scanEnd; i++)
                {
                    if (_scanBuf[i] == 0xA5 && _scanBuf[i + 1] == 0x5A && _scanBuf[i + 2] == 0x01)
                    {
                        headerPos = i;
                        break;
                    }
                }

                if (headerPos < 0)
                {
                    // No header found in scanned region.
                    // Safely discard all but the last 2 bytes (could be partial A5 5A).
                    int discard = Math.Max(peekLen - 2, 1);
                    _buffer.Discard(discard);
                    continue;
                }

                // Header found — discard garbage before it
                if (headerPos > 0)
                {
                    _buffer.Discard(headerPos);
                    continue;
                }

                // ── Phase 3: Header at position 0 — parse frame length ──
                // _scanBuf[0..4] is valid (we confirmed peekLen >= 5 via minTotalBytes)
                int rawLen = _scanBuf[LEN_LO] + _scanBuf[LEN_HI] * 256;
                int dataBytes = rawLen - _config.HeadLen;
                if (dataBytes <= 0)
                {
                    _buffer.Discard(1); // bogus frame, skip one byte and re-scan
                    continue;
                }

                int frameByteLen = _config.HeadLen + dataBytes + checksumLen;
                if (frameByteLen <= 0 || frameByteLen > _readBuf.Length)
                {
                    _buffer.Discard(1);
                    continue;
                }

                if (_buffer.Count < frameByteLen)
                    return; // Incomplete frame — wait for more data

                // ── Phase 4: Read entire frame in one batch ──
                _buffer.ReadBatch(_readBuf, 0, frameByteLen);

                // ── Phase 5: Checksum + extract ──
                ProcessFrame(frameByteLen);

                // ── Phase 6: Discard consumed frame, loop for next ──
                _buffer.Discard(frameByteLen);
            }
        }

        /// <summary>Process one complete frame sitting in _readBuf[0..frameByteLen-1].</summary>
        void ProcessFrame(int frameByteLen)
        {
            bool checksumOk = _config.SkipChecksum || SumCheck(_readBuf, 0, frameByteLen);
            long parseTick = System.Diagnostics.Stopwatch.GetTimestamp();

            if (!checksumOk)
            {
                _badFrames++;
                if (_badFrames <= 5)
                    DumpChecksumFail(frameByteLen);
                return;
            }

            _parsedFrames++;

            int total = _config.RowCount * _config.ColCount;
            int bitsPerSample = _config.BitsPerSample;
            int dataStart = _config.HeadLen;
            int[] result = new int[total];

            if (bitsPerSample == 8)
            {
                for (int i = 0; i < total; i++)
                    result[i] = _readBuf[dataStart + i];
            }
            else // 16-bit: scale 12-bit ADC (0-4095) → 8-bit (0-255), integer math
            {
                for (int i = 0; i < total; i++)
                {
                    int off = dataStart + i * 2;
                    // val ∈ [0, 4095]; val * 255 / 4096 fits in int, no float overhead
                    result[i] = ((_readBuf[off] + _readBuf[off + 1] * 256) * 255) / 4096;
                }
            }

            // ── Enqueue with backpressure ──
            // If main thread can't keep up, drop oldest frames rather than grow unbounded.
            while (_frameQueue.Count >= MaxQueueSize)
            {
                _frameQueue.TryDequeue(out _);
                _droppedFrames++;
            }

            _frameQueue.Enqueue(new TimedFrame
            {
                Data = result,
                FeedTick = _lastFeedTick,
                ParseTick = parseTick
            });
        }

        // ── Header scan helper ──

        int FindHeaderInBuffer(byte[] buf, int len)
        {
            for (int i = 0; i < len - 2; i++)
                if (buf[i] == 0xA5 && buf[i + 1] == 0x5A && buf[i + 2] == 0x01)
                    return i;
            return -1;
        }

        // ══════════════════════════════════════════════════════════
        //  Checksum implementations (lookup-table based, O(1)/byte)
        // ══════════════════════════════════════════════════════════

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

        int ChecksumLen()
        {
            return _config.Checksum switch
            {
                ChecksumType.CRC32_Mpeg2 => 4,
                _ => 2
            };
        }

        // ── Diagnostics ──

        void DumpChecksumFail(int frameByteLen)
        {
            var sb = new System.Text.StringBuilder();
            sb.AppendLine($"校验失败 #{_badFrames}: chk={_config.Checksum} bits={_config.BitsPerSample} len={frameByteLen}");

            if (_config.Checksum == ChecksumType.CRC16_Modbus)
            {
                ushort crc = 0xFFFF;
                for (int i = 0; i < frameByteLen - 2; i++)
                    crc = (ushort)((crc >> 8) ^ _crc16Table[(crc ^ _readBuf[i]) & 0xFF]);
                sb.AppendLine($"  Computed: {crc:X4}  Expected: {_readBuf[frameByteLen - 2]:X2}{_readBuf[frameByteLen - 1]:X2}");
            }
            else if (_config.Checksum == ChecksumType.Sum16)
            {
                short t = 0; for (int i = 0; i < frameByteLen - 2; i++) t += (short)_readBuf[i];
                sb.AppendLine($"  Computed: {(byte)(t & 0xFF):X2}{(byte)(t >> 8):X2}  Expected: {_readBuf[frameByteLen - 2]:X2}{_readBuf[frameByteLen - 1]:X2}");
            }

            sb.Append("  Hex: ").AppendLine(BitConverter.ToString(_readBuf, 0, Math.Min(frameByteLen, 64)).Replace("-", " "));
            OnError?.Invoke(sb.ToString());
        }

        // ══════════════════════════════════════════════════════════
        //  Main-thread dispatch
        // ══════════════════════════════════════════════════════════

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
                node.Update();
        }

        public static void Add(FrameDecoder decoder) { Instance._list.Add(decoder); }
        public static void Remove(FrameDecoder decoder) { Instance._list.Remove(decoder); }
    }
}

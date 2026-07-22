using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace DeviceDecoder
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

        /// <summary>Parsed frames waiting to be consumed by main thread.</summary>
        readonly System.Collections.Concurrent.ConcurrentQueue<int[]> _frameQueue = new();

        public System.Action<int[]> OnFrame;

        public int ParsedFrameCount => _parsedFrames;
        public int BadFrameCount => _badFrames;
        public int BufferedByteCount => _buffer.Count;
        public int QueuedFrameCount => _frameQueue.Count;

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
            _buffer.Write(data, offset, count);
            TryDecode();
        }

        public void Reset()
        {
            _buffer.Clear();
            _parsedFrames = 0;
            _badFrames = 0;
        }

        // ── Ported from DataAnalysis.DataAnalysisThread ──

        byte[] _readBuf = new byte[230400 / 5];

        void TryDecode()
        {
            // Row*Col threshold
            if (_buffer.Count < _config.RowCount * _config.ColCount) return;

            // Find header directly in ring buffer (no copy, no hex)
            int searchLen = Math.Min(_buffer.Count, 4096);
            int headBytePos = FindHeaderInBuffer(searchLen);
            if (headBytePos < 0)
            {
                if (_buffer.Count > _readBuf.Length / 2)
                    _buffer.Discard(_buffer.Count / 4);
                return;
            }
            if (headBytePos > 0) { _buffer.Discard(headBytePos); return; }

            // Verify header bytes
            if (_buffer.Count < 3) return;
            if (_buffer[0] != 0xA5 || _buffer[1] != 0x5A || _buffer[2] != 0x01) { _buffer.Discard(1); return; }

            // Read length field (bytes 3-4, little-endian)
            if (_buffer.Count < 5) return;
            int rawLen = _buffer[3] + _buffer[4] * 256;
            int dataBytes = rawLen - _config.HeadLen;
            if (dataBytes <= 0) { _buffer.Discard(1); return; }

            // Full frame = head(3) + len(2) + gap(1) + data + checksum
            int frameByteLen = _config.HeadLen + dataBytes + ChecksumLen();
            if (_buffer.Count < frameByteLen) return;

            // Copy frame to local buffer for checksum
            for (int i = 0; i < frameByteLen; i++)
                _readBuf[i] = _buffer[i];

            bool checksumOk = SumCheck(_readBuf, 0, frameByteLen);
            if (!checksumOk) { _badFrames++; DumpChecksumFail(0, frameByteLen * 2); }
            else _parsedFrames++;

            CommandAnalysis(frameByteLen, checksumOk);
            _buffer.Discard(frameByteLen);
        }

        int FindHeaderInBuffer(int len)
        {
            for (int i = 0; i < len - 2; i++)
                if (_buffer[i] == 0xA5 && _buffer[i + 1] == 0x5A && _buffer[i + 2] == 0x01)
                    return i;
            return -1;
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
            byte[] checkData = new byte[2];
            checkData[0] = cmd[start + len - 2];
            checkData[1] = cmd[start + len - 1];

            short temp = 0;
            for (int i = start; i < len + start - 2; i++)
                temp += (short)cmd[i];

            byte[] sum = new byte[2];
            sum[0] = (byte)(temp & 0xFF);
            sum[1] = (byte)(temp >> 8 & 0xFF);

            return sum.SequenceEqual(checkData);
        }

        bool SumCheck_CRC16(byte[] cmd, int start, int len)
        {
            byte[] checkData = new byte[2];
            checkData[0] = cmd[start + len - 2];
            checkData[1] = cmd[start + len - 1];

            ushort crc = 0xFFFF;
            for (int i = start; i < len + start - 2; i++)
            {
                crc ^= cmd[i];
                for (byte j = 0; j < 8; j++)
                {
                    if ((crc & 0x0001) != 0)
                    {
                        crc >>= 1;
                        crc ^= 0xA001;
                    }
                    else
                    {
                        crc >>= 1;
                    }
                }
            }

            return BitConverter.GetBytes(crc).SequenceEqual(checkData);
        }

        bool SumCheck_CRC32(byte[] cmd, int start, int len)
        {
            byte[] checkData = new byte[4];
            checkData[0] = cmd[start + len - 4];
            checkData[1] = cmd[start + len - 3];
            checkData[2] = cmd[start + len - 2];
            checkData[3] = cmd[start + len - 1];

            uint crc = 0xFFFFFFFF;
            for (int j = start; j < len + start - 4; j++)
            {
                crc ^= (uint)cmd[j] << 24;
                for (int i = 0; i < 8; ++i)
                {
                    if ((crc & 0x80000000) != 0)
                        crc = (crc << 1) ^ 0x04C11DB7;
                    else
                        crc <<= 1;
                }
            }

            byte[] result = new byte[4];
            result[3] = (byte)(crc & 0xFF);
            result[2] = (byte)((crc & 0xFF00) >> 8);
            result[1] = (byte)((crc & 0xFF0000) >> 16);
            result[0] = (byte)((crc >> 24) & 0xFF);

            return result.SequenceEqual(checkData);
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
                for (int i = s; i < l + s - 2; i++) { crc ^= _readBuf[i]; for (int j = 0; j < 8; j++) crc = (crc & 1) != 0 ? (ushort)((crc >> 1) ^ 0xA001) : (ushort)(crc >> 1); }
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

        void CommandAnalysis(int frameByteLen, bool checksumOk)
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
                _frameQueue.Enqueue(result);
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
            while (_frameQueue.TryDequeue(out var frame))
                OnFrame?.Invoke(frame);
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

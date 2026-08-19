using DeviceLink;
using UnityEngine;

namespace DevicePipe
{
    public class SerialPressureReader
    {
        public event System.Action<int[], int, int> OnFrame;

        readonly ProtocolConfig _config;
        SerialBridge _bridge;
        FrameDecoder _decoder;
        BitsPerSampleDetector _detector;

        int[] _data;
        PressureInfo[] _touches;
        ChessPieceInfo[] _pieces;

        public bool IsOpen => _bridge != null && _bridge.IsOpen;

        public float FrameRate => _decoder?.FramesPerSecond ?? 0f;
        public float WireFrameRate => _decoder?.WireFrameRate ?? 0f;
        public int FrameCount => _decoder?.ParsedFrameCount ?? 0;
        public int BadFrameCount => _decoder?.BadFrameCount ?? 0;
        public int DroppedFrameCount => _decoder?.DroppedFrameCount ?? 0;
        public int QueuedFrames => _decoder?.QueuedFrameCount ?? 0;
        public int BufferedBytes => _decoder?.BufferedByteCount ?? 0;

        public (long parseUs, long queueUs, long dispatchUs, long totalUs) LastFrameLatency =>
            _decoder?.LastFrameLatency ?? (0, 0, 0, 0);

        public SerialPressureReader(int row, int col)
            : this(new ProtocolConfig
            {
                HeaderHex = "A55A01",
                BitsPerSample = 16,
                HeaderByteLength = 3,
                Checksum = ChecksumType.CRC16_Modbus,
                RowCount = row,
                ColCount = col,
                SkipChecksum = true,
            }) { }

        public SerialPressureReader(ProtocolConfig config)
        {
            _config = config;
        }

        public void Open(string portName = null, int baudRate = 460800)
        {
            if (_decoder != null) return;

            _bridge = new SerialBridge();
            _bridge.OnError += e => Debug.LogWarning($"[Serial] {e}");
            _bridge.OnDataReceived += OnDetectData;

            _decoder = new FrameDecoder(_config);
            _decoder.OnFrame += UpdateData;

            if (_config.AutoDetectBitsPerSample)
            {
                int samples = _config.RowCount * _config.ColCount;
                _detector = new BitsPerSampleDetector(_config.HeaderBytes, _config.HeaderByteLength, samples);
            }

            var port = ResolvePort(portName, SerialBridge.GetPortNames());
            if (!string.IsNullOrEmpty(port)) _bridge.Open(port, baudRate);
        }

        void OnDetectData(byte[] data, int offset, int count)
        {
            if (_detector != null)
            {
                _detector.Feed(data, offset, count);

                if (_detector.DetectedBits > 0)
                {
                    _config.BitsPerSample = _detector.DetectedBits;
                    Debug.Log($"[SerialPressureReader] auto-detected {_detector.DetectedBits}-bit");
                    _detector = null;
                }
            }
            else
            {
                _decoder?.Feed(data, offset, count);
            }
        }

        static string ResolvePort(string requested, string[] available)
        {
            if (!string.IsNullOrEmpty(requested)) return requested;
            return available.Length > 0 ? available[available.Length - 1] : null;
        }

        public void Close()
        {
            _bridge?.Close();
            _decoder?.Dispose();
            _decoder = null;
            _bridge = null;
        }

        private void UpdateData(int[] data)
        {
            _data = data;
            _touches = null;
            _pieces = null;
            OnFrame?.Invoke(data, _config.RowCount, _config.ColCount);
        }

        public PressureInfo[] GetPressureInfo(RadiusMode mode = RadiusMode.Direction, bool enableFilter = false)
        {
            if (_data != null && _touches == null)
            {
                _touches = PressureAnalyzer.GetPressureInfo(_data, _config.RowCount, _config.ColCount, mode, enableFilter);
            }
            return _touches;
        }

        public ChessPieceInfo[] GetChessPieceInfo()
        {
            if (_data != null && _pieces == null)
            {
                _pieces = PressureAnalyzer.GetChessPieceInfo(_data, _config.RowCount, _config.ColCount);
            }
            return _pieces;
        }
    }
}

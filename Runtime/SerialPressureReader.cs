using DeviceLink;
using UnityEngine;

namespace DevicePipe
{
    /// <summary>
    /// Wraps serial bridge + frame decoder into one managed pipeline.
    /// Auto-connects to the last available COM port on Open().
    /// </summary>
    public class SerialPressureReader
    {
        public event System.Action<int[], int, int> OnFrame;

        readonly ProtocolConfig _config;
        readonly bool _useWin;

        DeviceLink.WinSerialBridge _winSerial;
        DeviceLink.SerialPortBridge _serial;
        FrameDecoder _decoder;

        int[] _data;
        PressureInfo[] _touches;
        ChessPieceInfo[] _pieces;

        public bool IsOpen
        {
            get
            {
                if (_useWin) return _winSerial != null && _winSerial.IsOpen;
                return _serial != null && _serial.IsOpen;
            }
        }

        public float FrameRate => _decoder?.FramesPerSecond ?? 0f;
        public int FrameCount => _decoder?.ParsedFrameCount ?? 0;
        public int BadFrameCount => _decoder?.BadFrameCount ?? 0;
        public int QueuedFrames => _decoder?.QueuedFrameCount ?? 0;
        public int BufferedBytes => _decoder?.BufferedByteCount ?? 0;

        public SerialPressureReader(int row, int col, bool useWinSerialBridge = false)
            : this(new ProtocolConfig
            {
                HeaderHex = "A55A01",
                BitsPerSample = 16,
                HeaderByteLength = 3,
                Checksum = ChecksumType.CRC16_Modbus,
                RowCount = row,
                ColCount = col,
                SkipChecksum = true,
            }, useWinSerialBridge) { }

        public SerialPressureReader(ProtocolConfig config, bool useWinSerialBridge = false)
        {
            _config = config;
            _useWin = useWinSerialBridge;
        }

        /// <summary>
        /// Open the specified port, or auto-detect the last available COM port if null/empty.
        /// </summary>
        public void Open(string portName = null, int baudRate = 460800)
        {
            if (_decoder != null) return; // already opened

            _decoder = new FrameDecoder(_config);
            _decoder.OnFrame += UpdateData;

            if (_useWin)
            {
                _winSerial = new DeviceLink.WinSerialBridge();
                _winSerial.OnDataReceived += (buf, off, len) => _decoder?.Feed(buf, off, len);
                var port = ResolvePort(portName, DeviceLink.WinSerialBridge.GetPortNames());
                if (!string.IsNullOrEmpty(port)) _winSerial.Open(port, baudRate);
            }
            else
            {
                _serial = new DeviceLink.SerialPortBridge();
                _serial.OnDataReceived += (buf, off, len) => _decoder?.Feed(buf, off, len);
                _serial.OnError += e => Debug.LogWarning($"[Serial] {e}");
                var port = ResolvePort(portName, DeviceLink.SerialPortBridge.GetPortNames());
                if (!string.IsNullOrEmpty(port)) _serial.Open(port, baudRate);
            }
        }

        static string ResolvePort(string requested, string[] available)
        {
            if (!string.IsNullOrEmpty(requested)) return requested;
            return available.Length > 0 ? available[available.Length - 1] : null;
        }

        public void Close()
        {
            _winSerial?.Close();
            _serial?.Close();
            _decoder?.Dispose();
            _decoder = null;
            _winSerial = null;
            _serial = null;
        }

        private void UpdateData(int[] data)
        {
            _data = data;
            _touches = null;
            _pieces = null;
            OnFrame?.Invoke(data, _config.RowCount, _config.ColCount);
        }

        public PressureInfo[] GetPressureInfo(RadiusMode mode = RadiusMode.Direction)
        {
            if (_data != null && _touches == null)
            {
                _touches = PressureAnalyzer.GetPressureInfo(_data, _config.RowCount, _config.ColCount, mode);
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

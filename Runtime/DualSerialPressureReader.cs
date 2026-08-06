using DeviceLink;
using UnityEngine;

namespace DevicePipe
{
    /// <summary>
    /// Wraps two SerialPressureReader instances into one merged pipeline.
    /// When both boards are active, waits for both frames then merges side-by-side
    /// (left + right) into a single (col*2 × row) array and fires OnFrame.
    /// When only one board is connected, the missing side is filled with zeros.
    /// </summary>
    public class DualSerialPressureReader
    {
        public event System.Action<int[], int, int> OnFrame;

        readonly int _row, _col;
        readonly ProtocolConfig _config;
        int _cellCount;

        SerialPressureReader _readerA;
        SerialPressureReader _readerB;
        bool _swapped;

        int[] _dataA;
        int[] _dataB;
        int[] _merged;
        int[] _zeros; // 单板模式或未就绪侧全零填充
        int _frameCountA, _frameCountB;

        int[] Zeros => _zeros ?? (_zeros = new int[_cellCount]);

        public bool IsOpen =>
            (_readerA != null && _readerA.IsOpen) ||
            (_readerB != null && _readerB.IsOpen);

        public int MergedWidth  => _col * 2;
        public int MergedHeight => _row;

        /// <summary>Left-side reader (respects swap).</summary>
        public SerialPressureReader LeftReader  => _swapped ? _readerB : _readerA;

        /// <summary>Right-side reader (respects swap).</summary>
        public SerialPressureReader RightReader => _swapped ? _readerA : _readerB;

        /// <summary>Raw reader A (independent of swap).</summary>
        public SerialPressureReader ReaderA => _readerA;

        /// <summary>Raw reader B (independent of swap).</summary>
        public SerialPressureReader ReaderB => _readerB;

        public bool Swapped
        {
            get => _swapped;
            set => _swapped = value;
        }

        public DualSerialPressureReader(int row, int col)
        {
            _row = row;
            _col = col;
            _cellCount = row * col;
        }

        public DualSerialPressureReader(ProtocolConfig config)
        {
            _row = config.RowCount;
            _col = config.ColCount;
            _cellCount = _row * _col;
            _config = config;
        }

        /// <summary>
        /// Open both ports. If a port string is null or empty, auto-detect from
        /// available serial ports. When both are empty, assigns the last two
        /// available ports (never the same one twice).
        /// </summary>
        public void Open(string portA, string portB, int baudRate = 460800)
        {
            var available = SerialBridge.GetPortNames();

            Debug.Log($"[DualReader] 可用串口: {(available.Length > 0 ? string.Join(", ", available) : "(无)")}");

            string resolvedA = ResolvePort(portA, available, exclude: null);
            string resolvedB = ResolvePort(portB, available, exclude: resolvedA);

            // 防止两个 reader 打开同一个串口
            if (!string.IsNullOrEmpty(resolvedA) && resolvedA == resolvedB)
                resolvedB = null;

            Debug.Log($"[DualReader] A={resolvedA ?? "(未开)"}  B={resolvedB ?? "(未开)"}");

            if (!string.IsNullOrEmpty(resolvedA))
            {
                _readerA = _config != null ? new SerialPressureReader(_config) : new SerialPressureReader(_row, _col);
                _readerA.OnFrame += OnFrameA;
                _readerA.Open(resolvedA, baudRate);
            }

            if (!string.IsNullOrEmpty(resolvedB))
            {
                _readerB = _config != null ? new SerialPressureReader(_config) : new SerialPressureReader(_row, _col);
                _readerB.OnFrame += OnFrameB;
                _readerB.Open(resolvedB, baudRate);
            }

            _merged = new int[_cellCount * 2];
        }

        static string ResolvePort(string requested, string[] available, string exclude)
        {
            // Explicitly specified → use as-is
            if (!string.IsNullOrEmpty(requested))
                return requested;

            // Auto-detect: pick the last available port that isn't already taken
            for (int i = available.Length - 1; i >= 0; i--)
            {
                if (available[i] != exclude)
                    return available[i];
            }
            return null;
        }

        public void Close()
        {
            _readerA?.Close();
            _readerB?.Close();
            _readerA = null;
            _readerB = null;
            _dataA = null;
            _dataB = null;
        }

        public void Swap() => _swapped = !_swapped;

        public PressureInfo[] GetPressureInfo(RadiusMode mode = RadiusMode.Direction)
        {
            if (_merged == null) return System.Array.Empty<PressureInfo>();
            return PressureAnalyzer.GetPressureInfo(_merged, _row, _col * 2, mode);
        }

        public ChessPieceInfo[] GetChessPieceInfo()
        {
            if (_merged == null) return System.Array.Empty<ChessPieceInfo>();
            return PressureAnalyzer.GetChessPieceInfo(_merged, _row, _col * 2);
        }

        void OnFrameA(int[] data, int w, int h)
        {
            if (++_frameCountA == 1)
                Debug.Log($"[DualReader] A 首帧到达  len={data.Length}  cellCount={_cellCount}  readerB={_readerB != null}");

            _dataA = data;
            int[] right = _readerB != null ? (_dataB ?? Zeros) : Zeros;
            Merge(_dataA, right);
        }

        void OnFrameB(int[] data, int w, int h)
        {
            if (++_frameCountB == 1)
                Debug.Log($"[DualReader] B 首帧到达  len={data.Length}  cellCount={_cellCount}  readerA={_readerA != null}");

            _dataB = data;
            int[] left = _readerA != null ? (_dataA ?? Zeros) : Zeros;
            Merge(left, _dataB);
        }

        void Merge(int[] dataA, int[] dataB)
        {
            int[] left  = _swapped ? dataB : dataA;
            int[] right = _swapped ? dataA : dataB;

            // Row-major interleave: each output row = left_row + right_row.
            int outH = _col * 2;
            int outW = _row;
            for (int r = 0, s = 0, d = 0; r < outW; r++, s += _col, d += outH)
            {
                System.Array.Copy(left,  s, _merged, d,          _col);
                System.Array.Copy(right, s, _merged, d + _col,   _col);
            }

            OnFrame?.Invoke(_merged, outW, outH); // width, height
        }
    }
}

namespace DevicePipe
{
    /// <summary>
    /// Auto-detects BitsPerSample (8 or 16) from raw frame bytes.
    /// Feed bytes via <see cref="Feed"/>; check <see cref="Done"/> and read <see cref="DetectedBits"/> when complete.
    /// </summary>
    public class BitsPerSampleDetector
    {
        readonly byte[] _header;
        readonly int _hdrLen;
        readonly int _samples;

        System.Collections.Generic.List<byte> _buf;
        int _detectedBits;
        int _matchPos;
        int _frameLenField;
        bool _done;

        /// <summary>Whether detection has finished (successfully or not).</summary>
        public bool Done => _done;

        /// <summary>Detected bits: 8 or 16. Only valid when <see cref="Done"/> is true and value is positive.</summary>
        public int DetectedBits => _detectedBits;

        /// <summary>Buffer position where the frame header was found (valid when <see cref="Done"/>).</summary>
        public int MatchPosition => _matchPos;

        /// <summary>Raw frame length field value at the time of match.</summary>
        public int FrameLenField => _frameLenField;

        /// <summary>Buffer size limit to prevent unbounded growth.</summary>
        public int MaxBufferSize = 4096;

        /// <summary>Number of buffered bytes past <paramref name="consumed"/>.</summary>
        public int RemainingAfter(int consumed) =>
            _buf == null ? 0 : System.Math.Max(0, _buf.Count - consumed);

        /// <summary>Copy buffered bytes from <paramref name="consumed"/> onward to <paramref name="dst"/>.</summary>
        public void CopyRemaining(int consumed, byte[] dst, int dstOffset)
        {
            int n = RemainingAfter(consumed);
            if (n > 0) _buf.CopyTo(consumed, dst, dstOffset, n);
        }

        /// <param name="header">Frame header bytes, e.g. 0xA5 0x5A 0x01</param>
        /// <param name="hdrLen">Header byte count used in length-field calculation</param>
        /// <param name="samples">Number of samples per frame (RowCount × ColCount)</param>
        public BitsPerSampleDetector(byte[] header, int hdrLen, int samples)
        {
            _header = header;
            _hdrLen = hdrLen;
            _samples = samples;
            _buf = new System.Collections.Generic.List<byte>(MaxBufferSize);
        }

        /// <summary>
        /// Feed a chunk of raw bytes. Returns true once a valid frame header is found
        /// and BitsPerSample has been inferred.
        /// </summary>
        public bool Feed(byte[] data, int offset, int count)
        {
            if (_done) return true;

            for (int i = offset; i < offset + count; i++)
                _buf.Add(data[i]);

            if (_buf.Count > MaxBufferSize)
                _buf.RemoveRange(0, _buf.Count - MaxBufferSize);

            return Scan();
        }

        bool Scan()
        {
            for (int i = 0; i <= _buf.Count - _hdrLen - 2; i++)
            {
                if (!MatchHeader(i)) continue;

                int frameLenField = _buf[i + _hdrLen] + (_buf[i + _hdrLen + 1] << 8);
                int bits = InferBits(frameLenField, _samples, _hdrLen);

                if (bits > 0)
                {
                    _detectedBits = bits;
                    _matchPos = i;
                    _frameLenField = frameLenField;
                    _done = true;
                    return true;
                }
            }
            return false;
        }

        bool MatchHeader(int pos)
        {
            for (int j = 0; j < _header.Length; j++)
                if (_buf[pos + j] != _header[j])
                    return false;
            return true;
        }

        /// <summary>
        /// frameLenField = headLen + samples × (bitsPerSample / 8).
        /// Tries headLen ∈ [hdrLen .. hdrLen+4] and returns the first valid 8 or 16.
        /// </summary>
        public static int InferBits(int frameLenField, int samples, int hdrLen)
        {
            for (int hl = hdrLen; hl <= hdrLen + 4; hl++)
            {
                int pay = frameLenField - hl;
                if (pay <= 0 || pay % samples != 0) continue;
                int bps = pay / samples;
                int bits = bps * 8;
                if (bits == 8 || bits == 16) return bits;
            }
            return -1;
        }

        /// <summary>Release the internal buffer.</summary>
        public void Reset()
        {
            _done = false;
            _detectedBits = -1;
            _buf?.Clear();
        }
    }
}

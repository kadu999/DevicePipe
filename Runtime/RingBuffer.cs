using System;

namespace DevicePipe
{
    /// <summary>Thread-safe circular byte buffer.</summary>
    public class RingBuffer
    {
        readonly byte[] _buf;
        int _start;
        int _end;
        int _count;
        readonly object _lock = new();

        public RingBuffer(int capacity) => _buf = new byte[capacity];

        public int Count { get { lock (_lock) return _count; } }
        public int Capacity => _buf.Length;
        public int Free => _buf.Length - Count;

        public void Write(byte[] src, int offset, int count)
        {
            if (count <= 0) return;
            lock (_lock)
            {
                if (Free < count) return; // overflow, discard

                int first = Math.Min(count, _buf.Length - _end);
                Array.Copy(src, offset, _buf, _end, first);
                int second = count - first;
                if (second > 0)
                    Array.Copy(src, offset + first, _buf, 0, second);

                _end = (_end + count) % _buf.Length;
                _count += count;
            }
        }

        public int Read(byte[] dst, int offset, int count)
        {
            lock (_lock)
            {
                int actual = Math.Min(count, _count);
                if (actual <= 0) return 0;

                int first = Math.Min(actual, _buf.Length - _start);
                Array.Copy(_buf, _start, dst, offset, first);
                int second = actual - first;
                if (second > 0)
                    Array.Copy(_buf, 0, dst, offset + first, second);

                _start = (_start + actual) % _buf.Length;
                _count -= actual;
                return actual;
            }
        }

        /// <summary>Peek without consuming.</summary>
        public byte this[int index]
        {
            get
            {
                lock (_lock)
                {
                    if (index >= _count) throw new IndexOutOfRangeException();
                    int pos = _start + index;
                    return pos < _buf.Length ? _buf[pos] : _buf[pos - _buf.Length];
                }
            }
        }

        public void Discard(int count)
        {
            lock (_lock)
            {
                int actual = Math.Min(count, _count);
                _start = (_start + actual) % _buf.Length;
                _count -= actual;
            }
        }

        public void Clear()
        {
            lock (_lock) { _start = _end = _count = 0; }
        }
    }
}

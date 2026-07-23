using System.Collections.Generic;
using UnityEngine;

namespace DevicePipe
{
    public struct PressureInfo
    {
        public float x, y;
        public int pressure;
        public float radius;
    }

    public enum RadiusMode { Direction, Square }

    public static class PressureAnalyzer
    {
        const int Threshold = 3;
        const float Sigma = 1.0f;
        const int Neighborhood = 5;
        const float MergeDist = 5.0f;

        static float[] _kernel;
        static int _kernelRadius;
        static float[,] _bufA, _bufB;
        static int _w, _h;

        static void Resize(int w, int h)
        {
            if (_w == w && _h == h) return;
            _w = w; _h = h;
            _bufA = new float[w, h];
            _bufB = new float[w, h];
        }

        public static PressureInfo[] GetPressureInfo(int[] data, int width, int height,
                                                      RadiusMode mode = RadiusMode.Direction)
        {
            EnsureKernel();
            Resize(width, height);

            int kr = _kernelRadius;

            // Horizontal Gaussian pass: data → _bufA
            for (int y = 0; y < height; y++)
                for (int x = 0; x < width; x++)
                {
                    float sum = 0, wsum = 0;
                    for (int k = -kr; k <= kr; k++)
                    {
                        int sx = x + k;
                        if ((uint)sx >= width) continue;
                        float kv = _kernel[k + kr];
                        sum += data[sx * height + y] * kv;
                        wsum += kv;
                    }
                    _bufA[x, y] = wsum > 0 ? sum / wsum : 0;
                }

            // Vertical Gaussian pass: _bufA → _bufB
            for (int x = 0; x < width; x++)
                for (int y = 0; y < height; y++)
                {
                    float sum = 0, wsum = 0;
                    for (int k = -kr; k <= kr; k++)
                    {
                        int sy = y + k;
                        if ((uint)sy >= height) continue;
                        float kv = _kernel[k + kr];
                        sum += _bufA[x, sy] * kv;
                        wsum += kv;
                    }
                    _bufB[x, y] = wsum > 0 ? sum / wsum : 0;
                }

            // Find local maxima in _bufB
            var peaks = new List<(int x, int y, float val)>(16);
            int half = Neighborhood / 2;
            for (int x = 0; x < width; x++)
                for (int y = 0; y < height; y++)
                {
                    float v = _bufB[x, y];
                    if (v <= Threshold) continue;
                    bool isMax = true;
                    for (int dx = -half; dx <= half && isMax; dx++)
                        for (int dy = -half; dy <= half && isMax; dy++)
                        {
                            int nx = x + dx, ny = y + dy;
                            if ((uint)nx >= width || (uint)ny >= height) continue;
                            if (_bufB[nx, ny] > v) { isMax = false; break; }
                        }
                    if (isMax) peaks.Add((x, y, v));
                }

            // Sort by pressure, merge close peaks
            peaks.Sort((a, b) => b.val.CompareTo(a.val));
            var result = new List<PressureInfo>();
            foreach (var p in peaks)
            {
                bool tooClose = false;
                foreach (var r in result)
                {
                    float d = Mathf.Sqrt((p.x - r.x) * (p.x - r.x) + (p.y - r.y) * (p.y - r.y));
                    if (d < MergeDist) { tooClose = true; break; }
                }
                if (!tooClose)
                {
                    float r = mode == RadiusMode.Square
                        ? ComputeRadiusBySquare(_bufB, p.x, p.y, absoluteThreshold: 3f)
                        : ComputeRadiusByDirection(p.x, p.y, p.val);
                    result.Add(new PressureInfo { x = p.x, y = p.y, pressure = (int)p.val, radius = r });
                }
            }
            return result.ToArray();
        }

        static float ComputeRadiusByDirection(int cx, int cy, float peakVal)
        {
            float threshold = Mathf.Max(peakVal * 0.3f, 3f);
            int maxDist = 50;

            int left = 0, right = 0, up = 0, down = 0;
            while (left < maxDist && cx - left > 0 && _bufB[cx - left - 1, cy] > threshold) left++;
            while (right < maxDist && cx + right + 1 < _w && _bufB[cx + right + 1, cy] > threshold) right++;
            while (up < maxDist && cy - up > 0 && _bufB[cx, cy - up - 1] > threshold) up++;
            while (down < maxDist && cy + down + 1 < _h && _bufB[cx, cy + down + 1] > threshold) down++;

            return (left + right + up + down) / 4f + 1f;
        }

        /// <summary>
        /// Python-style uniform square expansion radius estimator.
        /// Grows a square around the peak until any boundary pixel drops below absoluteThreshold.
        /// (Not currently used — saved for comparison.)
        /// </summary>
        static float ComputeRadiusBySquare(float[,] smoothed, int cx, int cy, float absoluteThreshold, int maxR = 50)
        {
            int h = smoothed.GetLength(1);
            int w = smoothed.GetLength(0);
            int r = 1;
            while (r < maxR && r < Mathf.Max(w, h))
            {
                int yMin = Mathf.Max(0, cy - r);
                int yMax = Mathf.Min(h - 1, cy + r);
                int xMin = Mathf.Max(0, cx - r);
                int xMax = Mathf.Min(w - 1, cx + r);

                bool below = false;
                for (int x = xMin; x <= xMax && !below; x++)
                {
                    if (smoothed[x, yMin] < absoluteThreshold) below = true;
                    if (smoothed[x, yMax] < absoluteThreshold) below = true;
                }
                for (int y = yMin; y <= yMax && !below; y++)
                {
                    if (smoothed[xMin, y] < absoluteThreshold) below = true;
                    if (smoothed[xMax, y] < absoluteThreshold) below = true;
                }

                if (below && r > 1) break;
                r++;
            }
            return r;
        }

        static void EnsureKernel()
        {
            if (_kernel != null) return;
            int radius = Mathf.CeilToInt(Sigma * 3);
            int size = radius * 2 + 1;
            _kernelRadius = radius;
            _kernel = new float[size];
            float sum = 0;
            for (int i = 0; i < size; i++)
            {
                float x = i - radius;
                _kernel[i] = Mathf.Exp(-(x * x) / (2 * Sigma * Sigma));
                sum += _kernel[i];
            }
            for (int i = 0; i < size; i++) _kernel[i] /= sum;
        }
    }
}

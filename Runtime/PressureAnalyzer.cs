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

    public struct ChessPieceInfo
    {
        public float pos_x, pos_y;
        public float radius;
        public float dir_x, dir_y;
        public float major, minor, angle;
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

        // ── Ring / Chess Piece Detection ──────────────────────
        const int SkeletonThresh = 25;
        const float CircThresh = 0.4f;
        const float AspectThresh = 3.0f;
        const int MinRingArea = 10;

        static byte[] _binBuf;
        static byte[] _skelA;
        static byte[] _skelB;
        static int[] _labelBuf;
        static int[] _bfsQueue;
        static int _bufCapacity;

        static readonly int[] NeighborDx8 = {  0,  1,  1,  1,  0, -1, -1, -1 };
        static readonly int[] NeighborDy8 = { -1, -1,  0,  1,  1,  1,  0, -1 };

        static void ResizeBufs(int size)
        {
            if (_bufCapacity >= size) return;
            _bufCapacity = size;
            _binBuf = new byte[size];
            _skelA = new byte[size];
            _skelB = new byte[size];
            _labelBuf = new int[size];
            _bfsQueue = new int[size];
        }

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

        // ── Ring / Chess Piece Detection ──────────────────────

        struct ComponentInfo
        {
            public int sumX, sumY;
            public int pixelCount;
            public int compId;
        }

        static void GetNeighbors8(byte[] buf, int x, int y, int w, int h, int[] n)
        {
            for (int d = 0; d < 8; d++)
            {
                int nx = x + NeighborDx8[d];
                int ny = y + NeighborDy8[d];
                n[d] = ((uint)nx < (uint)w && (uint)ny < (uint)h && buf[nx * h + ny] != 0) ? 1 : 0;
            }
        }

        static int CountNeighbors8(int[] n)
        {
            int sum = 0;
            for (int i = 0; i < 8; i++) sum += n[i];
            return sum;
        }

        static int TransitionCount(int[] n)
        {
            int transitions = 0;
            for (int i = 0; i < 8; i++)
                if (n[i] == 0 && n[(i + 1) % 8] == 1) transitions++;
            return transitions;
        }

        static void ApplyBinaryThreshold(int[] data, byte[] binary, int w, int h, int threshold)
        {
            int total = w * h;
            for (int i = 0; i < total; i++)
                binary[i] = data[i] > threshold ? (byte)1 : (byte)0;
        }

        /// <summary>Zhang-Suen thinning. Input in binary, result in _skelA.</summary>
        static void ZhangSuenSkeletonize(byte[] binary, byte[] skelA, byte[] skelB, int w, int h)
        {
            int total = w * h;
            System.Array.Copy(binary, skelA, total);
            int[] n = new int[8];
            bool changed;

            do
            {
                changed = false;

                // ── Sub-iteration 1 ──
                System.Array.Copy(skelA, skelB, total);
                for (int i = 0; i < total; i++)
                {
                    if (skelA[i] == 0) continue;
                    int x = i / h, y = i % h;
                    GetNeighbors8(skelA, x, y, w, h, n);
                    int B = CountNeighbors8(n);
                    if (B < 2 || B > 6) continue;
                    if (TransitionCount(n) != 1) continue;
                    // P2*P4*P6 == 0  (n[0]*n[2]*n[4])
                    if (n[0] != 0 && n[2] != 0 && n[4] != 0) continue;
                    // P4*P6*P8 == 0  (n[2]*n[4]*n[6])
                    if (n[2] != 0 && n[4] != 0 && n[6] != 0) continue;
                    skelB[i] = 0;
                    changed = true;
                }

                // ── Sub-iteration 2 ──
                System.Array.Copy(skelB, skelA, total);
                for (int i = 0; i < total; i++)
                {
                    if (skelB[i] == 0) continue;
                    int x = i / h, y = i % h;
                    GetNeighbors8(skelB, x, y, w, h, n);
                    int B = CountNeighbors8(n);
                    if (B < 2 || B > 6) continue;
                    if (TransitionCount(n) != 1) continue;
                    // P2*P4*P8 == 0  (n[0]*n[2]*n[6])
                    if (n[0] != 0 && n[2] != 0 && n[6] != 0) continue;
                    // P2*P6*P8 == 0  (n[0]*n[4]*n[6])
                    if (n[0] != 0 && n[4] != 0 && n[6] != 0) continue;
                    skelA[i] = 0;
                    changed = true;
                }
            } while (changed);
            // result is in _skelA
        }

        /// <summary>BFS connected-component labeling on skeleton (8-connected).</summary>
        static List<ComponentInfo> FindComponents(
            byte[] skeleton, int[] labels, int[] queue, int w, int h)
        {
            int total = w * h;
            System.Array.Clear(labels, 0, total);
            var components = new List<ComponentInfo>();
            int compId = 0;

            for (int seed = 0; seed < total; seed++)
            {
                if (skeleton[seed] == 0 || labels[seed] != 0) continue;

                compId++;
                int head = 0, tail = 0;
                queue[tail++] = seed;
                labels[seed] = compId;

                int sumX = 0, sumY = 0, count = 0;

                while (head < tail)
                {
                    int idx = queue[head++];
                    int x = idx / h, y = idx % h;
                    sumX += x; sumY += y; count++;

                    for (int d = 0; d < 8; d++)
                    {
                        int nx = x + NeighborDx8[d];
                        int ny = y + NeighborDy8[d];
                        if ((uint)nx >= (uint)w || (uint)ny >= (uint)h) continue;
                        int nidx = nx * h + ny;
                        if (skeleton[nidx] == 0 || labels[nidx] != 0) continue;
                        labels[nidx] = compId;
                        queue[tail++] = nidx;
                    }
                }

                components.Add(new ComponentInfo
                {
                    sumX = sumX, sumY = sumY,
                    pixelCount = count,
                    compId = compId
                });
            }

            return components;
        }

        /// <summary>Validate ring shape and fill ChessPieceInfo.</summary>
        static bool TryFitRing(byte[] skeleton, int[] labels, int compId,
                               ComponentInfo comp, int w, int h, out ChessPieceInfo result)
        {
            result = default;

            // ── Area filter ──
            if (comp.pixelCount < MinRingArea) return false;

            float cx = (float)comp.sumX / comp.pixelCount;
            float cy = (float)comp.sumY / comp.pixelCount;

            // ── Collect component pixels for ellipse + ordering ──
            int total = w * h;
            var points = new List<(int x, int y)>(comp.pixelCount);
            for (int i = 0; i < total; i++)
                if (labels[i] == compId)
                    points.Add((i / h, i % h));

            // Sort by angle around centroid (for polygon area/perimeter)
            points.Sort((a, b) =>
            {
                float angA = Mathf.Atan2(a.y - cy, a.x - cx);
                float angB = Mathf.Atan2(b.y - cy, b.x - cx);
                return angA.CompareTo(angB);
            });

            // ── Polygon area (shoelace) and perimeter ──
            float polyArea = 0f, perimeter = 0f;
            int npts = points.Count;
            for (int i = 0; i < npts; i++)
            {
                int j = (i + 1) % npts;
                float x1 = points[i].x, y1 = points[i].y;
                float x2 = points[j].x, y2 = points[j].y;
                polyArea += x1 * y2 - x2 * y1;
                float dx = x2 - x1, dy = y2 - y1;
                perimeter += Mathf.Sqrt(dx * dx + dy * dy);
            }
            polyArea = Mathf.Abs(polyArea) * 0.5f;

            // ── Circularity check ──
            if (perimeter > 1e-6f)
            {
                float circularity = 4f * Mathf.PI * polyArea / (perimeter * perimeter);
                if (circularity < CircThresh) return false;
            }

            // ── PCA ellipse fit ──
            float covXX = 0f, covYY = 0f, covXY = 0f;
            for (int i = 0; i < npts; i++)
            {
                float dx = points[i].x - cx;
                float dy = points[i].y - cy;
                covXX += dx * dx;
                covYY += dy * dy;
                covXY += dx * dy;
            }
            covXX /= npts; covYY /= npts; covXY /= npts;

            float trace = covXX + covYY;
            float det = covXX * covYY - covXY * covXY;
            float disc = Mathf.Sqrt(Mathf.Max(0f, trace * trace - 4f * det));
            float lambda1 = (trace + disc) * 0.5f; // larger
            float lambda2 = (trace - disc) * 0.5f; // smaller

            if (lambda2 < 1e-6f) return false; // degenerate

            float major = 2f * Mathf.Sqrt(2f * lambda1);
            float minor = 2f * Mathf.Sqrt(2f * lambda2);

            // Eigenvector for major axis → ellipse orientation angle
            float angle = 0f;
            float vx = lambda1 - covYY;
            float vy = covXY;
            float vlen = Mathf.Sqrt(vx * vx + vy * vy);
            if (vlen > 1e-6f)
                angle = Mathf.Atan2(vy / vlen, vx / vlen) * Mathf.Rad2Deg;

            // ── Aspect ratio check ──
            float aspect = major / minor;
            if (aspect > AspectThresh) return false;

            // ── Junction detection (nearest branch point) ──
            float jx = 0f, jy = 0f;
            float bestDist = float.MaxValue;
            bool found = false;
            for (int i = 0; i < npts; i++)
            {
                int x = points[i].x, y = points[i].y;
                int nCount = 0;
                for (int d = 0; d < 8; d++)
                {
                    int nx = x + NeighborDx8[d];
                    int ny = y + NeighborDy8[d];
                    if ((uint)nx < (uint)w && (uint)ny < (uint)h && skeleton[nx * h + ny] != 0)
                        nCount++;
                }
                if (nCount == 3)
                {
                    float dist = (x - cx) * (x - cx) + (y - cy) * (y - cy);
                    if (dist < bestDist) { bestDist = dist; jx = x; jy = y; found = true; }
                }
            }

            float dir_x = found ? jx - cx : 0f;
            float dir_y = found ? jy - cy : 0f;
            float radius = (major + minor) * 0.25f; // average semi-axis

            result = new ChessPieceInfo
            {
                pos_x = cx, pos_y = cy,
                radius = radius,
                dir_x = dir_x, dir_y = dir_y,
                major = major, minor = minor, angle = angle
            };
            return true;
        }

        /// <summary>Detect ring-shaped contacts (chess pieces) from pressure frame.</summary>
        public static ChessPieceInfo[] GetChessPieceInfo(int[] data, int width, int height)
        {
            if (data == null || width <= 0 || height <= 0)
                return System.Array.Empty<ChessPieceInfo>();

            int total = width * height;
            ResizeBufs(total);

            // Stage 1: binary threshold
            ApplyBinaryThreshold(data, _binBuf, width, height, SkeletonThresh);

            // Stage 2: Zhang-Suen skeletonization → _skelA
            ZhangSuenSkeletonize(_binBuf, _skelA, _skelB, width, height);

            // Stage 3: connected components
            var components = FindComponents(_skelA, _labelBuf, _bfsQueue, width, height);

            // Stage 4-5: filter + fit + junction → results
            var results = new List<ChessPieceInfo>();
            foreach (var comp in components)
            {
                if (TryFitRing(_skelA, _labelBuf, comp.compId, comp,
                               width, height, out var piece))
                    results.Add(piece);
            }

            return results.Count > 0 ? results.ToArray() : System.Array.Empty<ChessPieceInfo>();
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

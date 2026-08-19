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

        // ── Touch Filter (ported from ring_pressure_viewer.py) ──────────────
        // Master switch — when ON, GetPressureInfo applies the filter chain:
        //   area/strength → ring-boundary exclusion → inside-piece exclusion → temporal EMA
        public static bool FilterEnabled = false;

        // Stage 1: area & strength (python: π·r² ≥ 20 and peak ≥ 60)
        public static float FilterMinArea = 20f;
        public static float FilterMinPressure = 60f;

        // Detection (used only while the filter is ON — mirrors python TouchDetector):
        //   TOUCH_THRESHOLD = 12 and square-expansion radius until a pixel < 1
        public static float FilterDetectThreshold = 12f;

        // Stage 2: drop touches whose distance to a ring's ellipse boundary is < this
        public static bool FilterExcludeNearRings = true;
        public static float FilterRingExcludeDist = 4f;

        // Stage 3: drop touches inside a detected piece circle (python _detect_pieces,
        // which runs on peak-held data with tighter shape thresholds)
        public static bool FilterExcludeInsidePieces = true;

        // Stage 4 (extension beyond the python pipeline): temporal EMA smoothing
        // across frames (0 = off → behaves exactly like python; >0 blends each touch
        // with its nearest previous filtered touch)
        public static float FilterTemporalAlpha = 0f;

        static PressureInfo[] _prevFiltered;
        static int _prevFilterW, _prevFilterH;
        static bool _filterWasEnabled;

        // ── Piece detection state (python _apply_peak_hold + _detect_pieces) ──
        const int MinPieceArea = 10;
        const float PieceCircThresh = 0.60f;
        const float PieceAspectThresh = 1.7f;
        const float PieceRadiusMin = 4f;

        static float[] _peakBuf;
        static byte[] _dilBuf;
        static int _peakW, _peakH;

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
            return GetPressureInfo(data, width, height, mode, FilterEnabled);
        }

        public static PressureInfo[] GetPressureInfo(int[] data, int width, int height,
                                                      RadiusMode mode, bool enableFilter)
        {
            int inW = width, inH = height;

            // Reset temporal filter state on enable/disable or dimension change
            if (enableFilter != _filterWasEnabled ||
                (enableFilter && (inW != _prevFilterW || inH != _prevFilterH)))
            {
                _prevFiltered = null;
                if (_peakBuf != null) System.Array.Clear(_peakBuf, 0, _peakBuf.Length);
                _prevFilterW = inW;
                _prevFilterH = inH;
                _filterWasEnabled = enableFilter;
            }

            (width, height) = (height, width);

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
            // python TOUCH_THRESHOLD=12 while the filter is on, else the C# Threshold
            float thr = enableFilter ? FilterDetectThreshold : Threshold;
            for (int x = 0; x < width; x++)
                for (int y = 0; y < height; y++)
                {
                    float v = _bufB[x, y];
                    if (v <= thr) continue;
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
                    // python _estimate_radius: uniform square expansion until any pixel < 1
                    float r = enableFilter
                        ? ComputeRadiusBySquare(_bufB, p.x, p.y, absoluteThreshold: 1f)
                        : mode == RadiusMode.Square
                            ? ComputeRadiusBySquare(_bufB, p.x, p.y, absoluteThreshold: 3f)
                            : ComputeRadiusByDirection(p.x, p.y, p.val);
                    result.Add(new PressureInfo { x = p.x, y = p.y, pressure = (int)p.val, radius = r });
                }
            }

            PressureInfo[] touches = result.ToArray();
            if (enableFilter)
            {
                touches = ApplyTouchFilter(touches, data, inW, inH);
                touches = ApplyTemporalSmoothing(touches);
            }
            return touches;
        }

        // ── Touch Filter chain (ported from ring_pressure_viewer.py) ────────

        /// <summary>
        /// Post-detection filters, mirroring ring_pressure_viewer.py _render():
        ///   1. area & strength: keep π·r² ≥ FilterMinArea and pressure ≥ FilterMinPressure
        ///   2. ring-boundary exclusion: drop touches within FilterRingExcludeDist of a
        ///      detected ring's ellipse boundary (python _filter_touches_near_rings)
        ///   3. inside-piece exclusion: drop touches inside a detected piece circle
        ///      (python _detect_pieces on peak-held data)
        /// </summary>
        static PressureInfo[] ApplyTouchFilter(PressureInfo[] touches, int[] data, int inW, int inH)
        {
            // ── Stage 1: area & strength (python: π·r² ≥ 20 and peak ≥ 60) ──
            var list = new List<PressureInfo>(touches.Length);
            foreach (var t in touches)
            {
                if (FilterMinArea > 0f && Mathf.PI * t.radius * t.radius < FilterMinArea) continue;
                if (FilterMinPressure > 0f && t.pressure < FilterMinPressure) continue;
                list.Add(t);
            }
            if (list.Count == 0) return System.Array.Empty<PressureInfo>();
            PressureInfo[] current = list.ToArray();

            // ── Stage 2: ring-boundary exclusion (python _filter_touches_near_rings) ──
            if (FilterExcludeNearRings)
            {
                ChessPieceInfo[] rings = GetChessPieceInfo(data, inW, inH);
                var kept = new List<PressureInfo>(current.Length);
                foreach (var t in current)
                {
                    bool drop = false;
                    for (int i = 0; i < rings.Length && !drop; i++)
                        if (DistToEllipseBoundary(t.x, t.y, rings[i]) <= FilterRingExcludeDist)
                            drop = true;
                    if (!drop) kept.Add(t);
                }
                current = kept.ToArray();
                if (current.Length == 0) return current;
            }

            // ── Stage 3: inside-piece exclusion (python _detect_pieces) ──
            if (FilterExcludeInsidePieces)
            {
                var pieces = DetectPieces(data, inW, inH);
                var kept = new List<PressureInfo>(current.Length);
                foreach (var t in current)
                {
                    bool inside = false;
                    for (int i = 0; i < pieces.Count && !inside; i++)
                    {
                        float dx = t.x - pieces[i].Item1;
                        float dy = t.y - pieces[i].Item2;
                        if (dx * dx + dy * dy < pieces[i].Item3 * pieces[i].Item3)
                            inside = true;
                    }
                    if (!inside) kept.Add(t);
                }
                current = kept.ToArray();
            }

            return current;
        }

        /// <summary>
        /// Distance from point (px, py) to the boundary of ring's fitted ellipse.
        /// Python uses cv2.ellipse2Poly (1° steps → 360 points) to discretize the
        /// boundary; we sample the same 360 parametric points and take the min distance.
        /// </summary>
        static float DistToEllipseBoundary(float px, float py, ChessPieceInfo ring)
        {
            float a = ring.major * 0.5f;
            float b = ring.minor * 0.5f;
            if (a <= 0f || b <= 0f) return float.MaxValue;

            float cx = ring.pos_x, cy = ring.pos_y;
            float theta = ring.angle * Mathf.Deg2Rad;
            float cosT = Mathf.Cos(theta), sinT = Mathf.Sin(theta);

            float bestSq = float.MaxValue;
            const int N = 360;
            for (int i = 0; i < N; i++)
            {
                float t = i * (2f * Mathf.PI) / N;
                float ct = Mathf.Cos(t), st = Mathf.Sin(t);
                float ex = cx + a * ct * cosT - b * st * sinT;
                float ey = cy + a * ct * sinT + b * st * cosT;
                float dx = px - ex, dy = py - ey;
                float d = dx * dx + dy * dy;
                if (d < bestSq) bestSq = d;
            }
            return Mathf.Sqrt(bestSq);
        }

        // ── Piece detection (port of python _apply_peak_hold + _detect_pieces) ──

        /// <summary>
        /// Python _apply_peak_hold: cross-kernel dilate (data &gt; 0), then
        /// peak[dilated] = max(peak, data), peak[~dilated] = 0.
        /// </summary>
        static void ApplyPeakHold(int[] data, int inW, int inH)
        {
            int total = inW * inH;
            if (_peakBuf == null || _peakW != inW || _peakH != inH)
            {
                _peakBuf = new float[total];
                _peakW = inW;
                _peakH = inH;
            }
            if (_dilBuf == null || _dilBuf.Length < total)
                _dilBuf = new byte[total];

            // data layout: idx = x * inW + y  (x ∈ [0, inH), y ∈ [0, inW))
            for (int x = 0; x < inH; x++)
            {
                int rowBase = x * inW;
                for (int y = 0; y < inW; y++)
                {
                    int i = rowBase + y;
                    bool active = data[i] > 0;
                    if (!active)
                    {
                        if (x > 0 && data[i - inW] > 0) active = true;
                        else if (x + 1 < inH && data[i + inW] > 0) active = true;
                        else if (y > 0 && data[i - 1] > 0) active = true;
                        else if (y + 1 < inW && data[i + 1] > 0) active = true;
                    }
                    _dilBuf[i] = active ? (byte)1 : (byte)0;
                }
            }

            for (int i = 0; i < total; i++)
                _peakBuf[i] = _dilBuf[i] != 0 ? Mathf.Max(_peakBuf[i], data[i]) : 0f;
        }

        /// <summary>
        /// Python _detect_pieces: threshold peak-held data, find outer blobs, and keep
        /// blobs with area ≥ MIN_AREA, circularity ≥ 0.60, aspect ≤ 1.7, radius ≥ 4.
        /// Returns (cx, cy, radius) per piece.
        /// </summary>
        static List<(float, float, float)> DetectPieces(int[] data, int inW, int inH)
        {
            ApplyPeakHold(data, inW, inH);
            int total = inW * inH;
            ResizeBufs(total);

            // cv2.threshold(peak_data, SkeletonThresh, 255, THRESH_BINARY)
            for (int i = 0; i < total; i++)
                _binBuf[i] = _peakBuf[i] > SkeletonThresh ? (byte)1 : (byte)0;

            // cv2.findContours(RETR_EXTERNAL) ≈ 8-connected component labeling
            var components = FindComponents(_binBuf, _labelBuf, _bfsQueue, inH, inW);

            var pieces = new List<(float, float, float)>();
            foreach (var comp in components)
                if (TryFitPiece(_binBuf, _labelBuf, comp.compId, comp, inH, inW, out var p))
                    pieces.Add(p);
            return pieces;
        }

        /// <summary>
        /// Shape validation for one piece blob (python is_circle checks + fitEllipse):
        /// area, circularity (polygon area/perimeter), PCA ellipse aspect, radius.
        /// </summary>
        static bool TryFitPiece(byte[] binary, int[] labels, int compId, ComponentInfo comp,
                                int w, int h, out (float cx, float cy, float radius) piece)
        {
            piece = default;

            // python: area < MIN_AREA → reject
            if (comp.pixelCount < MinPieceArea) return false;

            int total = w * h;
            var points = new List<(int x, int y)>(comp.pixelCount);
            for (int i = 0; i < total; i++)
                if (labels[i] == compId)
                    points.Add((i / h, i % h));

            float cx = (float)comp.sumX / comp.pixelCount;
            float cy = (float)comp.sumY / comp.pixelCount;

            // Sort by angle around centroid (for polygon area/perimeter)
            points.Sort((a, b) =>
            {
                float angA = Mathf.Atan2(a.y - cy, a.x - cx);
                float angB = Mathf.Atan2(b.y - cy, b.x - cx);
                return angA.CompareTo(angB);
            });

            // ── Polygon area (shoelace) and perimeter → circularity ──
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

            if (perimeter > 1e-6f)
            {
                float circularity = 4f * Mathf.PI * polyArea / (perimeter * perimeter);
                if (circularity < PieceCircThresh) return false;
            }

            // ── PCA ellipse fit (python: cv2.fitEllipse) ──
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
            if (lambda2 < 1e-6f) return false;     // degenerate

            float major = 2f * Mathf.Sqrt(2f * lambda1);
            float minor = 2f * Mathf.Sqrt(2f * lambda2);

            // python: aspect = max(MA, ma) / min(MA, ma), reject if > 1.7
            float aspect = major / minor;
            if (aspect > PieceAspectThresh) return false;

            // python: radius = max(MA, ma) / 2.0, reject if < 4
            float radius = Mathf.Max(major, minor) * 0.5f;
            if (radius < PieceRadiusMin) return false;

            piece = (cx, cy, radius);
            return true;
        }

        /// <summary>
        /// Temporal EMA smoothing of reported touch positions/pressures across frames.
        /// Each new touch is blended with its nearest previous filtered touch (within 12px);
        /// unmatched touches pass through. State resets on empty frames, disable, or size change.
        /// </summary>
        static PressureInfo[] ApplyTemporalSmoothing(PressureInfo[] touches)
        {
            if (FilterTemporalAlpha <= 0f)
            {
                _prevFiltered = null;
                return touches;
            }
            if (touches.Length == 0)
            {
                _prevFiltered = System.Array.Empty<PressureInfo>();
                return touches;
            }
            if (_prevFiltered == null || _prevFiltered.Length == 0)
            {
                _prevFiltered = touches;
                return touches;
            }

            var result = new PressureInfo[touches.Length];
            const float matchSq = 12f * 12f;
            for (int i = 0; i < touches.Length; i++)
            {
                var t = touches[i];
                int best = -1;
                float bestSq = matchSq;
                for (int j = 0; j < _prevFiltered.Length; j++)
                {
                    float dx = t.x - _prevFiltered[j].x;
                    float dy = t.y - _prevFiltered[j].y;
                    float d = dx * dx + dy * dy;
                    if (d < bestSq) { bestSq = d; best = j; }
                }
                if (best >= 0)
                {
                    float a = FilterTemporalAlpha;
                    var p = _prevFiltered[best];
                    result[i] = new PressureInfo
                    {
                        x = p.x + (t.x - p.x) * a,
                        y = p.y + (t.y - p.y) * a,
                        pressure = Mathf.RoundToInt(p.pressure + (t.pressure - p.pressure) * a),
                        radius = p.radius + (t.radius - p.radius) * a,
                    };
                }
                else
                {
                    result[i] = t;
                }
            }
            _prevFiltered = result;
            return result;
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
        /// Python _estimate_radius: uniform square expansion radius estimator.
        /// Grows a square around the peak until any boundary pixel drops below absoluteThreshold.
        /// Used by the touch filter with absoluteThreshold=1 (python: region < 1).
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

            // ── Collect component pixels ──
            int total = w * h;
            var points = new List<(int x, int y)>(comp.pixelCount);
            for (int i = 0; i < total; i++)
                if (labels[i] == compId)
                    points.Add((i / h, i % h));

            // ── Robust center: filter out lever pixels by distance ──
            // Mean center is biased toward lever; ring pixels all sit at ~same
            // distance from true center. Filter to the dominant distance band.
            float cx0 = (float)comp.sumX / comp.pixelCount;
            float cy0 = (float)comp.sumY / comp.pixelCount;

            var dists = new float[points.Count];
            for (int i = 0; i < points.Count; i++)
                dists[i] = Mathf.Sqrt((points[i].x - cx0) * (points[i].x - cx0)
                                    + (points[i].y - cy0) * (points[i].y - cy0));
            System.Array.Sort(dists);
            float medDist = dists[points.Count / 2];

            // Recompute center from pixels near median distance (ring pixels)
            float cx = 0f, cy = 0f;
            int ringCount = 0;
            float lo = medDist * 0.5f, hi = medDist * 1.5f;
            for (int i = 0; i < points.Count; i++)
            {
                float d = Mathf.Sqrt((points[i].x - cx0) * (points[i].x - cx0)
                                   + (points[i].y - cy0) * (points[i].y - cy0));
                if (d >= lo && d <= hi)
                {
                    cx += points[i].x; cy += points[i].y; ringCount++;
                }
            }
            if (ringCount < MinRingArea) return false;
            cx /= ringCount; cy /= ringCount;

            // Sort by angle around robust centroid (for polygon area/perimeter)
            float sortCx = cx, sortCy = cy;
            points.Sort((a, b) =>
            {
                float angA = Mathf.Atan2(a.y - sortCy, a.x - sortCx);
                float angB = Mathf.Atan2(b.y - sortCy, b.x - sortCx);
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
            (width, height) = (height, width);

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

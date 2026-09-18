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

    public struct PieceInfo
    {
        public float pos_x, pos_y;
        public float radius;
        /// <summary>Stable per-piece ID assigned by PieceTracker (readers fill this in).</summary>
        public int id;
    }

    /// <summary>
    /// Assigns stable IDs to detected pieces across frames: each new detection is
    /// greedy-matched to the nearest previous piece within MatchDist and inherits
    /// its ID; unmatched detections get fresh IDs. Previous pieces stay alive for
    /// LostTimeout empty frames before their IDs are retired (python TrajectoryTracker
    /// semantics: MATCH_DIST_THRESHOLD = 30, LOST_TIMEOUT = 5).
    /// </summary>
    public class PieceTracker
    {
        public const float MatchDist = 30f;
        public const int LostTimeout = 5;

        struct Tracked
        {
            public PieceInfo info;
            public int misses;
        }

        Tracked[] _prev = System.Array.Empty<Tracked>();
        int _nextId = 1;

        // Reused per-tracker scratch (grown on demand, never exposed)
        bool[] _usedPrev = System.Array.Empty<bool>();
        int[] _idOf = System.Array.Empty<int>();
        readonly List<(float d, int pi, int di)> _pairs =
            new List<(float d, int pi, int di)>(64);

        public PieceInfo[] Track(PieceInfo[] detected)
        {
            int n = detected?.Length ?? 0;

            // Age out tracks that have been missing for too long
            int kept = 0;
            for (int i = 0; i < _prev.Length; i++)
                if (_prev[i].misses < LostTimeout) _prev[kept++] = _prev[i];
            if (kept != _prev.Length)
                System.Array.Resize(ref _prev, kept);

            if (n == 0)
            {
                for (int i = 0; i < _prev.Length; i++)
                    _prev[i].misses++;
                return System.Array.Empty<PieceInfo>();
            }

            var result = new PieceInfo[n];
            if (_usedPrev.Length < _prev.Length) _usedPrev = new bool[_prev.Length];
            System.Array.Clear(_usedPrev, 0, _prev.Length);
            if (_idOf.Length < n) _idOf = new int[n];
            else System.Array.Clear(_idOf, 0, n);
            var usedPrev = _usedPrev;
            var idOf = _idOf;

            // Greedy nearest-neighbor matching by ascending distance
            var pairs = _pairs;
            pairs.Clear();
            for (int d = 0; d < n; d++)
                for (int p = 0; p < _prev.Length; p++)
                {
                    float dx = detected[d].pos_x - _prev[p].info.pos_x;
                    float dy = detected[d].pos_y - _prev[p].info.pos_y;
                    float dist = Mathf.Sqrt(dx * dx + dy * dy);
                    if (dist <= MatchDist) pairs.Add((dist, p, d));
                }
            pairs.Sort((a, b) => a.d.CompareTo(b.d));
            foreach (var (d, pi, di) in pairs)
            {
                if (usedPrev[pi] || idOf[di] != 0) continue;
                usedPrev[pi] = true;
                idOf[di] = _prev[pi].info.id;
            }

            for (int i = 0; i < n; i++)
            {
                var p = detected[i];
                p.id = idOf[i] != 0 ? idOf[i] : _nextId++;
                result[i] = p;
            }

            // Next frame's tracks: current detections, plus unmatched old tracks
            // carried forward and aged (retired after LostTimeout misses)
            var next = new List<Tracked>(n + _prev.Length);
            for (int i = 0; i < n; i++)
                next.Add(new Tracked { info = result[i], misses = 0 });
            for (int p = 0; p < _prev.Length; p++)
                if (!usedPrev[p])
                    next.Add(new Tracked { info = _prev[p].info, misses = _prev[p].misses + 1 });
            _prev = next.ToArray();
            return result;
        }

        public void Reset()
        {
            _prev = System.Array.Empty<Tracked>();
            _nextId = 1;
        }
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
        //   area/strength → inside-piece exclusion → temporal EMA
        public static bool FilterEnabled = false;

        // Stage 1: area & strength (python: π·r² ≥ 20 and peak ≥ 60)
        public static float FilterMinArea = 20f;
        public static float FilterMinPressure = 60f;

        // Detection (used only while the filter is ON — mirrors python TouchDetector):
        //   TOUCH_THRESHOLD = 12 and square-expansion radius until a pixel < 1
        public static float FilterDetectThreshold = 12f;

        // Stage 2: drop touches inside a detected piece circle (python _detect_pieces,
        // which runs on peak-held data with tighter shape thresholds)
        public static bool FilterExcludeInsidePieces = true;

        // Stage 3 (extension beyond the python pipeline): temporal EMA smoothing
        // across frames (0 = off → behaves exactly like python; >0 blends each touch
        // with its nearest previous filtered touch)
        public static float FilterTemporalAlpha = 0f;

        static PressureInfo[] _prevFiltered;
        static int _prevFilterW, _prevFilterH;
        static bool _filterWasEnabled;

        // ── Piece detection state (python _apply_peak_hold + _detect_pieces) ──
        const int SkeletonThresh = 25;
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

        static byte[] _binBuf;
        static int[] _labelBuf;
        static int[] _bfsQueue;
        static int _bufCapacity;

        // ── Reused scratch collections (cleared on each use — never hold across calls) ──
        static readonly List<ComponentInfo> _components = new List<ComponentInfo>(64);
        static readonly List<(int x, int y)> _contour = new List<(int x, int y)>(256);
        static readonly List<(int x, int y)> _contourSimple = new List<(int x, int y)>(256);
        static readonly List<(float cx, float cy, float r)> _pieces = new List<(float, float, float)>(16);
        static readonly List<(int x, int y, float val)> _peaks = new List<(int x, int y, float val)>(16);
        static readonly List<PressureInfo> _piScratch = new List<PressureInfo>(16);

        static readonly int[] NeighborDx8 = {  0,  1,  1,  1,  0, -1, -1, -1 };
        static readonly int[] NeighborDy8 = { -1, -1,  0,  1,  1,  1,  0, -1 };

        static void ResizeBufs(int size)
        {
            if (_bufCapacity >= size) return;
            _bufCapacity = size;
            _binBuf = new byte[size];
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
            var peaks = _peaks;
            peaks.Clear();
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
            var result = _piScratch;
            result.Clear();
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
        ///   2. inside-piece exclusion: drop touches inside a detected piece circle
        ///      (python _detect_pieces on peak-held data)
        /// </summary>
        static PressureInfo[] ApplyTouchFilter(PressureInfo[] touches, int[] data, int inW, int inH)
        {
            // ── Stage 1: area & strength (python: π·r² ≥ 20 and peak ≥ 60) ──
            var list = _piScratch;
            list.Clear();
            foreach (var t in touches)
            {
                if (FilterMinArea > 0f && Mathf.PI * t.radius * t.radius < FilterMinArea) continue;
                if (FilterMinPressure > 0f && t.pressure < FilterMinPressure) continue;
                list.Add(t);
            }
            if (list.Count == 0) return System.Array.Empty<PressureInfo>();
            PressureInfo[] current = list.ToArray();

            // ── Stage 2: inside-piece exclusion (python _detect_pieces) ──
            if (FilterExcludeInsidePieces)
            {
                var pieces = DetectPieces(data, inW, inH);
                list.Clear();
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
                    if (!inside) list.Add(t);
                }
                current = list.ToArray();
            }

            // ── Stage 3: T-shape (stamp) exclusion — python :659-660 ──
            // The mask is rebuilt once per frame by TShapeDetector.GetTShapes, which
            // MatrixHeatmap must call before this; when no stamp was found (or the
            // feature is off) InTShapeMask is always false and nothing is dropped.
            list.Clear();
            foreach (var t in current)
                if (!TShapeDetector.InTShapeMask((int)t.x, (int)t.y))
                    list.Add(t);

            return list.ToArray();
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

            // SHARED scratch: valid only until the next DetectPieces call.
            // Both callers (GetPieceInfo, ApplyTouchFilter) consume it immediately.
            var pieces = _pieces;
            pieces.Clear();
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

            // python feeds cv2.findContours/area/arcLength/fitEllipse the outer boundary
            // contour only, in boundary-walk order. Moore-neighbor tracing gives that
            // ordering; sorting pixel centers by angle inflates the perimeter (consecutive
            // points can sit far apart across staircase corners) and kills circularity.
            List<(int x, int y)> full = TraceOuterContour(labels, compId, w, h);
            if (full.Count < 5) return false;

            // Then collapse collinear runs, i.e. cv2.CHAIN_APPROX_SIMPLE.  This matters
            // for polygons: python's `len(cnt) < 5` guard runs on the *simplified*
            // contour, where a square is exactly 4 corners, while the raw Moore walk
            // (~4·side points) would sail through — and a square has circularity
            // π/4 = 0.785 and aspect 1, so it would be reported as a circular piece.
            //
            // Polygon area and perimeter are unchanged by dropping collinear points, but
            // the PCA second moments are NOT (they are a mean over the point set) — and
            // that is what we want here, because python fits its ellipse to this same
            // simplified contour.  Verified: SimplifyContour reproduces
            // cv2.CHAIN_APPROX_SIMPLE's point count *and* point set exactly
            // (e.g. 70 -> 10, 106 -> 52, 76 -> 36).
            List<(int x, int y)> contour = SimplifyContour(full);
            if (contour.Count < 5) return false;

            float cx = (float)comp.sumX / comp.pixelCount;
            float cy = (float)comp.sumY / comp.pixelCount;

            // ── Polygon area (shoelace) and perimeter → circularity ──
            float polyArea = 0f, perimeter = 0f;
            int npts = contour.Count;
            for (int i = 0; i < npts; i++)
            {
                int j = (i + 1) % npts;
                float x1 = contour[i].x, y1 = contour[i].y;
                float x2 = contour[j].x, y2 = contour[j].y;
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

            // ── PCA ellipse fit (python: cv2.fitEllipse on contour points) ──
            float covXX = 0f, covYY = 0f, covXY = 0f;
            for (int i = 0; i < npts; i++)
            {
                float dx = contour[i].x - cx;
                float dy = contour[i].y - cy;
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
        /// Collapse collinear runs of a closed boundary walk (cv2.CHAIN_APPROX_SIMPLE):
        /// keep a point only when the direction of travel changes there.
        /// Returns SHARED scratch, valid until the next SimplifyContour call.
        /// </summary>
        static List<(int x, int y)> SimplifyContour(List<(int x, int y)> src)
        {
            var dst = _contourSimple;
            dst.Clear();
            int n = src.Count;
            if (n < 3)
            {
                for (int i = 0; i < n; i++) dst.Add(src[i]);
                return dst;
            }
            for (int i = 0; i < n; i++)
            {
                var prev = src[(i - 1 + n) % n];
                var cur = src[i];
                var next = src[(i + 1) % n];
                int d1x = System.Math.Sign(cur.x - prev.x), d1y = System.Math.Sign(cur.y - prev.y);
                int d2x = System.Math.Sign(next.x - cur.x), d2y = System.Math.Sign(next.y - cur.y);
                if (d1x != d2x || d1y != d2y) dst.Add(cur);
            }
            if (dst.Count == 0) for (int i = 0; i < n; i++) dst.Add(src[i]);
            return dst;
        }

        /// <summary>
        /// Moore-neighbor tracing of a component's outer boundary, in walk order
        /// (what cv2.findContours returns). Clockwise probe starting just after the
        /// backtrack pixel; Jacob-style stop on re-entering the start pixel.
        /// </summary>
        static List<(int x, int y)> TraceOuterContour(int[] labels, int compId, int w, int h)
        {
            // SHARED scratch: valid only until the next TraceOuterContour call
            // (TryFitPiece consumes it immediately).
            var contour = _contour;
            contour.Clear();
            int total = w * h;

            // Start: topmost, then leftmost component pixel
            int sx = -1, sy = -1;
            for (int y = 0; y < h && sx < 0; y++)
                for (int x = 0; x < w; x++)
                    if (labels[x * h + y] == compId) { sx = x; sy = y; break; }
            if (sx < 0) return contour;

            // Initial backtrack: any direction pointing at background/out of bounds
            int dir = 0;
            for (int d = 0; d < 8; d++)
            {
                int nx = sx + NeighborDx8[d], ny = sy + NeighborDy8[d];
                if ((uint)nx >= (uint)w || (uint)ny >= (uint)h || labels[nx * h + ny] != compId)
                { dir = d; break; }
            }

            int cx = sx, cy = sy;
            contour.Add((cx, cy));

            int maxSteps = total + 16; // safety cap; a closed border walk visits each pixel ≤ ~2×
            for (int step = 0; step < maxSteps; step++)
            {
                bool moved = false;
                for (int k = 1; k <= 8; k++)
                {
                    int nd = (dir + k) % 8;
                    int nx = cx + NeighborDx8[nd], ny = cy + NeighborDy8[nd];
                    if ((uint)nx >= (uint)w || (uint)ny >= (uint)h || labels[nx * h + ny] != compId)
                        continue;
                    if (nx == sx && ny == sy) return contour; // closed the loop
                    dir = (nd + 4) % 8; // backtrack pixel = where we came from
                    cx = nx; cy = ny;
                    contour.Add((cx, cy));
                    moved = true;
                    break;
                }
                if (!moved) break; // isolated pixel, nothing more to trace
            }
            return contour;
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

        // ── Connected components ──────────────────────

        struct ComponentInfo
        {
            public int sumX, sumY;
            public int pixelCount;
            public int compId;
        }

        /// <summary>BFS connected-component labeling (8-connected).
        /// Returns SHARED scratch _components: valid only until the next
        /// FindComponents call; callers must consume it immediately (and must not
        /// call anything that re-enters FindComponents while iterating).</summary>
        static List<ComponentInfo> FindComponents(
            byte[] skeleton, int[] labels, int[] queue, int w, int h)
        {
            int total = w * h;
            System.Array.Clear(labels, 0, total);
            var components = _components;
            components.Clear();
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

        /// <summary>Detect solid circular pieces from pressure frame.</summary>
        public static PieceInfo[] GetPieceInfo(int[] data, int width, int height)
        {
            if (data == null || width <= 0 || height <= 0)
                return System.Array.Empty<PieceInfo>();

            // No (width, height) swap: passing the caller's dims here matches the
            // touch-filter path's coordinate frame.
            var pieces = DetectPieces(data, width, height);
            if (pieces.Count == 0) return System.Array.Empty<PieceInfo>();

            var results = new PieceInfo[pieces.Count];
            int n = 0;
            for (int i = 0; i < pieces.Count; i++)
            {
                // python :661-662 — drop pieces whose centre lands in the stamp mask
                if (TShapeDetector.InTShapeMask((int)pieces[i].Item1, (int)pieces[i].Item2))
                    continue;
                results[n++] = new PieceInfo
                {
                    pos_x = pieces[i].Item1,
                    pos_y = pieces[i].Item2,
                    radius = pieces[i].Item3,
                };
            }
            if (n != results.Length) System.Array.Resize(ref results, n);
            return results;
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

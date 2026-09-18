using System.Collections.Generic;
using UnityEngine;

namespace DevicePipe
{
    /// <summary>
    /// One T-shaped stamp (印章) found by <see cref="TShapeDetector"/>.
    ///
    /// <para><b>Coordinate convention</b> — sensor pixels, same axis naming as
    /// <see cref="PieceInfo"/>: <c>Row</c> is the slow index (0..height-1) and
    /// <c>Col</c> the fast index (0..width-1), i.e. flat index = Row*width + Col.
    /// The python reference calls Col "x" and Row "y", so a python <c>(x, y)</c>
    /// pair is stored here as <c>(Row = y, Col = x)</c>.</para>
    ///
    /// <para>To place on a UGUI layer use
    /// <c>anchoredPosition = (Col * scale, Row * scale)</c> — exactly like
    /// TouchMarkerLayer / ChessPieceLayer.</para>
    /// </summary>
    public struct TShapeInfo
    {
        /// <summary>4-bit stamp identity, 0..15 (python <c>_compute_tshape_id</c>).</summary>
        public int id;
        /// <summary>0 = 圆形件 (id &gt; 7), 1 = 条形件 (id &lt;= 7).</summary>
        public int shape;
        /// <summary>Pixel count of the connected component that produced this shape.</summary>
        public int area;
        /// <summary>Mean raw pressure sampled along the stem (python <c>p</c>).</summary>
        public int pressure;

        // ── geometry, sensor px ─────────────────────────────────────────
        /// <summary>Where the bar meets the stem.</summary>
        public int junctionRow, junctionCol;
        /// <summary>Far end of the stem.</summary>
        public int stemRow, stemCol;
        /// <summary>The two ends of the bar.</summary>
        public int bar0Row, bar0Col, bar1Row, bar1Col;
        /// <summary>The three endpoints chosen by the detector.</summary>
        public int ep0Row, ep0Col, ep1Row, ep1Col, ep2Row, ep2Col;
        /// <summary>The four ID sampling points (python <c>_compute_tshape_endpoints</c>).</summary>
        public int p0Row, p0Col, p1Row, p1Col, p2Row, p2Col, p3Row, p3Col;

        // ── derived (python viewer :823-851) ────────────────────────────
        /// <summary>Stem midpoint — the shape's reported position.</summary>
        public float posRow, posCol;
        /// <summary>Reported radius in mm, shape 0 only (0 otherwise).</summary>
        public float radiusMm;
        /// <summary>Stem length in mm, shape 1 only (0 otherwise).</summary>
        public float stemLenMm;
        /// <summary>Bar length in mm, shape 1 only (0 otherwise).</summary>
        public float barLenMm;
        /// <summary>Stem direction, junction → stem tip (unnormalised, sensor px).</summary>
        public float dirRow, dirCol;

        // ── two-stroke fit diagnostics (see TShapeDetector.PassesStampFit) ──
        /// <summary>Fitted stroke thickness in px.</summary>
        public float strokeThickness;
        /// <summary>Fraction of the blob the bar+stem strokes explain.</summary>
        public float fitCoverage;
    }

    /// <summary>
    /// Port of the python stamp-type recogniser Unity was missing:
    /// <c>detect_tshapes</c> (ring_pressure_viewer.py:244-331),
    /// <c>_compute_tshape_endpoints</c> (:334-355), <c>_compute_tshape_id</c> (:358-367),
    /// the derived payload from the render loop (:820-851) and the
    /// <c>tshape_mask</c> exclusion mask (:636-662).
    ///
    /// <para>Everything runs on the same flat <c>int[]</c> the rest of DevicePipe uses,
    /// so no transpose is involved: a python <c>arr[y, x]</c> is exactly flat index
    /// <c>y*width + x</c>.</para>
    ///
    /// <para><b>Per-frame contract</b> — like <c>DetectPieces</c>, this keeps static
    /// state (the temporal EMA and the exclusion mask), so call
    /// <see cref="GetTShapes"/> exactly once per frame.  Calling it twice in one frame
    /// would advance the EMA twice.</para>
    ///
    /// <para>Two deliberate deviations from python: the morphology border handling
    /// matches OpenCV (out-of-bounds neighbours are ignored for both erode and dilate),
    /// and the mask's line/circle rasterisation approximates <c>cv2.line</c> /
    /// <c>cv2.circle</c> (≤1px boundary differences before the 3×3 dilate).</para>
    /// </summary>
    public static class TShapeDetector
    {
        // ── Knobs, python viewer trackbar defaults ────────────────────────
        /// <summary>detect_tshapes(thresh=…) — python 'TThresh' default 100.</summary>
        public static int Threshold = 100;
        /// <summary>detect_tshapes(area_min=…) — python 'Area Min' default 40.</summary>
        public static int AreaMin = 40;
        /// <summary>python smooth_alpha (ring_pressure_viewer.py:391).</summary>
        public static float SmoothAlpha = 0.3f;
        /// <summary>Mirrors the python viewer's <c>show_tshapes</c> switch.</summary>
        public static bool Enabled = true;
        /// <summary>px → mm factor the python viewer reports sizes with (:832-838).</summary>
        public const float PxToMm = 5f;

        // ── Arbitration with the circular-piece detector (see PassesStampFit) ──
        /// <summary>
        /// When true (default) a candidate must also be explained by a two-stroke T
        /// model before it is reported, and only accepted stamps enter the exclusion
        /// mask — so a round blob rejected here stays available to
        /// <see cref="PressureAnalyzer.GetPieceInfo"/> and both layers can coexist.
        /// <para>Set false for exact parity with ring_pressure_viewer.py, which reports
        /// any 3-extreme blob as a stamp and masks every large blob.</para>
        /// </summary>
        public static bool RequireStampFit = true;
        /// <summary>Smallest believable stroke thickness, px.</summary>
        public static float MinStrokeThickness = 1.5f;
        /// <summary>Upper bound on strokeThickness / min(barLen, stemLen).</summary>
        public static float MaxStrokeRatio = 0.35f;
        /// <summary>Coverage tolerance, as a fraction of the stroke thickness.</summary>
        public static float FitTolerance = 0.75f;
        /// <summary>Fraction of the blob the two strokes must explain.</summary>
        public static float MinFitCoverage = 0.75f;

        // ── Scratch, reused across frames ─────────────────────────────────
        static byte[] _bin, _tmp, _clean, _mask;
        static int[] _labels, _queue;
        static int _cap, _w, _h;
        static float[] _smooth;
        static bool _smoothInit;
        static int _maskW, _maskH, _lastCount;

        static readonly List<Comp> _comps = new List<Comp>(32);
        static readonly List<int> _px = new List<int>(512);
        static readonly List<int> _tipPx = new List<int>(256);
        static readonly List<float> _tipAng = new List<float>(256);
        static readonly List<float> _tipDist = new List<float>(256);
        static readonly List<int> _tipOrder = new List<int>(256);
        static readonly List<float> _sampleVals = new List<float>(64);

        static readonly int[] _epPx = new int[3];
        static readonly float[] _epAng = new float[3];
        static readonly int[] _epRow = new int[3];
        static readonly int[] _epCol = new int[3];
        static readonly int[] _pRow = new int[4];
        static readonly int[] _pCol = new int[4];
        static readonly float[] _pairDist = new float[3];
        static readonly int[] _pairOrder = new int[3];

        // pairs (0,1) (0,2) (1,2), newest-first order preserved
        static readonly int[] _pairA = { 0, 0, 1 };
        static readonly int[] _pairB = { 1, 2, 2 };

        // (col, row) offsets of the 8-neighbourhood
        static readonly int[] DCol8 = { 0, 1, 1, 1, 0, -1, -1, -1 };
        static readonly int[] DRow8 = { -1, -1, 0, 1, 1, 1, 0, -1 };

        struct Comp
        {
            public int label, area, sumRow, sumCol;
        }

        // ══════════════════════════════════════════════════════════════
        //  Public API
        // ══════════════════════════════════════════════════════════════

        /// <summary>
        /// Detect every T-shaped stamp in <paramref name="data"/> and rebuild the
        /// exclusion mask used by <see cref="InTShapeMask"/>.  Returns a fresh array.
        /// </summary>
        public static TShapeInfo[] GetTShapes(int[] data, int width, int height)
        {
            var result = new List<TShapeInfo>(4);
            if (!Enabled || data == null || width <= 0 || height <= 0)
            {
                _lastCount = 0;
                _maskW = _maskH = 0;
                return result.ToArray();
            }

            _w = width; _h = height;
            int total = width * height;
            EnsureBufs(total);

            // python _try_parse_frame: smoothed = a*data + (1-a)*smoothed, seeded on frame 1
            UpdateSmoothed(data, total);

            // cv2.threshold(np.clip(smoothed,0,255).astype(uint8), thresh, THRESH_BINARY)
            for (int i = 0; i < total; i++)
            {
                int u8 = (int)Mathf.Clamp(_smooth[i], 0f, 255f);   // astype(uint8) truncates
                _bin[i] = u8 > Threshold ? (byte)1 : (byte)0;
            }

            // MORPH_OPEN, 2x2 ones, iterations = 1
            Erode(_bin, _tmp, width, height, 2);
            Dilate(_tmp, _bin, width, height, 2);

            // drop components smaller than area_min
            Label(_bin, width, height);
            System.Array.Clear(_clean, 0, total);
            for (int i = 0; i < _comps.Count; i++)
            {
                if (_comps[i].area < AreaMin) continue;
                int lab = _comps[i].label;
                for (int p = 0; p < total; p++)
                    if (_labels[p] == lab) _clean[p] = 1;
            }

            // MORPH_CLOSE, 3x3 ones, iterations = 1
            Dilate(_clean, _tmp, width, height, 3);
            Erode(_tmp, _clean, width, height, 3);

            // ── analyse every surviving component ──
            System.Array.Clear(_mask, 0, total);
            int n = Label(_clean, width, height);
            int accepted = 0;
            for (int ci = 0; ci < n; ci++)
            {
                var comp = _comps[ci];
                if (comp.area < AreaMin) continue;
                if (!TryAnalyse(comp, data, width, height, out var shape)) continue;

                result.Add(shape);
                accepted++;

                if (RequireStampFit)
                {
                    // Claim only what was actually accepted.  A blob rejected by the
                    // stamp fit test must stay unmasked so PressureAnalyzer.GetPieceInfo
                    // still reports it as a circular piece.
                    for (int p = 0; p < total; p++)
                        if (_labels[p] == comp.label) _mask[p] = 1;
                }
            }

            // python :636-657 — the mask only exists when a stamp was actually found.
            if (accepted > 0)
            {
                // Parity mode keeps the reference behaviour of masking every large blob.
                if (!RequireStampFit) BuildMaskBase(width, height, total);
                for (int i = 0; i < result.Count; i++) DrawShapeOnMask(result[i]);
                Dilate(_mask, _tmp, width, height, 3);   // cv2.dilate(mask, 3x3, 1)
                System.Array.Copy(_tmp, _mask, total);
            }

            _maskW = width; _maskH = height; _lastCount = result.Count;
            return result.ToArray();
        }

        /// <summary>Number of shapes found by the most recent <see cref="GetTShapes"/> call.</summary>
        public static int LastTShapeCount => _lastCount;

        /// <summary>
        /// python: <c>tshape_mask[int(t[1]), int(t[0])] == 0</c> — true when this sensor
        /// pixel must be excluded from touch/piece reporting (python :659-662).
        /// </summary>
        public static bool InTShapeMask(int row, int col)
        {
            if (_mask == null || _maskW <= 0 || _maskH <= 0) return false;
            if ((uint)row >= (uint)_maskH || (uint)col >= (uint)_maskW) return false;
            return _mask[row * _maskW + col] != 0;
        }

        /// <summary>Drop the temporal EMA and the mask (reconnect / resolution change).</summary>
        public static void ResetState()
        {
            _smoothInit = false;
            _lastCount = 0;
            _maskW = _maskH = 0;
        }

        // ══════════════════════════════════════════════════════════════
        //  Buffers
        // ══════════════════════════════════════════════════════════════

        static void EnsureBufs(int size)
        {
            if (_cap >= size) return;
            _cap = size;
            _bin = new byte[size];
            _tmp = new byte[size];
            _clean = new byte[size];
            _mask = new byte[size];
            _labels = new int[size];
            _queue = new int[size];
        }

        static void UpdateSmoothed(int[] data, int total)
        {
            bool sizeChanged = _smooth == null || _smooth.Length < total;
            if (sizeChanged) _smooth = new float[total];

            if (!_smoothInit || sizeChanged)
            {
                for (int i = 0; i < total; i++) _smooth[i] = data[i];
                _smoothInit = true;
                return;
            }
            float a = SmoothAlpha;
            for (int i = 0; i < total; i++)
                _smooth[i] = a * data[i] + (1f - a) * _smooth[i];
        }

        // ══════════════════════════════════════════════════════════════
        //  Morphology — parity with cv2.morphologyEx
        // ══════════════════════════════════════════════════════════════

        // OpenCV anchors a k×k kernel of ones at (k/2, k/2) and treats out-of-bounds
        // reads as +inf for erode and -inf for dilate, i.e. ignores them in both cases.
        static void Erode(byte[] src, byte[] dst, int w, int h, int k)
        {
            int a = k / 2;
            for (int r = 0; r < h; r++)
            {
                int rowBase = r * w;
                for (int c = 0; c < w; c++)
                {
                    byte v = 1;
                    for (int j = 0; j < k && v != 0; j++)
                    {
                        int rr = r + j - a;
                        if ((uint)rr >= (uint)h) continue;
                        int base2 = rr * w;
                        for (int i = 0; i < k; i++)
                        {
                            int cc = c + i - a;
                            if ((uint)cc >= (uint)w) continue;
                            if (src[base2 + cc] == 0) { v = 0; break; }
                        }
                    }
                    dst[rowBase + c] = v;
                }
            }
        }

        static void Dilate(byte[] src, byte[] dst, int w, int h, int k)
        {
            int a = k / 2;
            for (int r = 0; r < h; r++)
            {
                int rowBase = r * w;
                for (int c = 0; c < w; c++)
                {
                    byte v = 0;
                    for (int j = 0; j < k && v == 0; j++)
                    {
                        int rr = r + j - a;
                        if ((uint)rr >= (uint)h) continue;
                        int base2 = rr * w;
                        for (int i = 0; i < k; i++)
                        {
                            int cc = c + i - a;
                            if ((uint)cc >= (uint)w) continue;
                            if (src[base2 + cc] != 0) { v = 1; break; }
                        }
                    }
                    dst[rowBase + c] = v;
                }
            }
        }

        // ══════════════════════════════════════════════════════════════
        //  8-connected labelling with stats (cv2.connectedComponentsWithStats)
        // ══════════════════════════════════════════════════════════════

        static int Label(byte[] src, int w, int h)
        {
            int total = w * h;
            System.Array.Clear(_labels, 0, total);
            _comps.Clear();
            int n = 0;

            for (int seed = 0; seed < total; seed++)
            {
                if (src[seed] == 0 || _labels[seed] != 0) continue;
                n++;
                int head = 0, tail = 0;
                _queue[tail++] = seed;
                _labels[seed] = n;
                int area = 0, sumRow = 0, sumCol = 0;

                while (head < tail)
                {
                    int idx = _queue[head++];
                    int r = idx / w, c = idx % w;
                    area++; sumRow += r; sumCol += c;

                    for (int d = 0; d < 8; d++)
                    {
                        int nc = c + DCol8[d], nr = r + DRow8[d];
                        if ((uint)nc >= (uint)w || (uint)nr >= (uint)h) continue;
                        int nidx = nr * w + nc;
                        if (src[nidx] == 0 || _labels[nidx] != 0) continue;
                        _labels[nidx] = n;
                        _queue[tail++] = nidx;
                    }
                }

                _comps.Add(new Comp { label = n, area = area, sumRow = sumRow, sumCol = sumCol });
            }
            return n;
        }

        // ══════════════════════════════════════════════════════════════
        //  Per-component T analysis (the body of python detect_tshapes)
        // ══════════════════════════════════════════════════════════════

        static bool TryAnalyse(Comp comp, int[] raw, int w, int h, out TShapeInfo shape)
        {
            shape = default;
            float cx = (float)comp.sumCol / comp.area;   // python centroid_x
            float cy = (float)comp.sumRow / comp.area;   // python centroid_y

            // component pixels in row-major order, matching np.where
            _px.Clear();
            int total = w * h;
            for (int p = 0; p < total; p++)
                if (_labels[p] == comp.label) _px.Add(p);

            float distSum = 0f;
            for (int k = 0; k < _px.Count; k++)
            {
                int idx = _px[k];
                float dr = idx / w - cy, dc = idx % w - cx;
                distSum += Mathf.Sqrt(dr * dr + dc * dc);
            }
            float avgThickness = distSum / _px.Count * 0.6f;
            int nbrThresh = Mathf.Max(2, Mathf.RoundToInt(avgThickness));

            // python: tip_mask = cc_mask & (nbr_count <= nbr_thresh); retry at +2 if < 3
            if (!CollectTips(comp, w, h, nbrThresh) &&
                !CollectTips(comp, w, h, nbrThresh + 2))
                return false;

            // python: sort by descending distance from the centroid
            _tipOrder.Clear();
            for (int k = 0; k < _tipPx.Count; k++) _tipOrder.Add(k);
            _tipOrder.Sort((a, b) =>
            {
                int cmp = _tipDist[b].CompareTo(_tipDist[a]);
                return cmp != 0 ? cmp : a.CompareTo(b);
            });

            // greedy pick 3 endpoints, each at least 45° from the ones already taken
            int found = 0;
            for (int k = 0; k < _tipOrder.Count && found < 3; k++)
            {
                int t = _tipOrder[k];
                float ang = _tipAng[t];
                bool ok = true;
                for (int u = 0; u < found; u++)
                {
                    float diff = Mathf.Abs(ang - _epAng[u]);
                    if (diff > Mathf.PI) diff = 2f * Mathf.PI - diff;
                    if (diff < Mathf.PI / 4f) { ok = false; break; }
                }
                if (!ok) continue;
                _epPx[found] = _tipPx[t];
                _epAng[found] = ang;
                found++;
            }
            if (found < 3) return false;

            for (int i = 0; i < 3; i++)
            {
                _epRow[i] = _epPx[i] / w;
                _epCol[i] = _epPx[i] % w;
            }

            // python: pairs = [(0,1),(0,2),(1,2)] sorted by endpoint distance
            for (int i = 0; i < 3; i++)
            {
                int a = _pairA[i], b = _pairB[i];
                _pairDist[i] = Mathf.Sqrt(Sq(_epCol[a] - _epCol[b]) + Sq(_epRow[a] - _epRow[b]));
                _pairOrder[i] = i;
            }
            System.Array.Sort(_pairOrder, (x, y) =>
            {
                int cmp = _pairDist[x].CompareTo(_pairDist[y]);
                return cmp != 0 ? cmp : x.CompareTo(y);
            });

            // the bar is the shortest pair whose midpoint lands on the shape
            int ja = -1, jb = -1;
            for (int i = 0; i < 3; i++)
            {
                int a = _pairA[_pairOrder[i]], b = _pairB[_pairOrder[i]];
                int mx = (_epCol[a] + _epCol[b]) / 2;
                int my = (_epRow[a] + _epRow[b]) / 2;
                if (mx < 0) mx = 0; else if (mx >= w) mx = w - 1;
                if (my < 0) my = 0; else if (my >= h) my = h - 1;
                if (_clean[my * w + mx] == 0) continue;
                ja = a; jb = b;
                break;
            }
            if (ja < 0)   // python's for/else fallback
            {
                ja = _pairA[_pairOrder[0]];
                jb = _pairB[_pairOrder[0]];
            }

            int stemIdx = 3 - ja - jb;   // the endpoint not in {ja, jb}
            int junctionCol = (_epCol[ja] + _epCol[jb]) / 2;
            int junctionRow = (_epRow[ja] + _epRow[jb]) / 2;

            // ── _compute_tshape_endpoints → the four ID sampling points ──
            float sdx = _epCol[stemIdx] - junctionCol;
            float sdy = _epRow[stemIdx] - junctionRow;
            float sdl = Mathf.Sqrt(sdx * sdx + sdy * sdy);
            if (sdl > 0f) { sdx /= sdl; sdy /= sdl; }
            float perpCol = -sdy, perpRow = sdx;
            float barLen = Mathf.Sqrt(Sq(_epCol[jb] - _epCol[ja]) + Sq(_epRow[jb] - _epRow[ja]));
            float half = barLen / 2f;
            float stemMidCol = (junctionCol + _epCol[stemIdx]) / 2f;
            float stemMidRow = (junctionRow + _epRow[stemIdx]) / 2f;

            for (int i = 0; i < 2; i++)
            {
                float ptx = i == 0 ? _epCol[stemIdx] : stemMidCol;
                float pty = i == 0 ? _epRow[stemIdx] : stemMidRow;
                // python int() truncates toward zero, same as a C# (int) cast
                _pCol[i * 2] = (int)(ptx + perpCol * half);
                _pRow[i * 2] = (int)(pty + perpRow * half);
                _pCol[i * 2 + 1] = (int)(ptx - perpCol * half);
                _pRow[i * 2 + 1] = (int)(pty - perpRow * half);
            }

            // ── _compute_tshape_id: sample the RAW frame around each point ──
            int tid = 0;
            for (int i = 0; i < 4; i++)
            {
                int rMin = Mathf.Max(0, _pRow[i] - 1), rMax = Mathf.Min(h - 1, _pRow[i] + 1);
                int cMin = Mathf.Max(0, _pCol[i] - 1), cMax = Mathf.Min(w - 1, _pCol[i] + 1);
                bool hit = false;
                for (int r = rMin; r <= rMax && !hit; r++)
                    for (int c = cMin; c <= cMax; c++)
                        if (raw[r * w + c] > 0) { hit = true; break; }
                if (hit) tid |= 1 << i;
            }

            // ── derived payload (python viewer :822-851) ──
            float posCol = (junctionCol + _epCol[stemIdx]) / 2f;
            float posRow = (junctionRow + _epRow[stemIdx]) / 2f;
            float stemLen = Mathf.Sqrt(Sq(_epCol[stemIdx] - junctionCol) +
                                       Sq(_epRow[stemIdx] - junctionRow));
            int sh = tid > 7 ? 0 : 1;

            // ── arbitration: reject blobs that are not explained by a two-stroke T ──
            float strokeT = 0f, fitCov = 0f;
            if (RequireStampFit &&
                !PassesStampFit(barLen, stemLen, comp.area,
                                junctionCol, junctionRow, _epCol[stemIdx], _epRow[stemIdx],
                                _epCol[ja], _epRow[ja], _epCol[jb], _epRow[jb],
                                out strokeT, out fitCov))
                return false;

            float radiusMm = 0f, stemLenMm = 0f, barLenMm = 0f;
            if (sh == 0)
            {
                float d1 = Mathf.Sqrt(Sq(posCol - _epCol[ja]) + Sq(posRow - _epRow[ja]));
                float d2 = Mathf.Sqrt(Sq(posCol - _epCol[jb]) + Sq(posRow - _epRow[jb]));
                radiusMm = Mathf.Max(d1, d2) * PxToMm;
            }
            else
            {
                stemLenMm = stemLen * PxToMm;
                barLenMm = barLen * PxToMm;
            }

            // python: mean raw pressure along np.linspace(junction → stem tip, max(int(stem_len), 1))
            int samples = Mathf.Max((int)stemLen, 1);
            _sampleVals.Clear();
            for (int i = 0; i < samples; i++)
            {
                float c, r;
                if (i == samples - 1) { c = _epCol[stemIdx]; r = _epRow[stemIdx]; }
                else
                {
                    float t = samples > 1 ? (float)i / (samples - 1) : 0f;
                    c = junctionCol + (_epCol[stemIdx] - junctionCol) * t;
                    r = junctionRow + (_epRow[stemIdx] - junctionRow) * t;
                }
                int ci = (int)c, ri = (int)r;
                if ((uint)ci < (uint)w && (uint)ri < (uint)h) _sampleVals.Add(raw[ri * w + ci]);
            }
            int pressure = 0;
            if (_sampleVals.Count > 0)
            {
                float s = 0f;
                for (int i = 0; i < _sampleVals.Count; i++) s += _sampleVals[i];
                pressure = (int)(s / _sampleVals.Count);
            }

            shape = new TShapeInfo
            {
                id = tid,
                shape = sh,
                area = comp.area,
                pressure = pressure,
                junctionRow = junctionRow, junctionCol = junctionCol,
                stemRow = _epRow[stemIdx], stemCol = _epCol[stemIdx],
                bar0Row = _epRow[ja], bar0Col = _epCol[ja],
                bar1Row = _epRow[jb], bar1Col = _epCol[jb],
                ep0Row = _epRow[0], ep0Col = _epCol[0],
                ep1Row = _epRow[1], ep1Col = _epCol[1],
                ep2Row = _epRow[2], ep2Col = _epCol[2],
                p0Row = _pRow[0], p0Col = _pCol[0],
                p1Row = _pRow[1], p1Col = _pCol[1],
                p2Row = _pRow[2], p2Col = _pCol[2],
                p3Row = _pRow[3], p3Col = _pCol[3],
                posRow = posRow, posCol = posCol,
                radiusMm = radiusMm, stemLenMm = stemLenMm, barLenMm = barLenMm,
                dirRow = _epRow[stemIdx] - junctionRow,
                dirCol = _epCol[stemIdx] - junctionCol,
                strokeThickness = strokeT,
                fitCoverage = fitCov,
            };
            return true;
        }

        static bool CollectTips(Comp comp, int w, int h, int nbrThresh)
        {
            _tipPx.Clear();
            _tipAng.Clear();
            _tipDist.Clear();
            float cx = (float)comp.sumCol / comp.area;
            float cy = (float)comp.sumRow / comp.area;

            for (int k = 0; k < _px.Count; k++)
            {
                int idx = _px[k];
                int r = idx / w, c = idx % w;

                // python cv2.filter2D(cc_mask, cross kernel, BORDER_CONSTANT) → the
                // count of in-component 8-neighbours
                int nbr = 0;
                for (int d = 0; d < 8; d++)
                {
                    int nc = c + DCol8[d], nr = r + DRow8[d];
                    if ((uint)nc >= (uint)w || (uint)nr >= (uint)h) continue;
                    if (_labels[nr * w + nc] == comp.label) nbr++;
                }
                if (nbr > nbrThresh) continue;

                _tipPx.Add(idx);
                _tipDist.Add(Mathf.Sqrt(Sq(c - cx) + Sq(r - cy)));
                _tipAng.Add(Mathf.Atan2(r - cy, c - cx));
            }
            return _tipPx.Count >= 3;
        }

        static float Sq(float v) => v * v;

        /// <summary>
        /// Two-stroke fit test — the arbitration rule that lets circular pieces and
        /// T-stamps coexist.
        ///
        /// <para>A real T is one bar and one stem of stroke thickness t, so
        /// <c>Area ≈ t*(barLen + stemLen) - t²</c> must have a positive solution, the
        /// solution must be genuinely thinner than the strokes are long, and the two
        /// segments must actually cover the blob.  The reference's 3-endpoint rule alone
        /// accepts any blob with three ≥45°-apart extremes, so a filled disc is reported
        /// as a stamp and its mask then starves the piece detector.</para>
        ///
        /// <para>Measured across 16 stamps × cell 3..6, discs r=4..20 and squares 8..30
        /// (python and the C# port agree):
        /// <list type="bullet">
        /// <item>stamp: t/minSeg 0.092..0.094, coverage 0.889..1.000 → accept</item>
        /// <item>disc: no real t (4A &gt; (barLen+stemLen)²), coverage 0.058..0.286 → reject</item>
        /// <item>square ≥20: t/minSeg 0.90..0.99, coverage 0.917..0.985 → reject</item>
        /// </list>
        /// The thin squares (≤16) have no real t either.</para>
        /// </summary>
        static bool PassesStampFit(float barLen, float stemLen, int area,
                                   float jCol, float jRow, float sCol, float sRow,
                                   float b0Col, float b0Row, float b1Col, float b1Row,
                                   out float strokeT, out float coverage)
        {
            strokeT = 0f;
            coverage = 0f;

            float sum = barLen + stemLen;
            if (sum <= 0f || area <= 0) return false;

            // Area = t*sum - t^2  →  t^2 - sum*t + area = 0
            float disc = sum * sum - 4f * (float)area;
            if (disc < 0f) return false;                       // no real stroke thickness
            strokeT = (sum - Mathf.Sqrt(disc)) * 0.5f;
            if (strokeT < MinStrokeThickness) return false;

            float minSeg = Mathf.Min(barLen, stemLen);
            if (minSeg <= 0f || strokeT > MaxStrokeRatio * minSeg) return false;

            float tol = Mathf.Max(1f, strokeT) * FitTolerance;
            int inside = 0;
            int n = _px.Count;
            for (int k = 0; k < n; k++)
            {
                int idx = _px[k];
                float c = idx % _w, r = idx / _w;
                float d1 = DistToSegment(c, r, b0Col, b0Row, b1Col, b1Row);
                float d2 = DistToSegment(c, r, jCol, jRow, sCol, sRow);
                if (Mathf.Min(d1, d2) <= tol) inside++;
            }
            coverage = n > 0 ? (float)inside / n : 0f;
            return coverage >= MinFitCoverage;
        }

        static float DistToSegment(float px, float py, float x0, float y0, float x1, float y1)
        {
            float vx = x1 - x0, vy = y1 - y0;
            float len2 = vx * vx + vy * vy;
            if (len2 <= 0f) return Mathf.Sqrt(Sq(px - x0) + Sq(py - y0));
            float t = ((px - x0) * vx + (py - y0) * vy) / len2;
            if (t < 0f) t = 0f; else if (t > 1f) t = 1f;
            return Mathf.Sqrt(Sq(px - (x0 + t * vx)) + Sq(py - (y0 + t * vy)));
        }

        // ══════════════════════════════════════════════════════════════
        //  tshape_mask (python :636-657)
        // ══════════════════════════════════════════════════════════════

        static void BuildMaskBase(int w, int h, int total)
        {
            Label(_bin, w, h);   // _bin is still the 2x2-opened binary, as in python
            for (int i = 0; i < _comps.Count; i++)
            {
                if (_comps[i].area < AreaMin) continue;
                int lab = _comps[i].label;
                for (int p = 0; p < total; p++)
                    if (_labels[p] == lab) _mask[p] = 1;
            }
        }

        static void DrawShapeOnMask(TShapeInfo s)
        {
            int w = _w, h = _h;
            // python cv2.line(..., 2) for the stem, the bar and both ID pairs
            DrawThickLine(w, h, s.junctionCol, s.junctionRow, s.stemCol, s.stemRow, 2);
            DrawThickLine(w, h, s.bar0Col, s.bar0Row, s.bar1Col, s.bar1Row, 2);
            DrawThickLine(w, h, s.p0Col, s.p0Row, s.p1Col, s.p1Row, 2);
            DrawThickLine(w, h, s.p2Col, s.p2Row, s.p3Col, s.p3Row, 2);
            // python cv2.circle(..., 2, 255, -1)
            DrawFilledCircle(w, h, s.p0Col, s.p0Row, 2);
            DrawFilledCircle(w, h, s.p1Col, s.p1Row, 2);
            DrawFilledCircle(w, h, s.p2Col, s.p2Row, 2);
            DrawFilledCircle(w, h, s.p3Col, s.p3Row, 2);
        }

        static void Plot(int w, int h, int c, int r)
        {
            if ((uint)c >= (uint)w || (uint)r >= (uint)h) return;
            _mask[r * w + c] = 1;
        }

        // Approximation of cv2.line(thickness=2): stamp a 2x2 block per Bresenham step.
        static void DrawThickLine(int w, int h, int c0, int r0, int c1, int r1, int thickness)
        {
            int t = Mathf.Max(1, thickness);
            int c = c0, r = r0;
            int dc = Mathf.Abs(c1 - c0), dr = Mathf.Abs(r1 - r0);
            int sc = c0 < c1 ? 1 : -1, sr = r0 < r1 ? 1 : -1;
            int err = dc - dr;
            int guard = 4 * (dc + dr) + 8;
            while (guard-- > 0)
            {
                for (int j = 0; j < t; j++)
                    for (int i = 0; i < t; i++)
                        Plot(w, h, c + i - t / 2, r + j - t / 2);
                if (c == c1 && r == r1) break;
                int e2 = 2 * err;
                if (e2 > -dr) { err -= dr; c += sc; }
                if (e2 < dc) { err += dc; r += sr; }
            }
        }

        static void DrawFilledCircle(int w, int h, int c, int r, int radius)
        {
            for (int dr = -radius; dr <= radius; dr++)
                for (int dc = -radius; dc <= radius; dc++)
                    if (dc * dc + dr * dr <= radius * radius) Plot(w, h, c + dc, r + dr);
        }
    }
}

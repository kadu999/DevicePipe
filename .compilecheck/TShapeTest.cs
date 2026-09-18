using System;
using DevicePipe;

/// <summary>
/// Regression tests for the ported T-shape (印章) recogniser.
///
/// The 16 physical stamps encode their identity in four optional dots, which
/// _compute_tshape_endpoints samples at (row10,col0), (row10,col10), (row5,col0)
/// and (row5,col10) — bit 0..3 of the reported id.  These tests regenerate that
/// exact geometry and require the detector to recover id == stamp index, which is
/// the same criterion the python reference was verified against at these scales.
/// </summary>
static class TShapeTest
{
    const int W = 100, H = 100;

    public static int Run()
    {
        int failures = 0;

        // ── 1. every real stamp must decode to its own id ──
        foreach (int cell in new[] { 3, 4 })
        {
            int ok = 0;
            string firstBad = "";
            for (int id = 0; id < 16; id++)
            {
                var shapes = Detect(RenderStamp(id, cell), W, H);
                int want = id > 7 ? 0 : 1;
                bool good = shapes.Length == 1 && shapes[0].id == id && shapes[0].shape == want;
                if (good) ok++;
                else if (firstBad.Length == 0)
                    firstBad = shapes.Length == 0
                        ? "no shape found"
                        : $"got {shapes.Length} shape(s), id={shapes[0].id}, shape={shapes[0].shape}";
            }
            Check(ref failures,
                $"all 16 stamps decode to their own id at cell={cell} ({ok}/16)" +
                (ok == 16 ? "" : $"  [{firstBad}]"),
                ok == 16);
        }

        // ── 2. nothing to find ──
        Check(ref failures, "empty frame -> no stamps", Detect(new int[W * H], W, H).Length == 0);
        Check(ref failures, "null frame -> no stamps", TShapeDetector.GetTShapes(null, W, H).Length == 0);

        // With arbitration on (the default) a disc is not accepted as a stamp.  The
        // reference's 3-endpoint rule does accept it, and that behaviour is pinned
        // separately under RequireStampFit = false (test 10).
        var disc = new int[W * H];
        for (int r = 42; r < 58; r++)
            for (int c = 42; c < 58; c++)
                if ((r - 50) * (r - 50) + (c - 50) * (c - 50) <= 64) disc[r * W + c] = 200;
        var discShapes = Detect(disc, W, H);
        Check(ref failures,
            $"solid disc is not a stamp under arbitration (got {discShapes.Length})",
            discShapes.Length == 0);

        // ── 3. a T is never mistaken for a circular chess piece ──
        TShapeDetector.ResetState();
        Check(ref failures, "plain T -> no circle pieces",
            PressureAnalyzer.GetPieceInfo(RenderStamp(0, 4), W, H).Length == 0);

        // ── 4. the exclusion mask exists exactly when a stamp was found ──
        var stamp = RenderStamp(0, 4);
        TShapeDetector.ResetState();
        var found = TShapeDetector.GetTShapes(stamp, W, H);
        bool maskHit = found.Length == 1 &&
                       TShapeDetector.InTShapeMask(found[0].junctionRow, found[0].junctionCol);
        Check(ref failures, "stamp -> junction pixel lands in the exclusion mask", maskHit);

        // an empty frame must leave the mask clear (python only builds it when a
        // stamp was detected).  ResetState first: otherwise the EMA still carries
        // the previous stamp's 140 and would legitimately re-detect it.
        TShapeDetector.ResetState();
        TShapeDetector.GetTShapes(new int[W * H], W, H);
        bool maskClear = true;
        for (int r = 0; r < H && maskClear; r++)
            for (int c = 0; c < W; c++)
                if (TShapeDetector.InTShapeMask(r, c)) { maskClear = false; break; }
        Check(ref failures, "no stamp -> exclusion mask is empty", maskClear);

        // ── 5. temporal EMA (python smooth_alpha = 0.3), pinned numerically ──
        // frame 1: stamp (200) seeds smoothed = 200
        // frame 2: blank -> 0.7*200 = 140 > threshold 100, still detected
        // frame 3: blank -> 0.7*140 =  98 < threshold 100, gone
        TShapeDetector.ResetState();
        var s1 = TShapeDetector.GetTShapes(stamp, W, H);
        var s2 = TShapeDetector.GetTShapes(new int[W * H], W, H);
        var s3 = TShapeDetector.GetTShapes(new int[W * H], W, H);
        Check(ref failures,
            $"EMA alpha=0.3: frame1={s1.Length}, frame2={s2.Length}, frame3={s3.Length} (expect 1,1,0)",
            s1.Length == 1 && s2.Length == 1 && s3.Length == 0);

        // ── 6. the master switch clears everything ──
        TShapeDetector.ResetState();
        TShapeDetector.Enabled = false;
        var off = TShapeDetector.GetTShapes(stamp, W, H);
        TShapeDetector.Enabled = true;
        Check(ref failures, "Enabled=false -> no stamps", off.Length == 0);

        // ── 7. a stamp must not be reported as a pressure touch either ──
        TShapeDetector.ResetState();
        TShapeDetector.GetTShapes(stamp, W, H);          // builds the mask, as MatrixHeatmap does
        var touches = PressureAnalyzer.GetPressureInfo(stamp, W, H,
                                                       RadiusMode.Direction, true);
        bool centreExcluded = true;
        if (found.Length == 1)
        {
            for (int i = 0; i < touches.Length; i++)
            {
                if (TShapeDetector.InTShapeMask((int)touches[i].x, (int)touches[i].y))
                    centreExcluded = false;
            }
        }
        Check(ref failures, "no touch survives inside the stamp mask", centreExcluded);

        // ── 8. arbitration decides who owns a blob ──
        // Default: a disc stays a circular piece even after the stamp pass, because
        // only accepted stamps enter the exclusion mask.
        // Parity mode: the reference masks every large blob, which starves the piece
        // detector — exactly the interference arbitration removes.
        TShapeDetector.RequireStampFit = true;
        TShapeDetector.ResetState();
        var arbBefore = PressureAnalyzer.GetPieceInfo(disc, W, H);
        TShapeDetector.GetTShapes(disc, W, H);
        var arbAfter = PressureAnalyzer.GetPieceInfo(disc, W, H);

        TShapeDetector.RequireStampFit = false;
        TShapeDetector.ResetState();
        var parityBefore = PressureAnalyzer.GetPieceInfo(disc, W, H);
        TShapeDetector.GetTShapes(disc, W, H);
        var parityAfter = PressureAnalyzer.GetPieceInfo(disc, W, H);
        TShapeDetector.RequireStampFit = true;

        Check(ref failures,
            $"arbitration keeps the piece: default {arbBefore.Length}->{arbAfter.Length} piece(s), " +
            $"parity mode {parityBefore.Length}->{parityAfter.Length}",
            arbBefore.Length == 1 && arbAfter.Length == 1 &&
            parityBefore.Length == 1 && parityAfter.Length == 0);

        // ── 9. arbitration (RequireStampFit) lets both detectors coexist ──
        // With the fit test on, a round blob is no longer claimed as a stamp, so its
        // pixels stay out of the exclusion mask and it is still reported as a piece.
        TShapeDetector.RequireStampFit = true;
        TShapeDetector.ResetState();
        var coexistingStamps = TShapeDetector.GetTShapes(disc, W, H);
        var coexistingPieces = PressureAnalyzer.GetPieceInfo(disc, W, H);
        Check(ref failures,
            $"disc with arbitration -> 0 stamps, 1 piece (got {coexistingStamps.Length}, {coexistingPieces.Length})",
            coexistingStamps.Length == 0 && coexistingPieces.Length == 1);

        // a large square also fits a T only degenerately (stroke as thick as it is long)
        var bigSquare = new int[W * H];
        for (int r = 40; r < 60; r++)
            for (int c = 40; c < 60; c++) bigSquare[r * W + c] = 200;
        Check(ref failures, "20x20 square with arbitration -> no stamps",
            Detect(bigSquare, W, H).Length == 0);

        // a square *frame* (the stamp's wall) is neither a stamp nor a circular piece.
        // The piece half of this needs cv2.CHAIN_APPROX_SIMPLE parity: a square has
        // circularity π/4 = 0.785 and aspect 1, so only the collapsed-contour
        // `len < 5` guard rejects it — as python does.
        var frame = new int[W * H];
        for (int r = 30; r < 70; r++)
            for (int c = 30; c < 70; c++)
                if (r < 34 || r >= 66 || c < 34 || c >= 70) frame[r * W + c] = 200;
        TShapeDetector.ResetState();
        var frameStamps = TShapeDetector.GetTShapes(frame, W, H);
        var framePieces = PressureAnalyzer.GetPieceInfo(frame, W, H);
        Check(ref failures,
            $"square frame -> no stamps and no circle pieces (got {frameStamps.Length}, {framePieces.Length})",
            frameStamps.Length == 0 && framePieces.Length == 0);

        // ── 10. the switch really is the arbitration, and parity mode is preserved ──
        TShapeDetector.RequireStampFit = false;
        var parityStamps = Detect(disc, W, H);
        TShapeDetector.RequireStampFit = true;
        Check(ref failures,
            $"RequireStampFit=false restores the reference behaviour (disc -> {parityStamps.Length} stamp, id=" +
            (parityStamps.Length > 0 ? parityStamps[0].id.ToString() : "-") + ")",
            parityStamps.Length == 1 && parityStamps[0].id == 14);

        // a real stamp must pass the fit test with room to spare
        TShapeDetector.RequireStampFit = true;
        var fitStamp = Detect(RenderStamp(0, 4), W, H);
        bool fitOk = fitStamp.Length == 1
                     && fitStamp[0].strokeThickness >= 1.5f
                     && fitStamp[0].fitCoverage >= 0.85f;
        Check(ref failures,
            "real stamp passes the fit test" +
            (fitStamp.Length == 1
                ? $" (t={fitStamp[0].strokeThickness:F2}, coverage={fitStamp[0].fitCoverage:F3})"
                : " (no shape)"),
            fitOk);

        // ── 11. one frame cannot advance the EMA twice ──
        // A frame legitimately reaches GetTShapes from both MatrixHeatmap and a
        // reader's GetTShapes(), so a repeat must not re-blend it (0.3 -> 0.51).
        // Sequence: 200 -> blank gives 0.7*200 = 140 (> 100, detected); a *second*
        // identical blank must be skipped, because advancing would give 98 (< 100).
        TShapeDetector.ResetState();
        var reused = new int[W * H];
        Array.Copy(stamp, reused, stamp.Length);
        var e1 = TShapeDetector.GetTShapes(reused, W, H);      // seeds smoothed = 200
        Array.Clear(reused, 0, reused.Length);
        var e2 = TShapeDetector.GetTShapes(reused, W, H);      // 0.7*200 = 140
        var e3 = TShapeDetector.GetTShapes(reused, W, H);      // same frame -> skipped
        Check(ref failures,
            $"same frame does not advance the EMA twice: {e1.Length},{e2.Length},{e3.Length} (expect 1,1,1)",
            e1.Length == 1 && e2.Length == 1 && e3.Length == 1);

        // a fresh array with identical content is a new frame and must advance
        TShapeDetector.ResetState();
        var f1 = TShapeDetector.GetTShapes(stamp, W, H);
        var f2 = TShapeDetector.GetTShapes(new int[W * H], W, H);
        var f3 = TShapeDetector.GetTShapes(new int[W * H], W, H);
        Check(ref failures,
            $"distinct frames still advance: {f1.Length},{f2.Length},{f3.Length} (expect 1,1,0)",
            f1.Length == 1 && f2.Length == 1 && f3.Length == 0);

        return failures;
    }

    static TShapeInfo[] Detect(int[] data, int w, int h)
    {
        TShapeDetector.ResetState();
        return TShapeDetector.GetTShapes(data, w, h);
    }

    /// <summary>
    /// Render one of the 16 real stamps onto the sensor.
    /// Layout: idx = row*W + col.  The T is row 0 plus column 5; the four identity
    /// dots are selected by bits of <paramref name="id"/>.
    /// </summary>
    static int[] RenderStamp(int id, int cell, int value = 200)
    {
        var data = new int[W * H];
        int off = (W - 11 * cell) / 2;

        void Cell(int row, int col)
        {
            for (int r = 0; r < cell; r++)
                for (int c = 0; c < cell; c++)
                    data[(off + row * cell + r) * W + off + col * cell + c] = value;
        }

        for (int c = 0; c < 11; c++) Cell(0, c);      // bar
        for (int r = 0; r < 11; r++) Cell(r, 5);      // stem
        if ((id & 1) != 0) Cell(10, 0);
        if ((id & 2) != 0) Cell(10, 10);
        if ((id & 4) != 0) Cell(5, 0);
        if ((id & 8) != 0) Cell(5, 10);
        return data;
    }

    static void Check(ref int failures, string name, bool cond)
    {
        Console.WriteLine((cond ? "PASS  " : "FAIL  ") + name);
        if (!cond) failures++;
    }
}

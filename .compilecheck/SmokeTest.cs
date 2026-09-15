using System;
using DevicePipe;

// Compile-check + smoke test harness for DevicePipe (run outside Unity).
// Usage: dotnet run -c Debug --project .compilecheck
static class SmokeTest
{
    static int _failures;

    static void Check(bool cond, string name)
    {
        Console.WriteLine((cond ? "PASS  " : "FAIL  ") + name);
        if (!cond) _failures++;
    }

    static int Main()
    {
        // data layout: idx = x * width + y, x ∈ [0, height)
        const int W = 40, H = 60;

        // ── Test 1: solid disk → GetPieceInfo ──
        int[] disk = new int[W * H];
        PaintDisk(disk, W, 30, 20, 8, 200);
        var pieces = PressureAnalyzer.GetPieceInfo(disk, W, H);
        Check(pieces.Length == 1, $"disk detected once (got {pieces.Length})");
        if (pieces.Length == 1)
        {
            Check(Math.Abs(pieces[0].pos_x - 30) < 1.5 && Math.Abs(pieces[0].pos_y - 20) < 1.5,
                $"disk center ({pieces[0].pos_x:F1},{pieces[0].pos_y:F1}) ≈ (30,20)");
            Check(pieces[0].radius > 5 && pieces[0].radius < 11,
                $"disk radius {pieces[0].radius:F1} in (5,11)");
        }

        // ── Test 2: two disks → two pieces ──
        var two = new int[W * H];
        PaintDisk(two, W, 10, 10, 6, 200);
        PaintDisk(two, W, 45, 30, 7, 200);
        Check(PressureAnalyzer.GetPieceInfo(two, W, H).Length == 2, "two disks → two pieces");

        // ── Test 3: elongated blob → rejected by aspect ratio ──
        var bar = new int[W * H];
        for (int px = 20; px < 50; px++)
            for (int py = 28; py < 32; py++)
                bar[px * W + py] = 200;
        Check(PressureAnalyzer.GetPieceInfo(bar, W, H).Length == 0, "elongated blob → no pieces");

        // ── Test 4: empty/null frame → empty results, no crash ──
        var empty = new int[W * H];
        Check(PressureAnalyzer.GetPieceInfo(empty, W, H).Length == 0, "empty → no pieces");
        Check(PressureAnalyzer.GetPieceInfo(null, W, H).Length == 0, "null → no pieces");

        // ── Test 5: PieceTracker — stable IDs across frames ──
        var tracker = new PieceTracker();
        var f1 = tracker.Track(new[] { new PieceInfo { pos_x = 30, pos_y = 20, radius = 8 } });
        Check(f1.Length == 1 && f1[0].id == 1, $"first piece gets id 1 (got {(f1.Length > 0 ? f1[0].id : -1)})");

        var f2 = tracker.Track(new[] { new PieceInfo { pos_x = 32, pos_y = 21, radius = 8 } });
        Check(f2.Length == 1 && f2[0].id == 1, $"moved piece keeps id 1 (got {(f2.Length > 0 ? f2[0].id : -1)})");

        var f3 = tracker.Track(new[]
        {
            new PieceInfo { pos_x = 32, pos_y = 21, radius = 8 },
            new PieceInfo { pos_x = 10, pos_y = 10, radius = 6 },
        });
        Check(f3.Length == 2 && f3[0].id == 1 && f3[1].id == 2,
            $"old piece id 1, new piece id 2 (got {(f3.Length > 1 ? $"{f3[0].id},{f3[1].id}" : "n/a")})");

        var f4 = tracker.Track(new[]
        {
            new PieceInfo { pos_x = 55, pos_y = 40, radius = 8 }, // moved > MatchDist → new id
            new PieceInfo { pos_x = 11, pos_y = 10, radius = 6 }, // small move → keeps id 2
        });
        Check(f4.Length == 2 && f4[0].id != 1 && f4[1].id == 2,
            $"far-moved gets new id, near-moved keeps id 2 (got {(f4.Length > 1 ? $"{f4[0].id},{f4[1].id}" : "n/a")})");

        tracker.Track(System.Array.Empty<PieceInfo>());
        var f6 = tracker.Track(new[] { new PieceInfo { pos_x = 11, pos_y = 10, radius = 6 } });
        Check(f6.Length == 1 && f6[0].id != 2,
            $"piece after empty-frame gap gets a fresh id (got {(f6.Length > 0 ? f6[0].id : -1)})");

        // ── Test 6: scratch reuse — interleaved calls stay correct over many frames ──
        bool stressOk = true;
        for (int iter = 0; iter < 200; iter++)
        {
            var p1 = PressureAnalyzer.GetPieceInfo(disk, W, H);
            var p2 = PressureAnalyzer.GetPieceInfo(two, W, H);
            var p3 = PressureAnalyzer.GetPieceInfo(bar, W, H);
            if (p1.Length != 1 || p2.Length != 2 || p3.Length != 0) { stressOk = false; break; }
        }
        Check(stressOk, "200× interleaved GetPieceInfo calls keep stable results");

        Console.WriteLine(_failures == 0 ? "ALL TESTS PASSED" : $"{_failures} TEST(S) FAILED");
        return _failures == 0 ? 0 : 1;
    }

    static void PaintDisk(int[] data, int w, int cx, int cy, int r, int v)
    {
        int h = data.Length / w;
        for (int px = cx - r; px <= cx + r; px++)
            for (int py = cy - r; py <= cy + r; py++)
            {
                if ((px - cx) * (px - cx) + (py - cy) * (py - cy) > r * r) continue;
                if ((uint)px >= (uint)h || (uint)py >= (uint)w) continue;
                data[px * w + py] = v;
            }
    }
}

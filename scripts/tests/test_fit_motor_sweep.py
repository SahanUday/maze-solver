import contextlib
import io
import tempfile
import unittest
from pathlib import Path

from support import load_script

fit = load_script("fit-motor-sweep.py")

DEAD, SLOPE = 100, 0.36  # rpm = SLOPE * (duty - DEAD) above the dead zone
CPR, WHEEL = 898, 65.0


def ladder():
    up = list(range(40, 761, 20))
    return [(d, "up") for d in up] + [(d, "down") for d in reversed(up)]


def counts_for(duty, window_s=0.2, gain=1.0):
    rpm = max(0.0, SLOPE * (duty - DEAD)) * gain
    return round(rpm / 60.0 * CPR * window_s)


def sweep_lines(*, microseconds=False, right_gain=1.0, skip=lambda row: False):
    """A full 296-row capture in the robot's layout (or the older microsecond one)."""
    lines = []
    for motor in "LR":
        for direction in ("forward", "reverse"):
            for duty, sweep in ladder():
                row = (motor, direction, sweep, duty)
                if skip(row):
                    continue
                n = counts_for(duty, gain=right_gain if motor == "R" else 1.0)
                n = -n if direction == "reverse" else n
                if microseconds:
                    lines.append(f"{motor},{direction},{sweep},{duty},{abs(n)},200000,0.0,0.0")
                else:
                    lines.append(f"{motor},{direction},{sweep},{duty},{n},200")
    return lines


def run_main(text, *extra):
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "capture.txt"
        path.write_text(text)
        out = io.StringIO()
        err = io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = fit.main([str(path), *extra])
        return code, out.getvalue(), err.getvalue()


class ParseTest(unittest.TestCase):
    def test_robot_rows_use_milliseconds_and_keep_the_sign(self):
        rows, _ = fit.parse("R,reverse,down,300,-42,201\n")
        self.assertEqual(rows, [{"motor": "R", "direction": "reverse", "sweep": "down",
                                 "duty": 300, "counts": -42, "dt_s": 0.201}])

    def test_older_file_rows_use_microseconds(self):
        rows, _ = fit.parse("L,forward,up,100,34,200016,11.36,38.7\n")
        self.assertAlmostEqual(rows[0]["dt_s"], 0.200016)

    def test_chatter_header_and_notes_are_ignored_and_notes_are_kept(self):
        text = "\n".join([
            "us mm F=120 L=0 R=0  (0 = no echo)",
            "run: self-test",
            "# counts_per_rev=898 wheel_mm=65 pack_v=11.4 surface=plywood",
            "motor,direction,sweep,duty,counts,dt_ms",
            "L,forward,up,40,0,200",
            "run: STOPPED - sweep complete (enc L=1 R=2)",
        ])
        rows, notes = fit.parse(text)
        self.assertEqual(len(rows), 1)
        self.assertEqual(notes["pack_v"], "11.4")
        self.assertEqual(notes["surface"], "plywood")

    def test_a_half_written_line_is_not_a_row(self):
        rows, _ = fit.parse("L,forward,up,100,3\nL,forward,up,1")
        self.assertEqual(rows, [])


class MathTest(unittest.TestCase):
    def test_one_revolution_a_second_is_sixty_rpm(self):
        rows = [{"counts": CPR, "dt_s": 1.0}]
        fit.speeds(rows, CPR, 65.0)
        self.assertAlmostEqual(rows[0]["rpm"], 60.0)
        self.assertAlmostEqual(rows[0]["mm_per_s"], 3.14159265 * 65.0, places=3)

    def test_a_reverse_count_gives_a_positive_speed(self):
        rows = [{"counts": -CPR, "dt_s": 1.0}]
        fit.speeds(rows, CPR, 65.0)
        self.assertGreater(rows[0]["rpm"], 0)

    def test_linear_fit_recovers_a_line(self):
        slope, intercept, r2 = fit.linear_fit([(x, 2.0 * x + 5.0) for x in range(10)])
        self.assertAlmostEqual(slope, 2.0)
        self.assertAlmostEqual(intercept, 5.0)
        self.assertAlmostEqual(r2, 1.0)

    def test_too_few_points_are_not_a_line(self):
        self.assertIsNone(fit.linear_fit([(0, 0), (1, 1)]))


class AnalyseTest(unittest.TestCase):
    def rows(self, **kw):
        rows, _ = fit.parse("\n".join(sweep_lines(**kw)))
        fit.speeds(rows, CPR, WHEEL)
        return rows

    def test_dead_zone_slope_and_zero_duty_come_back(self):
        a = fit.analyse(self.rows(), "L", "forward")
        self.assertEqual(a["starts"], 120)  # 100 is on the edge: 0 rpm there, ~7 at 120
        self.assertAlmostEqual(a["fit"]["slope"], SLOPE, delta=0.01)
        self.assertAlmostEqual(a["fit"]["zero_duty"], DEAD, delta=3)
        self.assertGreater(a["fit"]["r2"], 0.99)
        self.assertEqual((a["up"], a["down"]), (37, 37))

    def test_top_speed_is_at_the_top_rung(self):
        a = fit.analyse(self.rows(), "R", "reverse")
        self.assertEqual(a["top_duty"], 760)
        self.assertAlmostEqual(a["top_rpm"], SLOPE * (760 - DEAD), delta=1.0)

    def test_a_weaker_right_wheel_shows_in_the_speed_ratio(self):
        mean, low, high = fit.speed_ratio(self.rows(right_gain=0.9), "forward")
        self.assertAlmostEqual(mean, 0.9, delta=0.01)
        self.assertLessEqual(low, mean)
        self.assertGreaterEqual(high, mean)

    def test_a_missing_combo_is_none(self):
        rows = [r for r in self.rows() if r["motor"] == "L"]
        self.assertIsNone(fit.analyse(rows, "R", "forward"))


class MainTest(unittest.TestCase):
    def test_a_full_capture_with_chatter_passes_and_reports_the_rows(self):
        text = "\n".join(["boot noise", "# pack_v=11.4 surface=plywood"] + sweep_lines() + ["done"])
        code, out, _ = run_main(text)
        self.assertEqual(code, 0, out)
        self.assertIn("rows 296", out)
        self.assertIn("pack_v: 11.4", out)
        self.assertNotIn("INCOMPLETE", out)

    def test_the_older_microsecond_file_is_read_too(self):
        code, out, _ = run_main("\n".join(sweep_lines(microseconds=True)))
        self.assertEqual(code, 0, out)

    def test_a_cut_short_capture_is_flagged_and_exits_1(self):
        code, out, _ = run_main("\n".join(sweep_lines(skip=lambda r: r[0] == "R" and r[3] > 400)))
        self.assertEqual(code, 1)
        self.assertIn("INCOMPLETE", out)

    def test_a_capture_with_no_rows_exits_2(self):
        code, _, err = run_main("just chatter\n")
        self.assertEqual(code, 2)
        self.assertIn("no sweep rows", err)

    def test_a_missing_file_exits_2(self):
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = fit.main(["/nonexistent/capture.txt"])
        self.assertEqual(code, 2)

    def test_no_pack_voltage_is_called_out(self):
        _, out, _ = run_main("\n".join(sweep_lines()))
        self.assertIn("no pack voltage", out)

    def test_a_different_constant_in_the_capture_is_noted(self):
        text = "\n".join(["# counts_per_rev=1000"] + sweep_lines())
        _, out, _ = run_main(text, "--counts-per-rev", "898")
        self.assertIn("the capture says counts_per_rev=1000", out)

    def test_write_csv_matches_the_layout_the_plot_script_reads(self):
        with tempfile.TemporaryDirectory() as tmp:
            src = Path(tmp) / "in.txt"
            dst = Path(tmp) / "out.csv"
            src.write_text("\n".join(sweep_lines()))
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(fit.main([str(src), "--write-csv", str(dst)]), 0)
            lines = dst.read_text().splitlines()
        self.assertEqual(lines[0], "motor,direction,sweep,duty,counts,dt_us,rpm,mm_per_s")
        self.assertEqual(len(lines), 297)
        self.assertEqual(lines[-1].split(",")[:3], ["R", "reverse", "down"])
        again, _ = fit.parse("\n".join(lines))
        self.assertEqual(len(again), 296)


if __name__ == "__main__":
    unittest.main()

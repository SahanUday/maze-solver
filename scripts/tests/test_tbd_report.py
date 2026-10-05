import unittest

from support import Tree, load_script

tbd = load_script("tbd-report.py")

LISTING = """6 outstanding TBD placeholder(s):

include/RobotSpec.h:5://  `<<TBD HARDWARE>>` / `<<TBD CALIBRATION>>` = not yet measured/tuned.
include/RobotSpec.h:53:// ---- Wheel / drivetrain geometry --------------------- <<TBD HARDWARE>> --
include/RobotSpec.h:67:// ---- Motion PID gains -------------------------------- <<TBD CALIBRATION>> --
scripts/check-ram-budget.sh:5:# <<TBD FIRST-ISR>>: replace with a real stack high-water-mark measurement.
include/RobotConfig.h:9://  `<<TBD HARDWARE>>` = not yet measured. See scripts/list-tbds.sh.
"""


class ParseTest(unittest.TestCase):
    def test_real_placeholders_only_sorted_by_kind(self):
        items = tbd.parse(LISTING)
        self.assertEqual([(i["kind"], i["file"], i["line"]) for i in items], [("CALIBRATION", "include/RobotSpec.h", 67), ("FIRST-ISR", "scripts/check-ram-budget.sh", 5), ("HARDWARE", "include/RobotSpec.h", 53)])

    def test_description_is_cleaned_up(self):
        what = {i["kind"]: i["what"] for i in tbd.parse(LISTING)}
        self.assertEqual(what["HARDWARE"], "Wheel / drivetrain geometry")
        self.assertEqual(what["FIRST-ISR"], "replace with a real stack high-water-mark measurement.")

    def test_nothing_outstanding(self):
        self.assertEqual(tbd.parse("No outstanding TBD placeholders.\n"), [])


class RenderTest(unittest.TestCase):
    def setUp(self):
        self.items = tbd.parse(LISTING)

    def test_summary_and_table(self):
        text = tbd.render(self.items)
        self.assertIn("**3 open**: 1 CALIBRATION · 1 FIRST-ISR · 1 HARDWARE", text)
        self.assertIn("| HARDWARE | `include/RobotSpec.h:53` | Wheel / drivetrain geometry |", text)
        self.assertNotIn("This PR:", text)  # no base, no comparison

    def test_links_when_repo_and_sha_given(self):
        text = tbd.render(self.items, repo="o/r", sha="abc")
        self.assertIn("[include/RobotSpec.h:53](https://github.com/o/r/blob/abc/include/RobotSpec.h#L53)", text)

    def test_added_and_resolved_are_compared_without_line_numbers(self):
        base = [dict(i) for i in self.items]
        base[2]["line"] = 99  # same placeholder, moved: not a change
        base.append({"file": "include/RobotSpec.h", "line": 70, "kind": "HARDWARE", "what": "Old thing"})
        head = self.items + [{"file": "src/hal/x.cpp", "line": 3, "kind": "HARDWARE", "what": "New thing"}]
        text = tbd.render(head, base)
        self.assertIn("**1 added**, **1 resolved**", text)
        self.assertIn("Resolved here: Old thing (`include/RobotSpec.h`)", text)
        self.assertIn("New thing (new) |", text)
        self.assertNotIn("Wheel / drivetrain geometry (new)", text)

    def test_empty_and_pipe_escaping(self):
        self.assertIn("None - every placeholder is resolved.", tbd.render([]))
        text = tbd.render([{"file": "a.h", "line": 1, "kind": "HARDWARE", "what": "a | b"}])
        self.assertIn("a \\| b", text)

    def test_long_lists_collapse(self):
        many = [{"file": "a.h", "line": n, "kind": "HARDWARE", "what": f"thing {n}"} for n in range(1, 15)]
        text = tbd.render(many)
        self.assertIn("<details><summary>Show all 14</summary>", text)
        self.assertLess(text.index("<details>"), text.index("| Kind |"))


class CollectTest(unittest.TestCase):
    def test_runs_the_list_script_in_the_given_tree(self):
        tree = Tree({"include/A.h": "// ---- Motor limits ---- <<TBD HARDWARE>> --\n"})
        self.addCleanup(tree.close)
        script = tbd.Path(__file__).resolve().parent.parent / "list-tbds.sh"
        items = tbd.collect(tree.root, script)
        self.assertEqual([(i["file"], i["kind"], i["what"]) for i in items], [("include/A.h", "HARDWARE", "Motor limits")])


if __name__ == "__main__":
    unittest.main()

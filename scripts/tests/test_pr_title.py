import contextlib
import io
import os
import unittest
from unittest import mock

from support import load_script

title = load_script("check-pr-title.py")


class PrTitleTest(unittest.TestCase):
    def test_every_merged_title_so_far_is_accepted(self):
        for t in [
            "docs: Tidy RobotConfig comments and HAL section wording",
            "feat(hal): Add motor and encoder drivers for JGA25-370 + BTS7960",
            "feat: Project skeleton, fixed-period loop, native build",
            "feat(docs): Add docs/architecture foundation and hybrid abstraction ADR",
            "feat(CI): Add automated CI + pre-commit hooks for embedded code quality",
            "ci: Restructure CI and add pin, ISR and docs-drift checks",
        ]:
            self.assertEqual(title.problems(t), [], t)

    def test_breaking_marker_and_digit_subject(self):
        self.assertEqual(title.problems("feat(hal)!: Rework the motor API"), [])
        self.assertEqual(title.problems("fix: 4x decoding at counter wrap"), [])

    def test_not_conventional(self):
        for t in ["Add .gitignore", "SW-01: Project skeleton", "feat Add thing", "feat:Add thing", "feat(): Add thing", ""]:
            self.assertTrue(title.problems(t), t)

    def test_unknown_type_is_named(self):
        self.assertIn("type must be one of", title.problems("feature: Add thing")[0])

    def test_subject_style(self):
        self.assertTrue(title.problems("feat: add lower-case"))
        self.assertTrue(title.problems("feat: Ends with a period."))

    def test_length_limit(self):
        self.assertEqual(title.problems("feat: " + "A" * 66), [])
        self.assertTrue(title.problems("feat: " + "A" * 67))

    def test_main_exit_codes_and_no_annotation_off_actions(self):
        out = io.StringIO()
        with mock.patch.dict(os.environ, {"GITHUB_ACTIONS": ""}), contextlib.redirect_stdout(out):
            self.assertEqual(title.main(["feat(hal): Add IR array driver"]), 0)
            self.assertEqual(title.main(["nope"]), 1)
        self.assertNotIn("::error", out.getvalue())
        with mock.patch.dict(os.environ, {"PR_TITLE": ""}), contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(title.main([]), 2)

    def test_annotation_on_actions(self):
        out = io.StringIO()
        with mock.patch.dict(os.environ, {"GITHUB_ACTIONS": "true"}), contextlib.redirect_stdout(out):
            self.assertEqual(title.main(["nope"]), 1)
        self.assertIn("::error title=PR title::", out.getvalue())


if __name__ == "__main__":
    unittest.main()

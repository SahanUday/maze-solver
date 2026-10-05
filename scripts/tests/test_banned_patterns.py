"""Self-tests for scripts/check-banned-patterns.sh (run as a subprocess: it is
the shell script itself, including its grep/sed pipeline, that is under test)."""

import subprocess
import unittest

from support import SCRIPTS, Tree

SCRIPT = SCRIPTS / "check-banned-patterns.sh"

HAL_CALL = "void init() {\n    digitalWrite(13, 1);\n}\n"


class BannedPatternsTest(unittest.TestCase):
    def run_check(self, files):
        tree = Tree(files)
        self.addCleanup(tree.close)
        return subprocess.run([str(SCRIPT)], cwd=tree.root, capture_output=True, text=True)

    def test_clean_hal_passes(self):
        result = self.run_check({"src/hal/motors.cpp": "void init() {\n    DDRB |= 1;\n}\n"})
        self.assertEqual(result.returncode, 0, result.stdout)

    def test_arduino_gpio_in_hal_is_rejected(self):
        result = self.run_check({"src/hal/motors.cpp": HAL_CALL})
        self.assertEqual(result.returncode, 1)
        self.assertIn("src/hal/motors.cpp:2", result.stdout)

    # Regression: whole lines mentioning the API after a '//' were filtered out,
    # so a comment that happened to name the call hid the call itself.
    def test_a_comment_naming_the_api_does_not_hide_a_real_call(self):
        source = "void init() {\n    digitalWrite(13, 1);  // digitalWrite() is fine, honestly\n}\n"
        result = self.run_check({"src/hal/motors.cpp": source})
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertIn("src/hal/motors.cpp:2", result.stdout)

    def test_a_comment_explaining_why_the_api_is_avoided_still_passes(self):
        source = "// Register writes, not digitalWrite(), so the pulse is 2 cycles.\nvoid init() {\n    DDRB |= 1;\n}\n"
        result = self.run_check({"src/hal/motors.cpp": source})
        self.assertEqual(result.returncode, 0, result.stdout)

    def test_arduino_gpio_outside_hal_is_allowed(self):
        result = self.run_check({"src/main.cpp": HAL_CALL})
        self.assertEqual(result.returncode, 0, result.stdout)


if __name__ == "__main__":
    unittest.main()

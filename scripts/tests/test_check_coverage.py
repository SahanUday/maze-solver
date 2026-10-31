import json
import os
import shutil
import stat
import subprocess
import tempfile
import unittest
from pathlib import Path

from support import SCRIPTS, Tree

LOGIC = "inline int twice(int x)\n{\n    return x * 2;\n}\n"


@unittest.skipUnless(shutil.which("bash"), "check-coverage.sh needs bash")
class CheckCoverageWiringTest(unittest.TestCase):
    """Runs the real scripts/check-coverage.sh against a stub gcovr.

    The stub stands in for gcovr: it writes the --json-summary file it is
    given (or none) and exits with the status it is told to. What is under test
    is the wiring, that both checks always run and the exit status is right.
    """

    def run_script(self, files, gcovr_status, gcovr_report, floor="90"):
        tree = Tree({**files, ".pio/build/native_cov/.keep": ""})
        self.addCleanup(tree.close)
        bin_dir = tempfile.TemporaryDirectory()
        self.addCleanup(bin_dir.cleanup)
        write = f'printf %s {json.dumps(json.dumps(gcovr_report))} > "$out"' if gcovr_report is not None else ":"
        stub = Path(bin_dir.name) / "gcovr"
        stub.write_text(
            "#!/usr/bin/env bash\n"
            'while [ $# -gt 0 ]; do [ "$1" = --json-summary ] && out="$2"; shift; done\n'
            f"echo 'stub gcovr: floor {floor}'\n{write}\nexit {gcovr_status}\n"
        )
        stub.chmod(stub.stat().st_mode | stat.S_IEXEC)
        env = {**os.environ, "PATH": f"{bin_dir.name}{os.pathsep}{os.environ['PATH']}", "COVERAGE_FLOOR": floor}
        done = subprocess.run(["bash", str(SCRIPTS / "check-coverage.sh")], cwd=tree.root, env=env, capture_output=True, text=True)
        return done.returncode, done.stdout + done.stderr

    @staticmethod
    def report(*files):
        return {"files": [{"filename": f} for f in files]}

    def test_floor_and_headers_both_ok_exits_zero(self):
        code, out = self.run_script({"include/A.h": LOGIC}, 0, self.report("include/A.h"))
        self.assertEqual(code, 0, out)
        self.assertIn("untested-headers: ok", out)

    def test_an_untested_logic_header_fails_even_when_the_floor_passes(self):
        code, out = self.run_script({"include/A.h": LOGIC}, 0, self.report())
        self.assertEqual(code, 1, out)
        self.assertIn("include/A.h:1: [untested-header]", out)

    def test_a_failed_floor_fails_and_the_headers_check_still_runs(self):
        code, out = self.run_script({"include/A.h": LOGIC}, 2, self.report())
        self.assertEqual(code, 1, out)  # the header finding is the later status
        self.assertIn("[untested-header]", out)

    def test_a_failed_floor_alone_still_fails(self):
        code, out = self.run_script({"include/A.h": LOGIC}, 2, self.report("include/A.h"))
        self.assertEqual(code, 2, out)
        self.assertIn("untested-headers: ok", out)

    def test_no_report_written_fails_instead_of_passing(self):
        code, out = self.run_script({"include/A.h": LOGIC}, 0, None)
        self.assertEqual(code, 2, out)
        self.assertIn("cannot read the coverage report", out)

    def test_missing_build_dir_is_a_usage_error(self):
        tree = Tree({"include/A.h": LOGIC})
        self.addCleanup(tree.close)
        done = subprocess.run(["bash", str(SCRIPTS / "check-coverage.sh")], cwd=tree.root, capture_output=True, text=True)
        self.assertEqual(done.returncode, 2)
        self.assertIn("run 'pio test -e native_cov' first", done.stderr)


if __name__ == "__main__":
    unittest.main()

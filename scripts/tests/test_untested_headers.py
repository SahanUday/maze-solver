import contextlib
import io
import json
import unittest
from pathlib import Path

from support import Tree, load_script

untested = load_script("check-untested-headers.py")

LOGIC = "inline int twice(int x)\n{\n    return x * 2;\n}\n"
DATA = "#pragma once\nconstexpr int K = 3;\nstruct S {\n    int a = 0;\n    int b[2] = {0, 0};\n};\n"


def report(*files):
    return {"files": [{"filename": f} for f in files]}


class UntestedHeadersTest(unittest.TestCase):
    def run_check(self, files, covered=()):
        tree = Tree(files)
        self.addCleanup(tree.close)
        return untested.check(tree.root, report(*covered))

    def test_logic_header_in_the_report_passes(self):
        self.assertEqual(self.run_check({"include/A.h": LOGIC}, covered=["include/A.h"]), [])

    def test_logic_header_missing_from_the_report_is_reported(self):
        findings = self.run_check({"include/A.h": LOGIC})
        self.assertEqual([f.path for f in findings], ["include/A.h"])
        self.assertEqual(findings[0].line, 1)  # the signature, where the body starts
        self.assertIn("no test compiles it", findings[0].message)

    def test_data_only_headers_need_no_test(self):
        self.assertEqual(self.run_check({"include/Spec.h": DATA, "lib/m/Maze.h": "namespace m { constexpr int k = 1; }\n"}), [])

    def test_headers_under_lib_are_checked_too(self):
        findings = self.run_check({"lib/maze/Flood.h": LOGIC})
        self.assertEqual([f.path for f in findings], ["lib/maze/Flood.h"])

    def test_a_function_body_with_const_noexcept_or_trailing_return_counts(self):
        for body in ("int f() const {\n return 1;\n}\n", "int f() noexcept {\n return 1;\n}\n", "auto f() -> int {\n return 1;\n}\n"):
            with self.subTest(body=body):
                self.assertEqual(len(self.run_check({"include/A.h": body})), 1)

    def test_logic_in_a_comment_does_not_count(self):
        header = "// int f() { return 1; }\n/* int g() { return 2; } */\nconstexpr int K = 1;\n"
        self.assertEqual(self.run_check({"include/A.h": header}), [])

    def test_attributes_and_macro_bodies_are_not_functions(self):
        for header in (
            "struct S __attribute__((packed)) {\n    int a;\n};\n",
            "#define SQ(x) { x }\n",
            "#define MULTI(x) \\\n    { x; \\\n    }\nconstexpr int K = 1;\n",
        ):
            with self.subTest(header=header):
                self.assertEqual(self.run_check({"include/A.h": header}), [])

    def test_a_real_function_after_a_macro_is_still_found_on_the_right_line(self):
        header = "#define SQ(x) { x }\nconstexpr int K = 1;\n" + LOGIC
        findings = self.run_check({"include/A.h": header})
        self.assertEqual([(f.path, f.line) for f in findings], [("include/A.h", 3)])

    def test_a_constructor_body_is_logic(self):
        self.assertEqual(len(self.run_check({"include/A.h": "struct S {\n    S() : a(1) {}\n    int a;\n};\n"})), 1)

    def test_the_message_names_the_false_positive_escape(self):
        findings = self.run_check({"include/A.h": LOGIC})
        self.assertIn("false positive", findings[0].message)

    def test_exemption_with_a_reason_passes(self):
        header = "// coverage-exempt: needs Arduino.h, host cannot build it\n" + LOGIC
        self.assertEqual(self.run_check({"include/A.h": header}), [])

    def test_exemption_without_a_reason_is_reported_and_says_so(self):
        findings = self.run_check({"include/A.h": "// coverage-exempt:\n" + LOGIC})
        self.assertEqual(len(findings), 1)
        self.assertIn("needs a reason", findings[0].message)

    def test_non_header_files_are_ignored(self):
        self.assertEqual(self.run_check({"include/a.cpp": LOGIC, "src/b.h": LOGIC}), [])

    def test_report_paths_are_matched_as_posix_relative_paths(self):
        self.assertEqual(self.run_check({"lib/maze/Flood.h": LOGIC}, covered=["lib/maze/Flood.h"]), [])

    def run_main(self, files, report_text):
        tree = Tree({**files, "report.json": report_text})
        self.addCleanup(tree.close)
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = untested.main(["--root", str(tree.root), "--report", str(tree.root / "report.json")])
        return code, out.getvalue(), err.getvalue()

    def test_main_returns_nonzero_on_findings_and_zero_when_clean(self):
        code, out, _ = self.run_main({"include/A.h": LOGIC}, json.dumps(report()))
        self.assertEqual(code, 1)
        self.assertIn("include/A.h:1: [untested-header]", out)
        code, out, _ = self.run_main({"include/A.h": LOGIC}, json.dumps(report("include/A.h")))
        self.assertEqual((code, out.strip()), (0, "untested-headers: ok"))

    def test_main_reports_an_unreadable_report_instead_of_passing(self):
        code, _, err = self.run_main({"include/A.h": LOGIC}, "not json")
        self.assertEqual(code, 2)
        self.assertIn("cannot read the coverage report", err)


if __name__ == "__main__":
    unittest.main()

import subprocess
import unittest

from support import Tree, load_script

docs = load_script("check-docs-drift.py")

ADR = "docs/architecture/decisions/{}.md"


def adr(number, title="Some choice", status="Accepted", body="Because.\n"):
    return f"# {number}: {title}\n\n- **Status:** {status}\n- **Date:** 2026-10-03\n\n## Context\n\n{body}"


class StaticChecksTest(unittest.TestCase):
    def run_check(self, files):
        tree = Tree(files)
        self.addCleanup(tree.close)
        return docs.check(tree.root)

    def rules(self, files):
        return [f.rule for f in self.run_check(files)]

    def test_clean_docs_pass(self):
        files = {
            ADR.format("0001-first"): adr("0001"),
            ADR.format("0002-second"): adr("0002"),
            "src/hal/motors.cpp": "",
            "docs/architecture/modules/motors.md": "# motors\n",
        }
        self.assertEqual(self.run_check(files), [])

    def test_hal_module_without_a_doc_is_reported(self):
        self.assertEqual(self.rules({"src/hal/motors.cpp": ""}), ["modules"])

    def test_two_adrs_with_the_same_number_conflict(self):
        files = {ADR.format("0002-drive"): adr("0002"), ADR.format("0002-ir-array"): adr("0002")}
        findings = self.run_check(files)
        self.assertEqual([f.rule for f in findings], ["adr-names"])
        self.assertIn("0002", findings[0].message)

    def test_adr_naming_heading_and_status(self):
        self.assertEqual(self.rules({"docs/architecture/decisions/notes.md": "x"}), ["adr-names"])
        self.assertEqual(self.rules({ADR.format("0003-x"): adr("0004")}), ["adr-names"])
        self.assertEqual(self.rules({ADR.format("0003-x"): "# 0003: x\n\nno status\n"}), ["adr-names"])


class FrozenAdrTest(unittest.TestCase):
    def setUp(self):
        self.tree = Tree({ADR.format("0001-first"): adr("0001"), ADR.format("0002-draft"): adr("0002", status="Proposed")})
        self.addCleanup(self.tree.close)
        self.git("init", "-q")
        self.git("-c", "user.name=t", "-c", "user.email=t@t", "add", "-A")
        self.git("-c", "user.name=t", "-c", "user.email=t@t", "commit", "-q", "-m", "base")

    def git(self, *args):
        subprocess.run(["git", *args], cwd=self.tree.root, check=True, capture_output=True)

    def commit(self):
        self.git("-c", "user.name=t", "-c", "user.email=t@t", "add", "-A")
        self.git("-c", "user.name=t", "-c", "user.email=t@t", "commit", "-q", "-m", "change")

    def frozen(self):
        return [f.rule for f in docs.check(self.tree.root, base="HEAD~1")]

    def write(self, name, text):
        (self.tree.root / ADR.format(name)).write_text(text)

    def test_editing_an_accepted_adr_is_rejected(self):
        self.write("0001-first", adr("0001", body="Actually, no.\n"))
        self.commit()
        self.assertEqual(self.frozen(), ["adr-frozen"])

    def test_superseding_status_change_alone_is_allowed(self):
        self.write("0001-first", adr("0001", status="Superseded by 0003"))
        self.commit()
        self.assertEqual(self.frozen(), [])

    def test_status_change_plus_body_edit_is_rejected(self):
        self.write("0001-first", adr("0001", status="Superseded by 0003", body="Rewritten.\n"))
        self.commit()
        self.assertEqual(self.frozen(), ["adr-frozen"])

    def test_deleting_an_accepted_adr_is_rejected(self):
        (self.tree.root / ADR.format("0001-first")).unlink()
        self.commit()
        self.assertEqual(self.frozen(), ["adr-frozen"])

    def test_proposed_adrs_stay_editable_and_new_adrs_are_fine(self):
        self.write("0002-draft", adr("0002", status="Proposed", body="Reworked.\n"))
        self.write("0003-new", adr("0003"))
        self.commit()
        self.assertEqual(self.frozen(), [])


if __name__ == "__main__":
    unittest.main()

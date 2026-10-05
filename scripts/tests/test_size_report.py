import unittest
from pathlib import Path

from support import SCRIPTS, load_script

size = load_script("size-report.py")

AVR_SIZE = """AVR Memory Usage
----------------
Device: atmega2560

Program:    3602 bytes (1.4% Full)
(.text + .data + .bootloader)

Data:        248 bytes (3.0% Full)
(.data + .bss + .noinit)
"""

NM = """00001034 00000154 t HardwareSerial::write(unsigned char)
08389144 00000157 b Serial
00001724 00000160 T __vector_3
00002206 00001264 T main
08389000 00000004 d counter
00000010 00000002 N .comment
garbage line
"""

BUDGETS = {"flash": 253952, "ram": 8192}


class SizeReportTest(unittest.TestCase):
    def test_parse_avr_size(self):
        self.assertEqual(size.parse_avr_size(AVR_SIZE), {"flash": 3602, "ram": 248})
        with self.assertRaises(ValueError):
            size.parse_avr_size("nothing useful")

    def test_parse_nm_classifies_and_keeps_demangled_names_with_spaces(self):
        syms = size.parse_nm(NM)
        self.assertEqual(syms["HardwareSerial::write(unsigned char)"], ("flash", 154))
        self.assertEqual(syms["Serial"], ("ram", 157))
        self.assertEqual(syms["counter"], ("ram", 4))
        self.assertNotIn(".comment", syms)  # type N is neither flash nor SRAM
        self.assertEqual(len(syms), 5)

    def test_duplicate_symbol_names_are_summed(self):
        syms = size.parse_nm("00000001 00000010 t foo\n00000002 00000020 t foo\n")
        self.assertEqual(syms["foo"], ("flash", 30))

    def test_delta_format(self):
        self.assertEqual(size.delta(110, 100), "+10 B")
        self.assertEqual(size.delta(90, 100), "-10 B")
        self.assertEqual(size.delta(1000, 1000), "no change")
        self.assertEqual(size.delta(2000, 1000), "+1,000 B")

    def test_render_with_baseline_lists_biggest_changes_first(self):
        head = {"flash": 3900, "ram": 448}
        base = {"flash": 3600, "ram": 248}
        head_syms = {"main": ("flash", 1350), "g_pad": ("ram", 200), "old": ("flash", 10)}
        base_syms = {"main": ("flash", 1264), "old": ("flash", 10), "gone": ("flash", 5)}
        text = size.render(head, base, BUDGETS, head_syms, base_syms, "main (abc1234)")
        self.assertTrue(text.startswith(size.MARKER))
        self.assertIn("**+300 B**", text)
        self.assertIn("**+200 B**", text)
        self.assertIn("`g_pad` (new) | SRAM", text)
        self.assertIn("`gone` (removed)", text)
        self.assertNotIn("`old`", text)  # unchanged symbols are not listed
        self.assertLess(text.index("g_pad"), text.index("`main`"))  # +200 before +86

    def test_render_without_baseline(self):
        text = size.render({"flash": 3602, "ram": 248}, None, BUDGETS)
        self.assertIn("3,602 B (1.4%)", text)
        self.assertIn("No baseline build", text)
        self.assertNotIn("Change", text)

    def test_pipe_in_a_symbol_name_does_not_break_the_table(self):
        text = size.render({"flash": 2, "ram": 2}, {"flash": 1, "ram": 1}, BUDGETS, {"operator|": ("flash", 2)}, {"operator|": ("flash", 1)})
        self.assertIn("`operator\\|`", text)

    def test_budgets_come_from_the_budget_scripts(self):
        budgets = size.read_budgets(SCRIPTS.parent)
        self.assertEqual(budgets, {"flash": 253952, "ram": 8192})


if __name__ == "__main__":
    unittest.main()

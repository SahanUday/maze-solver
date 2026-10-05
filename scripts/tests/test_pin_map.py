import os
import unittest
from pathlib import Path

from support import Tree, load_script

pin_map = load_script("check-pin-map.py")

CONFIG = "include/RobotConfig.h"


def config(*lines):
    return {CONFIG: "\n".join(lines) + "\n"}


class PinMapTest(unittest.TestCase):
    def run_check(self, files):
        tree = Tree(files)
        self.addCleanup(tree.close)
        return pin_map.check(tree.root)

    def rules(self, files):
        return [f.rule for f in self.run_check(files)]

    def test_clean_config_passes(self):
        files = config(
            "constexpr uint8_t PIN_ENC_L_A = 2;  // PE4, INT4",
            "constexpr uint8_t PIN_US_ECHO = A13; // PK5",
            "constexpr uint8_t PIN_I2C_SDA = 20; // INT1",
            "constexpr uint8_t PIN_IR[2] = {A0, A1};",
        )
        self.assertEqual(self.run_check(files), [])

    def test_duplicate_pin_is_reported_once_with_both_names(self):
        findings = self.run_check(config("constexpr uint8_t PIN_A = 30;", "constexpr uint8_t PIN_B = 30;"))
        self.assertEqual(len(findings), 1)
        self.assertIn("PIN_A", findings[0].message)
        self.assertIn("PIN_B", findings[0].message)

    def test_duplicate_between_array_element_and_scalar(self):
        files = config("constexpr uint8_t PIN_IR[2] = {A0, A1};", "constexpr uint8_t PIN_X = 55;")
        self.assertEqual(self.rules(files), ["pins"])  # A1 is pin 55

    def test_alias_is_not_a_second_claim(self):
        files = config("constexpr uint8_t PIN_A = 30;", "constexpr uint8_t PIN_B = PIN_A;")
        self.assertEqual(self.run_check(files), [])

    def test_reserved_and_out_of_range_pins(self):
        self.assertEqual(self.rules(config("constexpr uint8_t PIN_X = 0;")), ["pins"])
        self.assertEqual(self.rules(config("constexpr uint8_t PIN_X = 21;")), ["pins"])
        self.assertEqual(self.rules(config("constexpr uint8_t PIN_X = 70;")), ["pins"])

    def test_i2c_pins_are_pinned_to_their_hardware_pins(self):
        self.assertEqual(self.rules(config("constexpr uint8_t PIN_I2C_SDA = 19;")), ["pins"])

    def test_unreadable_value_is_reported_not_skipped(self):
        self.assertEqual(self.rules(config("constexpr uint8_t PIN_X = LED_BUILTIN;")), ["pins"])

    def test_note_must_match_the_pin(self):
        self.assertEqual(self.rules(config("constexpr uint8_t PIN_X = 3; // PE4")), ["notes"])  # pin 3 is PE5
        self.assertEqual(self.rules(config("constexpr uint8_t PIN_X = 3; // PE5, INT4")), ["notes"])
        self.assertEqual(self.rules(config("constexpr uint8_t PIN_X = 6; // PH3, OC4A")), [])
        self.assertEqual(self.rules(config("constexpr uint8_t PIN_X = 6; // PWM6")), [])

    def test_hal_module_must_static_assert_the_pins_it_uses(self):
        files = config("constexpr uint8_t PIN_X = 30;")
        files["src/hal/foo.cpp"] = "void f() { int p = PIN_X; }\n"
        self.assertEqual(self.rules(files), ["asserts"])
        files["src/hal/foo.cpp"] = 'static_assert(PIN_X == 30, "PC7");\nvoid f() { int p = PIN_X; }\n'
        self.assertEqual(self.run_check(files), [])

    def test_pin_named_only_in_a_comment_needs_no_assert(self):
        files = config("constexpr uint8_t PIN_X = 30;")
        files["src/hal/foo.cpp"] = "// uses PIN_X somehow\nvoid f() {}\n"
        self.assertEqual(self.run_check(files), [])

    def test_two_modules_driving_one_timer_conflict(self):
        files = {
            "src/hal/a.cpp": "void a() { TCCR4A = 0; }\n",
            "src/hal/b.cpp": "void b() { OCR4B = 1; }\n",
        }
        findings = self.run_check(files)
        self.assertEqual([f.rule for f in findings], ["owners"])
        self.assertIn("timer4", findings[0].message)

    def test_declared_sharing_passes_only_if_every_module_declares_it(self):
        marker = "// pin-check: shared timer4 - both use compare channels\n"
        files = {
            "src/hal/a.cpp": marker + "void a() { TCCR4A = 0; }\n",
            "src/hal/b.cpp": "void b() { OCR4B = 1; }\n",
        }
        self.assertEqual(self.rules(files), ["owners"])
        files["src/hal/b.cpp"] = marker + files["src/hal/b.cpp"]
        self.assertEqual(self.run_check(files), [])

    def test_header_and_source_of_one_module_are_one_owner(self):
        files = {"src/hal/a.cpp": "void a() { TCCR4A = 0; }\n", "src/hal/a.h": "#define X TCCR4B\n"}
        self.assertEqual(self.run_check(files), [])

    def test_timer0_is_reserved_for_the_arduino_core(self):
        self.assertEqual(self.rules({"src/hal/a.cpp": "void a() { TCCR0A = 0; }\n"}), ["owners"])

    def test_interrupt_vectors_and_enable_bits_count_as_ownership(self):
        files = {
            "src/hal/a.cpp": "ISR(INT4_vect) {}\n",
            "src/hal/b.cpp": "void b() { EIMSK |= (1 << INT4); }\n",
        }
        self.assertEqual(self.rules(files), ["owners"])

    def test_comments_and_strings_are_not_register_use(self):
        files = {
            "src/hal/a.cpp": "void a() { TCCR4A = 0; }\n",
            "src/hal/b.cpp": '// OCR4B = 1;\nconst char *s = "TCCR4B";\n',
        }
        self.assertEqual(self.run_check(files), [])

    def test_main_returns_nonzero_on_findings(self):
        tree = Tree(config("constexpr uint8_t PIN_A = 30;", "constexpr uint8_t PIN_B = 30;"))
        self.addCleanup(tree.close)
        self.assertEqual(pin_map.main(["--root", str(tree.root)]), 1)

    def test_built_in_table_matches_the_arduino_core_when_installed(self):
        header = Path.home() / ".platformio/packages/framework-arduino-avr/variants/mega/pins_arduino.h"
        if not header.exists():
            self.skipTest("Arduino core not installed")
        self.assertEqual(pin_map.verify_core_table(header), [])


if __name__ == "__main__":
    unittest.main()

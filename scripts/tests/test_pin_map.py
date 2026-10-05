import contextlib
import io
import os
import unittest
from pathlib import Path
from unittest import mock

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

    def test_a_marked_alias_is_not_a_second_claim(self):
        files = config(
            "constexpr uint8_t PIN_A = 30;",
            "constexpr uint8_t PIN_B = PIN_A;  // pin-check: alias of PIN_A",
        )
        self.assertEqual(self.run_check(files), [])

    def test_an_unmarked_second_name_is_reported(self):
        files = config("constexpr uint8_t PIN_A = 30;", "constexpr uint8_t PIN_B = PIN_A;")
        findings = self.run_check(files)
        self.assertEqual([f.rule for f in findings], ["pins"])
        self.assertIn("pin-check: alias of PIN_A", findings[0].message)

    def test_an_alias_marker_naming_a_different_pin_is_reported(self):
        files = config(
            "constexpr uint8_t PIN_A = 30;",
            "constexpr uint8_t PIN_C = 31;",
            "constexpr uint8_t PIN_B = PIN_A;  // pin-check: alias of PIN_C",
        )
        findings = self.run_check(files)
        self.assertEqual([f.rule for f in findings], ["pins"])
        self.assertIn("but is defined as PIN_A", findings[0].message)

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

    def test_pin_named_only_inside_a_static_assert_is_not_a_use(self):
        # The assert is evidence about the register code, not a use of the pin
        # by it, so it neither demands nor satisfies an assert of its own.
        files = config("constexpr uint8_t PIN_X = 30;", "constexpr uint8_t PIN_Y = 31;")
        files["src/hal/foo.cpp"] = 'static_assert(PIN_X == 30, "PC7");\nvoid f() { PORTC |= 1 << 7; }\n'
        self.assertEqual(self.run_check(files), [])
        # ...while a pin the register code really does use still needs one.
        files["src/hal/foo.cpp"] = 'static_assert(PIN_X == 30, "PC7");\nvoid f() { int p = PIN_Y; }\n'
        self.assertEqual(self.rules(files), ["asserts"])

    def test_pin_named_only_in_a_comment_needs_no_assert(self):
        files = config("constexpr uint8_t PIN_X = 30;")
        files["src/hal/foo.cpp"] = "// uses PIN_X somehow\nvoid f() {}\n"
        self.assertEqual(self.run_check(files), [])

    def test_hal_port_register_needs_an_assert_on_a_pin_of_that_port(self):
        # PIN_X = 30 is PC7. A driver that pokes PORTC without asserting any
        # pin is invisible to the `asserts` rule: moving the pin in
        # RobotConfig.h would still pass.
        files = config("constexpr uint8_t PIN_X = 30; // PC7")
        files["src/hal/foo.cpp"] = "void f() { PORTC |= 1 << 7; }\n"
        findings = self.run_check(files)
        self.assertEqual([f.rule for f in findings], ["registers"])
        self.assertIn("port C", findings[0].message)
        files["src/hal/foo.cpp"] = 'static_assert(PIN_X == 30, "PC7");\nvoid f() { PORTC |= 1 << 7; }\n'
        self.assertEqual(self.run_check(files), [])

    def test_an_assert_on_a_pin_of_another_port_does_not_cover_the_register(self):
        files = config("constexpr uint8_t PIN_X = 30; // PC7")
        files["src/hal/foo.cpp"] = 'static_assert(PIN_X == 30, "PC7");\nvoid f() { DDRF = 0; }\n'
        findings = self.run_check(files)
        self.assertEqual([f.rule for f in findings], ["registers"])
        self.assertIn("port F", findings[0].message)
        self.assertIn("PIN_X", findings[0].message)

    def test_every_port_a_module_touches_needs_its_own_pin(self):
        files = config("constexpr uint8_t PIN_X = 30; // PC7", "constexpr uint8_t PIN_IR[2] = {A0, A1};")
        both = 'static_assert(PIN_X == 30, "PC7");\nstatic_assert(PIN_IR[0] == A0, "PF0");\n'
        files["src/hal/foo.cpp"] = both + "void f() { PORTC |= 1; DDRF = 0; }\n"
        self.assertEqual(self.run_check(files), [])  # an array's elements count for their port
        files["src/hal/foo.cpp"] = 'static_assert(PIN_X == 30, "PC7");\nvoid f() { PORTC |= 1; DDRF = 0; }\n'
        findings = self.run_check(files)
        self.assertEqual([f.rule for f in findings], ["registers"])
        self.assertIn("port F", findings[0].message)

    def test_a_header_and_source_share_the_module_assert(self):
        files = config("constexpr uint8_t PIN_X = 30; // PC7")
        files["src/hal/foo.h"] = 'static_assert(PIN_X == 30, "PC7");\n'
        files["src/hal/foo.cpp"] = "void f() { PINC; }\n"
        self.assertEqual(self.run_check(files), [])

    def test_register_look_alikes_comments_and_non_gpio_registers_are_not_port_access(self):
        files = config("constexpr uint8_t PIN_X = 30; // PC7")
        files["src/hal/foo.cpp"] = (
            "// PORTF is mentioned here only\n"
            'const char *s = "DDRF";\n'
            "void f() { int a = PIND2 + PINB0; ADMUX = 0; ADCSRA = 0; TCCR1A = 0; }\n"
        )
        self.assertEqual(self.run_check(files), [])

    def test_a_pin_bit_must_be_one_the_module_asserts(self):
        # The case a port-only rule missed: the code uses PC1, the only assert
        # is on PC7. Both are on PORTC, so the port matches and the bit is wrong.
        files = config("constexpr uint8_t PIN_A = 30; // PC7", "constexpr uint8_t PIN_B = 36; // PC1")
        files["src/hal/foo.cpp"] = 'static_assert(PIN_A == 30, "PC7");\nvoid f() { PORTC |= (1 << PC1); }\n'
        findings = self.run_check(files)
        self.assertEqual([f.rule for f in findings], ["registers"])
        self.assertIn("uses PC1", findings[0].message)
        self.assertIn("PIN_A", findings[0].message)
        files["src/hal/foo.cpp"] = 'static_assert(PIN_B == 36, "PC1");\nvoid f() { PORTC |= (1 << PC1); }\n'
        self.assertEqual(self.run_check(files), [])

    def test_a_pin_bit_with_no_assert_at_all_is_reported_once(self):
        files = config("constexpr uint8_t PIN_A = 30; // PC7")
        files["src/hal/foo.cpp"] = "void f() { DDRC |= (1 << PC7); PORTC |= (1 << PC7); }\n"
        findings = self.run_check(files)
        self.assertEqual([f.rule for f in findings], ["registers"])  # the bit, not the bit and the port
        self.assertIn("no static_assert in it names a PIN_*", findings[0].message)

    def test_every_element_of_an_asserted_pin_array_covers_its_bit(self):
        files = config("constexpr uint8_t PIN_IR[2] = {A0, A1};")
        assert_ = 'static_assert(PIN_IR[0] == A0, "PF0");\n'
        files["src/hal/foo.cpp"] = assert_ + "void f() { DDRF |= (1 << PF1); }\n"
        self.assertEqual(self.run_check(files), [])
        files["src/hal/foo.cpp"] = assert_ + "void f() { DDRF |= (1 << PF2); }\n"
        findings = self.run_check(files)
        self.assertEqual([f.rule for f in findings], ["registers"])
        self.assertIn("uses PF2", findings[0].message)

    def test_a_pin_bit_in_a_comment_or_an_assert_message_is_not_a_use(self):
        files = config("constexpr uint8_t PIN_A = 30; // PC7")
        files["src/hal/foo.cpp"] = '// PC1 is the emitter\nstatic_assert(PIN_A == 30, "PC1 is wrong");\nvoid f() { PORTC |= 1 << 7; }\n'
        self.assertEqual(self.run_check(files), [])

    def test_a_name_that_cannot_be_placed_on_a_port_skips_the_comparison(self):
        # An unresolved asserted name (an alias, a typo) makes the port check
        # a guess; the unknown-name case is reported by other rules, not here.
        files = config("constexpr uint8_t PIN_X = 30; // PC7")
        files["src/hal/foo.cpp"] = 'static_assert(PIN_GHOST == 1, "?");\nvoid f() { PORTF = 0; }\n'
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

    # Regression: a trailing comment on the first line of a multi-line
    # declaration used to swallow the rest of the file into one statement, so
    # every later pin went unparsed and duplicates were never reported.
    def test_trailing_comment_on_a_multiline_declaration_does_not_hide_later_pins(self):
        files = config(
            "constexpr uint8_t PIN_IR[8] = {  // left to right",
            "    A0, A1, A2, A3, A4, A5, A6, A7};",
            "constexpr uint8_t PIN_LED = 26;",
            "constexpr uint8_t PIN_DUP = 26;",
        )
        findings = self.run_check(files)
        self.assertEqual([f.rule for f in findings], ["pins"])
        self.assertIn("claimed twice", findings[0].message)

    def test_multiline_declaration_with_comments_on_every_line_parses(self):
        files = config(
            "constexpr uint8_t PIN_IR[3] = {  // left to right",
            "    A0,  // outer left",
            "    A1,  // centre",
            "    A2}; // outer right",
            "constexpr uint8_t PIN_LED = 26;",
        )
        self.assertEqual(self.run_check(files), [])

    def test_main_returns_nonzero_on_findings(self):
        tree = Tree(config("constexpr uint8_t PIN_A = 30;", "constexpr uint8_t PIN_B = 30;"))
        self.addCleanup(tree.close)
        # Not on Actions: main() would emit a real ::error annotation for the fixture.
        with mock.patch.dict(os.environ, {"GITHUB_ACTIONS": ""}), contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(pin_map.main(["--root", str(tree.root)]), 1)

    def test_built_in_table_matches_the_arduino_core_when_installed(self):
        header = Path.home() / ".platformio/packages/framework-arduino-avr/variants/mega/pins_arduino.h"
        if not header.exists():
            self.skipTest("Arduino core not installed")
        self.assertEqual(pin_map.verify_core_table(header), [])


if __name__ == "__main__":
    unittest.main()

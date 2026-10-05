import unittest

from support import load_script

isr = load_script("check-isr-atomicity.py")

HEAD = "#include <avr/interrupt.h>\n#include <util/atomic.h>\n"


def lines(source, path="src/hal/x.cpp"):
    """Line numbers of findings for a source snippet."""
    return [f.line for f in isr.check_file(path, HEAD + source)]


class IsrAtomicityTest(unittest.TestCase):
    def test_access_in_isr_and_atomic_block_is_fine(self):
        src = """
static volatile int32_t g = 0;
ISR(INT4_vect) { g = g + 1; }
int32_t read() {
    int32_t v;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { v = g; }
    return v;
}
"""
        self.assertEqual(lines(src), [])

    def test_unguarded_read_in_main_context_is_reported(self):
        src = """
static volatile int32_t g = 0;
ISR(INT4_vect) { g = g + 1; }
int32_t read() { return g; }
"""
        self.assertEqual(lines(src), [6])

    def test_unguarded_write_is_reported_too(self):
        src = """
static volatile uint16_t g = 0;
ISR(INT4_vect) { g++; }
void reset() { g = 0; }
"""
        self.assertEqual(lines(src), [6])

    def test_nested_blocks_inside_atomic_are_fine(self):
        src = """
static volatile uint16_t g = 0;
ISR(INT4_vect) { g++; }
void f(bool b) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { if (b) { for (;;) { g = 0; break; } } }
}
"""
        self.assertEqual(lines(src), [])

    def test_nonatomic_block_inside_atomic_is_not_protected(self):
        src = """
static volatile uint16_t g = 0;
ISR(INT4_vect) { g++; }
void f() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        NONATOMIC_BLOCK(NONATOMIC_RESTORESTATE) { g = 0; }
    }
}
"""
        self.assertEqual(lines(src), [8])

    def test_static_helper_called_only_from_isrs_is_fine(self):
        src = """
static volatile int32_t g = 0;
static inline void step() { g = g + 1; }
static void deeper() { step(); }
ISR(INT4_vect) { deeper(); }
ISR(INT5_vect) { step(); }
"""
        self.assertEqual(lines(src), [])

    def test_helper_also_called_from_unprotected_code_is_reported(self):
        src = """
static volatile int32_t g = 0;
static inline void step() { g = g + 1; }
ISR(INT4_vect) { step(); }
void poke() { step(); }
"""
        self.assertEqual(lines(src), [5])

    def test_helper_called_from_atomic_block_is_fine(self):
        src = """
static volatile int32_t g = 0;
static void zero() { g = 0; }
ISR(INT4_vect) { g++; }
void f() { ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { zero(); } }
"""
        self.assertEqual(lines(src), [])

    def test_non_static_helper_could_be_called_from_elsewhere(self):
        src = """
static volatile int32_t g = 0;
void step() { g = g + 1; }
ISR(INT4_vect) { step(); }
"""
        self.assertEqual(lines(src), [5])

    def test_extern_and_non_static_globals_are_rejected(self):
        self.assertEqual(lines("extern volatile uint32_t ticks;\n"), [3])
        self.assertEqual(lines("volatile uint32_t ticks = 0;\n"), [3])
        self.assertEqual(lines("namespace { volatile uint32_t ticks = 0; }\nISR(INT4_vect) { ticks++; }\n"), [])

    def test_single_byte_volatile_is_not_a_tearing_risk(self):
        src = """
static volatile uint8_t flag = 0;
static volatile bool done = false;
ISR(INT4_vect) { flag = 1; done = true; }
uint8_t f() { return flag; }
"""
        self.assertEqual(lines(src), [])

    def test_volatile_no_isr_touches_is_left_alone(self):
        self.assertEqual(lines("static volatile uint16_t spin = 0;\nvoid f() { spin = 1; }\n"), [])

    def test_pointer_to_volatile_register_is_ignored(self):
        src = "static volatile uint16_t *const reg = (volatile uint16_t *)0x84;\nISR(INT4_vect) { *reg = 1; }\nuint16_t f() { return *reg; }\n"
        self.assertEqual(lines(src), [])

    def test_volatile_array_elements_are_covered(self):
        src = """
static volatile uint16_t buf[4];
ISR(INT4_vect) { buf[0] = 1; }
uint16_t f() { return buf[0]; }
"""
        self.assertEqual(lines(src), [6])

    def test_reviewed_exception_marker(self):
        src = """
static volatile uint16_t g = 0;
ISR(INT4_vect) { g++; }
uint16_t f() {
    return g; // isr-safe: called with interrupts already off
}
"""
        self.assertEqual(lines(src), [])
        src = src.replace("// isr-safe: called with interrupts already off", "")
        self.assertEqual(lines(src), [7])

    def test_mentions_in_comments_and_strings_are_ignored(self):
        src = """
static volatile uint16_t g = 0;
ISR(INT4_vect) { g++; }
// g is read here
const char *s = "g";
"""
        self.assertEqual(lines(src), [])

    def test_the_real_encoder_driver_pattern(self):
        src = """
static volatile int32_t g_countL = 0;
static volatile uint8_t g_stateL = 0;
static inline void stepLeft() { const uint8_t s = 1; g_countL = g_countL + s; g_stateL = s; }
ISR(INT4_vect) { stepLeft(); }
ISR(INT5_vect) { stepLeft(); }
void encodersInit() { ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { g_countL = 0; g_stateL = 0; } }
void encodersRead(int32_t &left) { ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { left = g_countL; } }
"""
        self.assertEqual(lines(src), [])


if __name__ == "__main__":
    unittest.main()

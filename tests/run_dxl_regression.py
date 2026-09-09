"""Compile the production DXL implementation with a deterministic board double."""

from pathlib import Path
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[2]
TEST = ROOT / "firmware/tests/dxl_regression_test.c"

SHIM = r'''
#include <stdint.h>
static inline uint32_t k_cycle_get_32(void) { return 0u; }
static inline uint32_t k_uptime_get_32(void) { return 0u; }
static inline uint32_t k_us_to_cyc_ceil32(uint32_t value) { return value; }
static inline uint32_t k_cyc_to_us_floor32(uint32_t value) { return value; }
static inline uint32_t k_cyc_to_us_ceil32(uint32_t value) { return value; }
static inline void k_busy_wait(uint32_t value) { (void)value; }
static inline void k_sleep(uint32_t value) { (void)value; }
#define K_MSEC(value) (value)
'''


def main():
    with tempfile.TemporaryDirectory(prefix="head-dxl-regression-") as directory:
        include = Path(directory)
        (include / "zephyr").mkdir()
        (include / "zephyr/kernel.h").write_text(SHIM)
        executable = Path(directory) / "dxl_regression_test"
        subprocess.run([
            "cc", "-std=c99", "-Wall", "-Wextra", "-Wshadow",
            "-Werror", "-fsanitize=undefined", "-I" + str(include),
            "-I" + str(ROOT / "firmware/include"), str(TEST), "-o", str(executable),
        ], check=True)
        subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()

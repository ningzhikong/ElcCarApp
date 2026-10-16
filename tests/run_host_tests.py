"""Run real control math and experiment C logic with mocked HAL; never opens a port."""
import argparse
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
SCENARIOS = ('math', 'validation', 'cancel_pending', 'complete', 'heartbeat',
             'direction', 'stall', 'stop_key', 'uart_fault', 'frame', 'overflow', 'slow_motion', 'arm_expiry', 'pending_expiry')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gcc', default=r'D:\Dev-Cpp\MinGW64\bin\gcc.exe')
    args = parser.parse_args()
    output = ROOT / '.build/tests'
    output.mkdir(parents=True, exist_ok=True)
    executable = output / 'firmware_tests.exe'
    subprocess.run([args.gcc, '-std=c99', '-Wall', '-Wextra', '-Werror', '-pedantic',
                    '-DCAR_HOST_TEST', f'-I{ROOT / "tests"}', f'-I{ROOT / "Core/Inc"}',
                    str(ROOT / 'Core/Src/control_math.c'), str(ROOT / 'Core/Src/autotune.c'),
                    str(ROOT / 'tests/test_firmware.c'), '-lm', '-o', str(executable)], check=True)
    for scenario in SCENARIOS:
        subprocess.run([str(executable), scenario], check=True)
    engine = output / 'control_tests.exe'
    subprocess.run([args.gcc, '-std=c99', '-Wall', '-Wextra', '-Werror', '-pedantic',
                    '-DCAR_HOST_TEST', f'-I{ROOT / "tests"}', f'-I{ROOT / "Core/Inc"}',
                    str(ROOT / 'Core/Src/control_math.c'), str(ROOT / 'Core/Src/control.c'),
                    str(ROOT / 'tests/test_control_engine.c'), '-lm', '-o', str(engine)], check=True)
    engine_scenarios = ('sampling', 'stale', 'timing', 'stop', 'test_zero', 'cycle_wrap', 'invalid_target', 'sign_change', 'invalid_gains', 'fractional_limit')
    for scenario in engine_scenarios:
        subprocess.run([str(engine), scenario], check=True)
    print(f'{len(SCENARIOS) + len(engine_scenarios)} C host scenarios passed; hardware timing/IO not exercised.')
    subprocess.run([sys.executable, '-m', 'unittest', 'discover', '-s', str(ROOT / 'tests'),
                    '-p', 'test_*.py', '-v'], check=True)

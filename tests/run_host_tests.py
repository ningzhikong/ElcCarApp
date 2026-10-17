"""Run real control math and experiment C logic with mocked HAL; never opens a port."""
import argparse
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
SCENARIOS = ('math', 'validation', 'cancel_pending', 'complete', 'heartbeat',
             'direction', 'stall', 'stop_key', 'uart_fault', 'frame', 'overflow', 'slow_motion', 'arm_expiry', 'pending_expiry',
             'low_stall20', 'low_stall50', 'low_stall79', 'no_motion', 'sparse_motion',
             'jog_validation', 'jog_cancel', 'jog_expiry', 'jog_complete', 'jog_key',
             'jog_uart', 'jog_speed', 'jog_control_fault', 'jog_stop', 'rpm_profile', 'rpm_validation', 'rpm_hold')


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
                    str(ROOT / 'Core/Src/oled.c'),
                    str(ROOT / 'tests/test_control_engine.c'), '-lm', '-o', str(engine)], check=True)
    engine_scenarios = ('sampling', 'stale', 'timing', 'stop', 'test_zero', 'cycle_wrap', 'invalid_target',
                        'sign_change', 'invalid_gains', 'fractional_limit',
                        'oled_busy', 'oled_race', 'oled_frame', 'oled_error', 'quiet', 'motor_polarity',
                        'jog_output', 'jog_invalid')
    for scenario in engine_scenarios:
        subprocess.run([str(engine), scenario], check=True)
    tuned = output / 'tuned_profile_tests.exe'
    subprocess.run([args.gcc, '-std=c99', '-Wall', '-Wextra', '-Werror', '-pedantic',
                    '-DCAR_HOST_TEST', '-DCAR_TUNED_CONFIG_HEADER="tuned_profile_fixture.h"',
                    f'-I{ROOT / "tests"}', f'-I{ROOT / "Core/Inc"}',
                    str(ROOT / 'Core/Src/control_math.c'), str(ROOT / 'Core/Src/control.c'),
                    str(ROOT / 'Core/Src/oled.c'), str(ROOT / 'tests/test_control_engine.c'),
                    '-lm', '-o', str(tuned)], check=True)
    subprocess.run([str(tuned), 'tuned_profile'], check=True)
    route = output / 'fsm_tests.exe'
    subprocess.run([args.gcc, '-std=c99', '-Wall', '-Wextra', '-Werror', '-pedantic',
                    '-DCAR_HOST_TEST', f'-I{ROOT / "tests"}', f'-I{ROOT / "Core/Inc"}',
                    str(ROOT / 'Core/Src/control_math.c'), str(ROOT / 'Core/Src/fsm.c'),
                    str(ROOT / 'tests/test_fsm.c'), '-lm', '-o', str(route)], check=True)
    route_scenarios = ('vision_default', 'vision_noise', 'marker', 'initial_marker', 'half_counts', 'lap', 'two_laps', 'short_lap', 'counter_wrap', 'acbd', 'line',
                       'imu_init', 'imu_calibration', 'imu_read', 'imu_gap', 'adc', 'lost_line',
                       'timeout0', 'timeout1', 'timeout2', 'timeout3', 'control_fault', 'ack_motion', 'manual_stop')
    for scenario in route_scenarios:
        subprocess.run([str(route), scenario], check=True)
    mpu = output / 'mpu_tests.exe'
    subprocess.run([args.gcc, '-std=c99', '-Wall', '-Wextra', '-Werror', '-pedantic',
                    '-DCAR_HOST_TEST', f'-I{ROOT / "tests"}', f'-I{ROOT / "Core/Inc"}',
                    str(ROOT / 'Core/Src/mpu6050.c'), str(ROOT / 'tests/test_mpu.c'),
                    '-lm', '-o', str(mpu)], check=True)
    mpu_scenarios = ('init', 'valid', 'boundary', 'partial', 'missing', 'moving', 'vibration', 'gravity', 'too_short')
    for scenario in mpu_scenarios:
        subprocess.run([str(mpu), scenario], check=True)
    print(f'{len(SCENARIOS) + len(engine_scenarios) + len(route_scenarios) + len(mpu_scenarios) + 1} C host scenarios passed; hardware timing/IO not exercised.')
    subprocess.run([sys.executable, '-m', 'unittest', 'discover', '-s', str(ROOT / 'tests'),
                    '-p', 'test_*.py', '-v'], check=True)

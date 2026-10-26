"""Run real control math and experiment C logic with mocked HAL; never opens a port."""
import argparse
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
SCENARIOS = ('math', 'validation', 'cancel_pending', 'complete', 'heartbeat',
             'direction', 'stall', 'stop_key', 'uart_fault', 'frame', 'overflow', 'slow_motion', 'arm_expiry', 'pending_expiry',
             'low_stall20', 'low_stall50', 'low_stall79', 'no_motion', 'sparse_motion',
             'jog_validation', 'jog_cancel', 'jog_expiry', 'jog_complete', 'jog_raw', 'jog_key',
             'jog_uart', 'jog_speed', 'jog_control_fault', 'jog_stop', 'rpm_profile', 'rpm_validation', 'rpm_hold', 'rpm_hold30', 'raw_trace',
             'rpm_hold90', 'rpm_hold100', 'rpm_hold130', 'raw_period_overflow', 'hold15', 'hold15_heartbeat', 'hold15_stop', 'hold15_validation',
             'drive_profile', 'drive_step', 'drive_validation', 'drive_cancel')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gcc', default=r'D:\Dev-Cpp\MinGW64\bin\gcc.exe')
    args = parser.parse_args()
    output = ROOT / '.build/tests'
    output.mkdir(parents=True, exist_ok=True)
    heading_math = output / 'heading_tests.exe'
    subprocess.run([args.gcc, '-std=c99', '-Wall', '-Wextra', '-Werror', '-pedantic',
                    f'-I{ROOT / "Core/Inc"}', str(ROOT / 'Core/Src/heading_control.c'),
                    str(ROOT / 'tests/test_heading_control.c'), '-lm', '-o', str(heading_math)], check=True)
    heading_scenarios = ('direction', 'clamp_slew', 'damping', 'invalid', 'gap', 'wrap', 'reset')
    for scenario in heading_scenarios:
        subprocess.run([str(heading_math), scenario], check=True)
    line_math = output / 'line_follow_tests.exe'
    subprocess.run([args.gcc, '-std=c99', '-Wall', '-Wextra', '-Werror', '-pedantic',
                    f'-I{ROOT / "Core/Inc"}', str(ROOT / 'Core/Src/line_follow.c'),
                    str(ROOT / 'tests/test_line_follow.c'), '-lm', '-o', str(line_math)], check=True)
    line_math_scenarios = ('classification', 'direction', 'speed_plan', 'returning_pd', 'invalid', 'time_and_ceiling')
    for scenario in line_math_scenarios:
        subprocess.run([str(line_math), scenario], check=True)
    ground = output / 'ground_tests.exe'
    subprocess.run([args.gcc, '-std=c99', '-Wall', '-Wextra', '-Werror', '-pedantic',
                    '-DCAR_HOST_TEST', '-DCAR_GROUND_TEST', f'-I{ROOT / "tests"}', f'-I{ROOT / "Core/Inc"}',
                    str(ROOT / 'Core/Src/control_math.c'), str(ROOT / 'Core/Src/control.c'),
                    str(ROOT / 'Core/Src/ground_test.c'), str(ROOT / 'tests/test_ground.c'),
                    str(ROOT / 'Core/Src/heading_control.c'),
                    str(ROOT / 'Core/Src/line_follow.c'),
                    '-lm', '-o', str(ground)], check=True)
    ground_scenarios = ('filter', 'complete', 'wrap', 'boot_held', 'short_press', 'cancel', 'countdown_start',
                        'heartbeat', 'stall', 'direction', 'timing', 'stop', 'coasting')
    for scenario in ground_scenarios:
        subprocess.run([str(ground), scenario], check=True)
    ground_extra = ('ramp', 'imu_init_error', 'imu_calibration_error', 'imu_read_error',
                    'imu_stale', 'imu_nonfinite', 'heading_direction', 'heading_diverge', 'heading_reference', 'heading_scaling')
    for scenario in ground_extra:
        subprocess.run([str(ground), scenario], check=True)
    line_scenarios = ground_scenarios + ('white_start', 'black_start', 'line_lost', 'line_wide', 'line_stale', 'steering', 'countdown_line_lost', 'noise', 'speed_plan')
    line_count = 0
    for direction in (1, -1, 0):
        line = output / f'ground_line_{direction}_tests.exe'
        subprocess.run([args.gcc, '-std=c99', '-Wall', '-Wextra', '-Werror', '-pedantic',
                        '-DCAR_HOST_TEST', '-DCAR_GROUND_TEST', '-DCAR_GROUND_LINE', f'-DGROUND_IR_DIRECTION={direction}',
                        f'-I{ROOT / "tests"}', f'-I{ROOT / "Core/Inc"}',
                        str(ROOT / 'Core/Src/control_math.c'), str(ROOT / 'Core/Src/control.c'),
                        str(ROOT / 'Core/Src/ground_test.c'), str(ROOT / 'tests/test_ground.c'),
                        str(ROOT / 'Core/Src/line_follow.c'),
                        '-lm', '-o', str(line)], check=True)
        for scenario in (line_scenarios if direction else ('mapping_blocked',)):
            subprocess.run([str(line), scenario], check=True)
            line_count += 1
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
                        'sign_change', 'invalid_gains', 'fractional_limit', 'bench_pwm_boundary',
                        'oled_busy', 'oled_race', 'oled_frame', 'oled_error', 'oled_busy_resume', 'oled_init_busy', 'oled_init_retry', 'quiet', 'motor_polarity',
                        'jog_output', 'jog_invalid', 'rpm_feedback', 'rpm_feedback_guard', 'rpm_feedback_slow',
                        'drive_rpm', 'drive_windows', 'drive_invalid', 'drive_unconfigured', 'drive_reconfigure',
                        'drive_stall', 'drive_direction', 'drive_speed', 'drive_stale', 'drive_timing', 'drive_sparse')
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
                    str(ROOT / 'Core/Src/control_math.c'), str(ROOT / 'Core/Src/fsm.c'), str(ROOT / 'Core/Src/line_follow.c'),
                    str(ROOT / 'tests/test_fsm.c'), '-lm', '-o', str(route)], check=True)
    route_scenarios = ('vision_default', 'vision_noise', 'marker', 'initial_marker', 'half_counts', 'lap', 'two_laps', 'short_lap', 'counter_wrap', 'acbd', 'line',
                       'imu_init', 'imu_calibration', 'imu_read', 'imu_gap', 'lost_line',
                       'timeout0', 'timeout1', 'timeout2', 'timeout3', 'control_fault', 'ack_motion', 'manual_stop', 'rpm_units', 'ir_idle', 'ir_running',
                       'digital_polarity', 'digital_centroid', 'centered_speed', 'marker_noise')
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
    imu_diag = output / 'imu_diagnostic_tests.exe'
    subprocess.run([args.gcc, '-std=c99', '-Wall', '-Wextra', '-Werror', '-pedantic',
                    '-DCAR_HOST_TEST', f'-I{ROOT / "tests"}', f'-I{ROOT / "Core/Inc"}',
                    str(ROOT / 'Core/Src/imu_diagnostic.c'), str(ROOT / 'tests/test_imu_diagnostic.c'),
                    '-lm', '-o', str(imu_diag)], check=True)
    imu_diag_scenarios = ('stationary', 'left_y', 'left_x', 'negative', 'wrap', 'gap',
                          'read_error', 'nonfinite', 'init_error', 'calibration_error')
    for scenario in imu_diag_scenarios:
        subprocess.run([str(imu_diag), scenario], check=True)
    print(f'{len(SCENARIOS) + len(engine_scenarios) + len(route_scenarios) + len(mpu_scenarios) + len(ground_scenarios) + line_count + len(line_math_scenarios) + len(imu_diag_scenarios) + len(ground_extra) + len(heading_scenarios) + 1} C host scenarios passed; hardware timing/IO not exercised.')
    subprocess.run([sys.executable, '-m', 'unittest', 'discover', '-s', str(ROOT / 'tests'),
                    '-p', 'test_*.py', '-v'], check=True)

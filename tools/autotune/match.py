"""One fixed-gain equal-RPM bench experiment, using manual wheel CPR references."""
import argparse
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import re

import bench
import raw_trace


def hold_target(t, rpm):
    return rpm if 500 <= t < 4500 else 0.


def evaluate(rows, rpm, limit, cpr, hold=False):
    if (not math.isfinite(rpm) or not 1 <= rpm <= bench.RPM_BENCH_MAX or not 20 <= limit <= bench.RPM_BENCH_PWM_MAX
            or int(limit) != limit or len(cpr) != 2
            or any(type(c) is not int or not 0 < c <= 1000000 for c in cpr)
            or any(not 1 <= rpm * c / 6000 <= bench.RPM_BENCH_COUNTS_MAX for c in cpr)):
        raise ValueError('Invalid RPM experiment calibration or envelope')
    # Preserve all original raw/time/PWM checks; scale each wheel's target
    # and filtered speed to an equivalent common reference of 6000 CPR.
    # This produces RPM numerically, without claiming an autotune score.
    normalized = []
    profile = hold_target if hold else bench.target_at
    for row in rows:
        if len(row) != len(bench.FIELDS):
            raise ValueError('Malformed RPM trace')
        if not all(math.isfinite(v) for v in row):
            raise ValueError('Non-finite RPM trace')
        if max(abs(row[5]), abs(row[6])) > 100:
            raise ValueError('Encoder speed exceeds counts/10ms envelope')
        data = list(row)
        for target_i, speed_i, count in ((3, 5, cpr[0]), (4, 6, cpr[1])):
            expected = profile(row[0], rpm) * count / 6000
            if abs(row[target_i] - expected) > .00015:
                raise ValueError('RPM target conversion does not match calibration')
            data[target_i] = profile(row[0], rpm)
            data[speed_i] = row[speed_i] * 6000 / count
        normalized.append(data)
    bench.validate_trace(normalized, rpm, limit, target_profile=profile, peak_max=bench.RPM_BENCH_MAX,
                         pwm_max=bench.RPM_BENCH_PWM_MAX, speed_max=100 * 6000 / min(cpr))
    start, end = (2000, 4500) if hold else (2200, 3000)
    plateau = [r for r in normalized if start <= r[0] < end]
    means = [sum(r[i] for r in plateau) / len(plateau) for i in (5, 6)]
    mismatch = sum(abs(r[5] - r[6]) for r in plateau) / len(plateau)
    target_errors = [sum(abs(r[i] - rpm) for r in plateau) / len(plateau) for i in (5, 6)]
    matched = mismatch <= .05 * rpm and max(target_errors) <= .1 * rpm
    deviations = [(sum((r[i] - mean) ** 2 for r in plateau) / len(plateau)) ** .5
                  for i, mean in zip((5, 6), means)]
    smooth = max(deviations) <= .1 * rpm
    return {'trace_valid': True, 'matched_in_final_plateau': matched,
            'smooth_in_final_plateau': smooth, 'matched_and_smooth': matched and smooth,
            'requested_rpm': rpm, 'wheel_cpr_reference': list(cpr),
            'measurement_basis': 'manual_cpr_reference; not independent tachometer verification',
            'profile': 'hold' if hold else 'step', 'plateau_ms': [start, end],
            'mean_rpm_left': means[0], 'mean_rpm_right': means[1],
            'rpm_standard_deviation': deviations,
            'mean_absolute_rpm_difference': mismatch,
            'mean_absolute_target_error_rpm': target_errors,
            'criteria': {'mean_absolute_difference_fraction': .05, 'each_target_error_fraction': .1,
                         'each_rpm_standard_deviation_fraction': .1}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--build-id', required=True)
    parser.add_argument('--rpm', type=float, default=20., help='Wheel RPM; raised-wheel short bench maximum 130, normal drive maximum 20')
    parser.add_argument('--drive', action='store_true', help='Exercise fixed normal-drive RPM configuration/path')
    parser.add_argument('--raw-10ms', action='store_true', help='Require and export complete 100Hz encoder/PWM trace after stopping')
    parser.add_argument('--hold', action=argparse.BooleanOptionalAction, default=True,
                        help='Hold RPM for 4s (default); --no-hold selects speed steps')
    parser.add_argument('--cpr-left', type=int, default=1080)
    parser.add_argument('--cpr-right', type=int, default=1320)
    parser.add_argument('--sign-left', type=int, choices=(-1, 1), default=1)
    parser.add_argument('--sign-right', type=int, choices=(-1, 1), default=1)
    for side in ('left', 'right'):
        parser.add_argument(f'--kp-{side}', type=float, default=2.)
        parser.add_argument(f'--ki-{side}', type=float, default=1.)
    parser.add_argument('--pwm-limit', type=int, default=150)
    parser.add_argument('--output', type=Path, default=Path('.build/hardware_checks'))
    args = parser.parse_args()
    args.peak = args.rpm
    args.require_identified_firmware = True
    if not re.fullmatch('[0-9a-f]{16}', args.build_id):
        parser.error('Expected a 16-character firmware build ID')
    try:
        bench.validate_parameters(args, peak_max=bench.RPM_BENCH_MAX, pwm_max=bench.RPM_BENCH_PWM_MAX)
        if args.drive and (args.peak > 20 or args.pwm_limit != 150 or
                           [args.kp_left, args.ki_left, args.kp_right, args.ki_right] != [2., 1., 2., 1.] or
                           (args.sign_left, args.sign_right) != (1, 1)):
            raise ValueError('Normal drive check must use compiled-in configuration')
        if any(not 0 < c <= 1000000 or not 1 <= args.peak * c / 6000 <= bench.RPM_BENCH_COUNTS_MAX
               for c in (args.cpr_left, args.cpr_right)):
            raise ValueError('RPM conversion exceeds encoder bench bounds')
    except ValueError as exc:
        parser.error(str(exc))
    folder = args.output / ('match_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(parents=True, exist_ok=False)
    result = {'source': 'fixed_gain_rpm_hold_check' if args.hold else 'fixed_gain_rpm_check',
              'rpm_controller': 'ELC_RPM_PI_ADAPTIVE_V1',
              'profile': 'hold' if args.hold else 'step', 'build_id': args.build_id,
              'rpm': args.peak, 'pwm_limit': args.pwm_limit,
              'gains': [args.kp_left, args.ki_left, args.kp_right, args.ki_right],
              'cpr': [args.cpr_left, args.cpr_right]}
    if args.drive:
        result.update(source='normal_drive_rpm_check', drive_controller='ELC_DRIVE_RPM_V1')
    (folder / 'request.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    board = None
    try:
        board = bench.Board(args.port, folder)
        if args.raw_10ms:
            board.send('RAWINFO')
            if board.read() != raw_trace.CAPABILITY:
                raise RuntimeError('Firmware lacks 10ms capture; refusing to ARM')
        board.firmware_info = ('ELC_INC_PI_V1', args.build_id)
        rows = board.run(args, wheel_cpr=(args.cpr_left, args.cpr_right), hold=args.hold, drive=args.drive)
        with (folder / 'trace.csv').open('w', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            writer.writerow(bench.FIELDS)
            writer.writerows(rows)
        if args.raw_10ms:
            raw = raw_trace.capture(board, args.peak, hold=args.hold, drive=args.drive)
            with (folder / 'raw_10ms.csv').open('w', newline='', encoding='utf-8') as stream:
                writer = csv.writer(stream)
                writer.writerow(raw_trace.FIELDS)
                writer.writerows(raw)
            result['raw_analysis'] = raw_trace.analyze(raw, rows, args.peak, args.pwm_limit,
                                                     (args.cpr_left, args.cpr_right), hold=args.hold)
        result['evaluation'] = evaluate(rows, args.peak, args.pwm_limit, (args.cpr_left, args.cpr_right), hold=args.hold)
        result['outcome'] = 'completed'
    except BaseException as exc:
        result.update(outcome='stopped_or_failed', error=str(exc))
        if board is not None:
            try:
                status = board.status()
                result['failure_status'] = status
                if status[1] in (2, 3, 4) and 0 <= status[3] <= 112:
                    board.send('DUMP')
                    lines = [board.read()]
                    lines += [board.read() for _ in range(status[3])]
                    lines.append(board.read())
                    (folder / 'failure_dump.txt').write_text('\n'.join(lines), encoding='utf-8')
            except BaseException as diagnostic_error:
                result['diagnostic_error'] = str(diagnostic_error)
    finally:
        if board is not None:
            try:
                board.wait_stopped()
                result['stopped_and_quiet'] = True
            except BaseException as exc:
                result['stop_error'] = str(exc)
            board.close()
        (folder / 'result.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps(dict(result, folder=str(folder)), indent=2))
    return 0 if (result.get('outcome') == 'completed' and result.get('stopped_and_quiet')
                 and result.get('evaluation', {}).get('matched_and_smooth')) else 1


if __name__ == '__main__':
    raise SystemExit(main())

"""Bounded RPM PI search with local proposals and repeated dual-speed checks."""
import argparse
import copy
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import re

import bench
import match
import tune

CONTROLLER = 'ELC_RPM_PI_ADAPTIVE_V1'


def evaluate(rows, rpm, limit, cpr):
    quality = match.evaluate(rows, rpm, limit, cpr, hold=True)
    wheels = {}
    for side, index, count, deviation, error in zip(
            ('left', 'right'), (5, 6), cpr, quality['rpm_standard_deviation'],
            quality['mean_absolute_target_error_rpm']):
        # Include acceleration and settling, so smoothing alone cannot win.
        active = [r for r in rows if 500 <= r[0] < 4500]
        tracking = sum(abs(r[index] * 6000 / count - rpm) for r in active) / (len(active) * rpm)
        stop = [r for r in rows if 4500 <= r[0]]
        residual = sum(abs(r[index] * 6000 / count) for r in stop) / (len(stop) * rpm)
        score = (tracking + error / rpm + deviation / rpm + residual
                 + quality['mean_absolute_rpm_difference'] / rpm)
        if not quality['matched_and_smooth']:
            score += 1.
        wheels[side] = {'score': score, 'normalized_tracking_error': tracking,
                        'normalized_stop_residual': residual}
    return {'score': max(w['score'] for w in wheels.values()), 'wheels': wheels,
            'quality': quality, 'score_version': 'rpm_hold_multispeed_v1'}


def local_candidate(context, folder, round_number):
    gains = list(context['best_gains'])
    # One coordinate per round, never an output/target increase. This small
    # neighbourhood search makes no global-optimum claim.
    order = context.get('profile', {}).get('candidate_order', ['left', 'right'])
    index = 1 if order[round_number - 1] == 'left' else 3
    gains[index] = round(gains[index] * .8, 6)
    result = dict(zip(tune.GAIN_NAMES, gains))
    result.update(stop=gains == context['best_gains'],
                  reason='Reduce one wheel integral gain by 20 percent',
                  expected_effect='Check tracking and ripple at both calibrated RPM targets')
    return result


def validate(args):
    if not re.fullmatch('[0-9a-f]{16}', args.build_id):
        raise ValueError('Expected a 16-character firmware build ID')
    if (type(args.rounds) is not int or not 1 <= args.rounds <= 2
            or type(args.repeats) is not int or not 3 <= args.repeats <= 5):
        raise ValueError('Search requires 1..2 rounds and 3..5 repeats')
    if (not math.isfinite(args.min_improvement) or not .01 <= args.min_improvement <= .5
            or not math.isfinite(args.max_baseline_drift) or not 0 <= args.max_baseline_drift <= .5):
        raise ValueError('Invalid search comparison policy')
    if not 20 <= args.pwm_limit <= 150:
        raise ValueError('RPM search PWM limit must not exceed 150/999')
    bench.validate_parameters(args)  # Record exactly the precision sent to firmware.
    if getattr(args, 'first_wheel', 'left') not in ('left', 'right'):
        raise ValueError('Invalid first wheel')
    cpr = (args.cpr_left, args.cpr_right)
    for rpm in (10., 20.):
        request = copy.copy(args)
        request.peak = rpm
        bench.validate_parameters(request)
        if any(type(c) is not int or not 0 < c <= 1000000 or not 1 <= rpm * c / 6000 <= 25 for c in cpr):
            raise ValueError('Invalid calibration for dual-speed search')


class RPMExperiments:
    source = 'real_motor_rpm'

    def __init__(self, args, folder, factory=None):
        validate(args)  # Reject configuration before opening hardware.
        self.args, self.folder = args, Path(folder)
        self.factory = factory or (lambda path: bench.Board(args.port, path))
        self.identity = ('ELC_INC_PI_V1', args.build_id)
        self.sequence = 0
        first = getattr(args, 'first_wheel', 'left')
        self.profile = {'controller': CONTROLLER, 'firmware_info': list(self.identity),
                        'rpm_targets': [10, 20], 'profile': 'hold',
                        'cpr_reference': [args.cpr_left, args.cpr_right],
                        'pwm_limit': args.pwm_limit, 'sign_left': args.sign_left,
                        'sign_right': args.sign_right,
                        'candidate_order': [first, 'right' if first == 'left' else 'left'],
                        'measurement_basis': 'manual CPR; raised-wheel short bench only'}

    def measure(self, gains, label):
        group = self.folder / label
        group.mkdir(parents=True, exist_ok=False)
        results, board = [], None
        try:
            board = self.factory(group)
            board.firmware_info = self.identity
            for repeat in range(self.args.repeats):
                speeds = []
                for rpm in (10., 20.):
                    request = copy.copy(self.args)
                    request.peak = rpm
                    request.require_identified_firmware = True
                    for name, value in zip(tune.GAIN_NAMES, gains):
                        setattr(request, name, value)
                    bench.validate_parameters(request)
                    self.sequence += 1
                    prefix = group / f'{self.sequence:03d}_{int(rpm)}rpm'
                    tune.save(prefix.with_suffix('.request.json'),
                              {'gains': list(gains), 'rpm': rpm, 'profile': self.profile})
                    rows = board.run(request, self.sequence,
                                     wheel_cpr=(request.cpr_left, request.cpr_right), hold=True)
                    with prefix.with_suffix('.csv').open('w', newline='', encoding='utf-8') as stream:
                        writer = csv.writer(stream)
                        writer.writerow(bench.FIELDS)
                        writer.writerows(rows)
                    board.wait_stopped()
                    result = evaluate(rows, rpm, request.pwm_limit, (request.cpr_left, request.cpr_right))
                    result.update(source=self.source, gains=list(gains), experiment_id=self.sequence,
                                  firmware_info=list(self.identity), stopped_and_quiet=True,
                                  trace_file=prefix.with_suffix('.csv').relative_to(self.folder).as_posix())
                    tune.save(prefix.with_suffix('.json'), result)
                    speeds.append(result)
                # Each repeat is judged by its worst wheel at its worst speed.
                results.append({'score': max(r['score'] for r in speeds),
                                'wheels': {s: {'score': max(r['wheels'][s]['score'] for r in speeds)}
                                           for s in ('left', 'right')},
                                'speed_results': speeds, 'gains': list(gains), 'repeat': repeat + 1})
            tune.save(group / 'group.json', tune.group_stats(results))
            return results
        except BaseException as exc:
            tune.save(group / 'failure.json', {'source': self.source, 'error': str(exc)})
            if board is not None:
                try:
                    board.wait_stopped()
                except BaseException as stop_error:
                    tune.save(group / 'stop_failure.json', {'error': str(stop_error)})
            raise  # Never restart, clear faults, or try another candidate here.
        finally:
            if board is not None:
                board.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--build-id', required=True)
    parser.add_argument('--rounds', type=int, choices=(1, 2), default=1)
    parser.add_argument('--repeats', type=int, choices=(3, 4, 5), default=3)
    parser.add_argument('--first-wheel', choices=('left', 'right'), default='left')
    parser.add_argument('--cpr-left', type=int, default=1080)
    parser.add_argument('--cpr-right', type=int, default=1320)
    parser.add_argument('--sign-left', type=int, choices=(-1, 1), default=1)
    parser.add_argument('--sign-right', type=int, choices=(-1, 1), default=1)
    for side in ('left', 'right'):
        parser.add_argument(f'--kp-{side}', type=float, default=2.)
        parser.add_argument(f'--ki-{side}', type=float, default=1.)
    parser.add_argument('--pwm-limit', type=int, default=150)
    parser.add_argument('--min-improvement', type=float, default=.03)
    parser.add_argument('--max-baseline-drift', type=float, default=.25)
    parser.add_argument('--output', type=Path, default=Path('.build/hardware_checks'))
    args = parser.parse_args()
    args.peak, args.simulate = 20., False
    try:
        validate(args)
    except ValueError as exc:
        parser.error(str(exc))
    folder = args.output / ('rpm_search_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(parents=True, exist_ok=False)
    tune.save(folder / 'request.json', {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()})
    try:
        report = tune.optimize(RPMExperiments(args, folder), local_candidate, args, folder)
        passed = all(s['quality']['matched_and_smooth'] for r in report['best_experiments'] for s in r['speed_results'])
        print(json.dumps({'source': report['source'], 'best_gains': report['best_gains'],
                          'termination': report['termination'], 'quality_passed': passed,
                          'decisions': [d['action'] for d in report['decisions']], 'folder': str(folder)}, indent=2))
        return 0 if passed and report['termination'] in ('round_budget', 'provider_stop') else 1
    except BaseException as exc:
        tune.save(folder / 'failure.json', {'source': RPMExperiments.source, 'error': str(exc)})
        raise


if __name__ == '__main__':
    raise SystemExit(main())

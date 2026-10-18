"""Export verified real-motor search traces to a compiled-in PI configuration."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import re
from types import SimpleNamespace

import bench
from tune import GAIN_NAMES, group_stats


def close(a, b):
    return type(a) in (int, float) and math.isfinite(a) and math.isclose(a, b, rel_tol=1e-9, abs_tol=1e-9)


def render_profile(summary_path):
    summary_path = Path(summary_path).resolve()
    raw = summary_path.read_bytes()
    report = json.loads(raw)
    if not isinstance(report, dict):
        raise ValueError('Expected a search report object')
    if (report.get('source') != 'real_motor' or report.get('status') != 'complete'
            or report.get('termination') not in ('round_budget', 'provider_stop')):
        raise ValueError('Only completed real_motor searches without drift stops can be exported')
    profile = report['profile']
    if profile.get('protocol') != bench.HELLO or profile.get('controller') != bench.CONTROLLER:
        raise ValueError('Unsupported controller/profile; rerun with the current tools')
    identity = profile['firmware_info']
    if (not isinstance(identity, list) or len(identity) != 2 or identity[0] != 'ELC_INC_PI_V1'
            or not isinstance(identity[1], str) or not re.fullmatch('[0-9a-f]{16}', identity[1])):
        raise ValueError('Identified real firmware is required')
    gains = report['best_gains']
    if not isinstance(gains, list) or len(gains) != 4:
        raise ValueError('Expected four best gains')
    args = SimpleNamespace(**dict(zip(GAIN_NAMES, gains)), **{name: profile[name]
                           for name in ('peak', 'pwm_limit', 'sign_left', 'sign_right')})
    bench.validate_parameters(args)
    if gains != [getattr(args, name) for name in GAIN_NAMES] or args.peak != profile['peak']:
        raise ValueError('Parameters must match firmware precision')
    experiments = report['best_experiments']
    if not isinstance(experiments, list) or not 3 <= len(experiments) <= 5:
        raise ValueError('Three to five repeated best experiments are required')
    verified, paths, ids = [], set(), set()
    for experiment in experiments:
        if (experiment.get('source') != 'real_motor' or experiment.get('firmware_info') != identity
                or experiment.get('gains') != gains):
            raise ValueError('Experiment identity/gains mismatch')
        number = experiment['experiment_id']
        if type(number) is not int or not 1 <= number <= 1000000000 or number in ids:
            raise ValueError('Experiment ids must be distinct positive integers')
        ids.add(number)
        relative = Path(experiment['trace_file'])
        trace = (summary_path.parent / relative).resolve()
        if (relative.is_absolute() or not trace.is_relative_to(summary_path.parent)
                or trace.suffix != '.csv' or trace in paths):
            raise ValueError('Distinct CSV traces inside the search directory are required')
        paths.add(trace)
        with trace.open(newline='', encoding='utf-8') as stream:
            reader = csv.reader(stream)
            if next(reader) != list(bench.FIELDS):
                raise ValueError('Invalid telemetry header')
            rows = [tuple(float(value) for value in row) for row in reader]
        result = bench.evaluate(rows, args.peak, args.pwm_limit)
        if not experiment.get('valid') or not close(experiment.get('score'), result['score']):
            raise ValueError('Recorded score does not match raw telemetry')
        verified.append(result)
    actual, saved = group_stats(verified), report['best_statistics']
    if (not close(saved['median'], actual['median']) or not close(saved['worst'], actual['worst'])
            or any(not close(saved['wheels'][side], actual['wheels'][side]) for side in ('left', 'right'))):
        raise ValueError('Best statistics do not match repeated raw telemetry')
    lines = ['#ifndef CAR_MOTOR_TUNING_H', '#define CAR_MOTOR_TUNING_H',
             '/* Generated from repeated real_motor traces; recompile to apply. */',
             '#define CAR_TUNED_PI_ENABLED 1', '#define CAR_TUNED_PROFILE_VERSION 1',
             f'#define CAR_TUNED_SOURCE_BUILD_ID "{identity[1]}"',
             f'#define CAR_TUNED_REPORT_SHA256 "{hashlib.sha256(raw).hexdigest()}"']
    for name in GAIN_NAMES:
        lines.append(f'#define CAR_TUNED_{name.upper()} {getattr(args, name):.6f}f')
    lines.extend([f'#define CAR_TUNED_PWM_LIMIT {args.pwm_limit}',
                  f'#define CAR_TUNED_PEAK {args.peak:.4f}f',
                  f'#define CAR_TUNED_SIGN_LEFT ({args.sign_left})',
                  f'#define CAR_TUNED_SIGN_RIGHT ({args.sign_right})', '#endif', ''])
    return '\n'.join(lines)


def export(summary_path, output):
    content = render_profile(summary_path)  # Finish validation before any write.
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + '.tmp')
    temporary.write_text(content, encoding='utf-8')
    temporary.replace(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('summary', type=Path)
    parser.add_argument('--output', type=Path, required=True, help='Usually Core/Inc/motor_tuning.h')
    args = parser.parse_args()
    try:
        export(args.summary, args.output)
    except (ValueError, KeyError, TypeError, IndexError, StopIteration, OSError) as exc:
        parser.error(str(exc))
    print(f'Generated {args.output}; rebuild firmware to apply. No serial port was opened.')


if __name__ == '__main__':
    main()

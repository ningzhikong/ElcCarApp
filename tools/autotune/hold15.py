"""Bounded raised-wheel 15s RPM hold; compare cumulative encoder phase drift."""
import argparse
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import time

import bench

CAPABILITY = 'LONGINFO ELC_HOLD15_V1 150 16500 30'
CPR = (1080, 1320)
RAW_FIELDS = ('time_ms', 'sum_l', 'sum_r', 'pwm_l', 'pwm_r', 'summed_period_us')


def preflight(board, build_id):
    for command, expected in (
            ('HELLO', bench.HELLO), ('INFO', f'INFO ELC_INC_PI_V1 {build_id}'),
            ('CAL', 'CAL MANUAL_REF_V1 1080 1320'),
            ('RPMINFO', 'RPMINFO ELC_RPM_PI_ADAPTIVE_V1'), ('LONGINFO', CAPABILITY)):
        board.send(command)
        if board.read() != expected:
            raise ValueError(f'{command} capability/identity mismatch; not arming')


def dump(board, command, header, count, width, integer=False):
    board.send(command)
    if board.read() != header:
        raise ValueError(f'{command} header mismatch')
    rows = []
    deadline = time.monotonic() + 6
    for _ in range(count):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError('Dump deadline exceeded')
        row = tuple((int if integer else float)(v) for v in board.read(min(2, remaining)).split(','))
        if len(row) != width:
            raise ValueError('Malformed telemetry row')
        rows.append(row)
    if board.read() != ('RAWEND' if integer else 'END'):
        raise ValueError('Dump footer mismatch')
    return rows


def analyze(coarse, raw, rpm):
    if rpm not in (20, 25, 30) or len(coarse) != 111 or len(raw) != 551:
        raise ValueError('Incomplete fixed 15s profile')
    for i, row in enumerate(coarse):
        if len(row) != 11 or not all(math.isfinite(v) for v in row):
            raise ValueError('Invalid control trace')
        t, l, r, tl, tr, sl, sr, pl, pr, period, flags = row
        if t != i * 150 or any(v != int(v) for v in (t, l, r, pl, pr, period, flags)):
            raise ValueError('Control trace time/integer mismatch')
        on = 500 <= t < 15500
        if any(abs(target - (rpm * c / 6000 if on else 0)) > .00015
               for target, c in zip((tl, tr), CPR)):
            raise ValueError('Wrong target profile')
        if (not 5000 <= period <= 15000 or max(abs(l), abs(r), abs(sl), abs(sr)) > 100
                or not 0 <= pl <= 150 or not 0 <= pr <= 150
                or flags != (int(pl >= 150) | (int(pr >= 150) << 1))
                or (not on and (pl or pr))):
            raise ValueError('Control timing/output envelope violation')
    totals = [0, 0]
    phase = []
    for i, row in enumerate(raw):
        if len(row) != 6 or any(type(v) is not int for v in row):
            raise ValueError('Invalid count sum record')
        t, l, r, pl, pr, period = row
        if (t != i * 30 or max(abs(l), abs(r)) > (300 if i else 100)
                or not (15000 <= period <= 45000 if i else 5000 <= period <= 15000)
                or not 0 <= pl <= 150 or not 0 <= pr <= 150
                or (not 500 <= t < 15500 and (pl or pr))):
            raise ValueError('Count sum time/output envelope violation')
        if i % 5 == 0 and (pl, pr) != tuple(coarse[i // 5][7:9]):
            raise ValueError('Control and count sum output mismatch')
        totals[0] += l
        totals[1] += r
        turns = [total / c for total, c in zip(totals, CPR)]
        phase.append((t, *turns, (turns[0] - turns[1]) * 360))
    steady = [p for p in phase if 2000 <= p[0] < 15500]
    first, last = steady[0], steady[-1]
    seconds = (last[0] - first[0]) / 1000
    speeds = [(last[s] - first[s]) * 60 / seconds for s in (1, 2)]
    if min(speeds) < rpm * .1:
        raise ValueError('Insufficient encoder motion')
    mean_t = sum(p[0] / 1000 for p in steady) / len(steady)
    mean_phase = sum(p[3] for p in steady) / len(steady)
    slope = sum((p[0] / 1000 - mean_t) * (p[3] - mean_phase) for p in steady) / sum(
        (p[0] / 1000 - mean_t) ** 2 for p in steady)
    return dict(steady_interval_ms=[first[0], last[0]], count_integral_rpm=speeds,
                left_minus_right_rpm_from_phase_slope=slope / 6,
                steady_phase_change_deg=last[3] - first[3], final_phase_deg=phase[-1][3],
                full_trace_counts=totals, full_trace_turns=list(phase[-1][1:3]),
                diagnostic_only=True, phase_timeline=phase)


def run(board, rpm, build_id):
    preflight(board, build_id)
    board.wait_stopped()
    board.send('ARM 1 1')
    if board.read() != 'ARMED 1 1':
        raise ValueError('ARM not confirmed')
    board.send(f'HOLDRPM15 1 2 1 2 1 {rpm} 150')
    ack = board.read().split()
    if (len(ack) != 8 or ack[:2] != ['HOLD15ACK', '1']
            or list(map(float, ack[2:])) != [2, 1, 2, 1, rpm, 150]):
        raise ValueError('Applied 15s profile mismatch')
    deadline = time.monotonic() + 19
    while True:
        board.send('PING')
        status = board.status()
        request, state, fault, count, elapsed, armed, quiet = status
        if request != 1 or fault or state not in (1, 2):
            raise RuntimeError(f'MCU stopped/faulted: {status}')
        if state == 2:
            if count != 111 or elapsed != 16500 or armed:
                raise ValueError('Incomplete 15s experiment')
            break
        if time.monotonic() >= deadline:
            raise TimeoutError('Host experiment deadline exceeded')
        time.sleep(.1)
    coarse = dump(board, 'DUMP', 'BEGIN 1 111', 111, 11)
    raw = dump(board, 'RAWDUMP', f'RAWBEGIN15 1 551 1 1 {rpm:.4f}', 551, 6, integer=True)
    return coarse, raw


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--build-id', required=True)
    parser.add_argument('--rpm', type=int, choices=(20, 25, 30), default=30)
    parser.add_argument('--output', type=Path, default=Path('.build/hardware_checks'))
    args = parser.parse_args()
    if len(args.build_id) != 16 or any(c not in '0123456789abcdef' for c in args.build_id):
        parser.error('Expected a 16-digit firmware build id')
    folder = args.output / datetime.now().strftime('hold15_%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True, exist_ok=False)
    result = dict(rpm=args.rpm, build_id=args.build_id, duration_s=15, pwm_limit=150)
    board = None
    try:
        board = bench.Board(args.port, folder)
        coarse, raw = run(board, args.rpm, args.build_id)
        for name, fields, rows in (('trace_150ms.csv', bench.FIELDS, coarse),
                                   ('raw_30ms_sums.csv', RAW_FIELDS, raw)):
            with (folder / name).open('w', newline='', encoding='utf-8') as stream:
                writer = csv.writer(stream)
                writer.writerow(fields)
                writer.writerows(rows)
        result['analysis'] = analyze(coarse, raw, args.rpm)
        result['outcome'] = 'completed'
    except BaseException as exc:
        result.update(outcome='failed', error=str(exc))
    finally:
        if board is not None:
            try:
                board.wait_stopped()
                result['stopped_and_quiet'] = True
            except BaseException as exc:
                result['stop_error'] = str(exc)
            board.close()
        (folder / 'result.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    printable = dict(result, folder=str(folder))
    if 'analysis' in printable:
        printable['analysis'] = {k: v for k, v in result['analysis'].items() if k != 'phase_timeline'}
    print(json.dumps(printable, indent=2))
    return 0 if result.get('outcome') == 'completed' and result.get('stopped_and_quiet') else 1


if __name__ == '__main__':
    raise SystemExit(main())

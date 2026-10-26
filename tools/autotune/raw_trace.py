"""Stopped-only 100Hz encoder capture and offline estimator replay."""
from collections import deque
import math

CAPABILITY = 'RAWINFO ELC_RAW10_V1 10 552'
FIELDS = ('time_ms', 'raw_l', 'raw_r', 'pwm_l', 'pwm_r', 'period_us')
JOG_CAPABILITY = 'JOGRAWINFO ELC_JOG_RAW10_V1 10 800'


def capture_jog(board, side, pwm, duration):
    if side not in ('left', 'right') or not 1 <= pwm <= 150 or not 50 <= duration <= 800 or duration % 10:
        raise ValueError('Forward raw jog exceeds the diagnostic envelope')
    board.send('RAWDUMP')
    header = board.read().split()
    targets = [pwm, 0] if side == 'left' else [0, pwm]
    length = duration // 10 + 1
    if len(header) != 6 or header[0] != 'RAWJOG' or list(map(int, header[1:])) != [1, length, *targets, duration]:
        raise ValueError('Raw jog identity/output/duration mismatch')
    rows = [tuple(int(v) for v in board.read().split(',')) for _ in range(length)]
    if board.read() != 'RAWEND':
        raise ValueError('Missing raw jog end')
    for i, r in enumerate(rows):
        if len(r) != 6 or r[0] != i * 10 or max(abs(r[1]), abs(r[2])) > 100 or not 5000 <= r[5] <= 15000:
            raise ValueError('Incomplete or invalid raw jog samples')
        if list(r[3:5]) != (targets if r[0] < duration else [0, 0]):
            raise ValueError('PWM was not the requested constant pulse with a zero final output')
    return rows


def capture(board, rpm, *, hold, drive):
    board.send('RAWDUMP')
    header = board.read().split()
    if (len(header) != 6 or header[0] != 'RAWBEGIN' or
            list(map(int, header[1:5])) != [1, 551, 2 if drive else 1, int(hold)] or
            not math.isfinite(float(header[5])) or abs(float(header[5]) - rpm) > .0001):
        raise ValueError('Raw trace identity/profile mismatch')
    rows = [tuple(int(v) for v in board.read().split(',')) for _ in range(551)]
    if board.read() != 'RAWEND':
        raise ValueError('Missing raw trace end')
    return rows


def analyze(rows, coarse, rpm, limit, cpr, *, hold):
    if len(rows) != 551 or len(coarse) != 111:
        raise ValueError('Incomplete 10ms or 50ms trace')
    for i, r in enumerate(rows):
        if len(r) != 6 or any(type(v) is not int for v in r) or r[0] != i * 10:
            raise ValueError('Missing, duplicated or malformed 10ms sample')
        if max(abs(r[1]), abs(r[2])) > 100 or not 0 <= min(r[3:5]) <= max(r[3:5]) <= limit or not 5000 <= r[5] <= 15000:
            raise ValueError('Raw trace exceeds applied motion/timing envelope')
        if (r[0] < 500 or r[0] >= 4500) and (r[3] or r[4]):
            raise ValueError('Output was not zero outside the motion segment')
    for i, r in enumerate(coarse):
        raw = rows[i * 5]
        if any(abs(a-b) > .00001 for a, b in zip((r[0], r[1], r[2], r[7], r[8], r[9]), raw)):
            raise ValueError('10ms and 50ms traces do not describe the same samples')
    histories = [deque([0] * 40, maxlen=40), deque([0] * 40, maxlen=40)]
    windows = [10, 10]
    replay_error = [0., 0.]
    series = {width: [[], []] for width in (8, 10, 20, 40)}
    for r in rows:
        t = r[0]
        for side in (0, 1):
            histories[side].append(r[side + 1])
            history = list(histories[side])
            counts = sum(history[-windows[side]:]) / windows[side]
            if t % 50 == 0:
                replay_error[side] = max(replay_error[side], abs(counts - coarse[t // 50][5 + side]))
            if 2000 <= t < 4500:
                for width in series:
                    series[width][side].append(sum(history[-width:]) / width * 6000 / cpr[side])
        target = 0. if t < 500 or t >= 4500 else rpm if hold else .4 * rpm if t < 1500 else rpm if t < 3000 else .5 * rpm
        if target > 0:
            # The current tick estimates speed before the next window is set.
            windows = [max(8, min(25, int(200 / target + .5)))] * 2
    if max(replay_error) > .00011:
        raise ValueError('Raw counts cannot reproduce the firmware RPM feedback')
    plateau = [r for r in rows if 2000 <= r[0] < 4500]
    result = {'sample_count': len(rows), 'complete_10ms_timeline': True,
              'max_replay_error_counts_per_tick': replay_error,
              'offline_windows': {},
              'count_integral_rpm': [sum(r[i+1] for r in plateau) * 60 / (len(plateau) * .01 * cpr[i]) for i in (0, 1)],
              'raw_count_range': [[min(r[i+1] for r in plateau), max(r[i+1] for r in plateau)] for i in (0, 1)],
              'interpretation': 'Window comparisons are diagnostic only; reduced filtered variation is not independent proof of mechanical smoothness.'}
    for width, wheels in series.items():
        means = [sum(v) / len(v) for v in wheels]
        result['offline_windows'][str(width * 10)] = {
            'window_ms': width * 10, 'mean_rpm': means,
            'std_rpm': [(sum((x-m)**2 for x in v) / len(v))**.5 for v, m in zip(wheels, means)],
            'wheel_difference_mae': sum(abs(a-b) for a,b in zip(*wheels)) / len(wheels[0])}
    return result

"""Fixed-gain real-motor experiment over USART1. AI search is a separate next stage."""
import argparse
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import time

HELLO = 'HELLO ELC_AUTOTUNE_V1 INC_PI_COUNTS_10MS 10 50 5500'
FIELDS = ('time_ms', 'raw_l', 'raw_r', 'target_l', 'target_r', 'speed_l', 'speed_r',
          'pwm_l', 'pwm_r', 'period_us', 'flags')


def target_at(t, peak):
    return 0.0 if t < 500 or t >= 4500 else .4 * peak if t < 1500 else peak if t < 3000 else .5 * peak


def validate_trace(rows, peak, limit):
    """Reject invalid experiments before calculating either wheel's score."""
    if len(rows) != 111:
        raise ValueError('Expected 111 samples, including start and local stop')
    for index, row in enumerate(rows):
        if len(row) != 11 or not all(math.isfinite(value) for value in row):
            raise ValueError('Malformed/non-finite telemetry')
        t, raw_l, raw_r, target_l, target_r, speed_l, speed_r, pwm_l, pwm_r, period, flags = row
        if abs(t - index * 50) > 15 or (index and t <= rows[index - 1][0]):
            raise ValueError('Missing, duplicated or late samples')
        if not 5000 <= period <= 15000:
            raise ValueError('Invalid control period')
        expected = target_at(t, peak)
        if abs(target_l - expected) > .00015 or abs(target_r - expected) > .00015:
            raise ValueError('Target profile does not match the applied experiment')
        if max(abs(raw_l), abs(raw_r), abs(speed_l), abs(speed_r)) > 100:
            raise ValueError('Telemetry exceeds the initial bench speed envelope')
        if not 0 <= pwm_l <= limit or not 0 <= pwm_r <= limit:
            raise ValueError('Actual PWM exceeds applied limit')
        expected_flags = int(pwm_l >= limit) | (int(pwm_r >= limit) << 1)
        if flags != expected_flags:
            raise ValueError('Saturation flags disagree with actual PWM')
        if expected == 0 and (pwm_l != 0 or pwm_r != 0):
            raise ValueError('Drive was not cancelled in a zero-target segment')
    if rows[0][0] != 0 or not 5500 <= rows[-1][0] <= 5515:
        raise ValueError('Missing experiment boundaries')


def evaluate(rows, peak, limit):
    validate_trace(rows, peak, limit)
    wheels = {}
    for side, target_index, speed_index, pwm_index in (('left', 3, 5, 7), ('right', 4, 6, 8)):
        errors = [abs(r[target_index] - r[speed_index]) for r in rows]
        iae = sum((b[0] - a[0]) * .001 * (ea + eb) / 2
                  for a, b, ea, eb in zip(rows, rows[1:], errors, errors[1:]))
        segments = []
        for start, end, target, previous in ((500, 1500, .4 * peak, 0),
                                            (1500, 3000, peak, .4 * peak),
                                            (3000, 4500, .5 * peak, peak),
                                            (4500, 5501, 0, .5 * peak)):
            samples = [r for r in rows if start <= r[0] < end]
            if not samples:
                raise ValueError('Missing step response')
            tolerance = max(.5, .1 * target)
            settled = None
            for j, row in enumerate(samples):
                if end - row[0] >= 300 and all(abs(r[speed_index] - target) <= tolerance for r in samples[j:]):
                    settled = (row[0] - start) / 1000
                    break
            excursion = max(0., max((r[speed_index] - target) * (1 if target > previous else -1)
                                     for r in samples)) / abs(target - previous)
            segments.append({'target': target, 'settling_s': settled, 'excursion_fraction': excursion,
                             'duration_s': (end - start) / 1000})
        active = [r for r in rows if r[target_index] > 0]
        saturation = sum(r[pwm_index] >= limit for r in active) / len(active)
        variation = sum(abs(b[pwm_index] - a[pwm_index]) for a, b in zip(rows, rows[1:])) / (limit * (len(rows) - 1))
        settling = sum(1 if s['settling_s'] is None else s['settling_s'] / s['duration_s'] for s in segments) / 4
        score = iae / (2.65 * peak) + sum(s['excursion_fraction'] for s in segments) / 4 + settling + .05 * saturation + .05 * variation
        wheels[side] = {'score': score, 'iae_counts_s_per_10ms': iae, 'segments': segments,
                        'saturation_fraction': saturation, 'normalized_output_variation': variation}
    return {'valid': True, 'sample_count': len(rows), 'unit': 'encoder_counts_per_10ms',
            'algorithm': 'incremental_pi_v1', 'score_version': 'dual_bench_v1',
            'score': max(w['score'] for w in wheels.values()), 'wheels': wheels,
            'score_rule': 'worst wheel: normalized IAE + mean excursion + settling penalty + 0.05 saturation + 0.05 normalized output variation'}


class Board:
    def __init__(self, port, folder):
        import serial
        self.connection = serial.Serial(port, 115200, timeout=.1, write_timeout=.5)
        self.connection.reset_input_buffer()
        self.audit = (folder / 'serial.jsonl').open('w', encoding='utf-8')
        self.buffer = bytearray()

    def send(self, command):
        self.audit.write(json.dumps({'monotonic_s': time.monotonic(), 'tx': command}) + '\n')
        self.audit.flush()
        self.connection.write((command + '\n').encode('ascii'))

    def read(self, timeout=2):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if b'\n' in self.buffer:
                line, _, self.buffer = self.buffer.partition(b'\n')
                if len(line) > 512:
                    raise ValueError('Oversized serial frame')
                line = line.decode('ascii').strip()
                self.audit.write(json.dumps({'monotonic_s': time.monotonic(), 'rx': line}) + '\n')
                self.audit.flush()
                if line.startswith('READY '):
                    continue
                if line.startswith('ERR '):
                    raise RuntimeError(line)
                return line
            if len(self.buffer) > 512:
                raise ValueError('Oversized serial frame')
            self.buffer.extend(self.connection.read(min(self.connection.in_waiting or 1, 512)))
        raise TimeoutError('MCU response timeout')

    def stop(self):
        self.send('STOP')
        if self.read() != 'STOPPED':
            raise RuntimeError('Drive stop was not confirmed')

    def status(self):
        self.send('STATUS')
        data = self.read().split()
        if len(data) != 8 or data[0] != 'STATUS':
            raise ValueError('Invalid STATUS')
        return list(map(int, data[1:]))

    def run(self, args):
        self.send('HELLO')
        if self.read() != HELLO:
            raise RuntimeError('Wrong firmware, profile or boot mode')
        self.stop()
        deadline = time.monotonic() + 3
        while True:
            status = self.status()
            if status[-1] == 1:
                break
            if time.monotonic() > deadline:
                raise RuntimeError('Encoder motion has not settled')
            time.sleep(.1)
        self.send(f'ARM {args.sign_left} {args.sign_right}')
        if self.read() != f'ARMED {args.sign_left} {args.sign_right}':
            raise RuntimeError('ARM was not confirmed')
        gains = [round(g, 6) for g in (args.kp_left, args.ki_left, args.kp_right, args.ki_right)]
        self.send('RUN 1 ' + ' '.join(f'{g:.6f}' for g in gains) + f' {args.peak:.4f} {args.pwm_limit}')
        ack = self.read().split()
        if len(ack) != 8 or ack[:2] != ['ACK', '1']:
            raise ValueError('Invalid applied-parameter ACK')
        values = list(map(float, ack[2:]))
        if any(not math.isfinite(x) or abs(x - y) > .000001 for x, y in zip(values, gains + [args.peak, args.pwm_limit])):
            raise ValueError('Applied parameters do not match request')
        deadline = time.monotonic() + 8
        while True:
            self.send('PING')
            status = self.status()
            request, state, fault, count, elapsed, armed, quiet = status
            if request != 1 or fault or state not in (1, 2):
                raise RuntimeError(f'MCU experiment stopped/faulted: {status}')
            if state == 2:
                if count != 111 or not 5500 <= elapsed <= 5515:
                    raise RuntimeError('Incomplete experiment')
                break
            if time.monotonic() >= deadline:
                raise TimeoutError('Host experiment deadline exceeded')
            time.sleep(.1)
        self.send('DUMP')
        if self.read() != 'BEGIN 1 111':
            raise ValueError('Invalid dump start')
        rows = []
        for _ in range(111):
            row = self.read().split(',')
            if len(row) != len(FIELDS):
                raise ValueError('Invalid CSV sample')
            rows.append(tuple(float(v) for v in row))
        if self.read() != 'END':
            raise ValueError('Invalid dump end')
        return rows

    def close(self):
        self.connection.close()
        self.audit.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--sign-left', type=int, choices=(-1, 1), required=True)
    parser.add_argument('--sign-right', type=int, choices=(-1, 1), required=True)
    for side in ('left', 'right'):
        parser.add_argument(f'--kp-{side}', type=float, required=True)
        parser.add_argument(f'--ki-{side}', type=float, required=True)
    parser.add_argument('--peak', type=float, required=True, help='counts/10ms; not RPM')
    parser.add_argument('--pwm-limit', type=int, required=True)
    parser.add_argument('--output', type=Path, default=Path(__file__).parent / 'runs')
    args = parser.parse_args()
    limits = ((args.kp_left, 0, 10), (args.ki_left, 0, 2), (args.kp_right, 0, 10),
              (args.ki_right, 0, 2), (args.peak, 1, 25), (args.pwm_limit, 20, 200))
    if any(not math.isfinite(value) or not low <= value <= high for value, low, high in limits):
        parser.error('Experiment parameters outside initial bench bounds')
    args.peak = round(args.peak, 4)
    folder = args.output / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True, exist_ok=False)
    config = {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()}
    (folder / 'request.json').write_text(json.dumps(config, ensure_ascii=False, indent=2), encoding='utf-8')
    board = None
    stopped_and_quiet = False
    try:
        board = Board(args.port, folder)
        rows = board.run(args)
        with (folder / 'trace.csv').open('w', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            writer.writerow(FIELDS)
            writer.writerows(rows)
        result = evaluate(rows, args.peak, args.pwm_limit)
        result.update(source='real_motor', requested=config, protocol=HELLO)
        # Confirm cancellation and physical settling before returning success.
        board.stop()
        deadline = time.monotonic() + 3
        while True:
            status = board.status()
            if status[1] != 3 or status[2] != 0:
                raise RuntimeError(f'MCU fault after STOP: {status}')
            if status[-1] == 1:
                break
            if time.monotonic() >= deadline:
                raise RuntimeError('Drive cancelled, but encoder motion did not settle')
            time.sleep(.1)
        stopped_and_quiet = True
        (folder / 'summary.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
        print(f'Validated fixed-gain bench result: {folder}')
    except Exception as exc:
        (folder / 'failure.json').write_text(json.dumps({'error': str(exc)}), encoding='utf-8')
        raise
    finally:
        if board is not None:
            try:
                if not stopped_and_quiet:
                    board.stop()
            except Exception as exc:
                (folder / 'stop_failure.json').write_text(json.dumps({'error': str(exc)}), encoding='utf-8')
            finally:
                board.close()


if __name__ == '__main__':
    main()

from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools/autotune'))
import hold15


def traces(left_rpm=30, right_rpm=30):
    coarse, raw = [], []
    previous = [0, 0]
    for i in range(551):
        t = i * 30
        on = 500 <= t < 15500
        pwm = 100 if on else 0
        totals = [round(max(0, min(t, 15500) - 500) * c * speed / 60000)
                  for c, speed in zip(hold15.CPR, (left_rpm, right_rpm))]
        delta = [a - b for a, b in zip(totals, previous)]
        previous = totals
        raw.append((t, *delta, pwm, pwm, 30000 if i else 10000))
        if i % 5 == 0:
            coarse.append((t, 5, 7, 5.4 if on else 0, 6.6 if on else 0,
                           5.4, 6.6, pwm, pwm, 10000, 0))
    return coarse, raw


class Hold15Tests(unittest.TestCase):
    def test_equal_and_drifting_wheels(self):
        for right in (30, 29):
            result = hold15.analyze(*traces(30, right), 30)
            self.assertAlmostEqual(result['count_integral_rpm'][0], 30, delta=.01)
            self.assertAlmostEqual(result['count_integral_rpm'][1], right, delta=.01)
            self.assertAlmostEqual(result['left_minus_right_rpm_from_phase_slope'], 30-right, delta=.001)
            self.assertEqual(result['full_trace_counts'][0], 8100)

    def test_missing_corrupt_or_overpowered_trace_rejected(self):
        coarse, raw = traces()
        changes = [raw[:-1], raw.copy(), raw.copy(), raw.copy()]
        changes[1][80] = (2430, *raw[80][1:])
        changes[2][80] = (*raw[80][:-1], 50000)
        changes[3][-1] = (16500, 0, 0, 1, 0, 30000)
        for changed in changes:
            with self.assertRaises(ValueError): hold15.analyze(coarse, changed, 30)
        wrong = coarse.copy()
        wrong[10] = (*wrong[10][:7], 151, *wrong[10][8:])
        with self.assertRaises(ValueError): hold15.analyze(wrong, raw, 30)

    def test_wrong_capability_never_arms(self):
        class Board:
            sent = []
            replies = iter([hold15.bench.HELLO, 'INFO ELC_INC_PI_V1 abc',
                            'CAL MANUAL_REF_V1 1080 1320',
                            'RPMINFO ELC_RPM_PI_ADAPTIVE_V1', 'wrong'])
            def send(self, text): self.sent.append(text)
            def read(self): return next(self.replies)
        board = Board()
        with self.assertRaises(ValueError): hold15.run(board, 30, 'abc')
        self.assertFalse(any(s.startswith('ARM') for s in board.sent))

    def test_protocol_completes_and_exports_both_traces(self):
        coarse, raw = traces()
        class Board:
            def __init__(self):
                self.sent = []
                self.replies = iter([hold15.bench.HELLO, 'INFO ELC_INC_PI_V1 abc',
                    'CAL MANUAL_REF_V1 1080 1320', 'RPMINFO ELC_RPM_PI_ADAPTIVE_V1',
                    hold15.CAPABILITY, 'ARMED 1 1', 'HOLD15ACK 1 2 1 2 1 30 150',
                    'BEGIN 1 111', *[','.join(map(str, r)) for r in coarse], 'END',
                    'RAWBEGIN15 1 551 1 1 30.0000', *[','.join(map(str, r)) for r in raw], 'RAWEND'])
            def send(self, text): self.sent.append(text)
            def read(self, timeout=2): return next(self.replies)
            def wait_stopped(self): self.sent.append('WAIT_STOP')
            def status(self): return [1, 2, 0, 111, 16500, 0, 1]
        board = Board()
        actual = hold15.run(board, 30, 'abc')
        self.assertEqual(actual, (coarse, raw))
        self.assertIn('HOLDRPM15 1 2 1 2 1 30 150', board.sent)

    def test_host_deadline_without_done(self):
        class Board:
            def wait_stopped(self): pass
            def send(self, text): pass
            def read(self): return next(self.replies)
            def status(self): return [1, 1, 0, 1, 0, 1, 0]
        board = Board()
        board.replies = iter(['ARMED 1 1', 'HOLD15ACK 1 2 1 2 1 30 150'])
        with patch.object(hold15, 'preflight'), patch.object(hold15.time, 'monotonic', side_effect=[0, 20]):
            with self.assertRaises(TimeoutError): hold15.run(board, 30, 'abc')


if __name__ == '__main__':
    unittest.main()

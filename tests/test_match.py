import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools/autotune'))
import bench
import match


def trace(equal_counts=False):
    rows = []
    for t in range(0, 5501, 50):
        rpm = bench.target_at(t, 20)
        left, right = rpm * 1080 / 6000, rpm * 1320 / 6000
        speed_r = left if equal_counts else right
        pwm = 80 if rpm else 0
        rows.append([t, round(left), round(speed_r), left, right, left, speed_r, pwm, pwm, 10000, 0])
    return rows


class FakeBoard(bench.Board):
    def __init__(self, cal):
        self.sent = []
        self.replies = [bench.HELLO, 'INFO ELC_INC_PI_V1 0123456789abcdef', cal]

    def send(self, command):
        self.sent.append(command)

    def read(self, timeout=2):
        return self.replies.pop(0)


class MatchTests(unittest.TestCase):
    def test_equal_rpm_uses_different_count_speeds(self):
        result = match.evaluate(trace(), 20, 150, (1080, 1320))
        self.assertTrue(result['matched_in_final_plateau'])
        self.assertAlmostEqual(result['mean_rpm_left'], 20)
        self.assertAlmostEqual(result['mean_rpm_right'], 20)

    def test_equal_counts_is_not_equal_rpm(self):
        result = match.evaluate(trace(equal_counts=True), 20, 150, (1080, 1320))
        self.assertFalse(result['matched_in_final_plateau'])
        self.assertAlmostEqual(result['mean_rpm_right'], 20 * 1080 / 1320)

    def test_bad_targets_and_stopping_output_rejected(self):
        for index, field, value in ((40, 4, 3.6), (110, 7, 1), (40, 9, 20000), (40, 6, math.nan)):
            rows = trace()
            rows[index][field] = value
            with self.assertRaises(ValueError):
                match.evaluate(rows, 20, 150, (1080, 1320))

    def test_dead_wheel_cannot_be_matched(self):
        rows = trace()
        for row in rows:
            row[2] = row[6] = 0
        with self.assertRaises(ValueError):
            match.evaluate(rows, 20, 150, (1080, 1320))

    def test_cpr_mismatch_never_arms(self):
        board = FakeBoard('CAL MANUAL_REF_V1 1080 1080')
        args = SimpleNamespace(sign_left=1, sign_right=1, kp_left=2., ki_left=.3,
                               kp_right=2., ki_right=.3, peak=20., pwm_limit=150)
        with self.assertRaisesRegex(RuntimeError, 'calibration'):
            board.run(args, wheel_cpr=(1080, 1320))
        self.assertFalse(any(c.startswith(('ARM', 'RUN')) for c in board.sent))

    def test_rpm_protocol_roundtrip_and_ack_units(self):
        for prefix in ('RPMACK', 'ACK'):
            board = FakeBoard('CAL MANUAL_REF_V1 1080 1320')
            board.replies += ['STOPPED', 'STATUS 0 3 0 0 0 0 1', 'ARMED 1 1',
                              f'{prefix} 1 2.000000 0.300000 2.000000 0.300000 20.0000 150',
                              'STATUS 1 2 0 111 5500 0 0', 'BEGIN 1 111',
                              *[','.join(map(str, row)) for row in trace()], 'END']
            args = SimpleNamespace(sign_left=1, sign_right=1, kp_left=2., ki_left=.3,
                                   kp_right=2., ki_right=.3, peak=20., pwm_limit=150)
            if prefix == 'RPMACK':
                self.assertEqual(len(board.run(args, wheel_cpr=(1080, 1320))), 111)
                self.assertTrue(any(c.startswith('RUNRPM ') for c in board.sent))
            else:
                with self.assertRaisesRegex(ValueError, 'ACK'):
                    board.run(args, wheel_cpr=(1080, 1320))

    def test_raw_count_trace_cannot_enter_rpm_check(self):
        rows = trace()
        for row in rows:
            row[4] = row[3]
        with self.assertRaises(ValueError):
            match.evaluate(rows, 20, 150, (1080, 1320))


if __name__ == '__main__':
    unittest.main()

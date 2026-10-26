"""Validate the PC adapter against the actual C state machine's mocked trace."""
import importlib.util
import math
from pathlib import Path
import subprocess
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('bench', ROOT / 'tools/autotune/bench.py')
bench = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(bench)


class FakeBoard(bench.Board):
    def __init__(self, replies):
        self.replies = list(replies)
        self.sent = []

    def send(self, command):
        self.sent.append(command)

    def read(self, timeout=2):
        return self.replies.pop(0)


class BenchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        executable = ROOT / '.build/tests/firmware_tests.exe'
        result = subprocess.run([str(executable), 'complete', '--dump'],
                                capture_output=True, text=True, check=True)
        cls.dump_lines = result.stdout.splitlines()
        cls.rows = [tuple(map(float, line.split(','))) for line in cls.dump_lines[1:-1]]

    def test_c_trace_is_accepted(self):
        result = bench.evaluate(self.rows, 5., 100.)
        self.assertTrue(result['valid'])
        self.assertEqual(result['sample_count'], 111)
        self.assertEqual(result['score'], max(w['score'] for w in result['wheels'].values()))

    def test_missing_sample_rejected(self):
        with self.assertRaisesRegex(ValueError, '111'):
            bench.evaluate(self.rows[:-1], 5., 100.)

    def test_wrong_profile_rejected(self):
        with self.assertRaisesRegex(ValueError, 'profile'):
            bench.evaluate(self.rows, 6., 100.)

    def test_nonfinite_rejected(self):
        rows = [list(row) for row in self.rows]
        rows[5][5] = math.nan
        with self.assertRaisesRegex(ValueError, 'finite'):
            bench.evaluate(rows, 5., 100.)

    def test_output_limit_rejected(self):
        rows = [list(row) for row in self.rows]
        rows[20][7] = 101
        with self.assertRaisesRegex(ValueError, 'PWM'):
            bench.evaluate(rows, 5., 100.)

    def test_control_deadline_rejected(self):
        rows = [list(row) for row in self.rows]
        rows[20][9] = 20000
        with self.assertRaisesRegex(ValueError, 'period'):
            bench.evaluate(rows, 5., 100.)

    def test_stopping_sample_checked(self):
        rows = [list(row) for row in self.rows]
        rows[-1][7] = 1
        with self.assertRaisesRegex(ValueError, 'cancelled'):
            bench.evaluate(rows, 5., 100.)

    def test_board_adapter_roundtrip(self):
        replies = [bench.HELLO, 'INFO ELC_INC_PI_V1 1234567890abcdef', 'STOPPED', 'STATUS 0 3 0 0 0 0 1',
                   'ARMED 1 -1', 'ACK 1 2.000000 0.300000 2.000000 0.300000 5.0000 100',
                   'STATUS 1 2 0 111 5500 0 0',
                   self.dump_lines[0].replace('BEGIN 7 ', 'BEGIN 1 '), *self.dump_lines[1:]]
        board = FakeBoard(replies)
        args = SimpleNamespace(sign_left=1, sign_right=-1, kp_left=2., ki_left=.3,
                               kp_right=2., ki_right=.3, peak=5., pwm_limit=100)
        self.assertEqual(board.run(args), self.rows)
        self.assertIn('PING', board.sent)
        self.assertEqual(board.sent[-1], 'DUMP')

    def test_wrong_applied_gains_rejected(self):
        board = FakeBoard([bench.HELLO, 'INFO ELC_INC_PI_V1 1234567890abcdef', 'STOPPED', 'STATUS 0 3 0 0 0 0 1', 'ARMED 1 -1',
                           'ACK 1 nan 0.300000 2.000000 0.300000 5.0000 100'])
        args = SimpleNamespace(sign_left=1, sign_right=-1, kp_left=2., ki_left=.3,
                               kp_right=2., ki_right=.3, peak=5., pwm_limit=100)
        with self.assertRaisesRegex(ValueError, 'Applied'):
            board.run(args)
        self.assertNotIn('DUMP', board.sent)

    def test_fractional_counter_rejected(self):
        rows = [list(row) for row in self.rows]
        rows[20][1] = .5
        with self.assertRaisesRegex(ValueError, 'integers'):
            bench.evaluate(rows, 5., 100.)

    def test_unknown_firmware_cannot_start_search(self):
        board = FakeBoard([bench.HELLO, 'INFO ELC_INC_PI_V1 unidentified'])
        args = SimpleNamespace(sign_left=1, sign_right=-1, kp_left=2., ki_left=.3,
                               kp_right=2., ki_right=.3, peak=5., pwm_limit=100, require_identified_firmware=True)
        with self.assertRaisesRegex(RuntimeError, 'identified build'):
            board.run(args)
        self.assertFalse(any(command.startswith('ARM') for command in board.sent))

    def test_each_wheel_requires_measured_motion(self):
        for raw, speed, side in ((1, 5, 'left'), (2, 6, 'right')):
            rows = [list(row) for row in self.rows]
            for row in rows:
                row[raw] = row[speed] = 0
            with self.assertRaisesRegex(ValueError, f'motion on {side}'):
                bench.evaluate(rows, 5., 100.)

    def test_low_pwm_cap_cannot_hide_a_dead_encoder(self):
        rows = [list(row) for row in self.rows]
        for row in rows:
            row[1] = row[2] = row[5] = row[6] = 0
            row[7] = row[8] = 50 if row[3] else 0
            row[10] = 3 if row[3] else 0
        with self.assertRaisesRegex(ValueError, 'motion'):
            bench.evaluate(rows, 5., 50.)

    def test_decimated_raw_pulses_can_alias_without_a_dead_motor(self):
        rows, filtered = [], 0.
        for t in range(0, 5501, 10):
            target = bench.target_at(t, 1.)
            raw = int(target > 0 and (t // 10) % 5 in (1, 3))
            filtered += .125 * (raw - filtered)
            if t % 50 == 0:
                pwm = 50 if target else 0
                rows.append((t, raw, raw, target, target, filtered, filtered, pwm, pwm, 10000, 0))
        self.assertTrue(all(row[1] == 0 for row in rows))
        self.assertTrue(bench.evaluate(rows, 1., 100.)['valid'])


if __name__ == '__main__':
    unittest.main()

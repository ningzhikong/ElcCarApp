import sys
from pathlib import Path
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools/autotune'))
import bench
import jog
import raw_trace


class FakeBoard:
    def __init__(self, info='INFO ELC_INC_PI_V1 0123456789abcdef', enc='ENC 0 0 0'):
        self.replies = iter([bench.HELLO, info, enc, 'ARMED 1 1', 'JOGACK 1 0 -100 600', 'ENC 610 0 -12'])
        self.sent = []

    def wait_stopped(self):
        self.sent.append('STOP')

    def send(self, text):
        self.sent.append(text)

    def read(self):
        return next(self.replies)

    def status(self):
        return [1, 2, 0, 13, 600, 0, 0]


class JogTests(unittest.TestCase):
    def args(self):
        return SimpleNamespace(build_id='0123456789abcdef', side='right', pwm=-100, duration_ms=600)

    def test_wrong_build_cannot_arm(self):
        board = FakeBoard(info='INFO ELC_INC_PI_V1 ffffffffffffffff')
        with self.assertRaises(RuntimeError):
            jog.run(board, self.args())
        self.assertFalse(any(c.startswith(('ARM', 'JOG')) for c in board.sent))

    def test_missing_capability_cannot_arm(self):
        board = FakeBoard(enc='ERR COMMAND')
        with self.assertRaises(RuntimeError):
            jog.run(board, self.args())
        self.assertFalse(any(c.startswith(('ARM', 'JOG')) for c in board.sent))

    def test_single_pulse_is_diagnostic_only(self):
        board = FakeBoard()
        result = jog.run(board, self.args())
        self.assertEqual(board.sent.count('JOG 1 0 -100 600'), 1)
        self.assertEqual(result['encoder_after'], 'ENC 610 0 -12')
        self.assertNotIn('score', result)

    def test_forward_raw_pulse_exports_only_the_acknowledged_output(self):
        args = SimpleNamespace(build_id='0123456789abcdef', side='left', pwm=111, duration_ms=800, raw_10ms=True)
        rows = [(i * 10, 3 if i else 0, 0, 111 if i < 80 else 0, 0, 10000) for i in range(81)]
        board = FakeBoard()
        board.replies = iter([bench.HELLO, 'INFO ELC_INC_PI_V1 0123456789abcdef', raw_trace.JOG_CAPABILITY,
                              'ENC 0 0 0', 'ARMED 1 1', 'JOGACK 1 111 0 800', 'ENC 810 240 0',
                              'RAWJOG 1 81 111 0 800', *[','.join(map(str, r)) for r in rows], 'RAWEND'])
        board.status = lambda: [1, 2, 0, 17, 800, 0, 0]
        result = jog.run(board, args)
        self.assertEqual(result['raw_10ms'], rows)
        self.assertEqual(board.sent.count('JOG 1 111 0 800'), 1)

    def test_raw_capability_or_signed_output_failure_never_arms(self):
        args = self.args()
        args.raw_10ms = True
        board = FakeBoard()
        with self.assertRaises(ValueError):
            jog.run(board, args)
        self.assertFalse(any(c.startswith(('ARM', 'JOG')) for c in board.sent))
        args.pwm = 100
        board = FakeBoard()
        with self.assertRaises(RuntimeError):
            jog.run(board, args)
        self.assertFalse(any(c.startswith(('ARM', 'JOG 1')) for c in board.sent))


if __name__ == '__main__':
    unittest.main()

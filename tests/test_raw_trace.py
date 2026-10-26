from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools/autotune'))
import raw_trace


def traces():
    raw, coarse = [], []
    for i in range(551):
        t = i * 10
        counts = [round((i-50)*v) - round((i-51)*v) if 500 < t < 4500 else 0 for v in (5.4, 6.6)]
        pwm = 100 if 500 <= t < 4500 else 0
        raw.append((t, *counts, pwm, pwm, 10000))
        if i % 5 == 0:
            window = 10 if t <= 500 else 8
            feedback = [sum(r[side+1] for r in raw[max(0, i-window+1):i+1]) / window for side in (0, 1)]
            targets = [v if pwm else 0 for v in (5.4, 6.6)]
            coarse.append([t, *counts, *targets, *feedback, pwm, pwm, 10000, 0])
    return raw, coarse


class RawTraceTests(unittest.TestCase):
    def test_raw_jog_rejects_wrong_side_or_nonconstant_output(self):
        class Board:
            def __init__(self, header, bad=False):
                rows = [(i * 10, 3, 0, 111 if i < 80 else 0, 0, 10000) for i in range(81)]
                if bad: rows[40] = (400, 3, 0, 112, 0, 10000)
                self.replies = iter([header, *[','.join(map(str, r)) for r in rows], 'RAWEND'])
            def send(self, text): self.sent = text
            def read(self): return next(self.replies)
        for board in (Board('RAWJOG 1 81 0 111 800'), Board('RAWJOG 1 81 111 0 800', True)):
            with self.assertRaises(ValueError):
                raw_trace.capture_jog(board, 'left', 111, 800)

    def test_complete_trace_replays_feedback_without_changing_score(self):
        raw, coarse = traces()
        result = raw_trace.analyze(raw, coarse, 30, 150, (1080, 1320), hold=True)
        self.assertEqual(result['sample_count'], 551)
        self.assertEqual(result['max_replay_error_counts_per_tick'], [0, 0])
        self.assertAlmostEqual(result['count_integral_rpm'][0], 30)
        self.assertAlmostEqual(result['count_integral_rpm'][1], 30)
        self.assertLess(result['offline_windows']['200']['std_rpm'][0], result['offline_windows']['80']['std_rpm'][0])

    def test_missing_timing_and_output_errors_rejected(self):
        raw, coarse = traces()
        variants = [raw[:-1], raw.copy(), raw.copy(), raw.copy()]
        variants[1][60] = (610, *raw[60][1:])
        variants[2][60] = (*raw[60][:-1], 20000)
        variants[3][500] = (5000, 0, 0, 100, 0, 10000)
        for changed in variants:
            with self.assertRaises(ValueError):
                raw_trace.analyze(changed, coarse, 30, 150, (1080, 1320), hold=True)

    def test_coarse_and_raw_must_be_the_same_experiment(self):
        raw, coarse = traces()
        coarse[50][1] += 1
        with self.assertRaisesRegex(ValueError, 'same samples'):
            raw_trace.analyze(raw, coarse, 30, 150, (1080, 1320), hold=True)

    def test_wrong_estimator_rejected(self):
        raw, coarse = traces()
        coarse[50][5] += .5
        with self.assertRaisesRegex(ValueError, 'reproduce'):
            raw_trace.analyze(raw, coarse, 30, 150, (1080, 1320), hold=True)

    def test_export_checks_profile_and_sends_no_motion_command(self):
        raw, _ = traces()
        class Board:
            def __init__(self, header):
                self.replies = iter([header, *[','.join(map(str, r)) for r in raw], 'RAWEND'])
                self.sent = []
            def send(self, text): self.sent.append(text)
            def read(self): return next(self.replies)
        board = Board('RAWBEGIN 1 551 1 1 30.0000')
        self.assertEqual(raw_trace.capture(board, 30, hold=True, drive=False), raw)
        self.assertEqual(board.sent, ['RAWDUMP'])
        for header in ('RAWBEGIN 1 551 1 0 30.0000', 'RAWBEGIN 2 551 1 1 30.0000', 'RAWBEGIN 1 551 1 1 nan'):
            with self.assertRaises(ValueError):
                raw_trace.capture(Board(header), 30, hold=True, drive=False)


if __name__ == '__main__':
    unittest.main()

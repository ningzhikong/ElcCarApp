import copy
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools/autotune'))
import bench
import rpm_tune
import tune
from test_match import trace


def settings():
    return SimpleNamespace(port='fake', build_id='0123456789abcdef', rounds=1, repeats=3,
                           kp_left=2., ki_left=1., kp_right=2., ki_right=1.,
                           cpr_left=1080, cpr_right=1320, sign_left=1, sign_right=1,
                           peak=20., pwm_limit=150, min_improvement=.03,
                           max_baseline_drift=.25, simulate=False)


def rpm_trace(rpm):
    rows = trace(hold=True)
    for row in rows:
        for i in (3, 4, 5, 6):
            row[i] *= rpm / 20
        row[1], row[2] = round(row[5]), round(row[6])
    return rows


class FakeBoard:
    def __init__(self, fail=False):
        self.fail, self.calls, self.stops, self.closed = fail, [], 0, False

    def run(self, request, sequence, wheel_cpr=None, hold=False):
        self.calls.append((copy.copy(request), sequence, wheel_cpr, hold))
        if self.fail:
            raise RuntimeError('MCU fault')
        return rpm_trace(request.peak)

    def wait_stopped(self):
        self.stops += 1

    def close(self):
        self.closed = True


class RPMTuneTests(unittest.TestCase):
    def test_report_gains_match_firmware_precision(self):
        args = settings()
        args.ki_left = 1.00000001
        rpm_tune.validate(args)
        self.assertEqual(args.ki_left, 1.)

    def test_validates_before_opening_hardware(self):
        for field, value in (('build_id', 'unknown'), ('pwm_limit', 151),
                             ('repeats', 1), ('rounds', 3), ('cpr_left', 0),
                             ('ki_right', float('nan')), ('max_baseline_drift', float('inf'))):
            args = settings()
            setattr(args, field, value)
            with self.subTest(field=field), self.assertRaises(ValueError):
                rpm_tune.RPMExperiments(args, Path('unused'), lambda _: self.fail('opened hardware'))

    def test_coordinate_proposal_is_bounded_and_independent(self):
        for number, expected in ((1, [2., .8, 2., 1.]), (2, [2., 1., 2., .8])):
            proposal = rpm_tune.local_candidate({'best_gains': [2., 1., 2., 1.]}, None, number)
            checked = tune.validate_candidate(proposal, [2., 1., 2., 1.])
            self.assertEqual([checked[n] for n in tune.GAIN_NAMES], expected)

    def test_zero_integral_requests_stop(self):
        proposal = rpm_tune.local_candidate({'best_gains': [2., 0., 2., 1.]}, None, 1)
        self.assertTrue(tune.validate_candidate(proposal, [2., 0., 2., 1.])['stop'])

    def test_right_wheel_can_be_tested_first(self):
        context = {'best_gains': [2., 1., 2., 1.], 'profile': {'candidate_order': ['right', 'left']}}
        candidate = rpm_tune.local_candidate(context, None, 1)
        self.assertEqual(candidate['ki_left'], 1.)
        self.assertEqual(candidate['ki_right'], .8)
        tune.validate_candidate(candidate, context['best_gains'])

    def test_score_uses_rpm_not_equal_counts(self):
        good = rpm_tune.evaluate(trace(hold=True), 20, 150, (1080, 1320))
        bad = rpm_tune.evaluate(trace(equal_counts=True, hold=True), 20, 150, (1080, 1320))
        self.assertAlmostEqual(good['score'], 0.)
        self.assertGreater(bad['score'], 1.)
        self.assertFalse(bad['quality']['matched_and_smooth'])

    def test_acceleration_and_stop_are_scored(self):
        rows = rpm_trace(20)
        rows[10][5] = 0.
        rows[90][5] = 1.
        result = rpm_tune.evaluate(rows, 20, 150, (1080, 1320))
        self.assertTrue(result['quality']['matched_and_smooth'])
        self.assertGreater(result['wheels']['left']['normalized_tracking_error'], 0)
        self.assertGreater(result['wheels']['left']['normalized_stop_residual'], 0)

    def test_each_repeat_covers_two_speeds_and_stops(self):
        with tempfile.TemporaryDirectory() as folder:
            board = FakeBoard()
            exp = rpm_tune.RPMExperiments(settings(), folder, lambda _: board)
            results = exp.measure((2., 1., 2., 1.), 'baseline')
            self.assertEqual(len(results), 3)
            self.assertEqual([r[0].peak for r in board.calls], [10., 20.] * 3)
            self.assertTrue(all(c[2] == (1080, 1320) and c[3] for c in board.calls))
            self.assertTrue(all(c[0].require_identified_firmware for c in board.calls))
            self.assertEqual(board.firmware_info, ('ELC_INC_PI_V1', settings().build_id))
            self.assertEqual(board.stops, 6)
            self.assertTrue(board.closed)
            self.assertAlmostEqual(results[0]['score'], 0.)
            saved = json.loads(next((Path(folder) / 'baseline').glob('*.request.json')).read_text())
            self.assertEqual(saved['profile']['controller'], rpm_tune.CONTROLLER)
            self.assertNotEqual(exp.source, 'real_motor')  # Old IIR profile exporter must reject it.

    def test_fault_stops_and_does_not_retry(self):
        with tempfile.TemporaryDirectory() as folder:
            board = FakeBoard(fail=True)
            exp = rpm_tune.RPMExperiments(settings(), folder, lambda _: board)
            with self.assertRaisesRegex(RuntimeError, 'MCU fault'):
                exp.measure((2., 1., 2., 1.), 'baseline')
            self.assertEqual(len(board.calls), 1)
            self.assertEqual(board.stops, 1)
            self.assertTrue(board.closed)
            self.assertTrue((Path(folder) / 'baseline/failure.json').is_file())

    def test_report_preserves_rpm_contract(self):
        with tempfile.TemporaryDirectory() as folder:
            exp = rpm_tune.RPMExperiments(settings(), folder, lambda _: FakeBoard())
            report = tune.optimize(exp, lambda context, *unused: dict(
                zip(tune.GAIN_NAMES, context['best_gains']), stop=True,
                reason='baseline sufficient', expected_effect='no further motion'), settings(), folder)
            self.assertEqual(report['profile']['controller'], rpm_tune.CONTROLLER)
            self.assertEqual(report['profile']['rpm_targets'], [10, 20])
            self.assertEqual(report['source'], 'real_motor_rpm')
            self.assertNotIn('filter_alpha', report['profile'])


if __name__ == '__main__':
    unittest.main()

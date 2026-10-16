import copy
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/autotune'))
import tune

BEST = (2., .3, 2., .3)


def proposal(gains=(2.2, .33, 2.2, .33), stop=False):
    result = dict(zip(tune.GAIN_NAMES, gains))
    result.update(stop=stop, reason='test', expected_effect='test only')
    return result


def settings():
    return SimpleNamespace(kp_left=2., ki_left=.3, kp_right=2., ki_right=.3,
                           sign_left=1, sign_right=-1, peak=5., pwm_limit=100,
                           rounds=1, repeats=3, min_improvement=.03,
                           max_baseline_drift=.1, simulate=True, port=None)


class FakeExperiments:
    source = 'host_mock'
    identity = ('ELC_INC_PI_V1', 'host-mock-v1')

    def __init__(self, scores):
        self.scores = iter(scores)
        self.calls = []

    def measure(self, gains, label):
        self.calls.append((tuple(gains), label))
        score = next(self.scores)
        if isinstance(score, Exception):
            raise score
        return [{'score': score, 'wheels': {'left': {'score': score}, 'right': {'score': score}}}
                for _ in range(3)]


class TuneTests(unittest.TestCase):
    def test_candidate_step_and_finite_values(self):
        for bad in (True, float('nan'), float('inf'), 2.5):
            value = proposal()
            value['kp_left'] = bad
            with self.assertRaises(ValueError):
                tune.validate_candidate(value, BEST)

    def test_stop_cannot_apply_new_gains(self):
        with self.assertRaisesRegex(ValueError, 'current best'):
            tune.validate_candidate(proposal(stop=True), BEST)
        self.assertTrue(tune.validate_candidate(proposal(BEST, True), BEST)['stop'])

    def test_repeated_candidate_and_rounding_rejected(self):
        with self.assertRaisesRegex(ValueError, 'already tested'):
            tune.validate_candidate(proposal(), BEST, [(2.2, .33, 2.2, .33)])
        value = proposal(BEST)
        value['kp_left'] += 1e-9
        with self.assertRaisesRegex(ValueError, 'unchanged'):
            tune.validate_candidate(value, BEST)

    def test_zero_gain_step_is_bounded(self):
        tune.validate_candidate(proposal((.01, .001, .01, .001)), (0, 0, 0, 0))
        with self.assertRaisesRegex(ValueError, 'step policy'):
            tune.validate_candidate(proposal((.02, .001, .01, .001)), (0, 0, 0, 0))

    def run_search(self, scores):
        temporary = tempfile.TemporaryDirectory(dir=ROOT / '.build/tests')
        self.addCleanup(temporary.cleanup)
        experiments = FakeExperiments(scores)
        result = tune.optimize(experiments, lambda *unused: proposal(), settings(), Path(temporary.name))
        return result, experiments

    def test_accept_requires_confirmation(self):
        result, experiments = self.run_search([1., .8, 1.02, .81])
        self.assertEqual(result['decisions'][0]['action'], 'accepted')
        self.assertEqual(result['best_gains'], [2.2, .33, 2.2, .33])
        self.assertTrue(experiments.calls[-1][1].endswith('confirmation'))

    def test_rejection_restores_baseline(self):
        result, experiments = self.run_search([1., 1.2, 1.01])
        self.assertEqual(result['decisions'][0]['action'], 'rejected_baseline_restored')
        self.assertEqual(experiments.calls[-1][0], BEST)

    def test_failed_confirmation_rolls_back(self):
        result, experiments = self.run_search([1., .8, 1., 1.1, 1.])
        self.assertEqual(result['decisions'][0]['action'], 'confirmation_failed_rolled_back')
        self.assertEqual(experiments.calls[-1], (BEST, 'round_01_rollback'))

    def test_drift_stops_search_with_baseline_applied(self):
        result, experiments = self.run_search([1., .8, 1.3])
        self.assertEqual(result['termination'], 'baseline_drift')
        self.assertEqual(experiments.calls[-1][0], BEST)
        self.assertEqual(len(experiments.calls), 3)

    def test_drift_after_rollback_stops_search(self):
        result, experiments = self.run_search([1., .8, 1., 1.1, 1.3])
        self.assertEqual(result['termination'], 'rollback_drift')
        self.assertEqual(experiments.calls[-1][0], BEST)

    def test_one_wheel_cannot_hide_regression(self):
        old = [{'score': 1., 'wheels': {'left': {'score': .5}, 'right': {'score': 1.}}}]
        new = [{'score': .8, 'wheels': {'left': {'score': .6}, 'right': {'score': .8}}}]
        self.assertFalse(tune.improved(new, old, old, .03))

    def test_hardware_failure_never_starts_a_rollback_experiment(self):
        with tempfile.TemporaryDirectory(dir=ROOT / '.build/tests') as directory:
            experiments = FakeExperiments([1., RuntimeError('hardware fault')])
            with self.assertRaisesRegex(RuntimeError, 'hardware fault'):
                tune.optimize(experiments, lambda *unused: proposal(), settings(), Path(directory))
            self.assertEqual(len(experiments.calls), 2)

    def test_provider_runs_only_after_port_closed(self):
        active = []
        class TrackedMock(tune.MockBoard):
            def __init__(self):
                active.append(self)
            def close(self):
                active.remove(self)
        def provider(context, folder, round_number):
            self.assertFalse(active)
            return proposal(BEST, True)
        with tempfile.TemporaryDirectory(dir=ROOT / '.build/tests') as directory:
            args = settings()
            experiments = tune.Experiments(args, directory, lambda _: TrackedMock())
            report = tune.optimize(experiments, provider, args, Path(directory))
            self.assertEqual(report['source'], 'host_mock')
            self.assertEqual(report['termination'], 'provider_stop')
            self.assertFalse(active)

    def test_malformed_candidate_file_is_rejected_before_motion(self):
        with tempfile.TemporaryDirectory(dir=ROOT / '.build/tests') as directory:
            path = Path(directory) / 'candidate.json'
            value = proposal()
            value['kp_left'] = float('nan')
            path.write_text(json.dumps(value), encoding='utf-8')
            with self.assertRaises(ValueError):
                tune.CandidateFile(path)

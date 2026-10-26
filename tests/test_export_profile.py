"""Export validation with synthetic fixtures; never connects to a motor."""
import copy
import csv
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/autotune'))
import bench
import export_profile
import tune


class ExportTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(dir=ROOT / '.build/tests')
        self.addCleanup(self.temporary.cleanup)
        self.folder = Path(self.temporary.name)
        self.summary = self.folder / 'summary.json'
        self.output = self.folder / 'motor_tuning.h'
        args = SimpleNamespace(kp_left=2., ki_left=.3, kp_right=2., ki_right=.3,
                               peak=5., pwm_limit=100, sign_left=1, sign_right=-1)
        # Labelled only to exercise the real-export parser; all data is fictional.
        self.rows = tune.MockBoard().run(args)
        result = bench.evaluate(self.rows, 5, 100)
        experiments = []
        for number in range(1, 4):
            with (self.folder / f'{number}.csv').open('w', newline='', encoding='utf-8') as stream:
                writer = csv.writer(stream); writer.writerow(bench.FIELDS); writer.writerows(self.rows)
            entry = copy.deepcopy(result)
            entry.update(source='real_motor', gains=[2., .3, 2., .3], experiment_id=number,
                         firmware_info=['ELC_INC_PI_V1', '1234567890abcdef'], trace_file=f'{number}.csv')
            experiments.append(entry)
        self.report = {'source': 'real_motor', 'status': 'complete', 'termination': 'round_budget',
                       'profile': {'protocol': bench.HELLO, 'controller': copy.deepcopy(bench.CONTROLLER),
                                   'firmware_info': ['ELC_INC_PI_V1', '1234567890abcdef'],
                                   'peak': 5., 'pwm_limit': 100, 'sign_left': 1, 'sign_right': -1},
                       'best_gains': [2., .3, 2., .3], 'best_experiments': experiments,
                       'best_statistics': tune.group_stats(experiments)}

    def run_export(self):
        self.summary.write_text(json.dumps(self.report), encoding='utf-8')
        export_profile.export(self.summary, self.output)

    def test_verified_repeats_export_complete_envelope(self):
        self.run_export()
        header = self.output.read_text(encoding='utf-8')
        for text in ('CAR_TUNED_PI_ENABLED 1', 'CAR_TUNED_KP_LEFT 2.000000f',
                     'CAR_TUNED_SIGN_RIGHT (-1)', 'CAR_TUNED_PWM_LIMIT 100',
                     'CAR_TUNED_PEAK 5.0000f', '1234567890abcdef', 'CAR_TUNED_REPORT_SHA256'):
            self.assertIn(text, header)

    def test_mock_results_never_export(self):
        self.report['source'] = 'host_mock'
        with self.assertRaisesRegex(ValueError, 'real_motor'): self.run_export()
        self.assertFalse(self.output.exists())

    def test_drift_or_partial_search_never_export(self):
        for field, value in (('status', 'baseline_complete'), ('termination', 'baseline_drift')):
            original = self.report[field]; self.report[field] = value
            with self.assertRaisesRegex(ValueError, 'completed'): self.run_export()
            self.report[field] = original

    def test_changed_controller_is_rejected(self):
        self.report['profile']['controller']['kd'] = .05
        with self.assertRaisesRegex(ValueError, 'controller'): self.run_export()

    def test_score_tampering_cannot_overwrite_existing_config(self):
        self.output.write_text('previous config', encoding='utf-8')
        self.report['best_experiments'][0]['score'] += .1
        with self.assertRaisesRegex(ValueError, 'score'): self.run_export()
        self.assertEqual(self.output.read_text(encoding='utf-8'), 'previous config')

    def test_dead_wheel_raw_trace_is_rejected(self):
        rows = [list(row) for row in self.rows]
        for row in rows: row[1] = row[5] = 0
        with (self.folder / '1.csv').open('w', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream); writer.writerow(bench.FIELDS); writer.writerows(rows)
        with self.assertRaisesRegex(ValueError, 'motion'): self.run_export()

    def test_duplicate_or_external_trace_is_rejected(self):
        for path in ('1.csv', '../outside.csv'):
            self.report['best_experiments'][1]['trace_file'] = path
            with self.assertRaisesRegex(ValueError, 'CSV traces'): self.run_export()

    def test_inconsistent_identity_or_gains_is_rejected(self):
        for field, value in (('firmware_info', ['ELC_INC_PI_V1', 'fedcba0987654321']),
                             ('gains', [2.1, .3, 2., .3])):
            entry = self.report['best_experiments'][0]; previous = entry[field]; entry[field] = value
            with self.assertRaisesRegex(ValueError, 'mismatch'): self.run_export()
            entry[field] = previous

    def test_summary_statistics_are_recomputed(self):
        self.report['best_statistics']['median'] *= .5
        with self.assertRaisesRegex(ValueError, 'statistics'): self.run_export()

    def test_three_distinct_experiments_required(self):
        self.report['best_experiments'].pop()
        with self.assertRaisesRegex(ValueError, 'repeated'): self.run_export()


if __name__ == '__main__': unittest.main()

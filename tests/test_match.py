import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools/autotune'))
import bench
import match


def trace(equal_counts=False, hold=False, peak=20):
    rows = []
    for t in range(0, 5501, 50):
        rpm = (match.hold_target if hold else bench.target_at)(t, peak)
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
    def test_high_rpm_protocol_and_output_limit(self):
        for peak, limit in ((100, 350), (130, 450)):
            rows = trace(hold=True, peak=peak)
            board = FakeBoard('CAL MANUAL_REF_V1 1080 1320')
            board.replies += ['RPMINFO ELC_RPM_PI_ADAPTIVE_V1', 'STOPPED', 'STATUS 0 3 0 0 0 0 1', 'ARMED 1 1',
                              f'HOLDACK 1 2 1 2 1 {peak} {limit}', 'STATUS 1 2 0 111 5500 0 0', 'BEGIN 1 111',
                              *[','.join(map(str, row)) for row in rows], 'END']
            args = SimpleNamespace(sign_left=1, sign_right=1, kp_left=2., ki_left=1.,
                                   kp_right=2., ki_right=1., peak=float(peak), pwm_limit=limit)
            self.assertEqual(board.run(args, wheel_cpr=(1080, 1320), hold=True), list(map(tuple, rows)))
            self.assertTrue(match.evaluate(rows, peak, limit, (1080, 1320), hold=True)['matched_and_smooth'])

    def test_extended_target_counts_are_bounded_before_arm(self):
        args = SimpleNamespace(sign_left=1, sign_right=1, kp_left=2., ki_left=1.,
                               kp_right=2., ki_right=1., peak=130., pwm_limit=450)
        board = FakeBoard('unused')
        with self.assertRaises(ValueError): board.run(args, wheel_cpr=(1080, 1500), hold=True)
        self.assertFalse(any(s.startswith('ARM') for s in board.sent))

    def test_rpm_unit_conversion_preserves_original_speed_guard(self):
        rows = trace(hold=True, peak=100)
        for row in rows:
            if 500 <= row[0] < 4500:
                row[5] = 105 * 1080 / 6000
        self.assertTrue(match.evaluate(rows, 100, 350, (1080, 1320), hold=True)['trace_valid'])
        rows[50][5] = 101  # physical counts/10ms, not normalized RPM
        with self.assertRaises(ValueError): match.evaluate(rows, 100, 350, (1080, 1320), hold=True)

    def test_90rpm_short_bench_uses_calibrated_targets_and_300_cap(self):
        rows = trace(hold=True, peak=90)
        self.assertTrue(match.evaluate(rows, 90, 300, (1080, 1320), hold=True)['matched_and_smooth'])
        board = FakeBoard('CAL MANUAL_REF_V1 1080 1320')
        board.replies += ['RPMINFO ELC_RPM_PI_ADAPTIVE_V1', 'STOPPED', 'STATUS 0 3 0 0 0 0 1', 'ARMED 1 1',
                          'HOLDACK 1 2 1 2 1 90 300', 'STATUS 1 2 0 111 5500 0 0', 'BEGIN 1 111',
                          *[','.join(map(str, row)) for row in rows], 'END']
        args = SimpleNamespace(sign_left=1, sign_right=1, kp_left=2., ki_left=1.,
                               kp_right=2., ki_right=1., peak=90., pwm_limit=300)
        self.assertEqual(len(board.run(args, wheel_cpr=(1080, 1320), hold=True)), 111)
        self.assertIn('HOLDRPM 1 2.000000 1.000000 2.000000 1.000000 90.0000 300', board.sent)
        with self.assertRaises(ValueError): bench.validate_parameters(args)
        args.pwm_limit = 451
        rejected = FakeBoard('unused')
        with self.assertRaises(ValueError): rejected.run(args, wheel_cpr=(1080, 1320), hold=True)
        self.assertEqual(rejected.sent, [])
        with self.assertRaises(ValueError): bench.validate_parameters(args)

    def test_30rpm_hold_roundtrip_and_limits(self):
        rows = trace(hold=True, peak=30)
        self.assertTrue(match.evaluate(rows, 30, 150, (1080, 1320), hold=True)['matched_and_smooth'])
        board = FakeBoard('CAL MANUAL_REF_V1 1080 1320')
        board.replies += ['RPMINFO ELC_RPM_PI_ADAPTIVE_V1', 'STOPPED', 'STATUS 0 3 0 0 0 0 1', 'ARMED 1 1',
                          'HOLDACK 1 2.000000 1.000000 2.000000 1.000000 30.0000 150',
                          'STATUS 1 2 0 111 5500 0 0', 'BEGIN 1 111',
                          *[','.join(map(str, row)) for row in rows], 'END']
        args = SimpleNamespace(sign_left=1, sign_right=1, kp_left=2., ki_left=1.,
                               kp_right=2., ki_right=1., peak=30., pwm_limit=150)
        self.assertEqual(len(board.run(args, wheel_cpr=(1080, 1320), hold=True)), 111)
        self.assertTrue(any(c.startswith('HOLDRPM ') and '30.0000' in c for c in board.sent))
        for rpm, drive, calibration in ((131., False, (1080, 1320)), (30., True, (1080, 1320)), (26., False, None)):
            args.peak = rpm
            rejected = FakeBoard('unused')
            with self.assertRaises(ValueError):
                rejected.run(args, wheel_cpr=calibration, hold=True, drive=drive)
            self.assertEqual(rejected.sent, [])
        with self.assertRaises(ValueError):
            match.evaluate(rows, 131, 150, (1080, 1320), hold=True)

    def test_normal_drive_contract_and_distinct_ack(self):
        for hold, ack, mode in ((True, 'DRIVEACK', 'DRIVERPM'), (False, 'DRIVESTEPACK', 'DRIVESTEP')):
            board = FakeBoard('CAL MANUAL_REF_V1 1080 1320')
            board.replies += ['RPMINFO ELC_RPM_PI_ADAPTIVE_V1',
                              'DRIVEINFO ELC_DRIVE_RPM_V1 2.000000 1.000000 2.000000 1.000000 20.0000 150 1 1',
                              'STOPPED', 'STATUS 0 3 0 0 0 0 1', 'ARMED 1 1',
                              f'{ack} 1 2.000000 1.000000 2.000000 1.000000 20.0000 150',
                              'STATUS 1 2 0 111 5500 0 0', 'BEGIN 1 111',
                              *[','.join(map(str, row)) for row in trace(hold=hold)], 'END']
            args = SimpleNamespace(sign_left=1, sign_right=1, kp_left=2., ki_left=1.,
                                   kp_right=2., ki_right=1., peak=20., pwm_limit=150)
            self.assertEqual(len(board.run(args, wheel_cpr=(1080, 1320), hold=hold, drive=True)), 111)
            self.assertTrue(any(c.startswith(mode + ' ') for c in board.sent))

    def test_normal_drive_mismatch_never_arms(self):
        args = SimpleNamespace(sign_left=1, sign_right=1, kp_left=2., ki_left=1.,
                               kp_right=2., ki_right=1., peak=20., pwm_limit=150)
        board = FakeBoard('CAL MANUAL_REF_V1 1080 1320')
        board.replies += ['RPMINFO ELC_RPM_PI_ADAPTIVE_V1', 'DRIVEINFO OTHER_CONFIG']
        with self.assertRaisesRegex(RuntimeError, 'configuration'):
            board.run(args, wheel_cpr=(1080, 1320), hold=True, drive=True)
        self.assertFalse(any(c.startswith(('ARM', 'DRIVERPM')) for c in board.sent))
        args.ki_right = .8
        board = FakeBoard('unused')
        with self.assertRaises(ValueError):
            board.run(args, wheel_cpr=(1080, 1320), hold=True, drive=True)
        self.assertEqual(board.sent, [])

    def test_hold_and_step_profiles_cannot_be_confused(self):
        result = match.evaluate(trace(hold=True), 20, 150, (1080, 1320), hold=True)
        self.assertTrue(result['matched_and_smooth'])
        self.assertEqual(result['plateau_ms'], [2000, 4500])
        for rows, hold in ((trace(), True), (trace(hold=True), False)):
            with self.assertRaises(ValueError):
                match.evaluate(rows, 20, 150, (1080, 1320), hold=hold)

    def test_equal_average_cannot_hide_speed_fluctuations(self):
        rows = trace(hold=True)
        for row in rows[40:45]:
            row[5] *= 1.5
            row[6] *= 1.5
        result = match.evaluate(rows, 20, 150, (1080, 1320), hold=True)
        self.assertTrue(result['matched_in_final_plateau'])
        self.assertFalse(result['smooth_in_final_plateau'])
        self.assertFalse(result['matched_and_smooth'])

    def test_hold_protocol_ack(self):
        board = FakeBoard('CAL MANUAL_REF_V1 1080 1320')
        board.replies += ['RPMINFO ELC_RPM_PI_ADAPTIVE_V1', 'STOPPED', 'STATUS 0 3 0 0 0 0 1', 'ARMED 1 1',
                          'HOLDACK 1 2.000000 0.300000 2.000000 0.300000 20.0000 150',
                          'STATUS 1 2 0 111 5500 0 0', 'BEGIN 1 111',
                          *[','.join(map(str, row)) for row in trace(hold=True)], 'END']
        args = SimpleNamespace(sign_left=1, sign_right=1, kp_left=2., ki_left=.3,
                               kp_right=2., ki_right=.3, peak=20., pwm_limit=150)
        rows = board.run(args, wheel_cpr=(1080, 1320), hold=True)
        self.assertEqual(len(rows), 111)
        self.assertTrue(any(c.startswith('HOLDRPM ') for c in board.sent))

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
            board.replies += ['RPMINFO ELC_RPM_PI_ADAPTIVE_V1', 'STOPPED', 'STATUS 0 3 0 0 0 0 1', 'ARMED 1 1',
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

    def test_wrong_feedback_estimator_cannot_arm(self):
        board = FakeBoard('CAL MANUAL_REF_V1 1080 1320')
        board.replies += ['RPMINFO ELC_RPM_PI_IIR_V1']
        args = SimpleNamespace(sign_left=1, sign_right=1, kp_left=2., ki_left=1.,
                               kp_right=2., ki_right=1., peak=20., pwm_limit=150)
        with self.assertRaisesRegex(RuntimeError, 'estimator'):
            board.run(args, wheel_cpr=(1080, 1320), hold=True)
        self.assertFalse(any(c.startswith(('ARM', 'HOLDRPM')) for c in board.sent))


if __name__ == '__main__':
    unittest.main()

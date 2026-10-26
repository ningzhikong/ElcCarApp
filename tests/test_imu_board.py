from pathlib import Path
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import imu_board


def frame(**changes):
    fields = dict(magic=0x494d5531, seq=2, state=2, error=0, tick=1020, samples=1, period=20)
    fields.update({k: v for k, v in changes.items() if k != 'values'})
    values = changes.get('values', [0, 30, 0, 0, -30, 0, 1, 2, -1, 0, 1, 0])
    return struct.pack('<7I12f', *fields.values(), *values)


class ImuBoardTests(unittest.TestCase):
    def test_all_axes_and_sign_preserved(self):
        result = imu_board.decode(frame())
        self.assertEqual(result['angle_deg'], [0, -30, 0])
        self.assertEqual(result['gyro_dps'], [0, 30, 0])
        self.assertEqual(result['bias_dps'], [1, 2, -1])

    def test_wrong_image_or_partial_update_rejected(self):
        for raw in (frame(magic=0), frame(seq=3), frame()[:-1], frame(state=9)):
            with self.assertRaises(ValueError): imu_board.decode(raw)

    def test_invalid_timing_or_nonfinite_rejected(self):
        values = [0.] * 12; values[4] = float('nan')
        for raw in (frame(period=51), frame(period=0), frame(values=values)):
            with self.assertRaises(ValueError): imu_board.decode(raw)


if __name__ == '__main__':
    unittest.main()

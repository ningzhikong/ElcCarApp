from pathlib import Path
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import ground_board


class GroundReadbackTests(unittest.TestCase):
    def test_heading_trace_retains_signed_yaw_and_actual_differential(self):
        sample = bytearray(60)
        struct.pack_into('<I', sample, 0, 2000)
        struct.pack_into('<f', sample, 36, -5.5)
        struct.pack_into('<I', sample, 40, 20)
        struct.pack_into('<f', sample, 44, -1.5)
        struct.pack_into('<f', sample, 56, 88.5)
        row, = ground_board.decode_trace(sample)
        self.assertEqual((row[0], row[9], row[10], row[11], row[14]), (2000, -5.5, 20, -1.5, 88.5))
        self.assertEqual((row[14] - row[11], row[14] + row[11]), (90, 87))

    def test_line_mask_keeps_its_integer_layout(self):
        sample = bytearray(60)
        struct.pack_into('<I', sample, 36, 0xe7)
        row, = ground_board.decode_trace(sample, line=True)
        self.assertEqual(row[9], 0xe7)

    def test_truncated_trace_rejected(self):
        with self.assertRaises(ValueError): ground_board.decode_trace(bytes(59))


if __name__ == '__main__':
    unittest.main()

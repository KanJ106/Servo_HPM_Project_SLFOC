import pathlib
import struct
import unittest
from export_align_trace import decode, HEADER, SAMPLE, HEADER_FIELDS

ROOT = pathlib.Path(__file__).resolve().parents[1]
P = ROOT/'diagnostics/source_foc_check'

class AlignExport(unittest.TestCase):
    def test_fault_fixture_preserves_pretrip_output(self):
        h, r = decode((P/'align_fault_simulated.bin').read_bytes())
        self.assertEqual(h['retained_samples'], 524)
        self.assertEqual(h['omitted_cycles'], 0)
        self.assertFalse(r[-1]['pi_executed'])
        self.assertTrue(r[-1]['previous_output_valid'])
        self.assertEqual(r[-1]['previous_voltage_d'], r[-2]['voltage_d'])
        self.assertAlmostEqual(r[-1]['id_ref'], .03125*r[-1]['stage_cycles']/3200, places=8)
        self.assertEqual(r[-1]['current_failure'], 1)

    def test_late_completion_keeps_two_windows_and_marks_gap(self):
        h,r = decode((P/'align_complete_simulated.bin').read_bytes())
        self.assertEqual(h['total_samples'], 8000)
        self.assertEqual(h['omitted_cycles'], 7456)
        self.assertEqual(r[128]['gap_cycles'], 7456)
        self.assertFalse(r[128]['previous_output_valid'])
        self.assertTrue(all(x['gap_cycles']==0 for x in r[129:]))

    def test_reject_invalid_header(self):
        data=(P/'align_fault_simulated.bin').read_bytes()
        for field,value in [('active',1),('frozen',0),('sequence',3),('abi',2),
                            ('first_count',127),('tail_count',900),('tail_next',999)]:
            bad=bytearray(data)
            struct.pack_into('<I',bad,4*HEADER_FIELDS.index(field),value)
            with self.subTest(field=field),self.assertRaises(ValueError):decode(bad)

    def test_reject_internal_gap_and_wrong_union_length(self):
        data=(P/'align_fault_simulated.bin').read_bytes()
        bad=bytearray(data)
        struct.pack_into('<I',bad,HEADER.size+SAMPLE.size,123456)
        with self.assertRaises(ValueError):decode(bad)
        with self.assertRaises(ValueError):decode(data+b'\0'*172)

if __name__=='__main__':unittest.main()

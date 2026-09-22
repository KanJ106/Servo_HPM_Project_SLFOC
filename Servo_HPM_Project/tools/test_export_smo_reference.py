import unittest
import struct
from export_smo_trace import HEADER_V2, SAMPLE_V2, PARAMS, FIELDS_V2, decode

def fixture():
    h=[0x534D5431,2,192,160,2,160,1,0,1,32000,1,100000,90000,90000,0,40000,45088,32,64,5952]
    data=bytearray(HEADER_V2.pack(*h)+PARAMS.pack(1/16000,*([0.0]*16),800,160)+bytes(224*192))
    for base,count,state,stage,tick in [(0,160,4,32000,40000),(160,64,1,5952,13952)]:
        for i in range(count):
            r=dict.fromkeys(FIELDS_V2,0)
            r.update(tick=tick+32*i,stage_cycles=stage+32*i,state=state,direction=-1,
                     encoder_sequence=2+4*i,encoder_raw=(131070+i)%131072,encoder_valid=1)
            SAMPLE_V2.pack_into(data,152+(base+i)*192,*[r[k] for k in FIELDS_V2])
    return data

class Tests(unittest.TestCase):
    def test_complete_window_and_alignment(self):
        h,r=decode(fixture())
        self.assertEqual((len(r),len(h['alignment_samples'])),(160,64))
        self.assertAlmostEqual(h['window_span_ms'],318,places=4)
        self.assertAlmostEqual(h['longest_locked_ms'],5625,places=3)
        self.assertEqual(r[2]['encoder_raw'],0)
    def test_gap_rejected(self):
        b=fixture();struct.pack_into('<I',b,152+192,40033)
        with self.assertRaisesRegex(ValueError,'gap'):decode(b)
    def test_torn_encoder_rejected(self):
        b=fixture();struct.pack_into('<I',b,152+44,3)
        with self.assertRaisesRegex(ValueError,'encoder'):decode(b)
    def test_live_and_bounds_rejected(self):
        for index,value in [(7,1),(18,65),(17,16),(5,161)]:
            b=fixture();struct.pack_into('<I',b,index*4,value)
            with self.assertRaises(ValueError):decode(b)

if __name__=='__main__':unittest.main()

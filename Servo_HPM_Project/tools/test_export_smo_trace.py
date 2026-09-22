import unittest
from export_smo_trace import HEADER,PARAMS,SAMPLE,decode

def fixture():
    h=[0x534D5431,1,128,256,2,2,1,0,1,32000,1,1000,900,900,0,100,101]
    data=bytearray(HEADER.pack(*h)+PARAMS.pack(1/16000,*([0.0]*16),800,160)+bytes(256*128))
    for i in range(2):
        SAMPLE.pack_into(data,140+i*128,100+i,32000+i,4,-1,*([0]*11),*([0.0]*17))
    return data

class Tests(unittest.TestCase):
    def test_valid(self):
        h,r=decode(fixture());self.assertEqual(r[0]['direction'],-1)
        self.assertAlmostEqual(h['longest_locked_ms'],56.25,places=4)
    def test_live_or_torn(self):
        for index,value in [(4,3),(7,1),(6,0)]:
            b=fixture();b[index*4:index*4+4]=value.to_bytes(4,'little')
            with self.assertRaises(ValueError):decode(b)
    def test_bad_bounds_or_layout(self):
        for index,value in [(0,0),(1,2),(2,64),(3,128),(5,257),(12,1001)]:
            b=fixture();b[index*4:index*4+4]=value.to_bytes(4,'little')
            with self.assertRaises(ValueError):decode(b)
    def test_missing_cycle(self):
        b=fixture();b[268:272]=(102).to_bytes(4,'little')
        with self.assertRaises(ValueError):decode(b)
    def test_size(self):
        with self.assertRaises(ValueError):decode(fixture()[:-1])
if __name__=='__main__':unittest.main()

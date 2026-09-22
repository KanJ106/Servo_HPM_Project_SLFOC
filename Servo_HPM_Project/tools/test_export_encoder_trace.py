import struct, unittest
from export_encoder_trace import decode, HEADER, POINT, CAPACITY
def fixture(overflow=0, flags=3):
    h=[0]*41
    h[0]=2; h[1]=2; h[19]=1; h[20]=h[21]=h[22]=1
    h[23]=100; h[37]=3; h[38]=overflow
    data=bytearray(HEADER.pack(*h)+bytes(CAPACITY*POINT.size))
    for n,p in enumerate([(101,131060,524280,7),(106,20,2,flags|4),(111,52,12,3)]):
        POINT.pack_into(data,HEADER.size+n*POINT.size,*p)
    return data
class TestExport(unittest.TestCase):
    def test_wrap_and_speed(self):
        meta,rows=decode(fixture())
        self.assertTrue(meta["complete"])
        self.assertFalse(rows[0]["speed_valid"])
        self.assertAlmostEqual(rows[1]["motor_rpm"],32*60000/(131072*5))
        self.assertAlmostEqual(rows[1]["output_rpm"],10*60000/(524288*5))
    def test_invalid_interval(self):
        _,rows=decode(fixture(flags=1))
        self.assertFalse(rows[1]["speed_valid"])
        self.assertEqual(rows[1]["motor_rpm"],"")
        self.assertTrue(rows[2]["speed_valid"])
    def test_overflow(self):
        meta,_=decode(fixture(overflow=1)); self.assertFalse(meta["complete"])
    def test_reject(self):
        for offset,value in [(0,1),(4,3),(18*4,1),(37*4,513)]:
            data=fixture(); struct.pack_into("<I",data,offset,value)
            with self.assertRaises(ValueError): decode(data)
        with self.assertRaises(ValueError): decode(fixture()[:-1])
if __name__=="__main__": unittest.main()

import struct,unittest
from export_startup_timing import decode
class TimingTest(unittest.TestCase):
 def fixture(self):
  v=[0]*221
  v[:6]=[0x53544d31,1,2,600000000,16000,37500]
  v[9:19]=[7,4,4,1,52255,100,40000,12000,50,30]
  v[29+11*6:35+11*6]=[20,30000,52255,1,7,7]
  return v
 def test_units_and_bucket(self):
  r=decode(struct.pack("<221I",*self.fixture()))
  self.assertAlmostEqual(r["worst"]["elapsed_us"],87.09166667)
  self.assertEqual(r["bins"][0]["entry_state"],4)
  self.assertTrue(r["bins"][0]["trace_written"])
  self.assertEqual(r["period_us"],62.5)
 def test_reject(self):
  for index,value in [(0,0),(1,2),(2,3),(3,0),(4,0),(5,1)]:
   v=self.fixture();v[index]=value
   with self.assertRaises(ValueError):decode(struct.pack("<221I",*v))
  with self.assertRaises(ValueError):decode(b"")
 def test_actual_clock(self):
  v=self.fixture();v[3]=480000000;v[5]=30000
  self.assertEqual(decode(struct.pack("<221I",*v))["budget_cycles"],30000)
 def test_v3_commit(self):
  v=[0]*236;v[:6]=[0x53544d31,3,2,600000000,16000,37500]
  v[44+30*6:50+30*6]=[1,36200,36200,0,44000,0]
  r=decode(struct.pack("<236I",*v))
  self.assertEqual(r["bins"][0]["event_kind"],"frame_commit")
  self.assertEqual(r["bins"][0]["count"],1)
  self.assertAlmostEqual(r["bins"][0]["last_us"],60.333333333)
 def test_v2_detail(self):
  old=self.fixture();v=old[:9]+[0]*5+old[9:19]+[1,2,300,4,5]+old[19:29]+[0]*5+old[29:]
  v[1]=2
  r=decode(struct.pack("<236I",*v))
  self.assertEqual(r["worst"]["current_pi"],300)
  self.assertEqual(r["bins"][0]["entry_state"],4)
  v[1]=1
  with self.assertRaises(ValueError):decode(struct.pack("<236I",*v))
if __name__=="__main__":unittest.main()

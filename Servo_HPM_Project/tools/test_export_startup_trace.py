import struct, unittest
from export_startup_trace import decode, HEADER, SAMPLE, SAMPLE_V2, SAMPLE_V3

class TraceTest(unittest.TestCase):
    def fixture(self, abi):
        sample = {1:SAMPLE, 2:SAMPLE_V2, 3:SAMPLE_V3}[abi]
        data = bytearray(HEADER.pack(0x534c5437, abi, sample.size, 128, 2, 1, 1, 1, 0, 7))
        row = bytearray(sample.size)
        struct.pack_into("<I", row, 0, 160)
        if abi >= 2:
            struct.pack_into("<6I", row, SAMPLE.size, 123, 131000, 524000, 1, 2, 123)
        return data + row + bytes(sample.size * 127)
    def test_old_layout(self):
        meta, rows = decode(self.fixture(1))
        self.assertEqual(meta["trace_abi"], 1)
        self.assertNotIn("encoder_valid", rows[0])
    def test_chain_layout(self):
        data = self.fixture(3)
        struct.pack_into("<4i", data, HEADER.size+SAMPLE_V2.size, 159, 1, -1234, 2345)
        meta, rows = decode(data)
        self.assertEqual(meta["trace_abi"], 3)
        self.assertEqual(rows[0]["chain_tick"], 159)
        self.assertEqual(rows[0]["sdm_u"], -1234)
        self.assertEqual(rows[0]["sdm_v"], 2345)
        self.assertEqual(rows[0]["encoder_valid"], 1)
        struct.pack_into("<I", data, 8, SAMPLE_V2.size)
        with self.assertRaises(ValueError): decode(data)
    def test_new_layout(self):
        meta, rows = decode(self.fixture(2))
        self.assertEqual(meta["trace_abi"], 2)
        self.assertEqual(rows[0]["encoder_motor_raw"], 131000)
        self.assertEqual(rows[0]["encoder_valid"], 1)
        self.assertEqual(rows[0]["encoder_last_good_ms"], 123)
    def test_reject_wrong_stride_or_live(self):
        for offset, value in [(8,124), (16,3), (28,0), (32,1), (4,99)]:
            data = self.fixture(2)
            struct.pack_into("<I",data,offset,value)
            with self.assertRaises(ValueError): decode(data)
    def test_ring_order(self):
        data = self.fixture(2)
        struct.pack_into("<I",data,24,128)
        for i in range(128): struct.pack_into("<I",data,40+i*148,i)
        _, rows=decode(data)
        self.assertEqual([r["tick"] for r in rows],list(range(1,128))+[0])
if __name__ == "__main__": unittest.main()

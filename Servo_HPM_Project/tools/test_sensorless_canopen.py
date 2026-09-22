import pathlib
import struct
import unittest
import sensorless_canopen as sl

class WireTests(unittest.TestCase):
    def test_expedited_and_signed_float(self):
        s=sl.Sdo(lambda tx: struct.pack("<BHBI",0x43,0x2f00,1,1))
        self.assertEqual(s.read(0x2f00,1),1)
        self.assertEqual(sl.decode(sl.encode(-123,"int32_t"),"int32_t"),-123)
        self.assertEqual(sl.decode(sl.encode(0.25,"float"),"float"),0.25)
        with self.assertRaises(ValueError): sl.encode(float("nan"),"float")
        with self.assertRaises(ValueError): sl.encode(65536,"uint16_t")

    def test_segmented_trace_roundtrip(self):
        path=pathlib.Path(__file__).resolve().parents[1]/"diagnostics/source_foc_check/debug_chain_fault_simulated.bin"
        payload=path.read_bytes()
        cursor=0
        toggle=0
        def exchange(tx):
            nonlocal cursor,toggle
            if tx[0]==0x40:
                return struct.pack("<BHBI",0x41,0x2f07,0,len(payload))
            self.assertEqual(tx[0],0x60|(toggle<<4))
            chunk=payload[cursor:cursor+7];cursor+=len(chunk)
            last=cursor==len(payload)
            response=bytes([(toggle<<4)|((7-len(chunk))<<1)|int(last)])+chunk+bytes(7-len(chunk))
            toggle^=1
            return response
        self.assertEqual(sl.Sdo(exchange).upload(0x2f07),payload)

    def test_abort_object_and_toggle_errors(self):
        with self.assertRaises(sl.SdoError):
            sl.Sdo(lambda tx: struct.pack("<BHBI",0x80,0x2f00,1,0x08000022)).read(0x2f00,1)
        with self.assertRaises(sl.SdoError):
            sl.Sdo(lambda tx: struct.pack("<BHBI",0x43,0x2f01,1,1)).read(0x2f00,1)
        responses=iter([struct.pack("<BHBI",0x41,0x2f07,0,1),bytes([0x1d,42,0,0,0,0,0,0])])
        with self.assertRaises(sl.SdoError):
            sl.Sdo(lambda tx: next(responses)).upload(0x2f07)

    def test_recipe_validation_before_first_write(self):
        class Fake:
            writes=[]
            def read(self,i,s): return 1 if s==1 else 0
            def write(self,*args): self.writes.append(args)
        f=Fake();client=sl.Client(f);before=len(f.writes)
        with self.assertRaises(ValueError):
            client.stage(0x2f01,{"phase_rs_ohm":float("nan")})
        self.assertEqual(len(f.writes),before)

    def test_uncertain_start_still_stops(self):
        class Fake:
            def write(self,*args): pass
        c=object.__new__(sl.Client);c.sdo=Fake();c.pulse=lambda:None
        seen=[]
        def command(value):
            seen.append(value)
            if value==3: raise sl.SdoError("lost start acknowledgement")
        c.command=command
        with self.assertRaises(sl.SdoError):c.run(100,1,1)
        self.assertEqual(seen,[3,5])

if __name__=="__main__":
    unittest.main()


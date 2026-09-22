"""CANopen ABI 1 host client. No automatic platform enable or firmware download."""
import argparse
import json
import math
import pathlib
import struct
import time

SCHEMA = json.loads(pathlib.Path(__file__).with_name("sensorless_canopen_schema.json").read_text())
OBJECTS = {int(k, 16): v for k, v in SCHEMA["objects"].items()}

class SdoError(RuntimeError):
    pass

class Sdo:
    """One client on one node; exchange(tx_bytes) returns an 8-byte SDO response."""
    def __init__(self, exchange):
        self.exchange = exchange

    def request(self, payload, index=None, sub=None):
        response = bytes(self.exchange(bytes(payload)))
        if len(response) != 8:
            raise SdoError("invalid response length")
        if response[0] == 0x80:
            raise SdoError("SDO abort 0x%08X" % struct.unpack_from("<I", response, 4)[0])
        if index is not None and response[1:4] != struct.pack("<HB", index, sub):
            raise SdoError("response object does not match request")
        return response

    def write(self, index, sub, value):
        r = self.request(struct.pack("<BHBI", 0x23, index, sub, value & 0xFFFFFFFF), index, sub)
        if r[0] != 0x60:
            raise SdoError("invalid download acknowledgement")

    def upload(self, index, sub=0):
        r = self.request(struct.pack("<BHB4x", 0x40, index, sub), index, sub)
        if r[0] & 0xE0 != 0x40:
            raise SdoError("invalid upload initiation")
        if r[0] & 2:
            if not r[0] & 1:
                raise SdoError("expedited upload has no size")
            return r[4:8-((r[0] >> 2) & 3)]
        size = struct.unpack_from("<I", r, 4)[0] if r[0] & 1 else None
        if size is not None and size > 65536:
            raise SdoError("upload exceeds bound")
        out, toggle = bytearray(), 0
        while True:
            r = self.request(bytes([0x60 | toggle << 4]) + bytes(7))
            if r[0] & 0xE0 or (r[0] >> 4) & 1 != toggle:
                raise SdoError("invalid segment/toggle")
            last, unused = r[0] & 1, (r[0] >> 1) & 7
            if unused and not last:
                raise SdoError("short non-final segment")
            out.extend(r[1:8-unused])
            if len(out) > 65536:
                raise SdoError("upload exceeds bound")
            if last:
                break
            toggle ^= 1
        if size is not None and len(out) != size:
            raise SdoError("upload size mismatch")
        return bytes(out)

    def read(self, index, sub):
        b = self.upload(index, sub)
        if len(b) != 4:
            raise SdoError("expected 32-bit scalar")
        return struct.unpack("<I", b)[0]

def encode(value, kind):
    if kind == "float":
        value = float(value)
        if not math.isfinite(value):
            raise ValueError("non-finite parameter")
        return struct.unpack("<I", struct.pack("<f", value))[0]
    value = int(value, 0) if isinstance(value, str) else int(value)
    bounds = {"uint16_t": (0,65535), "int16_t": (-32768,32767),
              "int32_t": (-2147483648,2147483647), "uint32_t": (0,4294967295)}
    lo, hi = bounds[kind]
    if not lo <= value <= hi:
        raise ValueError("parameter out of range")
    return value & 0xFFFFFFFF

def decode(value, kind):
    if kind == "float":
        return struct.unpack("<f", struct.pack("<I", value))[0]
    return value-0x100000000 if kind.startswith("int") and value & 0x80000000 else value

class Client:
    def __init__(self, sdo):
        self.sdo = sdo
        if sdo.read(0x2F00, 1) != 1:
            raise SdoError("unsupported sensorless CAN protocol")
        self.heartbeat = sdo.read(0x2F00, 9)
        sdo.write(0x2F00, 1, 1)

    def pulse(self):
        self.heartbeat = (self.heartbeat + 1) & 0xFFFFFFFF
        self.sdo.write(0x2F00, 9, self.heartbeat)

    def command(self, command):
        if self.sdo.read(0x2F00, 12):
            raise SdoError("firmware command pending")
        seq = (self.sdo.read(0x2F00, 2) + 1) & 0xFFFFFFFF or 1
        self.sdo.write(0x2F00, 3, command)
        self.sdo.write(0x2F00, 2, seq)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            if self.sdo.read(0x2F00, 7) == seq:
                result = self.sdo.read(0x2F00, 8)
                if result != 1:
                    raise SdoError("command %d rejected, result=%d" % (command, result))
                return
            time.sleep(0.005)
        raise SdoError("command acknowledgement timeout; do not resend START blindly")

    def read_object(self, index):
        return {f["name"]: decode(self.sdo.read(index,f["sub"]),f["type"]) for f in OBJECTS[index]}

    def stage(self, index, values, complete=False):
        fields = {f["name"]: f for f in OBJECTS[index]}
        if set(values)-set(fields) or (complete and set(values)!=set(fields)):
            raise ValueError("unknown or missing fields")
        # Validate the entire file before sending its first field.
        prepared = [(fields[k]["sub"],encode(v,fields[k]["type"])) for k,v in values.items()]
        for sub, value in prepared:
            self.sdo.write(index, sub, value)

    def readback(self):
        for _ in range(3):
            before = self.sdo.read(0x2F00,15)
            data = {name:self.read_object(idx) for idx,name in
                    [(0x2F03,"recipe"),(0x2F04,"startup"),(0x2F05,"smo")]}
            after = self.sdo.read(0x2F00,15)
            if before == after and not after & 1:
                data["parameter_sequence"] = after
                return data
        raise SdoError("parameters changed during readback")

    def run(self, rpm, direction, seconds):
        if not 0 < seconds <= 60:
            raise ValueError("seconds must be in (0, 60]")
        self.sdo.write(0x2F00, 5, encode(direction,"int32_t"))
        self.sdo.write(0x2F00, 6, encode(rpm,"int32_t"))
        self.pulse()
        self.sdo.write(0x2F00, 4, 0x534C)
        # Even an uncertain START acknowledgement must be followed by STOP.
        try:
            self.command(3)
            end = time.monotonic()+seconds
            while time.monotonic()<end:
                self.pulse()
                if self.sdo.read(0x2F06,9):
                    return
                time.sleep(0.2)
        finally:
            self.command(5)

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--interface", required=True, help="python-can interface matching your adapter")
    p.add_argument("--channel", required=True)
    p.add_argument("--bitrate", type=int, choices=[500000,1000000], required=True)
    p.add_argument("--node", type=int, required=True)
    sub = p.add_subparsers(dest="action", required=True)
    sub.add_parser("status")
    a = sub.add_parser("apply"); a.add_argument("recipe",type=pathlib.Path);a.add_argument("--tuning",type=pathlib.Path)
    a = sub.add_parser("start");a.add_argument("--rpm",type=int,required=True);a.add_argument("--direction",type=int,choices=[-1,1],default=1);a.add_argument("--seconds",type=float,required=True)
    sub.add_parser("stop");sub.add_parser("clear")
    a = sub.add_parser("dump");a.add_argument("output",type=pathlib.Path)
    args = p.parse_args()
    if not 1 <= args.node <= 127:
        p.error("node must be 1..127")
    try:
        import can
    except ImportError:
        p.error("python-can is required for hardware access; offline tests need only Python")
    bus = can.Bus(interface=args.interface,channel=args.channel,bitrate=args.bitrate,
                  can_filters=[{"can_id":0x580+args.node,"can_mask":0x7FF,"extended":False}])
    def exchange(data):
        bus.send(can.Message(arbitration_id=0x600+args.node,is_extended_id=False,data=data),timeout=1)
        until=time.monotonic()+1
        while time.monotonic()<until:
            msg=bus.recv(max(0,until-time.monotonic()))
            if msg is not None and not msg.is_extended_id and not msg.is_error_frame and not msg.is_remote_frame and msg.arbitration_id==0x580+args.node:
                return msg.data
        raise SdoError("CAN SDO response timeout")
    try:
        client=Client(Sdo(exchange))
        if args.action=="status":
            print(json.dumps({"control":client.read_object(0x2F00),"status":client.read_object(0x2F06),
                              "effective":client.readback()},indent=2))
        elif args.action=="apply":
            recipe=json.loads(args.recipe.read_text(encoding="utf-8-sig"))
            client.stage(0x2F01,recipe.get("source_recipe",recipe),True)
            client.command(12)
            if args.tuning:
                tuning=json.loads(args.tuning.read_text(encoding="utf-8-sig"))
                client.stage(0x2F02,tuning.get("startup_debug",tuning))
                client.command(2)
            print(json.dumps(client.readback(),indent=2))
        elif args.action=="start":
            client.run(args.rpm,args.direction,args.seconds)
            print(json.dumps(client.read_object(0x2F06),indent=2))
        elif args.action in ("stop","clear"):
            client.command(5 if args.action=="stop" else 6)
        else:
            before=client.sdo.read(0x2F06,8)
            data=client.sdo.upload(0x2F07)
            after=client.sdo.read(0x2F06,8)
            from export_startup_trace import decode as decode_trace
            meta,_=decode_trace(data)
            if before!=after or meta["record_id"]!=after:
                raise SdoError("record changed while exporting")
            args.output.write_bytes(data)
            metadata={"provenance":"hardware","node":args.node,"bitrate":args.bitrate,
                      "firmware_version":client.sdo.read(0x2F00,13),
                      "effective":client.readback(),"trace":meta}
            args.output.with_suffix(".session.json").write_text(json.dumps(metadata,indent=2),encoding="utf-8")
            print("Saved frozen trace:",args.output)
    finally:
        bus.shutdown()

if __name__ == "__main__":
    main()


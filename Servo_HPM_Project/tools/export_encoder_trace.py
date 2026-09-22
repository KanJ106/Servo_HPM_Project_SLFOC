"""Export ABI-2 read-only encoder trajectory; never write target memory."""
import argparse, csv, json, pathlib, struct
HEADER = struct.Struct("<29I6i6I")
POINT = struct.Struct("<4I")
CAPACITY = 512

def delta(now, before, modulus):
    d = now - before
    if d > modulus // 2: d -= modulus
    if d < -modulus // 2: d += modulus
    return d

def decode(data):
    if len(data) != HEADER.size + CAPACITY * POINT.size:
        raise ValueError("wrong size; use matching ELF and encoder ABI 2")
    h = HEADER.unpack_from(data)
    count, overflow = h[37:39]
    if h[0] != 2 or h[1] & 1 or h[18]:
        raise ValueError("incompatible ABI, incoherent snapshot or drive active")
    if not h[19] or not 2 <= count <= CAPACITY:
        raise ValueError("no completed trajectory")
    rows, prev = [], None
    for i in range(count):
        t,m,o,flags = POINT.unpack_from(data, HEADER.size+i*POINT.size)
        if m >= 131072 or o >= 524288 or flags & ~7:
            raise ValueError("invalid raw position or flags")
        dt = ((t-prev[0]) & 0xffffffff) if prev else 0
        valid = bool(prev and 0 < dt <= 5 and (prev[3]&1) and (flags&3)==3)
        row = dict(tick_ms=t, relative_ms=(t-h[23])&0xffffffff,
            motor_raw=m, output_raw=o, position_valid=int(bool(flags&1)),
            interval_valid=int(bool(flags&2)), drive_active=int(bool(flags&4)),
            speed_valid=int(valid), motor_rpm="", output_rpm="")
        if valid:
            row["motor_rpm"] = delta(m,prev[1],131072)*60000/(131072*dt)
            row["output_rpm"] = delta(o,prev[2],524288)*60000/(524288*dt)
        rows.append(row); prev=(t,m,o,flags)
    if rows[-1]["drive_active"]:
        raise ValueError("missing terminal sample")
    meta=dict(abi=2, record_id=h[19], samples=count, overflow=bool(overflow),
        run_valid=bool(h[22]), run_errors=h[36],
        complete=bool(not overflow and h[20] and h[21] and h[22]),
        speed_source="encoder raw finite difference; not observer",
        period_ms=5, duration_ms=rows[-1]["relative_ms"])
    return meta, rows

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("dump",type=pathlib.Path)
    p.add_argument("--provenance",required=True,choices=["hardware","simulated"])
    a=p.parse_args()
    meta,rows=decode(a.dump.read_bytes()); meta["provenance"]=a.provenance
    with a.dump.with_suffix(".csv").open("w",newline="",encoding="utf-8-sig") as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
    a.dump.with_suffix(".json").write_text(json.dumps(meta,indent=2),encoding="utf-8")
    print(json.dumps(meta))
if __name__=="__main__": main()

"""Decode frozen SLT7 memory exported with symbols from the matching ELF."""
import argparse, csv, json, pathlib, struct
FIELDS = "tick mode state stage_cycles total_cycles first_fault open_theta_q24 observer_theta_q24 control_theta_q24 angle_error_q24 speed_ref_rpm observer_speed_rpm reference_speed_rpm reference_valid id_ref iq_ref id_fb iq_fb id_limited iq_limited bus_pu ud uq emf phase_error_rad blend saturated quality locked lock_reason current_failure".split()
HEADER = struct.Struct("<10I")
SAMPLE = struct.Struct("<6I4i3fI12f5I")
ENCODER_FIELDS = "encoder_tick_ms encoder_motor_raw encoder_output_raw encoder_valid encoder_sequence encoder_last_good_ms".split()
SAMPLE_V2 = struct.Struct("<6I4i3fI12f11I")
CHAIN_FIELDS = "chain_tick chain_valid sdm_u sdm_v adc_u adc_v adc_w zero_u zero_v zero_w iu_q24 iv_q24 iw_q24 clark_u_q24 clark_v_q24 alpha_q24 beta_q24 sample_window phase_mode ualpha_q24 ubeta_q24 pwm_u pwm_v pwm_w pwm_period duty_a_q24 duty_b_q24 duty_c_q24 bus_reference_q24".split()
SAMPLE_V3 = struct.Struct("<6I4i3fI12f11I29i")
def decode(data):
    if len(data) < HEADER.size: raise ValueError("short trace header")
    magic, abi, stride, cap, seq, pos, count, frozen, active, record = HEADER.unpack_from(data)
    sample = {1: SAMPLE, 2: SAMPLE_V2, 3: SAMPLE_V3}.get(abi)
    fields = FIELDS + (ENCODER_FIELDS if abi >= 2 else []) + (CHAIN_FIELDS if abi == 3 else [])
    if magic != 0x534c5437 or sample is None or stride != sample.size or cap != 128:
        raise ValueError("incompatible trace layout; use the matching trace ELF")
    if seq & 1 or not frozen or active: raise ValueError("trace not frozen/coherent")
    if count > cap or pos >= cap or len(data) != HEADER.size + cap * stride:
        raise ValueError("invalid trace length or ring bounds")
    rows = []
    for n in range(count):
        slot = (pos - count + n) % cap
        rows.append(dict(zip(fields, sample.unpack_from(data, HEADER.size + slot * stride))))
    return {"record_id": record, "samples": count, "trace_abi": abi,
            "first_fault": next((r["first_fault"] for r in rows if r["first_fault"]), 0)}, rows
def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("dump", type=pathlib.Path)
    p.add_argument("--sample-hz", required=True, type=float)
    p.add_argument("--provenance", required=True, choices=["simulated", "hardware"])
    p.add_argument("--output", type=pathlib.Path)
    a = p.parse_args()
    if not 1000 <= a.sample_hz <= 1000000: p.error("invalid sample rate")
    meta, rows = decode(a.dump.read_bytes())
    for r in rows:
        r["stage_ms"] = r["stage_cycles"] * 1000 / a.sample_hz
        r["total_ms"] = r["total_cycles"] * 1000 / a.sample_hz
        r["phase_error_deg"] = r["phase_error_rad"] * 180 / 3.141592653589793
    out = a.output or a.dump.with_suffix(".csv")
    with out.open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS+(ENCODER_FIELDS if meta["trace_abi"] >= 2 else [])+(CHAIN_FIELDS if meta["trace_abi"] == 3 else [])+["stage_ms","total_ms","phase_error_deg"])
        w.writeheader(); w.writerows(rows)
    meta.update(sample_hz=a.sample_hz, provenance=a.provenance, source=str(a.dump))
    out.with_suffix(".json").write_text(json.dumps(meta,indent=2),encoding="utf-8")
    print(json.dumps(meta))
if __name__ == "__main__": main()

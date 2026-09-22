"""Decode a stopped SMT1 observer window from its matching ELF symbol dump."""
import argparse
import csv
import json
import math
from pathlib import Path
import struct

HEADER_FIELDS='magic abi sample_bytes capacity sequence count frozen active record_id trigger_stage_cycles stop_reason total_samples locked_samples longest_locked_run locked_run first_tick last_tick'.split()
PARAM_FIELDS='sample_time_s stator_resistance_pu current_model_gain switching_gain boundary_pu emf_lpf_alpha emf_delay_comp_gain pll_kp pll_ki pll_speed_lpf_alpha weak_emf_speed_decay min_emf_pu unlock_emf_ratio lock_phase_error max_electrical_rad_s max_accel_e_rad_s2 current_hat_limit_pu lock_cycles unlock_cycles'.split()
FIELDS='tick stage_cycles state direction chain_tick chain_valid sample_window pwm_u pwm_v pwm_w pwm_period locked lock_reason lock_counter unlock_counter voltage_alpha voltage_beta current_alpha current_beta current_hat_alpha current_hat_beta switching_alpha switching_beta emf_alpha emf_beta theta_raw theta_pll omega_tracking omega_filtered phase_error_rad emf_magnitude phase_advance'.split()
HEADER=struct.Struct('<17I')
PARAMS=struct.Struct('<17f2H')
SAMPLE=struct.Struct('<3Ii11I17f')
HEADER_V2=struct.Struct('<20I')
SAMPLE_V2=struct.Struct('<3Ii13I2i6f6I17f')
FIELDS_V2=FIELDS[:11]+'encoder_sequence encoder_raw encoder_tick_ms encoder_request_tick encoder_read_tick encoder_valid forced_theta_q24 control_theta_q24 id_ref iq_ref id_fb iq_fb ud uq saturated current_failure'.split()+FIELDS[11:]

def decode_v2(data):
    if len(data)!=HEADER_V2.size+PARAMS.size+(160+64)*SAMPLE_V2.size:
        raise ValueError('wrong SMT1 ABI 2 dump size')
    h=dict(zip(HEADER_FIELDS+['sample_divider','align_count','align_trigger_cycles'],HEADER_V2.unpack_from(data)))
    if (h['magic'],h['abi'],h['sample_bytes'],h['capacity'],h['sample_divider'])!=(0x534D5431,2,192,160,32):
        raise ValueError('incompatible SMT1 ABI 2 layout')
    if h['sequence']&1 or h['active'] or not h['frozen']:
        raise ValueError('SMT1 attempt is not stopped/coherent')
    if h['count']>160 or h['align_count']>64 or h['locked_samples']>h['total_samples'] or h['longest_locked_run']>h['locked_samples']:
        raise ValueError('invalid SMT1 ABI 2 bounds/counters')
    p=dict(zip(PARAM_FIELDS,PARAMS.unpack_from(data,HEADER_V2.size)))
    if not math.isfinite(p['sample_time_s']) or not .000001<=p['sample_time_s']<=.001:
        raise ValueError('invalid sample period')
    offset=HEADER_V2.size+PARAMS.size
    def rows_at(start,count,state):
        rows=[dict(zip(FIELDS_V2,SAMPLE_V2.unpack_from(data,start+i*SAMPLE_V2.size))) for i in range(count)]
        for r in rows:
            if r['state']!=state:
                raise ValueError('unexpected capture state')
            if not all(math.isfinite(r[k]) for k in ['id_ref','iq_ref','id_fb','iq_fb','ud','uq']+FIELDS[-17:]):
                raise ValueError('nonfinite captured value')
            if r['encoder_valid'] and (r['encoder_raw']>=131072 or r['encoder_sequence']&1):
                raise ValueError('invalid encoder snapshot marked valid')
        for a,b in zip(rows,rows[1:]):
            if ((b['tick']-a['tick'])&0xffffffff)!=32 or b['stage_cycles']!=a['stage_cycles']+32:
                raise ValueError('gap in decimated observer window')
        return rows
    rows=rows_at(offset,h['count'],4)
    align=rows_at(offset+160*SAMPLE_V2.size,h['align_count'],1)
    if rows and (rows[0]['tick']!=h['first_tick'] or rows[-1]['tick']!=h['last_tick']):
        raise ValueError('window bounds do not match samples')
    h.update(params=p,sample_hz=1/(32*p['sample_time_s']),control_hz=1/p['sample_time_s'],
             longest_locked_ms=h['longest_locked_run']*p['sample_time_s']*1000,
             window_span_ms=((h['last_tick']-h['first_tick'])&0xffffffff)*p['sample_time_s']*1000 if rows else 0,
             alignment_samples=align,
             scope='Post-SMO software snapshot. Encoder request/FIFO service read ticks are host timestamps, NOT physical latch timestamps. ALIGN samples are candidate zero evidence, not certified calibration.')
    return h,rows

def decode(data):
    if len(data)>=8 and struct.unpack_from('<I',data,4)[0]==2:
        return decode_v2(data)
    if len(data)!=HEADER.size+PARAMS.size+256*SAMPLE.size:
        raise ValueError('wrong SMT1 dump size')
    h=dict(zip(HEADER_FIELDS,HEADER.unpack_from(data)))
    if h['magic']!=0x534D5431 or h['abi']!=1 or h['sample_bytes']!=128 or h['capacity']!=256:
        raise ValueError('incompatible SMT1 layout')
    if h['sequence']&1 or h['active'] or not h['frozen']:
        raise ValueError('SMT1 attempt is not stopped/coherent; frozen window alone is insufficient')
    if h['count']>256 or h['locked_samples']>h['total_samples'] or h['longest_locked_run']>h['locked_samples']:
        raise ValueError('invalid SMT1 bounds/counters')
    p=dict(zip(PARAM_FIELDS,PARAMS.unpack_from(data,HEADER.size)))
    if not math.isfinite(p['sample_time_s']) or not .000001<=p['sample_time_s']<=.001:
        raise ValueError('invalid sample period')
    offset=HEADER.size+PARAMS.size
    rows=[dict(zip(FIELDS,SAMPLE.unpack_from(data,offset+i*SAMPLE.size))) for i in range(h['count'])]
    if rows and (rows[0]['tick']!=h['first_tick'] or rows[-1]['tick']!=h['last_tick']):
        raise ValueError('window bounds do not match samples')
    for a,b in zip(rows,rows[1:]):
        if ((b['tick']-a['tick'])&0xffffffff)!=1 or b['stage_cycles']!=a['stage_cycles']+1:
            raise ValueError('nonconsecutive observer window')
    h['params']=p
    h['sample_hz']=1/p['sample_time_s']
    h['longest_locked_ms']=h['longest_locked_run']*p['sample_time_s']*1000
    h['scope']='Synchronous software snapshot after SmoPll_Step; PWM fields are current calculated commands, input voltage is preceding cached interval. No physical latch timestamp.'
    return h,rows

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dump',type=Path)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--provenance',required=True,choices=['hardware','simulated'])
    args=parser.parse_args()
    meta,rows=decode(args.dump.read_bytes())
    out=args.output or args.dump.with_suffix('.csv')
    fields=FIELDS_V2 if meta['abi']==2 else FIELDS
    with out.open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)
    if meta['abi']==2:
        align=meta.pop('alignment_samples')
        align_out=out.with_name(out.stem+'_alignment.csv')
        with align_out.open('w',newline='',encoding='utf-8-sig') as f:
            w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(align)
        meta['alignment_csv']=str(align_out)
    meta.update(provenance=args.provenance,source=str(args.dump))
    out.with_suffix('.json').write_text(json.dumps(meta,indent=2),encoding='utf-8')
    print(json.dumps(meta))
if __name__=='__main__':main()

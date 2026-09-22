"""Export frozen ALG1 ALIGN records from the matching firmware ELF layout."""
import argparse
import csv
import json
import pathlib
import struct

HEADER = struct.Struct('<16I8f')
SAMPLE = struct.Struct('<2I4f4i3f2HI')
HEADER_FIELDS = ('magic abi sample_bytes capacity sequence active frozen record_id '
                 'total_samples first_count tail_count tail_next stop_reason sample_hz first_tick last_tick '
                 'current_base_a id_command current_limit measured_trip kp_d kp_q ki_step align_angle_turns').split()
FIELDS = ('tick stage_cycles id_ref iq_ref id_fb iq_fb sdm_u sdm_v zero_u zero_v '
          'voltage_d voltage_q voltage_limit saturated current_failure flags').split()
DERIVED = 'stage_ms elapsed_ms segment gap_cycles raw_sdm pi_executed previous_output_valid previous_voltage_d previous_voltage_q'.split()
FIRST, TAIL = 128, 416

def decode(data):
    if len(data) != HEADER.size + SAMPLE.size * (FIRST + TAIL):
        raise ValueError('wrong length; read ALIGN member size, not the larger union size')
    h = dict(zip(HEADER_FIELDS, HEADER.unpack_from(data)))
    if (h['magic'], h['abi'], h['sample_bytes'], h['capacity']) != (0x414c4731, 1, 60, 544):
        raise ValueError('wrong ALG1 layout')
    if h['active'] or h['frozen'] != 1 or h['sequence'] & 1:
        raise ValueError('record is active or not a coherent frozen snapshot')
    if not 1000 <= h['sample_hz'] <= 1000000 or h['stop_reason'] not in (1, 2, 3):
        raise ValueError('invalid rate or stop reason')
    total = h['total_samples']
    if h['first_count'] != min(total, FIRST) or h['tail_count'] != min(max(total-FIRST, 0), TAIL):
        raise ValueError('invalid counts')
    if h['tail_next'] != max(total-FIRST, 0) % TAIL:
        raise ValueError('invalid ring position')
    slots = list(range(h['first_count']))
    slots += [FIRST + (h['tail_next']-h['tail_count']+n) % TAIL for n in range(h['tail_count'])]
    rows = [dict(zip(FIELDS, SAMPLE.unpack_from(data, HEADER.size+s*SAMPLE.size))) for s in slots]
    if rows and (rows[0]['tick'] != h['first_tick'] or rows[-1]['tick'] != h['last_tick']):
        raise ValueError('invalid endpoint ticks')
    for n, row in enumerate(rows):
        distance = (row['tick'] - rows[n-1]['tick']) & 0xffffffff if n else 1
        at_gap = n == h['first_count'] and total > FIRST + TAIL
        expected = total - (FIRST + TAIL) + 1 if at_gap else 1
        if distance != expected:
            raise ValueError('nonconsecutive record inside retained window')
        if row['flags'] & ~0xff03 or row['saturated'] > 1:
            raise ValueError('invalid sample flags')
        row.update(stage_ms=row['stage_cycles']*1000/h['sample_hz'],
                   elapsed_ms=((row['tick']-h['first_tick']) & 0xffffffff)*1000/h['sample_hz'],
                   segment='begin' if n < h['first_count'] else 'tail',
                   gap_cycles=distance-1, raw_sdm=bool(row['flags'] & 1),
                   pi_executed=bool(row['flags'] & 2), previous_output_valid=bool(n and distance == 1),
                   previous_voltage_d=rows[n-1]['voltage_d'] if n and distance == 1 else None,
                   previous_voltage_q=rows[n-1]['voltage_q'] if n and distance == 1 else None)
    h.update(retained_samples=len(rows), omitted_cycles=max(total-FIRST-TAIL, 0),
             scope='ALIGN only; first 128 and final 416 consecutive software PWM cycles. '
                   'Post-decision PI outputs; trip outputs are reset, preceding row preserves prior output. '
                   'SDM raw fields are usable only when raw_sdm=true. No physical latch timestamp.')
    return h, rows

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('dump', type=pathlib.Path)
    p.add_argument('--output', required=True, type=pathlib.Path)
    p.add_argument('--provenance', required=True, choices=['simulated', 'hardware'])
    a = p.parse_args()
    h, rows = decode(a.dump.read_bytes())
    with a.output.open('w', encoding='utf-8-sig', newline='') as f:
        w = csv.DictWriter(f, fieldnames=FIELDS+DERIVED)
        w.writeheader()
        w.writerows(rows)
    h.update(source=str(a.dump), provenance=a.provenance)
    a.output.with_suffix('.json').write_text(json.dumps(h, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(h))

if __name__ == '__main__':
    main()

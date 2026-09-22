"""Decode STM1 ABI1/2/3; ABI3 reserves stage14/bin30 for one frame-commit PWM."""
import argparse,json,pathlib,struct
EVENT="tick entry_state exit_state trace_written elapsed pre torque post communication scope".split()
DETAIL="acquisition transform current_pi pwm monitor".split()
def decode(data):
 if len(data) not in (884,944):raise ValueError("wrong timing size")
 v=struct.unpack("<%dI"%(len(data)//4),data)
 magic,abi,seq,cpu,hz,budget=v[:6]
 if magic!=0x53544d31 or abi not in (1,2,3) or seq&1:raise ValueError("wrong ABI or incoherent snapshot")
 if len(data)!={1:884,2:944,3:944}[abi]:raise ValueError("ABI/size mismatch")
 fields=EVENT+(DETAIL if abi>=2 else [])
 event_start=14 if abi>=2 else 9
 bins_start=event_start+2*len(fields)
 if not cpu or not hz or budget!=cpu//hz:raise ValueError("clock/budget not initialized")
 def event(offset):
  e=dict(zip(fields,v[offset:offset+len(fields)]));e["elapsed_us"]=e["elapsed"]*1e6/cpu;return e
 bins=[]
 for i in range(32):
  x=v[bins_start+i*6:bins_start+(i+1)*6]
  if not x[0]:continue
  b=dict(zip("count last max over max_tick first_over_tick".split(),x))
  b.update(bucket=i,entry_state=None if i//2==0 else i//2-1,drive_enabled=bool(i//2),trace_written=bool(i%2),max_us=x[2]*1e6/cpu)
  if abi==3 and i==30:b.update(event_kind="frame_commit",last_us=x[1]*1e6/cpu)
  bins.append(b)
 return dict(abi=abi,cpu_hz=cpu,sample_hz=hz,budget_cycles=budget,period_us=1e6/hz,worst=event(event_start),first_over=event(event_start+len(fields)),bins=bins,
  scope="Instrumented elapsed cycles; excludes ISR prologue/epilogue and timing aggregation; can include preemption/debug stalls.")
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument("dump",type=pathlib.Path);p.add_argument("--output",type=pathlib.Path)
 a=p.parse_args();s=json.dumps(decode(a.dump.read_bytes()),indent=2)
 if a.output:a.output.write_text(s,encoding="utf-8")
 print(s)
if __name__=="__main__":main()

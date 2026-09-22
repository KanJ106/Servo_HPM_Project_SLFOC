"""Check ABI3 software current/PWM chain; not physical current calibration."""
import argparse,json,math,pathlib
from export_startup_trace import decode
Q=1<<24
def check(rows):
    errors={}; checked=0
    def expect(name,actual,wanted,tolerance=0):
        delta=abs(actual-wanted)
        if delta>tolerance:
            errors.setdefault(name,{"count":0,"max_error":0})
            errors[name]["count"]+=1
            errors[name]["max_error"]=max(errors[name]["max_error"],delta)
    def sat(v):return max(-32768,min(32767,v))
    def signed16(v):return ((v+32768)%65536)-32768
    for r in rows:
        if not r.get("chain_valid"):continue
        checked+=1
        expect("cycle",(r["chain_tick"]+1)&0xffffffff,r["tick"])
        expect("sdm_u_sign",r["adc_u"],signed16(-r["sdm_u"]))
        expect("sdm_v_sign",r["adc_v"],signed16(-r["sdm_v"]))
        expect("w_reconstruction",r["adc_w"],sat(-r["adc_u"]-r["adc_v"]))
        for phase in "uvw":
            expect("offset_"+phase,r["i"+phase+"_q24"],sat(r["adc_"+phase]-r["zero_"+phase])*512)
        u,v,w=(r["i"+phase+"_q24"] for phase in "uvw")
        if r["sample_window"]==1:u=-(v+w)
        elif r["sample_window"]!=0:v=-(u+w)
        if r["phase_mode"]:u=-(u+v)
        expect("clark_u",r["clark_u_q24"],u)
        expect("clark_v",r["clark_v_q24"],v)
        expect("alpha",r["alpha_q24"],u)
        expect("beta",r["beta_q24"],((u+2*v)*9686330)>>24,2)
        angle=r["control_theta_q24"]/Q*2*math.pi
        co,si=math.cos(angle),math.sin(angle)
        expect("park_d",r["id_fb"],(r["alpha_q24"]*co+r["beta_q24"]*si)/Q,2e-6)
        expect("park_q",r["iq_fb"],(r["beta_q24"]*co-r["alpha_q24"]*si)/Q,2e-6)
        expect("ipark_alpha",r["ualpha_q24"]/Q,r["ud"]*co-r["uq"]*si,2e-6)
        expect("ipark_beta",r["ubeta_q24"]/Q,r["uq"]*co+r["ud"]*si,2e-6)
        if r["bus_pu"]<=0:
            expect("bus_positive",0,1);continue
        gain=(r["bus_reference_q24"]/Q)/r["bus_pu"]
        a=r["ualpha_q24"]/Q*gain;b=r["ubeta_q24"]/Q*gain
        phases=[a,-a/2+b*math.sqrt(3)/2,-a/2-b*math.sqrt(3)/2]
        common=(max(phases)+min(phases))/2
        for k,v in zip("abc",phases):
            expect("duty_"+k,r["duty_"+k+"_q24"]/Q,max(-1,min(1,.5+v-common)),3e-6)
        duty=[r["duty_"+k+"_q24"] for k in "abc"]
        if r["phase_mode"]:duty.reverse()
        half=r["pwm_period"]>>1
        for k,v in zip("uvw",duty):
            expect("pwm_"+k,r["pwm_"+k],max(0,min(half,((Q-v)*half)>>24)),1)
    return {"checked_samples":checked,"invalid_samples":len(rows)-checked,"errors":errors,
            "software_chain_consistent":checked>0 and not errors,
            "scope":"Software arithmetic only; dead-time compensation must remain disabled. Not physical calibration."}
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("dump",type=pathlib.Path);p.add_argument("--output",type=pathlib.Path)
    a=p.parse_args();m,r=decode(a.dump.read_bytes())
    if m["trace_abi"]!=3:p.error("ABI3 required")
    result=check(r);text=json.dumps(result,indent=2)
    if a.output:a.output.write_text(text,encoding="utf-8")
    print(text)
if __name__=="__main__":main()

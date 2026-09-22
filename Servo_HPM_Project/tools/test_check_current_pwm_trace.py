import unittest
from check_current_pwm_trace import check,Q
class ChainTest(unittest.TestCase):
 def row(self):
  r=dict(chain_valid=1,chain_tick=0,tick=1,sdm_u=-100,sdm_v=50,adc_u=100,adc_v=-50,adc_w=-50,
   zero_u=0,zero_v=0,zero_w=0,iu_q24=51200,iv_q24=-25600,iw_q24=-25600,
   sample_window=0,phase_mode=0,clark_u_q24=51200,clark_v_q24=-25600,alpha_q24=51200,beta_q24=0,
   control_theta_q24=0,id_fb=51200/Q,iq_fb=0,ualpha_q24=0,ubeta_q24=0,ud=0,uq=0,
   bus_pu=.6,bus_reference_q24=int(.6*Q),pwm_period=1000,
   duty_a_q24=Q//2,duty_b_q24=Q//2,duty_c_q24=Q//2,pwm_u=250,pwm_v=250,pwm_w=250)
  return r
 def test_good(self):self.assertTrue(check([self.row()])["software_chain_consistent"])
 def test_bad(self):
  for field,error in [("adc_u","sdm_u_sign"),("zero_u","offset_u"),("pwm_u","pwm_u"),("chain_tick","cycle")]:
   r=self.row();r[field]+=20
   self.assertIn(error,check([r])["errors"])
 def test_invalid(self):
  r=self.row();r["chain_valid"]=0
  self.assertFalse(check([r])["software_chain_consistent"])
if __name__=="__main__":unittest.main()

# Load symbols from the matching BANK0 ELF before sourcing this file.
# These commands only submit mailbox requests; inspect acknowledge_seq/result.
define sl7_mode
  if g_sensorless_production.magic != 0x534c4637 || g_sensorless_production.abi_version != 0x00070000
    echo REJECTED: expected mailbox ABI 7.\n
  else
    if StateMachine.RegulFlg != 0 || g_sensorless_shadow.startup.state != 0
      echo REJECTED: Servo OFF and IDLE required.\n
    else
      if g_sensorless_production.request_seq != g_sensorless_production.acknowledge_seq
        echo REJECTED: previous request still pending.\n
      else
        if $arg0 < 0 || $arg0 > 2
          echo REJECTED: mode 0=full, 1=align, 2=IF.\n
        else
          set g_sensorless_production.tuning.mask = 0x200
          set g_sensorless_production.tuning.debug_mode = $arg0
          set g_sensorless_production.tuning.current_ramp_ms = $arg1
          set g_sensorless_production.tuning.if_hold_ms = $arg2
          set g_sensorless_production.tuning.iq_hold_ms = $arg3
          set g_sensorless_production.tuning.ramp_timeout_ms = $arg4
          set g_sensorless_production.tuning.total_timeout_ms = $arg5
          set g_sensorless_production.tuning.run_test_ms = $arg6
          set g_sensorless_production.command = 2
          set g_sensorless_production.arm_key = 0
          set g_sensorless_production.request_seq = g_sensorless_production.request_seq + 1
          echo Submitted. Run firmware service, verify ack/result and effective_startup before enabling.\n
        end
      end
    end
  end
end
document sl7_mode
sl7_mode MODE CURRENT_RAMP_MS IF_HOLD_MS IQ_HOLD_MS RAMP_TIMEOUT_MS TOTAL_TIMEOUT_MS RUN_TEST_MS
Requires ABI 7, Servo OFF, IDLE, no pending request. Does not start the motor.
end

define sl7_readback
  if g_sensorless_production.magic != 0x534c4637 || g_sensorless_production.abi_version != 0x00070000
    echo REJECTED: expected mailbox ABI 7.\n
  else
    p g_sensorless_production.firmware_version
    p g_sensorless_production.parameter_version
    p g_sensorless_production.command_result
    p g_sensorless_production.acknowledge_seq
    p g_sensorless_production.effective_recipe
    p g_sensorless_production.effective_startup
    p g_sensorless_production.effective_smo
    p g_sensorless_production.effective_motor
    p g_sensorless_production.effective_current
    p g_sensorless_production.first_failure
    p g_sensorless_diag
  end
end

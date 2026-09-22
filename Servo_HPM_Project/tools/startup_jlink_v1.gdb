# Matching BANK0 ELF required. Sourcing defines commands only.
# Do not halt/step while PWM is active. Deadline requires a running CPU.
define slj_status
  p g_sensorless_jlink
  p g_sensorless_production.firmware_version
  p g_sensorless_production.command_result
  p g_sensorless_production.first_failure
  p g_source_foc_ready
  p StateMachine
  p FaultP
  p DrvCoeff
  p AdRead
  p g_sensorless_production.effective_recipe
  p g_sensorless_production.effective_startup
  p g_sensorless_diag
  p g_sl_sampling
end

define slj_run
  if $argc != 3
    echo Usage: slj_run DIRECTION MOTOR_RPM DEADLINE_MS\n
  else
    if g_sensorless_jlink.abi != 1 || g_sensorless_production.abi_version != 0x70000
      echo REJECTED: mailbox ABI mismatch.\n
    else
      if StateMachine.RegulFlg || StateMachine.RegilFlgISR || StateMachine.SvpwmOnFlg || StateMachine.SrvOnStatus || g_sensorless_shadow.startup.state != 0
        echo REJECTED: OFF and IDLE required.\n
      else
        if g_sensorless_jlink.request_seq != g_sensorless_jlink.acknowledge_seq || g_sensorless_production.request_seq != g_sensorless_production.acknowledge_seq
          echo REJECTED: pending request.\n
        else
          if !g_source_foc_ready || (g_sensorless_shadow.startup.config.debug_mode != 1 && g_sensorless_shadow.startup.config.debug_mode != 2)
            echo REJECTED: valid recipe and ALIGN/IF mode required.\n
          else
            if ($arg0 != 1 && $arg0 != -1) || $arg1 <= 0 || $arg2 < 1 || $arg2 > 10000
              echo REJECTED: direction +/-1, positive motor RPM, deadline 1..10000 ms.\n
            else
              set g_sensorless_jlink.direction = $arg0
              set g_sensorless_jlink.target_speed_rpm = $arg1
              set g_sensorless_jlink.duration_ms = $arg2
              set g_sensorless_jlink.command = 1
              set g_sensorless_jlink.arm_key = 0x4a4c494e
              set g_sensorless_jlink.request_seq = g_sensorless_jlink.request_seq + 1
              echo Resuming: firmware will validate, enable normally, then run once. Do not halt during motion.\n
              continue
            end
          end
        end
      end
    end
  end
end
document slj_run
Explicit motion command; never invoked by sourcing this file.
Prepare/verify parameters with power stage OFF. Remove breakpoints first.
The firmware deadline includes time waiting for drive readiness.
Do not use debugger Halt as emergency stop. Use independent power disable.
end

define slj_abort
  set g_sensorless_jlink.command = 2
  set g_sensorless_jlink.arm_key = 0
  set g_sensorless_jlink.request_seq = g_sensorless_jlink.request_seq + 1
  echo Abort queued; firmware must run to consume it. Not a hardware emergency stop.\n
end

define slj_release
  if StateMachine.RegulFlg || StateMachine.RegilFlgISR || StateMachine.SvpwmOnFlg || StateMachine.SrvOnStatus || g_sensorless_jlink.enable_requested
    echo REJECTED: wait for OFF before releasing ownership.\n
  else
    set g_sensorless_jlink.command = 3
    set g_sensorless_jlink.arm_key = 0x4a4c494e
    set g_sensorless_jlink.request_seq = g_sensorless_jlink.request_seq + 1
    echo Release queued. Existing CAN heartbeat has been invalidated.\n
  end
end

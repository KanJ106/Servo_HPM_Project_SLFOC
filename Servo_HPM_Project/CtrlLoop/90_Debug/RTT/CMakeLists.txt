# Copyright (c) 2022 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

sdk_inc(.)

sdk_src(SEGGER_RTT.c)
sdk_src(SEGGER_RTT_printf.c)
sdk_ses_src(SEGGER_RTT_Syscalls_SES.c)
sdk_gcc_src(SEGGER_RTT_Syscalls_GCC.c)

sdk_src(segger_rtt_interface.c)

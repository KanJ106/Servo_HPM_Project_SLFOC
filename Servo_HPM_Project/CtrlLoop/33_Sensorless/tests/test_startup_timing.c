#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "StartupTiming.h"
int main(void) {
 STARTUP_TIMING s; STARTUP_TIMING_EVENT e;
 memset(&s,0,sizeof(s));memset(&e,0,sizeof(e));s.budget_cycles=100;
 e.tick=7;e.entry_state=4;e.exit_state=4;e.elapsed=99;
 StartupTiming_Accumulate(&s,&e,1);
 assert(s.bins[10].count==1 && !s.bins[10].over);
 e.tick=8;e.elapsed=100;e.trace_written=1;e.torque=70;e.current_pi=50;e.pwm=15;
 StartupTiming_Accumulate(&s,&e,1);
 assert(s.bins[11].over==1 && s.bins[11].first_over_tick==8);
 assert(s.first_over.tick==8 && s.first_over.torque==70 && s.first_over.current_pi==50 && s.first_over.pwm==15);
 e.tick=9;e.elapsed=150;
 StartupTiming_Accumulate(&s,&e,1);
 assert(s.worst.tick==9 && s.first_over.tick==8 && s.bins[11].over==2);
 assert(s.bins[10].max==99 && s.bins[11].max==150);
 e.trace_written=0;e.elapsed=20;
 StartupTiming_Accumulate(&s,&e,0);assert(s.bins[0].count==1);
 s.bins[0].count=0xffffffffU; StartupTiming_Accumulate(&s,&e,0);
 assert(s.bins[0].count==0xffffffffU && !(s.sequence&1));
 e.entry_state=999;e.elapsed=120;
 StartupTiming_Accumulate(&s,&e,1);assert(s.bins[30].over==1);
 assert(sizeof(STARTUP_TIMING)==944);
 puts("timing buckets, budget boundary, latches, saturation and layout: PASS");
 return 0;
}

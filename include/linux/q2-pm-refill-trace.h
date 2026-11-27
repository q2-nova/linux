/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef Q2_REFILL_TRACE_H
#define Q2_REFILL_TRACE_H
/* Separate from checkpoint7f40..7f60. First-entry-only v2; previous entry/pre-ERET v1 remains historical. */
#define Q2_RT_OFFSET 0x7f60
#define Q2_RT_WORDS 20
#define Q2_RT_MAGIC 0x51525431U
#define Q2_RT_VERSION 2
#define Q2_RT_ENTRY 0x55
#define Q2_RT_EXIT 0xaa
#define Q2_RT_ARMED 0x33
extern unsigned int q2_refill_trace_enable;
static inline unsigned int q2_rt_phase(unsigned int marker)
{
 return marker==0?0:marker==Q2_RT_ENTRY?1:2;
}
static inline int q2_rt_match(const unsigned int tag[4],const unsigned int *seen)
{
 unsigned int i,part,phase=q2_rt_phase(seen[6]);
 if (!(tag[0]|tag[1]|tag[2]|tag[3]) || phase>1) return 0;
 for (i=0;i<4;i++) if (seen[i]!=tag[i]) return 0;
 if (seen[4]!=Q2_RT_MAGIC || seen[5]!=Q2_RT_VERSION ||
     (seen[7] && seen[7]!=Q2_RT_ARMED) || (phase && seen[7]!=Q2_RT_ARMED)) return 0;
 for (part=0;part<phase;part++)
  for (i=0;i<3;i++)
   if (seen[8+part*6+3+i]!=~seen[8+part*6+i]) return 0;
 return 1;
}
#endif


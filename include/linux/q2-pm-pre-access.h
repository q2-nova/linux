/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef Q2_PRE_ACCESS_H
#define Q2_PRE_ACCESS_H
/* Replaces the inactive ErrorEPC intervention's48-byte slot. Distinct magic;
 * captures Status/EPC/ErrorEPC without modifying architectural state. */
#define Q2_PA_OFFSET 0x7a50
#define Q2_PA_WORDS 12
#define Q2_PA_MAGIC 0x51504131U
#define Q2_PA_VERSION 1
#define Q2_PA_CAPTURED 0x55
#define Q2_PA_RETURNED 0xaa
static inline unsigned int q2_pa_phase(unsigned int marker)
{
 return !marker?0:marker==Q2_PA_CAPTURED?1:marker==Q2_PA_RETURNED?2:3;
}
static inline int q2_pa_match(const unsigned int tag[4],const unsigned int *v)
{
 unsigned int i;
 if (!(tag[0]|tag[1]|tag[2]|tag[3])) return 0;
 for (i=0;i<4;i++) if (tag[i]!=v[i]) return 0;
 if (v[4]!=Q2_PA_MAGIC || q2_pa_phase(v[5])>2) return 0;
 if (v[5]) for (i=0;i<3;i++) if (v[9+i]!=~v[6+i]) return 0;
 return 1;
}
#endif

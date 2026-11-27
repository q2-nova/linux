/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef Q2_ICACHE_PROBE_RECORD_H
#define Q2_ICACHE_PROBE_RECORD_H
/* The64-byte gap after the origin records, before the MMU records. */
#define Q2_CI_OFFSET 0x7bc0
#define Q2_CI_WORDS 16
#define Q2_CI_MAGIC 0x51434931U
#define Q2_CI_VERSION 2
#define Q2_CI_BUDGET 240000U /*10ms at the admitted24MHz global OST*/
#define Q2_CI_SPINS 0x100000U
#define Q2_CI_LINES 1024U
enum q2_ci_mode { Q2_CI_NONE, Q2_CI_DWELL, Q2_CI_FULL };
static inline unsigned int q2_ci_phase(unsigned int marker)
{ return marker==0?0:marker==0x55?1:marker==0xaa?2:3; }
static inline int q2_ci_match(const unsigned int tag[4],const unsigned int *v)
{
 unsigned int i;
 if (!(tag[0]|tag[1]|tag[2]|tag[3])) return 0;
 for (i=0;i<4;i++) if (tag[i]!=v[i]) return 0;
 if (v[4]!=Q2_CI_MAGIC || v[5]!=Q2_CI_VERSION || q2_ci_phase(v[6])>2 || v[7]!=4) return 0;
 if (!v[6]) return 1;
 for (i=0;i<4;i++) if (v[12+i]!=~v[8+i]) return 0;
 if (v[8]>Q2_CI_FULL) return 0;
 return 1;
}
#endif

/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef Q2_GENERAL_TRACE_H
#define Q2_GENERAL_TRACE_H
/* Follows refill7f60..7fb0;14 words end at7fe8, below32KiB SRAM end. */
#define Q2_GT_OFFSET 0x7fb0
#define Q2_GT_WORDS 14
#define Q2_GT_MAGIC 0x51475431
#define Q2_GT_VERSION 1
#define Q2_GT_ENTRY 0x55
#define Q2_GT_ARMED 0x33
#ifndef __ASSEMBLY__
extern unsigned int q2_general_trace_enable;
static inline int q2_gt_match(const unsigned int tag[4],const unsigned int *seen)
{
 unsigned int i;
 if (!(tag[0]|tag[1]|tag[2]|tag[3])) return 0;
 for (i=0;i<4;i++) if (seen[i]!=tag[i]) return 0;
 if (seen[4]!=Q2_GT_MAGIC || seen[5]!=Q2_GT_VERSION ||
     (seen[6] && seen[6]!=Q2_GT_ENTRY) ||
     (seen[7] && seen[7]!=Q2_GT_ARMED) ||
     (seen[6] && seen[7]!=Q2_GT_ARMED)) return 0;
 if (seen[6]) for (i=0;i<3;i++) if (seen[11+i]!=~seen[8+i]) return 0;
 return 1;
}
#endif
#endif

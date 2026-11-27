/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef Q2_FAULT_ORIGIN_RECORD_H
#define Q2_FAULT_ORIGIN_RECORD_H
/* Independent of MMUv3/RFSv1. Both records fit between auxiliary and MMU. */
#define Q2_FOR_OFFSET 0x7a80
#define Q2_FOR_STRIDE 0xa0
#define Q2_FOR_SLOTS 2
#define Q2_FOR_VALUES 16
#define Q2_FOR_WORDS (8 + 2 * Q2_FOR_VALUES)
#define Q2_FOR_MAGIC 0x51464f31U
#define Q2_FOR_VERSION 1
enum q2_fault_origin {
 Q2_ORIGIN_PAGE_FAULT=1, Q2_ORIGIN_ADE_INSTRUCTION=2,
 Q2_ORIGIN_OTHER_DIE=3, Q2_ORIGIN_NMI=4
};
enum q2_vector_reason {
 Q2_VECTOR_NONE=0, Q2_VECTOR_ATTEMPT=1, Q2_VECTOR_ALREADY=2,
 Q2_VECTOR_GLOBAL_MISMATCH=3, Q2_VECTOR_RANGE=4,
 Q2_VECTOR_CACHED_COMMITTED=5, Q2_VECTOR_BOTH_COMMITTED=6,
 Q2_VECTOR_BUSY=7
};
enum q2_fault_origin_field {
 Q2_FOR_ORIGIN, Q2_FOR_MMU_KIND, Q2_FOR_SAVED_EPC, Q2_FOR_LIVE_EPC,
 Q2_FOR_ERROR_EPC, Q2_FOR_SAVED_STATUS, Q2_FOR_SAVED_CAUSE,
 Q2_FOR_PRE_VALID, Q2_FOR_ATTEMPTED, Q2_FOR_COMMITTED, Q2_FOR_REASON,
 Q2_FOR_LAST_LIVE_EBASE, Q2_FOR_LAST_GLOBAL_EBASE,
 Q2_FOR_FAULT_LIVE_EBASE, Q2_FOR_FAULT_GLOBAL_EBASE, Q2_FOR_CAPTURE_BUSY
};
static const char * const q2_for_names[Q2_FOR_VALUES] = {
 "origin","mmu_kind","saved_epc","live_epc","errorepc","saved_status","saved_cause",
 "pre_valid","attempted","committed","reason","last_live_ebase",
 "last_global_ebase","fault_live_ebase","fault_global_ebase","capture_busy"
};
/* Exact bounded literal recognition; never emit or retain arbitrary messages. */
static inline unsigned int q2_fault_die_origin(const char *reason)
{
 static const char target[]="Kernel unaligned instruction access";
 unsigned int i;
 if (!reason) return Q2_ORIGIN_OTHER_DIE;
 for (i=0;i<sizeof(target);i++)
  if (reason[i]!=target[i]) return Q2_ORIGIN_OTHER_DIE;
 return Q2_ORIGIN_ADE_INSTRUCTION;
}
static inline int q2_for_pack(unsigned int *out,const unsigned int tag[4],
 unsigned int slot,const unsigned int *value)
{
 unsigned int i;
 if (slot>=Q2_FOR_SLOTS || !(tag[0]|tag[1]|tag[2]|tag[3])) return 0;
 for (i=0;i<4;i++) out[i]=tag[i];
 out[4]=Q2_FOR_MAGIC;out[5]=Q2_FOR_VERSION;out[6]=slot;out[7]=~slot;
 for (i=0;i<Q2_FOR_VALUES;i++) {out[8+i]=value[i];out[8+Q2_FOR_VALUES+i]=~value[i];}
 return 1;
}
static inline int q2_for_match(const unsigned int tag[4],unsigned int slot,
 const unsigned int *seen)
{
 unsigned int i;
 if (slot>=Q2_FOR_SLOTS || !(tag[0]|tag[1]|tag[2]|tag[3])) return 0;
 for (i=0;i<4;i++) if (seen[i]!=tag[i]) return 0;
 if (seen[4]!=Q2_FOR_MAGIC || seen[5]!=Q2_FOR_VERSION ||
     seen[6]!=slot || seen[7]!=~slot) return 0;
 for (i=0;i<Q2_FOR_VALUES;i++)
  if (seen[8+Q2_FOR_VALUES+i]!=~seen[8+i]) return 0;
 return 1;
}
#endif


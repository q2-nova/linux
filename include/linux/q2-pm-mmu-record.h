/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef Q2_MMU_CAPTURE_RECORD_H
#define Q2_MMU_CAPTURE_RECORD_H
/* Three independent owned records, outside code/context and both old journals.
 * Only kernel numeric context; complements detect torn data, not forgery. */
#define Q2_MMU_OFFSET 0x7c00
#define Q2_MMU_STRIDE 0x100
#define Q2_MMU_SLOTS 3
#define Q2_MMU_VERSION 3
#define Q2_MMU_VALUES 28
#define Q2_MMU_WORDS (8 + 2 * Q2_MMU_VALUES)
#define Q2_MMU_MAGIC 0x51324d4dU
enum q2_mmu_field {
 Q2_MMU_STATUS, Q2_MMU_CAUSE, Q2_MMU_EPC, Q2_MMU_BADVADDR,
 Q2_MMU_EBASE, Q2_MMU_CONTEXT, Q2_MMU_ENTRYHI, Q2_MMU_PAGEMASK,
 Q2_MMU_PAGEGRAIN, Q2_MMU_CONFIG0, Q2_MMU_CONFIG3, Q2_MMU_CONFIG4,
 Q2_MMU_CONFIG5, Q2_MMU_CONFIG7, Q2_MMU_WIRED, Q2_MMU_LLADDR,
 Q2_MMU_KS2, Q2_MMU_KS3, Q2_MMU_KS4, Q2_MMU_KS5,
 Q2_MMU_KS6, Q2_MMU_KS7, Q2_MMU_NODE, Q2_MMU_OPS,
 Q2_MMU_RA, Q2_MMU_SP, Q2_MMU_S0, Q2_MMU_V0
};
static const char * const q2_mmu_names[Q2_MMU_VALUES] = {
 "status","cause","epc","badvaddr","ebase","context","entryhi",
 "pagemask","pagegrain","config0","config3","config4","config5",
 "config7","wired","lladdr","ks2","ks3","ks4","ks5","ks6","ks7",
 "node","ops","ra","sp","s0","v0"
};
static inline int q2_mmu_kind(unsigned int slot, unsigned int kind)
{
 return slot < Q2_MMU_SLOTS &&
  ((slot == 0 && kind == 1) || (slot == 1 && kind == 2) ||
   (slot == 2 && (kind == 3 || kind == 4)));
}
static inline void q2_mmu_pack(unsigned int out[Q2_MMU_WORDS],
 const unsigned int tag[4], unsigned int kind,
 const unsigned int value[Q2_MMU_VALUES])
{
 unsigned int i;
 for (i=0;i<4;i++) out[i]=tag[i];
 out[4]=Q2_MMU_MAGIC;out[5]=Q2_MMU_VERSION;out[6]=kind;out[7]=~kind;
 for (i=0;i<Q2_MMU_VALUES;i++) {
  out[8+i]=value[i];out[8+Q2_MMU_VALUES+i]=~value[i];
 }
}
static inline int q2_mmu_match(const unsigned int tag[4], unsigned int slot,
 const unsigned int seen[Q2_MMU_WORDS])
{
 unsigned int i;
 if (!(tag[0]|tag[1]|tag[2]|tag[3])) return 0;
 for (i=0;i<4;i++) if (tag[i]!=seen[i]) return 0;
 if (seen[4]!=Q2_MMU_MAGIC || seen[5]!=Q2_MMU_VERSION || seen[7]!=~seen[6] ||
     !q2_mmu_kind(slot,seen[6])) return 0;
 for (i=0;i<Q2_MMU_VALUES;i++)
  if (seen[8+Q2_MMU_VALUES+i]!=~seen[8+i]) return 0;
 return 1;
}
#endif

/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef Q2_REFILL_SNAPSHOT_RECORD_H
#define Q2_REFILL_SNAPSHOT_RECORD_H
/* Four data-view snapshots of the owned generated vector, then one first-fault
 * auxiliary record. Context ends below7100; existing MMU records start7c00. */
#define Q2_RFS_VECTOR_OFFSET 0x7100
#define Q2_RFS_VECTOR_STRIDE 0x240
#define Q2_RFS_VECTOR_SLOTS 4
#define Q2_RFS_CODE_WORDS 64
#define Q2_RFS_VECTOR_VALUES (1 + Q2_RFS_CODE_WORDS)
#define Q2_RFS_AUX_OFFSET 0x7a00
#define Q2_RFS_AUX_VALUES 6
#define Q2_RFS_MAGIC 0x51325253U
#define Q2_RFS_VERSION 1
#define Q2_RFS_WORDS(n) (8 + 2 * (n))
#define Q2_RFS_MAX_WORDS Q2_RFS_WORDS(Q2_RFS_VECTOR_VALUES)
static inline unsigned int q2_rfs_values(unsigned int kind)
{
 if (kind>=1 && kind<=4) return Q2_RFS_VECTOR_VALUES;
 if (kind==19 || kind==20) return Q2_RFS_AUX_VALUES;
 return 0;
}
static inline unsigned int q2_rfs_offset(unsigned int kind)
{
 if (kind>=1 && kind<=4)
  return Q2_RFS_VECTOR_OFFSET+(kind-1)*Q2_RFS_VECTOR_STRIDE;
 if (kind==19 || kind==20) return Q2_RFS_AUX_OFFSET;
 return 0;
}
static inline int q2_rfs_pack(unsigned int *out,const unsigned int tag[4],
 unsigned int kind,const unsigned int *value)
{
 unsigned int i,n=q2_rfs_values(kind);
 if (!n || !(tag[0]|tag[1]|tag[2]|tag[3])) return 0;
 for (i=0;i<4;i++) out[i]=tag[i];
 out[4]=Q2_RFS_MAGIC;out[5]=Q2_RFS_VERSION;out[6]=kind;out[7]=~kind;
 for (i=0;i<n;i++) {out[8+i]=value[i];out[8+n+i]=~value[i];}
 return 1;
}
static inline int q2_rfs_match(const unsigned int tag[4],unsigned int kind,
 const unsigned int *seen)
{
 unsigned int i,n=q2_rfs_values(kind);
 if (!n || !(tag[0]|tag[1]|tag[2]|tag[3])) return 0;
 for (i=0;i<4;i++) if (seen[i]!=tag[i]) return 0;
 if (seen[4]!=Q2_RFS_MAGIC || seen[5]!=Q2_RFS_VERSION ||
     seen[6]!=kind || seen[7]!=~kind) return 0;
 for (i=0;i<n;i++) if (seen[8+n+i]!=~seen[8+i]) return 0;
 return 1;
}
#endif

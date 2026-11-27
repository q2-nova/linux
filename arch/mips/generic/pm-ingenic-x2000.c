// SPDX-License-Identifier: GPL-2.0-or-later
/* X2000 deep sleep: Q2 stock kernel and Ingenic X2000 pm_sleep.c reference. */
#include <linux/init.h>
#include <linux/build_bug.h>
#include <linux/clk.h>
#include <linux/of_address.h>
#include <linux/q2-pm-checkpoint.h>
#include <linux/q2-pm-checkpoint-record.h>
#include <linux/q2-pm-mmu-record.h>
#include <linux/q2-pm-refill-snapshot.h>
#include <linux/q2-pm-fault-origin.h>
#include <linux/q2-pm-refill-trace.h>
#include <linux/q2-pm-general-trace.h>
#include <linux/q2-pm-icache-probe.h>
#include <linux/q2-pm-pre-access.h>
#include <asm/cacheops.h>
#include <asm/hazards.h>
#include <asm/setup.h>
#include <asm/ptrace.h>
#include <linux/ingenic-x2000-watchdog.h>
#include <linux/pid.h>
#include <linux/sched.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/iopoll.h>
#include <linux/kobject.h>
#include <linux/of.h>
#include <linux/sizes.h>
#include <linux/smp.h>
#include <linux/suspend.h>
#include <linux/sysfs.h>
#include <asm/addrspace.h>
#include <asm/cacheflush.h>
#include <asm/mipsregs.h>
#include <asm/tlbflush.h>
#include "pm-ingenic-x2000-journal.h"

static_assert(Q2_FOR_OFFSET+Q2_FOR_STRIDE*Q2_FOR_SLOTS<=Q2_CI_OFFSET);
static_assert(Q2_CI_OFFSET+Q2_CI_WORDS*4<=Q2_MMU_OFFSET);
static_assert(Q2_RFS_AUX_OFFSET+Q2_RFS_WORDS(Q2_RFS_AUX_VALUES)*4<=Q2_PA_OFFSET);
static_assert(Q2_PA_OFFSET+Q2_PA_WORDS*4<=Q2_FOR_OFFSET);

#define SRAM_PHYS 0x12400000
#define SRAM_CONTEXT 0x7000
#define SRAM_JOURNAL 0x7f00
#define JOURNAL_BASE 200
#define JOURNAL_ACTIVE 204
#define ROUNDTRIP 208
#define CANARY_POINTER 212
#define CANARY_SEED 216
#define CPU_ONLY 220
#define RESET_RETURN 236
#define SAVED_EPC 240
#define EPC_RESTORE 244
#define CPM_PHYS 0x10000000
#define CCU_PHYS 0x12200000
#define RTC_PHYS 0x10003000
#define RTC_RTCCR 0x00
#define RTC_ENABLE BIT(0)
#define RTC_USE_EXCLK BIT(1)
#define CPM_CPCCR 0x00
#define CPM_LCR 0x04
#define CPM_OPCR 0x24
#define CPM_CLKGR0 0x20
#define CPM_RTC_GATED BIT(27)
#define CCU_CSRR 0x40
#define CCU_RER 0xf00

extern char x2000_sram_start[], x2000_sram_sleep[], x2000_sram_end[];
extern int x2000_suspend_save(void *entry);
extern void x2000_epc_probe(unsigned int treatment);
extern char q2_pm_exl_keep[];
extern void q2_pm_precise_trap(void);
/* Only the admitted IRQ-disabled CPU0 probe arms this one-shot gate. */
u32 q2_pm_trap_gate;
static void __iomem *sram, *cpm, *ccu, *rtc;
static u32 attempts, core_returns, core_failures;
static u32 roundtrip_returns, roundtrip_failures;
static u32 cpu_only_returns, cpu_only_failures;
static u32 ddr_canary[16] __aligned(64);
static struct kobject *proof_kobj;
static bool journal_owner, journal_armed;
static bool admission_normal;
static bool normal_retain_exclk = true;
static u32 journal_tag[4];
/* Inactive diagnostic profile: ordinary automatic sleep is not qualified.
 * A referenced task identity, not a reusable numeric PID, owns admission.
 */
static struct pid *admission_pid;
static unsigned int admission_seconds;
static u32 admission_generation, next_generation;
static bool admission_entered, admission_roundtrip, admission_keep_exclk, admission_idle_ddr, admission_pd0;
static bool pd0_retain_exclk = true;
static u32 admission_cpu_only;
static bool admission_checkpoint;

/* Opt-in numeric context only. No user memory, page walks or allocation.
 * The first C fault is retained separately from the ordinary checkpoint. */
static bool mmu_busy, mmu_window;
static bool mmu_active(void)
{
 return admission_checkpoint && mmu_window && admission_pid &&
  task_pid(current)==admission_pid && !smp_processor_id();
}
/* Inactive cache-state discriminator. One admitted return, fixed I-cache
 * geometry and bounded OST budget. No D/L2 intervention or new sleep policy. */
static u32 cache_probe_mode, admission_cache_probe;
static bool cache_probe_ran;
static void __iomem *cache_probe_ost;
static unsigned long cache_probe_rate;
/* Read the existing global counter. Never change its clock, control or value. */
static inline u32 cache_probe_ticks(void)
{
 return readl(cache_probe_ost+0x10);
}
static void __init cache_probe_init_clock(void)
{
 struct device_node *np;
 struct clk *clock;
 struct resource resource;
 np=of_find_compatible_node(NULL,NULL,"ingenic,x2000-ost");
 if (!np) return;
 clock=of_clk_get_by_name(np,"ost");
 if (!IS_ERR(clock)) {
  cache_probe_rate=clk_get_rate(clock);clk_put(clock);
  if (cache_probe_rate==24000000 && !of_address_to_resource(np,0,&resource) &&
      resource.start==0x12000000 && resource_size(&resource)==0x20)
   cache_probe_ost=of_iomap(np,0);
 }
 of_node_put(np);
}
static bool cache_probe_read(u32 out[Q2_CI_WORDS])
{
 unsigned int i;
 if (!journal_owner) return false;
 for (i=0;i<4;i++) {
  out[i]=readl(sram+Q2_CI_OFFSET+i*4);
  if (out[i]!=journal_tag[i]) return false;
 }
 for (i=4;i<Q2_CI_WORDS;i++) out[i]=readl(sram+Q2_CI_OFFSET+i*4);
 return q2_ci_match(journal_tag,out);
}
static void cache_probe_publish(unsigned int phase,u32 elapsed,u32 lines)
{
 unsigned int i;u32 v[Q2_CI_WORDS]={0};void __iomem *p=sram+Q2_CI_OFFSET;
 for (i=0;i<4;i++) v[i]=journal_tag[i];
 v[4]=Q2_CI_MAGIC;v[5]=Q2_CI_VERSION;v[6]=phase==1?0x55:phase==2?0xaa:0;v[7]=4;
 v[8]=admission_cache_probe;v[9]=elapsed;v[10]=lines;
 v[11]=admission_cache_probe?Q2_CI_BUDGET:0;
 for (i=0;i<4;i++) v[12+i]=~v[8+i];
 writel(0,p+16);wmb();
 for (i=0;i<Q2_CI_WORDS;i++) if (i!=4) writel(v[i],p+i*4);
 wmb();writel(v[4],p+16);wmb();readl(p+16);
}
static void cache_probe_prepare(void)
{
 cache_probe_ran=false;
 cache_probe_publish(0,0,0);
}
static int cache_probe_choose(unsigned int mode)
{
 if (mode>Q2_CI_FULL) return -EINVAL;
 if (admission_pid || admission_generation || journal_armed) return -EBUSY;
 if (mode && (!cache_probe_ost || cache_probe_rate!=24000000 ||
     readl(cache_probe_ost)!=0 || readl(cache_probe_ost+4)!=1 ||
     read_c0_prid()!=0x00132000 ||
     current_cpu_data.icache.linesz!=32 || current_cpu_data.icache.ways!=8 ||
     current_cpu_data.icache.waysize!=4096)) return -EOPNOTSUPP;
 cache_probe_mode=mode;return 0;
}
static inline void cache_probe_line(unsigned long address,bool invalidate)
{
 if (invalidate)
  __asm__ __volatile__("cache %0, 0(%1)"::"i"(Index_Invalidate_I),"r"(address):"memory");
 else __asm__ __volatile__("nop":::"memory");
}
static inline void cache_probe_barrier(void)
{
 mb();instruction_hazard();
}
static noinline void cache_probe_run(void)
{
 u32 start=0,elapsed=0,spin=0,lines=0;
 unsigned long address;
 if (!mmu_active() || !journal_owner || cache_probe_ran) return;
 cache_probe_ran=true;cache_probe_publish(1,0,0);
 if (admission_cache_probe) {
  start=cache_probe_ticks();
  for (address=0x80000000UL;address<0x80008000UL;address+=32) {
   cache_probe_line(address,admission_cache_probe==Q2_CI_FULL);lines++;
  }
  cache_probe_barrier();
  do {
   elapsed=cache_probe_ticks()-start;
   if (elapsed>=Q2_CI_BUDGET) break;
  } while (++spin<Q2_CI_SPINS);
 }
 /* phase2 says the bounded trial returned, not that a frozen OST advanced.
  * Consumers separately require elapsed>=budget for active modes. */
 cache_probe_publish(2,elapsed,lines);
}
static ssize_t cache_probe_show(struct kobject *kobj,struct kobj_attribute *attr,char *buf)
{
 unsigned int flags=lock_system_sleep();u32 v[Q2_CI_WORDS];
 bool matched=cache_probe_read(v);ssize_t size;
 size=sysfs_emit(buf,"version=%u selected=%u matched=%u phase=%u",Q2_CI_VERSION,cache_probe_mode,matched,matched?q2_ci_phase(v[6]):0);
 if (matched && v[6])
  size+=sysfs_emit_at(buf,size," mode=%u elapsed=%u lines=%u budget=%u",v[8],v[9],v[10],v[11]);
 size+=sysfs_emit_at(buf,size,"\n");unlock_system_sleep(flags);return size;
}
static ssize_t cache_probe_store(struct kobject *kobj,struct kobj_attribute *attr,const char *buf,size_t count)
{
 unsigned int mode,flags;int ret;
 if (sysfs_streq(buf,"none")) mode=Q2_CI_NONE;
 else if (sysfs_streq(buf,"dwell")) mode=Q2_CI_DWELL;
 else if (sysfs_streq(buf,"full")) mode=Q2_CI_FULL;
 else return -EINVAL;
 flags=lock_system_sleep();ret=cache_probe_choose(mode);unlock_system_sleep(flags);
 return ret?ret:(ssize_t)count;
}
static struct kobj_attribute cache_probe_attr=__ATTR(cache_probe,0600,cache_probe_show,cache_probe_store);

static bool mmu_read(unsigned int slot,u32 out[Q2_MMU_WORDS])
{
 unsigned int i;
 void __iomem *record;
 if (!journal_owner || slot>=Q2_MMU_SLOTS) return false;
 record=sram+Q2_MMU_OFFSET+slot*Q2_MMU_STRIDE;
 for (i=0;i<4;i++) {
  out[i]=readl(record+i*4);
  if (out[i]!=journal_tag[i]) return false;
 }
 for (i=4;i<Q2_MMU_WORDS;i++) out[i]=readl(record+i*4);
 return q2_mmu_match(journal_tag,slot,out);
}
/* Data aliases of the actual boot-admitted vector. These do not certify the
 * I-cache contents executed by the CPU. Never walk arbitrary exception memory. */
static bool refill_busy;
static void origin_prepare(void);
static void refill_trace_prepare(void);
static void general_trace_prepare(void);
static void pre_access_prepare(void);
static void origin_note(unsigned int after,unsigned int reason,
 unsigned long live,unsigned long global,u32 committed);

static bool refill_read(unsigned int kind, u32 words[Q2_RFS_MAX_WORDS])
{
 unsigned int i, n=q2_rfs_values(kind), offset=q2_rfs_offset(kind);
 void __iomem *record;
 if (!journal_owner || !n || !offset) return false;
 record=sram+offset;
 for (i=0;i<4;i++) {
  words[i]=readl(record+i*4);
  if (words[i]!=journal_tag[i]) return false;
 }
 for (i=4;i<Q2_RFS_WORDS(n);i++) words[i]=readl(record+i*4);
 return q2_rfs_match(journal_tag,kind,words);
}
static void refill_store(unsigned int kind, const u32 *value)
{
 u32 words[Q2_RFS_MAX_WORDS];
 unsigned int i, n=q2_rfs_values(kind), offset=q2_rfs_offset(kind);
 void __iomem *record;
 if (!journal_owner || !q2_rfs_pack(words,journal_tag,kind,value)) return;
 record=sram+offset;
 writel(0,record+16);wmb();
 for (i=0;i<Q2_RFS_WORDS(n);i++) if (i!=4) writel(words[i],record+i*4);
 wmb();writel(words[4],record+16);wmb();readl(record+16);
}
static void refill_prepare(void)
{
 unsigned int i;
 for (i=1;i<=Q2_RFS_VECTOR_SLOTS;i++)
  writel(0,sram+q2_rfs_offset(i)+16);
 writel(0,sram+Q2_RFS_AUX_OFFSET+16);wmb();
 refill_busy=false;origin_prepare();refill_trace_prepare();general_trace_prepare();pre_access_prepare();
}
static void refill_vector(bool after)
{
 u32 value[Q2_RFS_VECTOR_VALUES], words[Q2_RFS_MAX_WORDS];
 unsigned int i, kind=after?3:1;
 unsigned long base;
 if (!mmu_active()) return;
 base=read_c0_ebase() & ~0xfffUL;
 if (refill_busy) {
  origin_note(after,Q2_VECTOR_BUSY,base,ebase,0);return;
 }
 refill_busy=true;
 origin_note(after,Q2_VECTOR_ATTEMPT,base,ebase,0);
 /* The first post-resume/fault view is immutable for this owned attempt. */
 if (after && refill_read(3,words)) {
  origin_note(after,Q2_VECTOR_ALREADY,base,ebase,0);goto done;
 }
 if (base!=ebase) {
  origin_note(after,Q2_VECTOR_GLOBAL_MISMATCH,base,ebase,0);goto done;
 }
 if (base<0x80000000UL || base>0x9ffff000UL) {
  origin_note(after,Q2_VECTOR_RANGE,base,ebase,0);goto done;
 }
 value[0]=base;
 for (i=0;i<Q2_RFS_CODE_WORDS;i++)
  value[1+i]=READ_ONCE(((const u32 *)base)[i]);
 refill_store(kind,value);
 origin_note(after,Q2_VECTOR_CACHED_COMMITTED,base,ebase,1U<<(kind-1));
 for (i=0;i<Q2_RFS_CODE_WORDS;i++)
  value[1+i]=READ_ONCE(((const u32 *)CKSEG1ADDR(base))[i]);
 refill_store(kind+1,value);
 origin_note(after,Q2_VECTOR_BOTH_COMMITTED,base,ebase,3U<<(kind-1));
done:
 refill_busy=false;
}
static void refill_fault(unsigned int kind, const u32 value[Q2_RFS_AUX_VALUES])
{
 if (!mmu_active() || refill_busy || (kind!=3 && kind!=4)) return;
 refill_store(kind+16,value);
 refill_vector(true);
}
static ssize_t refill_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
 unsigned int flags=lock_system_sleep(), slot, i;
 u32 words[Q2_RFS_MAX_WORDS];ssize_t size=0;
 for (slot=0;slot<5;slot++) {
  unsigned int kind=slot+1;
  bool matched;
  if (slot==4) kind=19;
  matched=refill_read(kind,words);
  if (slot==4 && !matched) {kind=20;matched=refill_read(kind,words);}
  size+=sysfs_emit_at(buf,size,"version=1 slot=%u matched=%u kind=%u",
   slot,matched,matched?kind:0);
  if (matched && slot<4) {
   size+=sysfs_emit_at(buf,size," ebase=%08x words=",words[8]);
   for (i=0;i<Q2_RFS_CODE_WORDS;i++)
    size+=sysfs_emit_at(buf,size,"%08x",words[9+i]);
  } else if (matched) {
   size+=sysfs_emit_at(buf,size,
    " epc=%08x cause=%08x status=%08x entrylo0=%08x entrylo1=%08x errorepc=%08x",
    words[8],words[9],words[10],words[11],words[12],words[13]);
  }
  size+=sysfs_emit_at(buf,size,"\n");
 }
 unlock_system_sleep(flags);return size;
}
static struct kobj_attribute refill_attr=__ATTR(refill,0400,refill_show,NULL);

/* Small independent records survive even when a later vector copy cannot.
 * Slot0 is first-fault entry; slot1 is completion of optional extra capture. */
static u32 origin_pending, origin_pending_epc;
static u32 vector_attempted, vector_committed, vector_reason;
static u32 vector_live, vector_global;
static bool origin_read(unsigned int slot,u32 words[Q2_FOR_WORDS])
{
 unsigned int i;
 void __iomem *record;
 if (!journal_owner || slot>=Q2_FOR_SLOTS) return false;
 record=sram+Q2_FOR_OFFSET+slot*Q2_FOR_STRIDE;
 for (i=0;i<4;i++) {
  words[i]=readl(record+i*4);
  if (words[i]!=journal_tag[i]) return false;
 }
 for (i=4;i<Q2_FOR_WORDS;i++) words[i]=readl(record+i*4);
 return q2_for_match(journal_tag,slot,words);
}
static void origin_store(unsigned int slot,const u32 value[Q2_FOR_VALUES])
{
 unsigned int i;u32 words[Q2_FOR_WORDS];
 void __iomem *record;
 if (!journal_owner || !q2_for_pack(words,journal_tag,slot,value)) return;
 record=sram+Q2_FOR_OFFSET+slot*Q2_FOR_STRIDE;
 writel(0,record+16);wmb();
 for (i=0;i<Q2_FOR_WORDS;i++) if (i!=4) writel(words[i],record+i*4);
 wmb();writel(words[4],record+16);wmb();readl(record+16);
}
static void origin_prepare(void)
{
 unsigned int i;
 for (i=0;i<Q2_FOR_SLOTS;i++)
  writel(0,sram+Q2_FOR_OFFSET+i*Q2_FOR_STRIDE+16);
 wmb();
 origin_pending=origin_pending_epc=0;
 vector_attempted=vector_committed=vector_reason=vector_live=vector_global=0;
}
static void origin_note(unsigned int after,unsigned int reason,
 unsigned long live,unsigned long global,u32 committed)
{
 vector_attempted|=1U<<after;vector_committed|=committed;
 vector_reason=reason;vector_live=live;vector_global=global;
}
static void origin_current(u32 value[Q2_FOR_VALUES])
{
 value[Q2_FOR_ATTEMPTED]=vector_attempted;
 value[Q2_FOR_COMMITTED]=vector_committed;
 value[Q2_FOR_REASON]=vector_reason;
 value[Q2_FOR_LAST_LIVE_EBASE]=vector_live;
 value[Q2_FOR_LAST_GLOBAL_EBASE]=vector_global;
}
static void origin_capture(unsigned int kind,const struct pt_regs *regs)
{
 u32 words[Q2_FOR_WORDS],old[Q2_RFS_MAX_WORDS],value[Q2_FOR_VALUES]={0};
 unsigned int i;
 if (origin_read(0,words)) return;
 value[Q2_FOR_ORIGIN]=origin_pending?origin_pending:
  kind==3?Q2_ORIGIN_PAGE_FAULT:Q2_ORIGIN_OTHER_DIE;
 value[Q2_FOR_MMU_KIND]=kind;
 value[Q2_FOR_SAVED_EPC]=regs->cp0_epc;
 value[Q2_FOR_LIVE_EPC]=origin_pending?origin_pending_epc:read_c0_epc();
 value[Q2_FOR_ERROR_EPC]=read_c0_errorepc();
 value[Q2_FOR_SAVED_STATUS]=regs->cp0_status;
 value[Q2_FOR_SAVED_CAUSE]=regs->cp0_cause;
 for (i=1;i<=Q2_RFS_VECTOR_SLOTS;i++)
  if (refill_read(i,old)) value[Q2_FOR_PRE_VALID]|=1U<<(i-1);
 origin_current(value);
 value[Q2_FOR_FAULT_LIVE_EBASE]=read_c0_ebase();
 value[Q2_FOR_FAULT_GLOBAL_EBASE]=ebase;
 value[Q2_FOR_CAPTURE_BUSY]=refill_busy;
 origin_store(0,value);
}
static void origin_complete(void)
{
 u32 words[Q2_FOR_WORDS],value[Q2_FOR_VALUES];
 unsigned int i;
 if (origin_read(1,words) || !origin_read(0,words)) return;
 for (i=0;i<Q2_FOR_VALUES;i++) value[i]=words[8+i];
 origin_current(value);origin_store(1,value);
}
static ssize_t origin_show(struct kobject *kobj,struct kobj_attribute *attr,char *buf)
{
 unsigned int flags=lock_system_sleep(),slot,i;
 u32 words[Q2_FOR_WORDS];ssize_t size=0;
 for (slot=0;slot<Q2_FOR_SLOTS;slot++) {
  bool matched=origin_read(slot,words);
  size+=sysfs_emit_at(buf,size,"version=1 slot=%u matched=%u",slot,matched);
  if (matched) for (i=0;i<Q2_FOR_VALUES;i++)
   size+=sysfs_emit_at(buf,size," %s=%08x",q2_for_names[i],words[8+i]);
  size+=sysfs_emit_at(buf,size,"\n");
 }
 unlock_system_sleep(flags);return size;
}
static struct kobj_attribute origin_attr=__ATTR(origin,0400,origin_show,NULL);

/* Arm only immediately before an owned vmalloc ops access. No early-boot SRAM
 * gate: the generated handler tests the zero-initialized DRAM enable word. */
static bool refill_trace_fault_closed;
static bool refill_trace_read(u32 words[Q2_RT_WORDS])
{
 unsigned int i,phase;
 if (!journal_owner) return false;
 for (i=0;i<4;i++) {
  words[i]=readl(sram+Q2_RT_OFFSET+i*4);
  if (words[i]!=journal_tag[i]) return false;
 }
 for (i=4;i<8;i++) words[i]=readl(sram+Q2_RT_OFFSET+i*4);
 phase=q2_rt_phase(words[6]);
 if (phase>1 || words[4]!=Q2_RT_MAGIC || words[5]!=Q2_RT_VERSION) return false;
 for (i=8;i<8+phase*6;i++) words[i]=readl(sram+Q2_RT_OFFSET+i*4);
 return q2_rt_match(journal_tag,words);
}
static void refill_trace_prepare(void)
{
 unsigned int i;
 WRITE_ONCE(q2_refill_trace_enable,0);wmb();
 refill_trace_fault_closed=false;
 for (i=0;i<Q2_RT_WORDS;i++) writel(0,sram+Q2_RT_OFFSET+i*4);
 for (i=0;i<4;i++) writel(journal_tag[i],sram+Q2_RT_OFFSET+i*4);
 writel(Q2_RT_VERSION,sram+Q2_RT_OFFSET+20);wmb();
 writel(Q2_RT_MAGIC,sram+Q2_RT_OFFSET+16);wmb();
 readl(sram+Q2_RT_OFFSET+16);
}
static void refill_trace_probe(unsigned long ops)
{
 u32 words[Q2_RT_WORDS];
 if (!mmu_active() || refill_trace_fault_closed || ops<0xc0000000UL || ops>=0xe0000000UL) return;
 if (!refill_trace_read(words) || q2_rt_phase(words[6])) return;
 writel(Q2_RT_ARMED,sram+Q2_RT_OFFSET+28);wmb();readl(sram+Q2_RT_OFFSET+28);
 WRITE_ONCE(q2_refill_trace_enable,1);wmb();
}
/* Stop before any first-C-fault record work. Fatal handling and a recovered
 * fault's later callbacks must never create a new refill-entry/exit witness. */
static void refill_trace_fault_close(void)
{
 if (!mmu_active()) return;
 WRITE_ONCE(q2_general_trace_enable,0);
 WRITE_ONCE(q2_refill_trace_enable,0);wmb();
 refill_trace_fault_closed=true;
}
static ssize_t refill_trace_show(struct kobject *kobj,struct kobj_attribute *attr,char *buf)
{
 unsigned int flags=lock_system_sleep(),phase=0;
 u32 words[Q2_RT_WORDS];ssize_t size;
 bool matched=refill_trace_read(words);
 if (matched) phase=q2_rt_phase(words[6]);
 size=sysfs_emit(buf,"version=%u matched=%u phase=%u armed=%u",Q2_RT_VERSION,matched,phase,matched && words[7]==Q2_RT_ARMED);
 if (matched && phase) size+=sysfs_emit_at(buf,size,
  " entry_status=%08x entry_epc=%08x entry_errorepc=%08x",words[8],words[9],words[10]);
 size+=sysfs_emit_at(buf,size,"\n");
 unlock_system_sleep(flags);return size;
}
static struct kobj_attribute refill_trace_attr=__ATTR(refill_trace,0400,refill_trace_show,NULL);

/* Independent first general-exception witness. The runtime vector data aliases
 * below certify copied data, not executed I-cache contents or fault-time bytes. */
static bool general_trace_read(u32 words[Q2_GT_WORDS])
{
 unsigned int i;
 if (!journal_owner) return false;
 for (i=0;i<4;i++) {
  words[i]=readl(sram+Q2_GT_OFFSET+i*4);
  if (words[i]!=journal_tag[i]) return false;
 }
 for (i=4;i<8;i++) words[i]=readl(sram+Q2_GT_OFFSET+i*4);
 if (words[6] && words[6]!=Q2_GT_ENTRY) return false;
 if (words[6]) for (i=8;i<Q2_GT_WORDS;i++) words[i]=readl(sram+Q2_GT_OFFSET+i*4);
 return q2_gt_match(journal_tag,words);
}
static void general_trace_prepare(void)
{
 unsigned int i;
 WRITE_ONCE(q2_general_trace_enable,0);wmb();
 for (i=0;i<Q2_GT_WORDS;i++) writel(0,sram+Q2_GT_OFFSET+i*4);
 for (i=0;i<4;i++) writel(journal_tag[i],sram+Q2_GT_OFFSET+i*4);
 writel(Q2_GT_VERSION,sram+Q2_GT_OFFSET+20);wmb();
 writel(Q2_GT_MAGIC,sram+Q2_GT_OFFSET+16);wmb();readl(sram+Q2_GT_OFFSET+16);
}
static void general_trace_probe(unsigned long ops)
{
 u32 words[Q2_GT_WORDS];
 if (!mmu_active() || refill_trace_fault_closed || ops<0xc0000000UL || ops>=0xe0000000UL) return;
 if (!general_trace_read(words) || words[6]) return;
 writel(Q2_GT_ARMED,sram+Q2_GT_OFFSET+28);wmb();readl(sram+Q2_GT_OFFSET+28);
 WRITE_ONCE(q2_general_trace_enable,1);wmb();
}
static ssize_t general_trace_show(struct kobject *kobj,struct kobj_attribute *attr,char *buf)
{
 unsigned int flags=lock_system_sleep(),i;
 unsigned long base=read_c0_ebase() & ~0xfffUL;
 u32 words[Q2_GT_WORDS];ssize_t size;
 bool matched=general_trace_read(words);
 bool vector=matched && base==ebase && base>=0x80000000UL && base<=0x9ffff000UL;
 size=sysfs_emit(buf,"version=1 matched=%u phase=%u armed=%u",matched,
  matched && words[6]==Q2_GT_ENTRY,matched && words[7]==Q2_GT_ARMED);
 if (matched && words[6]) size+=sysfs_emit_at(buf,size,
  " epc=%08x badvaddr=%08x cause=%08x",words[8],words[9],words[10]);
 size+=sysfs_emit_at(buf,size," vector=%u",vector);
 if (vector) {
  size+=sysfs_emit_at(buf,size," ebase=%08lx cached=",base);
  for (i=0;i<32;i++) size+=sysfs_emit_at(buf,size,"%08x",READ_ONCE(((const u32 *)(base+0x180))[i]));
  size+=sysfs_emit_at(buf,size," uncached=");
  for (i=0;i<32;i++) size+=sysfs_emit_at(buf,size,"%08x",READ_ONCE(((const u32 *)CKSEG1ADDR(base+0x180))[i]));
 }
 size+=sysfs_emit_at(buf,size,"\n");unlock_system_sleep(flags);return size;
}
static struct kobj_attribute general_trace_attr=__ATTR(general_trace,0400,general_trace_show,NULL);

/* Passive first mapped-access boundary. No CP0 writes or active repair. */
static bool pre_access_spent;
static bool exception_probe_spent;
static unsigned int exception_probe_mode;
static bool pre_access_read(u32 words[Q2_PA_WORDS])
{
 unsigned int i;
 if (!journal_owner) return false;
 for (i=0;i<Q2_PA_WORDS;i++) words[i]=readl(sram+Q2_PA_OFFSET+i*4);
 return q2_pa_match(journal_tag,words);
}
static void pre_access_store(u32 words[Q2_PA_WORDS],unsigned int phase)
{
 unsigned int i;
 writel(~0U,sram+Q2_PA_OFFSET+20);wmb();
 for (i=6;i<9;i++) {
  writel(words[i],sram+Q2_PA_OFFSET+i*4);
  writel(~words[i],sram+Q2_PA_OFFSET+(i+3)*4);
 }
 wmb();writel(phase==1?Q2_PA_CAPTURED:Q2_PA_RETURNED,sram+Q2_PA_OFFSET+20);wmb();readl(sram+Q2_PA_OFFSET+20);
}
static void pre_access_prepare(void)
{
 unsigned int i;
 pre_access_spent=false;exception_probe_spent=false;
 WRITE_ONCE(q2_pm_trap_gate,0);
 for (i=0;i<Q2_PA_WORDS;i++) writel(0,sram+Q2_PA_OFFSET+i*4);
 for (i=0;i<4;i++) writel(journal_tag[i],sram+Q2_PA_OFFSET+i*4);
 wmb();writel(Q2_PA_MAGIC,sram+Q2_PA_OFFSET+16);wmb();readl(sram+Q2_PA_OFFSET+16);
}
static noinline void pre_access_probe(unsigned long ops)
{
 u32 words[Q2_PA_WORDS];
 if (!mmu_active() || pre_access_spent || cache_probe_mode ||
     ops<0xc0000000UL || ops>=0xe0000000UL || !pre_access_read(words) || words[5]) return;
 pre_access_spent=true;
 words[6]=read_c0_status();words[7]=read_c0_epc();words[8]=read_c0_errorepc();
 pre_access_store(words,1);
}
static noinline void pre_access_complete(void)
{
 u32 words[Q2_PA_WORDS];
 if (!mmu_active() || !pre_access_spent || !pre_access_read(words) || words[5]!=Q2_PA_CAPTURED) return;
 pre_access_store(words,2);
}
static ssize_t pre_access_show(struct kobject *kobj,struct kobj_attribute *attr,char *buf)
{
 unsigned int flags=lock_system_sleep();u32 words[Q2_PA_WORDS];
 bool matched=pre_access_read(words);ssize_t size;
 size=sysfs_emit(buf,"version=%u matched=%u phase=%u",Q2_PA_VERSION,matched,matched?q2_pa_phase(words[5]):0);
 if (matched && words[5]) size+=sysfs_emit_at(buf,size," status=%08x epc=%08x errorepc=%08x",words[6],words[7],words[8]);
 size+=sysfs_emit_at(buf,size,"\n");unlock_system_sleep(flags);return size;
}
static struct kobj_attribute pre_access_attr=__ATTR(pre_access,0400,pre_access_show,NULL);

static void mmu_capture(unsigned int kind,unsigned long node,unsigned long ops,
 const struct pt_regs *regs,unsigned long address)
{
 u32 words[Q2_MMU_WORDS],value[Q2_MMU_VALUES]={0};
 u32 auxiliary[Q2_RFS_AUX_VALUES]={0};
 unsigned int i,slot=kind>2?2:kind-1;
 void __iomem *record;
 if (!mmu_active() || mmu_busy || !q2_mmu_kind(slot,kind) || (slot==2 && !regs)) return;
 if (slot==2) refill_trace_fault_close();
 mmu_busy=true;
 if (slot==2) {
  auxiliary[0]=regs->cp0_epc;auxiliary[1]=regs->cp0_cause;
  auxiliary[2]=regs->cp0_status;auxiliary[3]=read_c0_entrylo0();
  auxiliary[4]=read_c0_entrylo1();auxiliary[5]=read_c0_errorepc();
 }
 /* Do not replace the first fault with a subsequent die() notification. */
 if (slot==2 && mmu_read(slot,words)) goto done;
 value[Q2_MMU_STATUS]=read_c0_status();value[Q2_MMU_CAUSE]=read_c0_cause();
 value[Q2_MMU_EBASE]=read_c0_ebase();value[Q2_MMU_CONTEXT]=read_c0_context();
 value[Q2_MMU_ENTRYHI]=read_c0_entryhi();value[Q2_MMU_PAGEMASK]=read_c0_pagemask();
 value[Q2_MMU_PAGEGRAIN]=read_c0_pagegrain();value[Q2_MMU_CONFIG0]=read_c0_config();
 value[Q2_MMU_CONFIG3]=read_c0_config3();value[Q2_MMU_CONFIG4]=read_c0_config4();
 value[Q2_MMU_CONFIG5]=read_c0_config5();value[Q2_MMU_CONFIG7]=read_c0_config7();
 value[Q2_MMU_WIRED]=read_c0_wired();value[Q2_MMU_LLADDR]=read_c0_lladdr();
 value[Q2_MMU_KS2]=__read_32bit_c0_register($31,2);
 value[Q2_MMU_KS3]=__read_32bit_c0_register($31,3);
 value[Q2_MMU_KS4]=__read_32bit_c0_register($31,4);
 value[Q2_MMU_KS5]=__read_32bit_c0_register($31,5);
 value[Q2_MMU_KS6]=__read_32bit_c0_register($31,6);
 value[Q2_MMU_KS7]=__read_32bit_c0_register($31,7);
 value[Q2_MMU_NODE]=node;value[Q2_MMU_OPS]=ops;
 if (slot==2) {
  value[Q2_MMU_STATUS]=regs->cp0_status;value[Q2_MMU_CAUSE]=regs->cp0_cause;
  value[Q2_MMU_EPC]=regs->cp0_epc;value[Q2_MMU_BADVADDR]=address;
  value[Q2_MMU_RA]=regs->regs[31];value[Q2_MMU_SP]=regs->regs[29];
  value[Q2_MMU_S0]=regs->regs[16];value[Q2_MMU_V0]=regs->regs[2];
 }
 q2_mmu_pack(words,journal_tag,kind,value);
 record=sram+Q2_MMU_OFFSET+slot*Q2_MMU_STRIDE;
 writel(0,record+16);wmb();
 for (i=0;i<Q2_MMU_WORDS;i++) if (i!=4) writel(words[i],record+i*4);
 wmb();writel(words[4],record+16);wmb();readl(record+16);
 /* Commit the existing first-fault record before any extra data copy. */
 if (slot==2) origin_capture(kind,regs);
 if (slot==2) refill_fault(kind,auxiliary);
 if (slot==2) origin_complete();
done:
 mmu_busy=false;
}
/* Inactive genuine non-TLB exception discriminator. Every byte belongs to
 * this built-in kernel; do not substitute an arbitrary physical address.
 * The first general witness is armed BEFORE this load; refill is armed AFTER
 * its verified return. No observer gate is rearmed or overwritten.
 */
static const u32 exception_probe_buffer[2] __aligned(8)={0x44332211,0x88776655};
extern char q2_pm_adel_fault[],q2_pm_adel_resume[];
static noinline u32 exception_probe_load(void)
{
 u32 value;
 __asm__ __volatile__(
  ".set push\n.set noreorder\nnop\n"
  ".globl q2_pm_adel_fault\nq2_pm_adel_fault:\n"
  "lw %0,1(%1)\n"
  ".globl q2_pm_adel_resume\nq2_pm_adel_resume:\nnop\n.set pop\n"
  : "=&r"(value):"r"(exception_probe_buffer):"memory");
 return value;
}
static noinline bool exception_restore_epc(unsigned long epc)
{
 write_c0_epc(epc);back_to_back_c0_hazard();
 return read_c0_epc()==epc;
}
static noinline bool exception_probe_run(unsigned long ops)
{
 unsigned long status,epc;u32 value;
 if (!mmu_active() || !journal_owner || exception_probe_spent || cache_probe_mode ||
     ops<0xc0000000UL || ops>=0xe0000000UL) return false;
 exception_probe_spent=true;
 WRITE_ONCE(q2_pm_trap_gate,0);
 /* Same-binary exception-state comparison:0 none,1 AdEL,2 EPC seed,3 SRAM ERET,4 restore EPC,5 EXL ERET,6 EPC pulse. */
 if (!exception_probe_mode || exception_probe_mode==3 || exception_probe_mode==4) return true;
 status=read_c0_status();
 if (status&7) return false;
 if (exception_probe_mode==14) {
  bool spent;
  WRITE_ONCE(q2_pm_trap_gate,1);
  barrier();q2_pm_precise_trap();
  spent=READ_ONCE(q2_pm_trap_gate)==0;
  WRITE_ONCE(q2_pm_trap_gate,0);
  return spent && read_c0_status()==status &&
   read_c0_epc()==(unsigned long)q2_pm_exl_keep &&
   (read_c0_cause()&(CAUSEF_BD|CAUSEF_EXCCODE))==(13U<<2);
 }
 if (exception_probe_mode==5 || exception_probe_mode==6 ||
     exception_probe_mode==8 || exception_probe_mode==9 ||
     exception_probe_mode==10 || exception_probe_mode==11 || exception_probe_mode==12 || exception_probe_mode==13) {
  epc=read_c0_epc();value=exception_probe_mode==5 || exception_probe_mode==8 || exception_probe_mode>=10;
  if (exception_probe_mode==8 || exception_probe_mode==9 || exception_probe_mode>=10) value|=2;
  if (exception_probe_mode==10 || exception_probe_mode==12 || exception_probe_mode==13) value|=4;
  if (exception_probe_mode==11) value|=8;
  if (exception_probe_mode==12 || exception_probe_mode==13) value|=16;
  if (exception_probe_mode==13) value|=32;
  x2000_epc_probe(value);
  return read_c0_epc()==((value&2)?(unsigned long)q2_pm_exl_keep:epc) && read_c0_status()==status;
 }
 if (exception_probe_mode==2) {
  epc=read_c0_epc();write_c0_epc(epc);back_to_back_c0_hazard();
  return read_c0_epc()==epc && read_c0_status()==status;
 }
 epc=exception_probe_mode==7?read_c0_epc():0;
 value=exception_probe_load();
 if (value!=0x55443322 || read_c0_status()!=status ||
     (read_c0_cause()&(CAUSEF_BD|CAUSEF_EXCCODE))!=(4U<<2) ||
     read_c0_epc()!=(unsigned long)q2_pm_adel_resume) return false;
 return exception_probe_mode!=7 ||
  (exception_restore_epc(epc) && read_c0_status()==status);
}
/* Selection is frozen by the same sleep/admission serialization as the
 * existing cache diagnostic. Normal production has no such interface.
 */
static int exception_mode_choose(unsigned int mode)
{
 if (mode>14) return -EINVAL;
 if (admission_pid || admission_generation || journal_armed) return -EBUSY;
 if (mode && cache_probe_mode) return -EBUSY;
 WRITE_ONCE(q2_pm_trap_gate,0);
 exception_probe_mode=mode;return 0;
}
static ssize_t exception_mode_show(struct kobject *kobj,struct kobj_attribute *attr,char *buf)
{
 return sysfs_emit(buf,"version=1 selected=%u\n",READ_ONCE(exception_probe_mode));
}
static ssize_t exception_mode_store(struct kobject *kobj,struct kobj_attribute *attr,const char *buf,size_t count)
{
 unsigned int mode,flags;int ret;
 if (sysfs_streq(buf,"none")) mode=0;
 else if (sysfs_streq(buf,"adel")) mode=1;
 else if (sysfs_streq(buf,"epc-seed")) mode=2;
 else if (sysfs_streq(buf,"reset-eret")) mode=3;
 else if (sysfs_streq(buf,"restore-epc")) mode=4;
 else if (sysfs_streq(buf,"exl-eret")) mode=5;
 else if (sysfs_streq(buf,"epc-pulse")) mode=6;
 else if (sysfs_streq(buf,"adel-restore")) mode=7;
 else if (sysfs_streq(buf,"exl-keep")) mode=8;
 else if (sysfs_streq(buf,"pulse-keep")) mode=9;
 else if (sysfs_streq(buf,"exl-status-first")) mode=10;
 else if (sysfs_streq(buf,"exl-epc-barrier")) mode=11;
 else if (sysfs_streq(buf,"exl-preclear")) mode=12;
 else if (sysfs_streq(buf,"exl-cycle")) mode=13;
 else if (sysfs_streq(buf,"trap-entry")) mode=14;
 else return -EINVAL;
 flags=lock_system_sleep();ret=exception_mode_choose(mode);unlock_system_sleep(flags);
 return ret?ret:(ssize_t)count;
}
static struct kobj_attribute exception_mode_attr=__ATTR(exception_mode,0600,exception_mode_show,exception_mode_store);
/* Orthogonal owned-SLEEP clock policy. Select only without a live admission;
 * journal_store snapshots it into the caller/generation-bound request.
 * The normal production image exposes no diagnostic admission interface.
 */
static int pd0_clock_choose(bool retain)
{
 if (admission_pid || admission_generation || journal_armed) return -EBUSY;
 pd0_retain_exclk=retain;return 0;
}
static ssize_t pd0_clock_show(struct kobject *kobj,struct kobj_attribute *attr,char *buf)
{
 return sysfs_emit(buf,"version=1 retain=%u\n",READ_ONCE(pd0_retain_exclk));
}
static ssize_t pd0_clock_store(struct kobject *kobj,struct kobj_attribute *attr,const char *buf,size_t count)
{
 unsigned int flags;int ret;bool retain;
 if (sysfs_streq(buf,"retain")) retain=true;
 else if (sysfs_streq(buf,"stop")) retain=false;
 else return -EINVAL;
 flags=lock_system_sleep();ret=pd0_clock_choose(retain);unlock_system_sleep(flags);
 return ret?ret:(ssize_t)count;
}
static struct kobj_attribute pd0_clock_attr=__ATTR(pd0_clock,0600,pd0_clock_show,pd0_clock_store);

/* Explicit architectural reset-return comparison. Overwrite the SRAM choice
 * on every admitted entry, before any clock change or final cache drain.
 */
static noinline int reset_return_prepare(void)
{
 if (exception_probe_mode==3 && (read_c0_status()&7)) return -EINVAL;
 writel(exception_probe_mode==3,sram+SRAM_CONTEXT+RESET_RETURN);
 return 0;
}
/* Original EPC is captured by the actual save leaf, not sampled after reset. */
static noinline int epc_restore_prepare(void)
{
 if (exception_probe_mode==4 && (read_c0_status()&7)) return -EINVAL;
 writel(exception_probe_mode==4,sram+SRAM_CONTEXT+EPC_RESTORE);
 return 0;
}
void q2_pm_mmu_probe(unsigned long node,unsigned long ops)
{
 mmu_capture(2,node,ops,NULL,0);general_trace_probe(ops);
 if (!exception_probe_run(ops)) return;
 refill_trace_probe(ops);pre_access_probe(ops);
}
void q2_pm_mmu_fault(const struct pt_regs *regs,unsigned long address,unsigned int kind)
{
 if (regs && (kind==3 || kind==4)) mmu_capture(kind,0,0,regs,address);
}

void q2_pm_mmu_fault_origin(const struct pt_regs *regs,unsigned long address,
 unsigned int kind,unsigned int origin)
{
 if (!regs || !mmu_active() || mmu_busy || kind!=4 ||
     origin<Q2_ORIGIN_ADE_INSTRUCTION || origin>Q2_ORIGIN_NMI) return;
 origin_pending=origin;origin_pending_epc=read_c0_epc();
 mmu_capture(kind,0,0,regs,address);
 origin_pending=origin_pending_epc=0;
}
static ssize_t mmu_show(struct kobject *kobj,struct kobj_attribute *attr,char *buf)
{
 unsigned int flags=lock_system_sleep(),slot,i;
 u32 words[Q2_MMU_WORDS];ssize_t size=0;
 for (slot=0;slot<Q2_MMU_SLOTS;slot++) {
  bool matched=mmu_read(slot,words);
  size+=sysfs_emit_at(buf,size,"version=%u slot=%u matched=%u kind=%u",Q2_MMU_VERSION,slot,matched,matched?words[6]:0);
  if (matched) for (i=0;i<Q2_MMU_VALUES;i++)
   size+=sysfs_emit_at(buf,size," %s=%08x",q2_mmu_names[i],words[8+i]);
  size+=sysfs_emit_at(buf,size,"\n");
 }
 unlock_system_sleep(flags);return size;
}
static struct kobj_attribute mmu_attr=__ATTR(mmu,0400,mmu_show,NULL);

/* Only the sleep-lock owner writes this record; no asynchronous callbacks or
 * allocation/locking/DRAM logging in the IRQ-disabled syscore path. */
void q2_pm_checkpoint(unsigned int phase, unsigned long callback)
{
 void __iomem *record;
 /* syscore_suspend errors bypass SYSCORE_RESUMED. Close before IRQs/CPUs,
  * even when the remaining checkpoint admission has already disappeared. */
 if (phase==Q2_CP_IRQ_ENABLE || phase==Q2_CP_CPU_ENABLE) WRITE_ONCE(q2_pm_trap_gate,0);
 if (phase==Q2_CP_IRQ_ENABLE || phase==Q2_CP_CPU_ENABLE) WRITE_ONCE(q2_general_trace_enable,0);
 if (phase==Q2_CP_IRQ_ENABLE || phase==Q2_CP_CPU_ENABLE) {
  mmu_window=false;WRITE_ONCE(q2_refill_trace_enable,0);
 }
 if (!admission_checkpoint || !admission_pid ||
     task_pid(current) != admission_pid ||
     !q2_checkpoint_payload_valid(phase, (u32)callback))
  return;
 record = sram + Q2_CHECKPOINT_OFFSET;
 writel(~0U, record + 16); /* invalidate before replacing either payload */
 wmb();
 writel((u32)callback, record + 20);
 writel(~phase, record + 24);
 writel(~(u32)callback, record + 28);
 wmb();
 writel(phase, record + 16);
 wmb();
 readl(record + 16); /* drain posted SRAM writes before the callback */
 if (phase==Q2_CP_SYSCORE_SUSPEND) {
  mmu_window=true;mmu_capture(1,0,0,NULL,0);refill_vector(false);
 }
 if (phase==Q2_CP_SYSCORE_RESUMED) {pre_access_complete();refill_vector(true);mmu_window=false;WRITE_ONCE(q2_refill_trace_enable,0);WRITE_ONCE(q2_general_trace_enable,0);WRITE_ONCE(q2_pm_trap_gate,0);}
 if (phase==Q2_CP_PLATFORM_PRE_FLUSH) {mmu_capture(1,0,0,NULL,0);refill_vector(false);}
 if (phase==Q2_CP_SYSCORE_RESUME_NODE_REACHED) mmu_capture(2,0,0,NULL,0);
}

static bool checkpoint_read(u32 words[Q2_CHECKPOINT_WORDS])
{
 unsigned int i;
 if (!journal_owner) return false;
 for (i=0;i<4;i++) {
  words[i]=readl(sram + Q2_CHECKPOINT_OFFSET + 4*i);
  if (words[i] != journal_tag[i]) return false;
 }
 for (i=4;i<Q2_CHECKPOINT_WORDS;i++)
  words[i]=readl(sram + Q2_CHECKPOINT_OFFSET + 4*i);
 return q2_checkpoint_match(journal_tag,words);
}
static ssize_t checkpoint_show(struct kobject *kobj,
 struct kobj_attribute *attr, char *buf)
{
 unsigned int flags=lock_system_sleep();
 u32 words[Q2_CHECKPOINT_WORDS];
 bool matched=checkpoint_read(words);
 ssize_t size=sysfs_emit(buf,"version=1 matched=%u phase=%u callback=%08x\n",
  matched,matched?words[4]:0,matched?words[5]:0);
 unlock_system_sleep(flags);
 return size;
}
static struct kobj_attribute checkpoint_attr = __ATTR(checkpoint, 0400, checkpoint_show, NULL);


static int admission_cancel(void)
{
	int ret = x2000_wdt_recovery_cancel(admission_generation);

	admission_generation = 0;
	admission_normal = false;
	admission_entered = false;
	admission_roundtrip = false;
	admission_keep_exclk = false;
	admission_idle_ddr = false;
	admission_pd0 = false;
	admission_cpu_only = false;
	admission_checkpoint = false;
	admission_cache_probe = 0;
	mmu_window = false;
	WRITE_ONCE(q2_pm_trap_gate,0);
	WRITE_ONCE(q2_general_trace_enable,0);
	WRITE_ONCE(q2_refill_trace_enable,0);
	journal_armed = false;
	if (admission_pid) {
		put_pid(admission_pid);
		admission_pid = NULL;
	}
	return ret;
}


/* Caller holds the system sleep lock. Never reveal an unknown SRAM word. */
static bool journal_read(u32 *code)
{
	u32 seen[4];
	int i;

	if (!journal_owner)
		return false;
	for (i = 0; i < 3; i++) {
		seen[i] = readl(sram + SRAM_JOURNAL + 4 * i);
		if (seen[i] != journal_tag[i])
			return false;
	}
	seen[3] = readl(sram + SRAM_JOURNAL + 12);
	return q2_pm_journal_match(journal_tag, seen, 1, code);
}

static ssize_t journal_show(struct kobject *kobj, struct kobj_attribute *attr,
			    char *buf)
{
	unsigned int flags = lock_system_sleep();
	u32 code = 0;
	bool matched = journal_read(&code);
	ssize_t size = sysfs_emit(buf, "armed=%u matched=%u stage=%u failed=%u\n",
			journal_armed, matched, matched ? code & 0x7f : 0,
			matched ? !!(code & 0x80) : 0);
	unlock_system_sleep(flags);
	return size;
}

static ssize_t journal_store(struct kobject *kobj, struct kobj_attribute *attr,
			     const char *buf, size_t count)
{
	unsigned int flags;
	u32 tag[4], seen[4], code = 0, seconds = 0;
	bool observe = false, arm = false, roundtrip = false, keep_exclk = false, idle_ddr = false, pd0 = false, checkpoint = false;
	u32 cpu_only = 0;
	struct pid *owner = NULL;
	int ret;
	char text[64], *cursor, *word;
	int i;

	if (sysfs_streq(buf, "cancel")) {
		flags = lock_system_sleep();
		ret = admission_cancel();
		unlock_system_sleep(flags);
		return ret ?: (ssize_t)count;
	}
	if (sysfs_streq(buf, "clear")) {
		flags = lock_system_sleep();
		if (!journal_read(&code)) {
			unlock_system_sleep(flags);
			return -ESTALE;
		}
		{
   u32 words[Q2_CHECKPOINT_WORDS];
   if (checkpoint_read(words))
    for (i=0;i<Q2_CHECKPOINT_WORDS;i++)
     writel(0,sram + Q2_CHECKPOINT_OFFSET + i*4);
  }
  for (i = 0; i < 4; i++)
   writel(0, sram + SRAM_JOURNAL + i * 4);
		/* Complete the owned clear before releasing journal ownership. */
		wmb();
		journal_owner = false;
		ret = admission_cancel();
		unlock_system_sleep(flags);
		return ret ?: (ssize_t)count;
	}
	if (!count || count >= sizeof(text))
		return -EINVAL;
	memcpy(text, buf, count);
	text[count] = 0;
	cursor = strim(text);
	if (!strncmp(cursor, "observe ", 8)) {
		observe = true;
		cursor += 8;
	} else if (!strncmp(cursor, "arm ", 4) ||
		   !strncmp(cursor, "roundtrip ", 10) ||
		   !strncmp(cursor, "cpu-only ", 9) ||
		   !strncmp(cursor, "cpu-trace ", 10) ||
		   !strncmp(cursor, "osc-keep ", 9) ||
		   !strncmp(cursor, "idle-ddr ", 9) ||
		   !strncmp(cursor, "sleep-pd0 ", 10) ||
		   !strncmp(cursor, "core-trace ", 11)) {
		arm = true;
		checkpoint = !strncmp(cursor, "core-trace ", 11);
		roundtrip = !strncmp(cursor, "roundtrip ", 10);
		pd0 = !strncmp(cursor, "sleep-pd0 ", 10);
		keep_exclk = !strncmp(cursor, "osc-keep ", 9) || (pd0 && pd0_retain_exclk);
		idle_ddr = !strncmp(cursor, "idle-ddr ", 9);
		cpu_only = !strncmp(cursor, "cpu-trace ", 10) ? 2 :
			   !strncmp(cursor, "cpu-only ", 9);
		cursor += checkpoint ? 11 : (roundtrip || cpu_only == 2 || pd0) ? 10 : (cpu_only || keep_exclk || idle_ddr) ? 9 : 4;
		word = strsep(&cursor, ",");
		if (!word || kstrtou32(word, 10, &seconds) ||
		    seconds < 10 || seconds > 60)
			return -EINVAL;
	}
	for (i = 0; i < 4; i++) {
		word = strsep(&cursor, ",");
		if (!word || kstrtou32(word, 10, &tag[i]))
			return -EINVAL;
	}
	if (cursor || !(tag[0] | tag[1] | tag[2] | tag[3]))
		return -EINVAL;
	flags = lock_system_sleep();
	if (observe) {
		/* Adopt only the caller's previously generated marker. Do not
		 * reseed after reset or replace an armed transaction's owner.
		 */
		if (journal_armed || admission_pid || admission_generation) {
			unlock_system_sleep(flags);
			return -EBUSY;
		}
		for (i = 0; i < 4; i++)
			seen[i] = readl(sram + SRAM_JOURNAL + 4 * i);
		if (!q2_pm_journal_match(tag, seen, 1, &code)) {
			unlock_system_sleep(flags);
			return -ESTALE;
		}
		memcpy(journal_tag, tag, sizeof(tag));
		journal_owner = true;
		unlock_system_sleep(flags);
		return count;
	}
	if (arm) {
		owner = get_task_pid(current, PIDTYPE_PID);
		if (!owner) {
			unlock_system_sleep(flags);
			return -ESRCH;
		}
	}
	ret = admission_cancel();
	if (ret) {
		if (owner)
			put_pid(owner);
		unlock_system_sleep(flags);
		return ret;
	}
	admission_pid = owner;
	admission_roundtrip = roundtrip;
	admission_keep_exclk = keep_exclk;
	admission_idle_ddr = idle_ddr;
	admission_pd0 = pd0;
	admission_cpu_only = cpu_only;
	admission_checkpoint = checkpoint;
	admission_cache_probe = checkpoint ? cache_probe_mode : 0;
	admission_seconds = seconds;
	memcpy(journal_tag, tag, sizeof(tag));
	for (i = 0; i < 4; i++)
		writel(tag[i], sram + SRAM_JOURNAL + i * 4);
	/* Publish the entire generated tag before allowing an entry. */
	wmb();
	journal_owner = journal_armed = true;
 if (checkpoint) {
  refill_prepare();cache_probe_prepare();
  for (i=0;i<Q2_MMU_SLOTS;i++) writel(0,sram+Q2_MMU_OFFSET+i*Q2_MMU_STRIDE+16);
  wmb();
  writel(~0U, sram + Q2_CHECKPOINT_OFFSET + 16);
  wmb();
  for (i=0;i<4;i++) writel(tag[i],sram + Q2_CHECKPOINT_OFFSET + i*4);
  wmb();
  q2_pm_checkpoint(Q2_CP_ARMED,0);
 }
	unlock_system_sleep(flags);
	return count;
}
static struct kobj_attribute journal_attr = __ATTR(journal, 0600, journal_show, journal_store);

static u32 proof_read(bool valid, u32 offset)
{
	return valid ? readl(sram + SRAM_CONTEXT + offset) : 0;
}

/* Public numeric evidence only. Bind this to the boot ID and standard PM
	* success/failure counters: a core return precedes later device callbacks.
	* RTC samples have one-second resolution; this is not a nanosecond timer.
	*/
static ssize_t proof_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	u32 seq = READ_ONCE(attempts);
	bool valid = seq && readl(sram + SRAM_CONTEXT + 168) == 0x51325350 &&
		     readl(sram + SRAM_CONTEXT + 172) == seq;

	return sysfs_emit(buf, "attempts=%u core_returns=%u core_failures=%u "
		"valid=%u stage=%u error=%d entry_ddrc=0x%08x entry_phy=0x%08x "
		"saved_status=0x%08x resume_status=0x%08x rtc_enter=%u rtc_exit=%u "
		"roundtrip=%u roundtrip_returns=%u roundtrip_failures=%u "
		"cpu_only=%u cpu_only_returns=%u cpu_only_failures=%u entry_opcr=0x%08x entry_lcr=0x%08x saved_epc=0x%08x epc_restore=%u\n",
		seq, READ_ONCE(core_returns), READ_ONCE(core_failures), valid,
		proof_read(valid, 148), (int)proof_read(valid, 144),
		proof_read(valid, 176), proof_read(valid, 180),
		proof_read(valid, 52), proof_read(valid, 184),
		proof_read(valid, 188), proof_read(valid, 192),
		proof_read(valid, ROUNDTRIP), READ_ONCE(roundtrip_returns),
		READ_ONCE(roundtrip_failures), proof_read(valid, CPU_ONLY),
		READ_ONCE(cpu_only_returns), READ_ONCE(cpu_only_failures),
		proof_read(valid, 224), proof_read(valid, 228),
		proof_read(valid, SAVED_EPC), proof_read(valid, EPC_RESTORE));
}
static struct kobj_attribute proof_attr = __ATTR_RO(proof);

static ssize_t entry_policy_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "version=1 normal_retained=%u\n",
			 IS_ENABLED(CONFIG_INGENIC_X2000_NORMAL_SLEEP));
}
static struct kobj_attribute entry_policy_attr = __ATTR_RO(entry_policy);

static ssize_t normal_exclk_show(struct kobject *kobj,
                                 struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "version=1 retain=%u\n", READ_ONCE(normal_retain_exclk));
}

static ssize_t normal_exclk_store(struct kobject *kobj, struct kobj_attribute *attr,
                                 const char *buf, size_t count)
{
	unsigned int flags;
	bool retain;

	if (sysfs_streq(buf, "retain"))
		retain = true;
	else if (sysfs_streq(buf, "stop"))
		retain = false;
	else
		return -EINVAL;

	flags = lock_system_sleep();
	WRITE_ONCE(normal_retain_exclk, retain);
	unlock_system_sleep(flags);
	return count;
}
static struct kobj_attribute normal_exclk_attr = __ATTR(normal_exclk, 0600, normal_exclk_show, normal_exclk_store);

static bool x2000_normal_exclk_can_stop(void)
{
	/* Keep the independent RTC available. Ordinary suspend has
	 * already proved that its watchdog is stopped. The common sleep
	 * programming selects RTCLK for the watchdog in either policy.
	 */
	return rtc && (readl(rtc + RTC_RTCCR) & (RTC_ENABLE | RTC_USE_EXCLK)) == RTC_ENABLE &&
	       !(readl(cpm + CPM_CLKGR0) & CPM_RTC_GATED);
}

static int x2000_pm_normal_begin(suspend_state_t state)
{
	struct pid *owner;

	if (state != PM_SUSPEND_MEM)
		return -EINVAL;
	if (admission_pid || admission_generation || journal_armed || journal_owner ||
	    exception_probe_mode || cache_probe_mode)
		return -EBUSY;
	owner = get_task_pid(current, PIDTYPE_PID);
	if (!owner)
		return -ESRCH;
	/* Ordinary Linux sleep admission: no diagnostic generation/deadline.
	 * Force the physically qualified retained-CPU SLEEP parameters; never
	 * inherit a diagnostic selector or turn a roundtrip into normal sleep.
	 */
	admission_pid = owner;
	admission_normal = true;
	admission_entered = false;
	admission_roundtrip = false;
	admission_idle_ddr = false;
	admission_cpu_only = 0;
	admission_checkpoint = false;
	admission_cache_probe = 0;
	admission_keep_exclk = READ_ONCE(normal_retain_exclk);
	admission_pd0 = true;
	memset(journal_tag, 0, sizeof(journal_tag));
	return 0;
}

/* begin/end execute under the existing system-sleep lock; never recurse. */
static int x2000_pm_begin(suspend_state_t state)
{
	u32 generation;
	int ret;

	if (IS_ENABLED(CONFIG_INGENIC_X2000_NORMAL_SLEEP))
		return x2000_pm_normal_begin(state);

	if (state != PM_SUSPEND_MEM || !journal_armed || !admission_pid ||
	    task_pid(current) != admission_pid || admission_generation)
		return -EACCES;
	generation = ++next_generation;
	if (!generation)
		generation = ++next_generation;
	ret = x2000_wdt_recovery_request(generation, admission_seconds);
	if (ret)
		return ret;
	admission_generation = generation;
	admission_entered = false;
	q2_pm_checkpoint(Q2_CP_BEGIN,0);
	return 0;
}

static void x2000_pm_end(void)
{
	int ret = admission_cancel();

	if (ret)
		pr_err("X2000 diagnostic watchdog cleanup failed: %d\n", ret);
}

static int x2000_pm_enter(suspend_state_t state)
{
	u32 lcr, opcr, rer, cpccr, status;
	bool record, roundtrip = admission_roundtrip;
	bool keep_exclk = admission_keep_exclk;
	bool idle_ddr = admission_idle_ddr;
	bool pd0 = admission_pd0;
	u32 cpu_only = admission_cpu_only;
	int i;
	int clock_ret;
	int ret;

	attempts++;
	if ((!admission_normal && !admission_generation) || !admission_pid ||
	    task_pid(current) != admission_pid || admission_entered) {
		core_failures++;
		return -EACCES;
	}
	/* Spend before any CPU/SRAM/PM change; duplicate entry grants nothing. */
	admission_entered = true;
	if (admission_normal ? !x2000_wdt_normal_sleep_ready() :
	    !x2000_wdt_recovery_ready(admission_generation)) {
		core_failures++;
		return -EACCES;
	}

	if (admission_normal && !keep_exclk && !x2000_normal_exclk_can_stop()) {
		core_failures++;
		return -EOPNOTSUPP;
	}

	/* The SRAM index walk is the Q2's admitted D32/L2-64 geometry. CPU1
	 * must be offline too; never execute a partial walk on another geometry.
	 */
	if ((read_c0_config1() & 0xff80U) != 0x3380U ||
	    (read_c0_config2() & 0xfffU) != 0x35fU) {
		core_failures++;
		return -EOPNOTSUPP;
	}
	/* CPU1 must have flushed its caches and be held in hardware reset. */
	if (state != PM_SUSPEND_MEM || smp_processor_id() ||
	    num_online_cpus() != 1 || !(readl(ccu + CCU_CSRR) & BIT(1))) {
		core_failures++;
		return -EBUSY;
	}
	ret = reset_return_prepare();
	if (ret) {
		core_failures++;
		return ret;
	}
	ret = epc_restore_prepare();
	if (ret) {
		core_failures++;
		return ret;
	}
	/* Generic CPU-PM syscore already saved FPU/DSP and CPU context.
	 * Nested cpu_pm_enter()/exit() here violates that API's pairing contract.
	 * Its syscore resume restores ASID/UserLocal/watch after platform return.
	 */
	lcr = readl(cpm + CPM_LCR);
	opcr = readl(cpm + CPM_OPCR);
	rer = readl(ccu + CCU_RER);
	cpccr = readl(cpm + CPM_CPCCR);
	memcpy_toio(sram, x2000_sram_start, x2000_sram_end - x2000_sram_start);
	/* One actual entry consumes arming; devices-only PM rehearsals do not. */
	record = journal_armed;
	journal_armed = false;
	writel(record, sram + SRAM_CONTEXT + JOURNAL_ACTIVE);
	if (record) {
		writel(journal_tag[3], sram + SRAM_CONTEXT + JOURNAL_BASE);
		for (i = 0; i < 4; i++)
			writel(journal_tag[i], sram + SRAM_JOURNAL + i * 4);
	}

	writel((lcr & ~3U) | ((cpu_only || idle_ddr) ? 2 : 1), cpm + CPM_LCR);
	/* Stock CPU/L2, RAM/ROM sleep and RTC-source controls. Bits20..22
	 * are RAM/ROM sleep controls, not oscillator/PLL stable-time enables.
	 */
	if (cpu_only || idle_ddr) {
		/* Stock IDLE_PD: retain oscillator/DDR and use LCR mode2.
		 * Explicit SRAM retention preserves the owned reset context.
		 * idle-ddr uses the same CPU controls but runs the existing manual
		 * DDR/PHY sequence (CPU_ONLY remains zero). Neither is full SLEEP.
		 */
		writel((opcr & ~(BIT(31) | BIT(26) | BIT(3))) |
		       BIT(30) | BIT(21) | BIT(2), cpm + CPM_OPCR);
	} else {
		/* Owned osc-keep retains EXCLK; sleep-pd0 snapshots pd0_clock. Normal
		 * SLEEP snapshots its independent clock policy. Diagnostic
		 * arm/roundtrip keep the original oscillator-off entry.
		 */
		writel((opcr & ~(BIT(31) | BIT(26) | BIT(4) | BIT(3))) |
		       BIT(30) | BIT(22) | BIT(21) | BIT(20) |
		       (keep_exclk ? BIT(4) : 0) | (pd0 ? 0 : BIT(3)) | BIT(2),
		       cpm + CPM_OPCR);
	}
	writel(readl(cpm + CPM_OPCR), sram + SRAM_CONTEXT + 224);
	writel(readl(cpm + CPM_LCR), sram + SRAM_CONTEXT + 228);
	writel(SRAM_PHYS | 0xa0000000, ccu + CCU_RER);
	writel(0, sram + SRAM_CONTEXT + 168);
	writel(attempts, sram + SRAM_CONTEXT + 172);
	/* Owned generated DRAM, checked through KSEG1 after full DDR restoration.
	 * No private media/configuration is used as a memory-integrity fixture.
	 */
	status = journal_tag[0] ^ journal_tag[1] ^ journal_tag[2] ^
		 journal_tag[3] ^ attempts;
	for (i = 0; i < (int)ARRAY_SIZE(ddr_canary); i++)
		ddr_canary[i] = status ^ i;
	writel(roundtrip, sram + SRAM_CONTEXT + ROUNDTRIP);
	writel(cpu_only, sram + SRAM_CONTEXT + CPU_ONLY);
	writel((u32)CKSEG1ADDR(ddr_canary), sram + SRAM_CONTEXT + CANARY_POINTER);
	writel(status, sram + SRAM_CONTEXT + CANARY_SEED);
	/* Primary flush includes I-cache. Its wrapper can dirty bookkeeping;
	 * the SRAM leaf performs the final D/L2 drain after saving context.
	 */
	q2_pm_checkpoint(Q2_CP_PLATFORM_PRE_FLUSH,0);
	__flush_cache_all();
	ret = x2000_suspend_save((void __force *)sram +
				(x2000_sram_sleep - x2000_sram_start));
	q2_pm_checkpoint(Q2_CP_CONTEXT_RETURN,0);
	if (cpu_only == 2) {
		/* CPU restore trace v1 has a distinct journal family,0x10..0x17. */
		writel(0x16, sram + SRAM_CONTEXT + 148);
		if (record)
			writel(journal_tag[3] ^ 0x16, sram + SRAM_JOURNAL + 12);
		wmb();
	}
	writel(0, sram + SRAM_CONTEXT + JOURNAL_ACTIVE);
	local_flush_tlb_all();
	cache_probe_run();
	q2_pm_checkpoint(Q2_CP_TLB_RETURN,0);
	if (cpu_only == 2) {
		writel(0x18, sram + SRAM_CONTEXT + 148);
		if (record)
			writel(journal_tag[3] ^ 0x18, sram + SRAM_JOURNAL + 12);
		wmb();
	}
	writel(rer, ccu + CCU_RER);
	q2_pm_checkpoint(Q2_CP_RER_WRITE_ISSUED,0);
	if (cpu_only == 2) {
		writel(0x19, sram + SRAM_CONTEXT + 148);
		if (record)
			writel(journal_tag[3] ^ 0x19, sram + SRAM_JOURNAL + 12);
		wmb();
	}
	/* Leave low-power mode before touching the CPU clock, as stock does.
	 * CPCCR's live mux/other-divider state must survive CPU reset. Disable
	 * changes without changing the divider, update only CPU_DIV, then commit.
	 */
	writel(lcr, cpm + CPM_LCR);
	q2_pm_checkpoint(Q2_CP_LCR_WRITE_ISSUED,0);
	status = readl(cpm + CPM_CPCCR) & ~BIT(22);
	q2_pm_checkpoint(Q2_CP_CPU_DIV_DISABLE_PRE,0);
	writel(status, cpm + CPM_CPCCR);
	q2_pm_checkpoint(Q2_CP_CPU_DIV_DISABLE_ISSUED,0);
	if (cpu_only == 2) {
		writel(0x1a, sram + SRAM_CONTEXT + 148);
		if (record)
			writel(journal_tag[3] ^ 0x1a, sram + SRAM_JOURNAL + 12);
		wmb();
	}
	status = (status & ~0xfU) | (cpccr & 0xfU);
	q2_pm_checkpoint(Q2_CP_CPU_DIV_UPDATE_PRE,0);
	writel(status, cpm + CPM_CPCCR);
	q2_pm_checkpoint(Q2_CP_CPU_DIV_UPDATE_ISSUED,0);
	q2_pm_checkpoint(Q2_CP_CPU_DIV_COMMIT_PRE,0);
	writel(status | BIT(22), cpm + CPM_CPCCR);
	q2_pm_checkpoint(Q2_CP_CPU_DIV_COMMIT_ISSUED,0);
	if (cpu_only == 2) {
		writel(0x1b, sram + SRAM_CONTEXT + 148);
		if (record)
			writel(journal_tag[3] ^ 0x1b, sram + SRAM_JOURNAL + 12);
		wmb();
	}
	/* Completion is required before exposing the original CPU frequency. */
	q2_pm_checkpoint(Q2_CP_CLOCK_POLL_PRE,0);
	clock_ret = readl_poll_timeout_atomic(cpm + 0xd4, status,
					     !(status & 1), 1, 100000);
	q2_pm_checkpoint(Q2_CP_CLOCK_POLL_RETURNED,0);
	if (cpu_only == 2) {
		writel(0x1c, sram + SRAM_CONTEXT + 148);
		if (record)
			writel(journal_tag[3] ^ 0x1c, sram + SRAM_JOURNAL + 12);
		wmb();
	}
	q2_pm_checkpoint(Q2_CP_OPCR_WRITE_PRE,0);
	writel(opcr, cpm + CPM_OPCR);
	q2_pm_checkpoint(Q2_CP_OPCR_WRITE_ISSUED,0);
	if (cpu_only == 2) {
		writel(0x1d, sram + SRAM_CONTEXT + 148);
		if (record)
			writel(journal_tag[3] ^ 0x1d, sram + SRAM_JOURNAL + 12);
		wmb();
	}
	if (cpu_only == 2) {
		writel(0x17, sram + SRAM_CONTEXT + 148);
		if (record)
			writel(journal_tag[3] ^ 0x17, sram + SRAM_JOURNAL + 12);
		wmb();
	}
	/* Diagnostic DDR roundtrips must never certify CPU power-down/wake. */
	if (cpu_only) {
		if (ret || clock_ret)
			cpu_only_failures++;
		else
			cpu_only_returns++;
	} else if (roundtrip) {
		if (ret || clock_ret)
			roundtrip_failures++;
		else
			roundtrip_returns++;
	} else if (ret || clock_ret) {
		core_failures++;
	} else {
		core_returns++;
	}
	q2_pm_checkpoint(Q2_CP_ENTER_ATTEMPT_FINISHED,0);
	return ret ?: clock_ret;
}

/* Generic suspend-to-idle owns device PM and pending wake IRQs. Keep its
 * admission separate from the retained-DDR deep sleep admission.
 */
static bool allow_s2idle;

static ssize_t s2idle_show(struct kobject *kobj, struct kobj_attribute *attr,
			   char *buf)
{
	return sysfs_emit(buf, "%u\n", READ_ONCE(allow_s2idle));
}

static ssize_t s2idle_store(struct kobject *kobj, struct kobj_attribute *attr,
			    const char *buf, size_t count)
{
	unsigned int flags;
	bool allow;
	int ret;

	ret = kstrtobool(buf, &allow);
	if (ret)
		return ret;

	flags = lock_system_sleep();
	WRITE_ONCE(allow_s2idle, allow);
	unlock_system_sleep(flags);
	return count;
}
static struct kobj_attribute s2idle_attr = __ATTR(s2idle, 0600, s2idle_show, s2idle_store);

static int x2000_s2idle_begin(void)
{
	return READ_ONCE(allow_s2idle) ? 0 : -EINVAL;
}

static void x2000_s2idle_end(void)
{
	/* The generic PM core falls back to the deep end hook if absent. */
}

static const struct platform_s2idle_ops x2000_s2idle_ops = {
	.begin = x2000_s2idle_begin,
	.end = x2000_s2idle_end,
};

static const struct platform_suspend_ops x2000_pm_ops = {
	.begin = x2000_pm_begin,
	.end = x2000_pm_end,
	.valid = suspend_valid_only_mem,
	.enter = x2000_pm_enter,
};

static int __init x2000_pm_init(void)
{
	unsigned long size = x2000_sram_end - x2000_sram_start;
	int ret;

	/* Q2-qualified sequencing only; other X2000 boards need their own
	 * regulator/retention/wake verification before using this backend.
	 */
	if (!of_machine_is_compatible("shanling,q2"))
		return 0;
	if (!size || size > SRAM_CONTEXT)
		return -E2BIG;
	if (!request_mem_region(SRAM_PHYS, SZ_32K, "x2000-suspend"))
		return -EBUSY;
	sram = ioremap(SRAM_PHYS, SZ_32K);
	cpm = ioremap(CPM_PHYS, SZ_4K);
	ccu = ioremap(CCU_PHYS, SZ_4K);
	if (!sram || !cpm || !ccu) {
		iounmap(sram);
		iounmap(cpm);
		iounmap(ccu);
		release_mem_region(SRAM_PHYS, SZ_32K);
		return -ENOMEM;
	}
	if (IS_ENABLED(CONFIG_INGENIC_X2000_NORMAL_SLEEP))
		rtc = ioremap(RTC_PHYS, SZ_4K);
	else
		cache_probe_init_clock();
	proof_kobj = kobject_create_and_add("q2_suspend", kernel_kobj);
	if (!proof_kobj) {
		ret = -ENOMEM;
		goto unmap;
	}
	{
  static struct attribute *attributes[] = {
   &entry_policy_attr.attr, &proof_attr.attr, &journal_attr.attr, &checkpoint_attr.attr, &mmu_attr.attr, &refill_attr.attr, &origin_attr.attr, &refill_trace_attr.attr, &general_trace_attr.attr, &cache_probe_attr.attr, &pre_access_attr.attr, &exception_mode_attr.attr, &pd0_clock_attr.attr, NULL
  };
  static const struct attribute_group group = { .attrs = attributes };
  static struct attribute *normal_attributes[] = {
   &entry_policy_attr.attr, &normal_exclk_attr.attr, &proof_attr.attr, &s2idle_attr.attr, NULL
  };
  static const struct attribute_group normal_group = { .attrs = normal_attributes };
  ret = sysfs_create_group(proof_kobj,
    IS_ENABLED(CONFIG_INGENIC_X2000_NORMAL_SLEEP) ? &normal_group : &group);
  if (ret) {
   kobject_put(proof_kobj);
   goto unmap;
  }
 }
	/* Do not read/clear a predecessor journal merely because we booted. */
	writel(0, sram + SRAM_CONTEXT + JOURNAL_ACTIVE);
	suspend_set_ops(&x2000_pm_ops);
	if (IS_ENABLED(CONFIG_INGENIC_X2000_NORMAL_SLEEP))
		s2idle_set_ops(&x2000_s2idle_ops);
	pr_info("X2000: retained SRAM deep-sleep backend registered (%lu bytes)\n", size);
	return 0;
unmap:
	iounmap(rtc);
	iounmap(cache_probe_ost);
	iounmap(sram);
	iounmap(cpm);
	iounmap(ccu);
	release_mem_region(SRAM_PHYS, SZ_32K);
	return ret;
}
late_initcall(x2000_pm_init);

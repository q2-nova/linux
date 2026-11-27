/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_Q2_PM_CHECKPOINT_RECORD_H
#define _LINUX_Q2_PM_CHECKPOINT_RECORD_H
/* Separate from the legacy journal/context. Only generated nonce ownership
 * permits exposing this public diagnostic phase/callback address. Complements
 * reject torn stores; this is not authentication or a reset-retention promise. */
#define Q2_CHECKPOINT_OFFSET 0x7f40
#define Q2_CHECKPOINT_WORDS 8
#define Q2_CHECKPOINT_BYTES (4 * Q2_CHECKPOINT_WORDS)
enum q2_checkpoint_phase {
 Q2_CP_ARMED = 0, Q2_CP_BEGIN, Q2_CP_CONSOLE_SUSPEND,
 Q2_CP_DEVICE_SUSPEND, Q2_CP_DEVICE_SUSPENDED,
 Q2_CP_PLATFORM_PREPARE, Q2_CP_DEVICE_LATE, Q2_CP_DEVICE_LATE_DONE,
 Q2_CP_PLATFORM_LATE, Q2_CP_DEVICE_NOIRQ, Q2_CP_DEVICE_NOIRQ_DONE,
 Q2_CP_PLATFORM_NOIRQ, Q2_CP_CPU_DISABLE, Q2_CP_CPU_DISABLED,
 Q2_CP_IRQ_DISABLE, Q2_CP_SYSCORE_SUSPEND, Q2_CP_SYSCORE_SUSPENDED,
 Q2_CP_CORE_DELAY, Q2_CP_PLATFORM_ENTER, Q2_CP_PLATFORM_RETURNED,
 Q2_CP_SYSCORE_RESUME, Q2_CP_SYSCORE_RESUMED, Q2_CP_IRQ_ENABLE,
 Q2_CP_CPU_ENABLE, Q2_CP_CPU_ENABLED, Q2_CP_PLATFORM_WAKE,
 Q2_CP_DEVICE_NOIRQ_RESUME, Q2_CP_PLATFORM_EARLY,
 Q2_CP_DEVICE_EARLY_RESUME, Q2_CP_PLATFORM_FINISH,
 Q2_CP_DEVICE_RESUME, Q2_CP_DEVICE_RESUMED, Q2_CP_CONSOLE_RESUME,
 Q2_CP_PLATFORM_END,
 Q2_CP_SYSCORE_SUSPEND_CALL, Q2_CP_SYSCORE_SUSPEND_RETURN,
 Q2_CP_SYSCORE_RESUME_CALL, Q2_CP_SYSCORE_RESUME_RETURN,
 /* Additive C-return family. Old0..37 encodings retain their meanings. */
 Q2_CP_PLATFORM_PRE_FLUSH, Q2_CP_CONTEXT_RETURN, Q2_CP_TLB_RETURN,
 Q2_CP_RER_WRITE_ISSUED, Q2_CP_LCR_WRITE_ISSUED,
 Q2_CP_CPU_DIV_DISABLE_PRE, Q2_CP_CPU_DIV_DISABLE_ISSUED,
 Q2_CP_CPU_DIV_UPDATE_PRE, Q2_CP_CPU_DIV_UPDATE_ISSUED,
 Q2_CP_CPU_DIV_COMMIT_PRE, Q2_CP_CPU_DIV_COMMIT_ISSUED,
 Q2_CP_CLOCK_POLL_PRE, Q2_CP_CLOCK_POLL_RETURNED,
 Q2_CP_OPCR_WRITE_PRE, Q2_CP_OPCR_WRITE_ISSUED,
 Q2_CP_ENTER_ATTEMPT_FINISHED,
 /* Additive loop/guard boundaries after the observed phase37 return. */
 Q2_CP_SYSCORE_RESUME_NODE_REACHED, Q2_CP_SYSCORE_RESUME_IRQ_CHECK,
 Q2_CP_SYSCORE_RESUME_IRQ_CHECKED, Q2_CP_SYSCORE_RESUME_NODE_DONE,
 Q2_CP_SYSCORE_RESUME_TRACE_ENTER, Q2_CP_SYSCORE_RESUME_TRACE_DONE,
 Q2_CP_PHASES
};
static inline int q2_checkpoint_payload_valid(unsigned int phase, unsigned int callback)
{
 return phase < Q2_CP_PHASES &&
  (!!callback == (phase >= Q2_CP_SYSCORE_SUSPEND_CALL && phase <= Q2_CP_SYSCORE_RESUME_RETURN));
}
static inline int q2_checkpoint_match(const unsigned int tag[4],
 const unsigned int seen[Q2_CHECKPOINT_WORDS])
{
 unsigned int i;
 if (!(tag[0] | tag[1] | tag[2] | tag[3])) return 0;
 for (i=0;i<4;i++) if (tag[i] != seen[i]) return 0;
 return q2_checkpoint_payload_valid(seen[4],seen[5]) &&
  seen[6] == ~seen[4] && seen[7] == ~seen[5];
}
#endif

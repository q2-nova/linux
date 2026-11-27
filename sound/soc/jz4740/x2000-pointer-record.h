/* SPDX-License-Identifier: GPL-2.0-only */
/* Passive numeric cache. Types u32/u64 are supplied by kernel or C controls. */
#define Q2_PTR_CAPACITY 256
#define Q2_PTR_POSITION_POLL 1
#define Q2_PTR_POSITION_ALSA 2
#define Q2_PTR_POLL_END 3
#define Q2_PTR_TRIGGER 4
#define Q2_PTR_SYNC_STOP 5
#define Q2_PTR_PREPARE 6
#define Q2_PTR_ACCEPTED 1
#define Q2_PTR_INVALID 2
#define Q2_PTR_RETRY_EXHAUSTED 3
#define Q2_PTR_REGRESSED 4
struct q2_ptr_event {
	u64 sequence, ns;
	u32 kind, generation, state, running, command;
	u32 before, fsr, after, tries, disposition, previous, position;
	u32 last_pos, period_pos, delta, buffer_bytes, dma_addr;
	u32 hw_ptr, hw_base, appl_ptr, buffer_frames, period_frames;
	u32 boundary, stop_threshold, rate;
};
struct q2_ptr_record {
	u64 total, overwritten, after_freeze;
	u32 used, next, generation, frozen, saturated, arm_id;
	struct q2_ptr_event events[Q2_PTR_CAPACITY];
};
static void q2_ptr_append(struct q2_ptr_record *r, struct q2_ptr_event e,
			  bool actual_xrun)
{
	if (r->frozen) {
		if (r->after_freeze != ~(u64)0)
			r->after_freeze++;
		else
			r->saturated = 1;
		return;
	}
	if (r->total == ~(u64)0) {
		r->saturated = r->frozen = 1;
		return;
	}
	e.sequence = ++r->total;
	e.generation = r->generation;
	r->events[r->next] = e;
	r->next = (r->next + 1) % Q2_PTR_CAPACITY;
	if (r->used < Q2_PTR_CAPACITY)
		r->used++;
	else
		r->overwritten++;
	if (actual_xrun)
		r->frozen = 1;
}

/* Control is serialized with PCM open/close publication and cache writers.
 * Disarm preserves the completed evidence; arm explicitly starts a fresh cache.
 */
static int q2_ptr_control(struct q2_ptr_record *r, bool *enabled, u32 *arm_id,
			  bool pcm_open, bool arm)
{
	if (pcm_open)
		return 1;
	if (!arm) {
		WRITE_ONCE(*enabled, false);
		return 0;
	}
	if (*arm_id == ~(u32)0)
		return 2;
	WRITE_ONCE(*enabled, false);
	memset(r, 0, sizeof(*r));
	r->arm_id = ++*arm_id;
	WRITE_ONCE(*enabled, true);
	return 0;
}

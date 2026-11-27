/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef INGENIC_X2000_POWER_TRANSACTION_H
#define INGENIC_X2000_POWER_TRANSACTION_H

/* Shared by the real driver and deterministic hardware/fault controls.
 * update changes only mask under the CGU lock. wait is bounded by its caller.
 * The clock must remain enabled until either transition or rollback completes.
 */
struct x2000_power_io {
	u32 (*read)(void *context);
	void (*update)(void *context, u32 mask, u32 value);
	int (*wait)(void *context, u32 mask, u32 value);
	void *context;
};

struct x2000_power_result {
	int error;
	int rollback_error;
	bool changed;
	bool state_known;
};

static struct x2000_power_result
x2000_power_change(const struct x2000_power_io *io, u32 request,
		  u32 acknowledge, bool off)
{
	struct x2000_power_result result = { .state_known = true };
	u32 before = io->read(io->context);
	bool was_off = !!(before & request);

	/* Do not adopt an incomplete transition owned by another actor. */
	if (!!(before & acknowledge) != was_off) {
		result.error = -EBUSY;
		result.state_known = false;
		return result;
	}
	if (was_off == off)
		return result;
	result.changed = true;
	io->update(io->context, request, off ? request : 0);
	result.error = io->wait(io->context, acknowledge, off ? acknowledge : 0);
	if (result.error) {
		/* Restore only our request bit, preserving concurrent other domains
		 * and all CPU sleep/stability controls in this shared register.
		 */
		io->update(io->context, request, was_off ? request : 0);
		result.rollback_error = io->wait(io->context, acknowledge,
						was_off ? acknowledge : 0);
		result.state_known = !result.rollback_error;
	}
	return result;
}
#endif

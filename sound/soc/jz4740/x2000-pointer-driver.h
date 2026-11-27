/* SPDX-License-Identifier: GPL-2.0-only */
/* Included after x2000_audio. All producers hold the ALSA stream lock,
 * except prepare after sync_stop/timer retirement; its caller owns the PCM.
 * The separate lock protects cache-only debugfs readers, never MMIO.
 */
static void x2000_diag_event(struct x2000_audio *audio, u32 kind, u32 command,
			     u32 before, u32 fsr, u32 after, u32 tries,
			     u32 disposition, u32 previous, u32 position,
			     u32 delta)
{
	struct snd_pcm_runtime *rt = audio->substream->runtime;
	struct q2_ptr_event e = {
		.ns = ktime_get_ns(), .kind = kind, .state = rt->state,
		.running = audio->running, .command = command,
		.before = before, .fsr = fsr, .after = after, .tries = tries,
		.disposition = disposition, .previous = previous,
		.position = position, .last_pos = audio->last_pos,
		.period_pos = audio->period_pos, .delta = delta,
		.buffer_bytes = audio->buffer_bytes, .dma_addr = rt->dma_addr,
		.hw_ptr = rt->status->hw_ptr, .hw_base = rt->hw_ptr_base,
		.appl_ptr = READ_ONCE(rt->control->appl_ptr),
		.buffer_frames = rt->buffer_size, .period_frames = rt->period_size,
		.boundary = rt->boundary, .stop_threshold = rt->stop_threshold,
		.rate = rt->rate,
	};
	unsigned long flags;

	spin_lock_irqsave(&audio->diag_lock, flags);
	if (kind == Q2_PTR_PREPARE && rt->state != SNDRV_PCM_STATE_XRUN &&
	    !audio->diag->frozen) {
		if (audio->diag->generation == U32_MAX)
			audio->diag->saturated = audio->diag->frozen = 1;
		else
			audio->diag->generation++;
	}
	q2_ptr_append(audio->diag, e, rt->state == SNDRV_PCM_STATE_XRUN);
	spin_unlock_irqrestore(&audio->diag_lock, flags);
}

/* Read only a frozen cache, so the dump cannot perturb active pre-XRUN
 * sampling. The owned copy is allocated in process context, not on stack.
 * No MMIO or PCM pointer operation is performed by this interface.
 */
static int x2000_diag_show(struct seq_file *file, void *unused)
{
	struct x2000_audio *audio = file->private;
	struct q2_ptr_record *copy;
	unsigned long flags;
	u32 i;

	copy = kvzalloc(sizeof(*copy), GFP_KERNEL);
	if (!copy)
		return -ENOMEM;
	spin_lock_irqsave(&audio->diag_lock, flags);
	if (!audio->diag->frozen) {
		spin_unlock_irqrestore(&audio->diag_lock, flags);
		kvfree(copy);
		return -EAGAIN;
	}
	memcpy(copy, audio->diag, sizeof(*copy));
	spin_unlock_irqrestore(&audio->diag_lock, flags);
	seq_printf(file, "version=1 arm_id=%u capacity=%u total=%llu overwritten=%llu after_freeze=%llu used=%u generation=%u frozen=%u saturated=%u\n",
		copy->arm_id, Q2_PTR_CAPACITY, copy->total, copy->overwritten, copy->after_freeze,
		copy->used, copy->generation, copy->frozen, copy->saturated);
	for (i = 0; i < copy->used; i++) {
		const struct q2_ptr_event *e = &copy->events[
			(copy->next + Q2_PTR_CAPACITY - copy->used + i) % Q2_PTR_CAPACITY];
		seq_printf(file, "E %llu %llu %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u\n",
			e->sequence, e->ns, e->kind, e->generation, e->state,
			e->running, e->command, e->before, e->fsr, e->after,
			e->tries, e->disposition, e->previous, e->position,
			e->last_pos, e->period_pos, e->delta, e->buffer_bytes,
			e->dma_addr, e->hw_ptr, e->hw_base, e->appl_ptr,
			e->buffer_frames, e->period_frames, e->boundary,
			e->stop_threshold, e->rate);
	}
	kvfree(copy);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(x2000_diag);

/* No PCM is allowed to be open, even paused/prepared. File lifetime is
 * protected by debugfs proxy removal before devres frees this device cache.
 */
static ssize_t x2000_diag_control(struct file *file, const char __user *buf,
				  size_t count, loff_t *position)
{
	struct x2000_audio *audio = file->private_data;
	char command[8];
	unsigned long flags;
	bool arm;
	int ret;

	if (*position || (count != 4 && count != 7))
		return -EINVAL;
	if (copy_from_user(command, buf, count))
		return -EFAULT;
	if (count == 4 && !memcmp(command, "arm\n", 4))
		arm = true;
	else if (count == 7 && !memcmp(command, "disarm\n", 7))
		arm = false;
	else
		return -EINVAL;
	spin_lock_irqsave(&audio->diag_lock, flags);
	ret = q2_ptr_control(audio->diag, &audio->diag_enabled,
		&audio->diag_arm_id, audio->substream != NULL || audio->diag_opening, arm);
	spin_unlock_irqrestore(&audio->diag_lock, flags);
	if (ret)
		return ret == 1 ? -EBUSY : -EOVERFLOW;
	*position += count;
	return count;
}
static const struct file_operations x2000_diag_control_fops = {
	.owner = THIS_MODULE, .open = simple_open, .write = x2000_diag_control,
	/* NULL llseek: fs/open.c clears FMODE_LSEEK; seeks return ESPIPE. */
};

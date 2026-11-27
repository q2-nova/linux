/* SPDX-License-Identifier: GPL-2.0-only */
/* Configuration registers already read by the AXP2101 regulator driver.
 * No status/IRQ clear-on-read registers, no configuration writes or retries.
 */
#ifndef AXP2101_RAIL_SNAPSHOT_H
#define AXP2101_RAIL_SNAPSHOT_H

static const unsigned int axp2101_rail_registers[] = {
	0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
	0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97,
	0x98, 0x99, 0x9a,
};
#define AXP2101_RAIL_WORDS 19
#define AXP2101_RAIL_DCDC_WORDS 8

struct axp2101_rail_sample {
	u64 begin_ns, end_ns;
	unsigned int valid;
	int error;
	u8 values[AXP2101_RAIL_WORDS];
};

struct axp2101_rail_io {
	int (*read)(void *, unsigned int, unsigned int *);
	int (*read_block)(void *, u8, u8 *, size_t);
	u64 (*clock)(void *);
	void *context;
};

static inline void axp2101_rail_capture_blocks(struct axp2101_rail_sample *s,
					     const struct axp2101_rail_io *io)
{
	const unsigned int ldo_words = AXP2101_RAIL_WORDS - AXP2101_RAIL_DCDC_WORDS;
	u8 block[AXP2101_RAIL_WORDS - AXP2101_RAIL_DCDC_WORDS];

	/* A failed transfer cannot certify a partial range. Retain only the
	 * earlier completed prefix and report the first transport failure.
	 */
	s->error = io->read_block(io->context, axp2101_rail_registers[0],
				  block, AXP2101_RAIL_DCDC_WORDS);
	if (s->error)
		return;
	memcpy(s->values, block, AXP2101_RAIL_DCDC_WORDS);
	s->valid = AXP2101_RAIL_DCDC_WORDS;

	s->error = io->read_block(io->context, axp2101_rail_registers[s->valid],
				  block, ldo_words);
	if (s->error)
		return;
	memcpy(s->values + s->valid, block, ldo_words);
	s->valid += ldo_words;
}

static inline void axp2101_rail_capture(struct axp2101_rail_sample *s,
				      const struct axp2101_rail_io *io)
{
	unsigned int i, value;

	memset(s, 0, sizeof(*s));
	s->begin_ns = io->clock(io->context);
	if (io->read_block) {
		axp2101_rail_capture_blocks(s, io);
		goto end;
	}

	for (i = 0; i < AXP2101_RAIL_WORDS; i++) {
		s->error = io->read(io->context, axp2101_rail_registers[i], &value);
		if (s->error)
			break;
		if (value > 0xff) {
			s->error = -EIO;
			break;
		}
		s->values[i] = value;
		s->valid++;
	}
end:
	s->end_ns = io->clock(io->context);
	if (s->end_ns < s->begin_ns && !s->error)
		s->error = -EIO;
}
#endif

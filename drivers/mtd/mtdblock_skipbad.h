/* SPDX-License-Identifier: GPL-2.0-only */
/* Logical eraseblocks follow the good physical eraseblocks, as nandwrite does.
 * The caller supplies errno definitions. Kept independent of MTD I/O so the
 * exact mapping used by the driver can be exercised with generated fixtures.
 */
#ifndef MTDBLOCK_SKIPBAD_H
#define MTDBLOCK_SKIPBAD_H

static inline int skipbad_build_map(unsigned int *map, unsigned int blocks,
		int (*is_bad)(void *context, unsigned int block), void *context)
{
	unsigned int physical, good = 0;
	int ret;

	for (physical = 0; physical < blocks; physical++) {
		ret = is_bad(context, physical);
		if (ret < 0)
			return ret;
		if (!ret)
			map[good++] = physical;
	}
	return good ? (int)good : -ENOSPC;
}

static inline int skipbad_offset(const unsigned int *map, unsigned int good,
		unsigned int erasesize, unsigned long long sector,
		unsigned long long *offset)
{
	unsigned long long logical;
	unsigned int shift = 0;

	if (erasesize < 512 || (erasesize & (erasesize - 1)) ||
	    sector >= (unsigned long long)good * erasesize / 512)
		return -EIO;
	while ((1U << shift) != erasesize)
		shift++;
	logical = sector * 512;
	*offset = (unsigned long long)map[logical >> shift] * erasesize +
		(logical & (erasesize - 1));
	return 0;
}
#endif

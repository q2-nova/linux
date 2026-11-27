/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef Q2_PM_JOURNAL_H
#define Q2_PM_JOURNAL_H

/* The last word is the caller's random tag XOR a small, closed stage domain.
 * The legacy sixteen encodings retain124 bits of accidental-match resistance.
 * The additional CPU-restore-v1 family makes44 encodings and about122.54 bits overall.
 * Old codes retain their meanings;0x10..0x1d is a separate trace namespace.
 * This is an ownership discriminator, not an ECC or authentication protocol.
 */
static inline int q2_pm_journal_code_valid(unsigned int code)
{
	return code <= 7 || (code >= 0x80 && code <= 0x87) ||
	       (code >= 0x10 && code <= 0x1d) ||
	       (code >= 0x90 && code <= 0x9d);
}

static inline int q2_pm_journal_match(const unsigned int tag[4],
		const unsigned int seen[4], int journal, unsigned int *code)
{
	unsigned int value;
	int i;

	for (i = 0; i < 3; i++)
		if (seen[i] != tag[i])
			return 0;
	value = seen[3] ^ tag[3];
	if ((!journal && value) || !q2_pm_journal_code_valid(value))
		return 0;
	*code = value;
	return 1;
}
#endif

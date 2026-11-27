/* inffast.c -- fast decoding
 * Copyright (C) 1995-2004 Mark Adler
 * For conditions of distribution and use, see copyright notice in zlib.h
 */

#include <linux/zutil.h>
#include "inftrees.h"
#include "inflate.h"
#include "inffast.h"

#ifndef ASMINF

union uu {
	unsigned short us;
	unsigned char b[2];
};

/* Endian independent version */
static inline unsigned short
get_unaligned16(const unsigned short *p)
{
	union uu  mm;
	unsigned char *b = (unsigned char *)p;

	mm.b[0] = b[0];
	mm.b[1] = b[1];
	return mm.us;
}

/*
   Decode literal, length, and distance codes and write out the resulting
   literal and match bytes until either not enough input or output is
   available, an end-of-block is encountered, or a data error is encountered.
   When large enough input and output buffers are supplied to inflate(), for
   example, a 16K input buffer and a 64K output buffer, more than 95% of the
   inflate execution time is spent in this routine.

   Entry assumptions:

        state->mode == LEN
        strm->avail_in >= 6
        strm->avail_out >= 258
        start >= strm->avail_out
        state->bits < 8

   On return, state->mode is one of:

        LEN -- ran out of enough output space or enough available input
        TYPE -- reached end of block code, inflate() to interpret next block
        BAD -- error in block data

   Notes:

    - The maximum input bits used by a length/distance pair is 15 bits for the
      length code, 5 bits for the length extra, 15 bits for the distance code,
      and 13 bits for the distance extra.  This totals 48 bits, or six bytes.
      Therefore if strm->avail_in >= 6, then there is enough input to avoid
      checking for available input while decoding.

    - The maximum bytes that a single length/distance pair can output is 258
      bytes, which is the maximum length that can be coded.  inflate_fast()
      requires strm->avail_out >= 258 for each loop to avoid checking for
      output space.

    - @start:	inflate()'s starting value for strm->avail_out
 */
void inflate_fast(z_streamp strm, unsigned start)
{
    struct inflate_state *state;
    const unsigned char *in;    /* local strm->next_in */
    const unsigned char *last;  /* while in < last, enough input available */
    unsigned char *out;         /* local strm->next_out */
    unsigned char *beg;         /* inflate()'s initial strm->next_out */
    unsigned char *end;         /* while out < end, enough space available */
#ifdef INFLATE_STRICT
    unsigned dmax;              /* maximum distance from zlib header */
#endif
    unsigned wsize;             /* window size or zero if not using window */
    unsigned whave;             /* valid bytes in the window */
    unsigned write;             /* window write index */
    unsigned char *window;      /* allocated sliding window, if wsize != 0 */
    unsigned long hold;         /* local strm->hold */
    unsigned bits;              /* local strm->bits */
    code const *lcode;          /* local strm->lencode */
    code const *dcode;          /* local strm->distcode */
    unsigned lmask;             /* mask for first level of length codes */
    unsigned dmask;             /* mask for first level of distance codes */
    code this;                  /* retrieved table entry */
    unsigned op;                /* code bits, operation, extra bits, or */
                                /*  window position, window bytes to copy */
    unsigned len;               /* match length, unused bytes */
    unsigned dist;              /* match distance */
    unsigned char *from;        /* where to copy match from */

    /* copy state to local variables */
    state = (struct inflate_state *)strm->state;
    in = strm->next_in;
    last = in + (strm->avail_in - 5);
    out = strm->next_out;
    beg = out - (start - strm->avail_out);
    end = out + (strm->avail_out - 257);
#ifdef INFLATE_STRICT
    dmax = state->dmax;
#endif
    wsize = state->wsize;
    whave = state->whave;
    write = state->write;
    window = state->window;
    hold = state->hold;
    bits = state->bits;
    lcode = state->lencode;
    dcode = state->distcode;
    lmask = (1U << state->lenbits) - 1;
    dmask = (1U << state->distbits) - 1;

    /* decode literals and length/distances until end-of-block or not enough
       input data or output space */
    do {
        if (bits < 15) {
            hold += (unsigned long)(*in++) << bits;
            bits += 8;
            hold += (unsigned long)(*in++) << bits;
            bits += 8;
        }
        this = lcode[hold & lmask];
      dolen:
        op = (unsigned)(this.bits);
        hold >>= op;
        bits -= op;
        op = (unsigned)(this.op);
        if (op == 0) {                          /* literal */
            *out++ = (unsigned char)(this.val);
        }
        else if (op & 16) {                     /* length base */
            len = (unsigned)(this.val);
            op &= 15;                           /* number of extra bits */
            if (op) {
                if (bits < op) {
                    hold += (unsigned long)(*in++) << bits;
                    bits += 8;
                }
                len += (unsigned)hold & ((1U << op) - 1);
                hold >>= op;
                bits -= op;
            }
            if (bits < 15) {
                hold += (unsigned long)(*in++) << bits;
                bits += 8;
                hold += (unsigned long)(*in++) << bits;
                bits += 8;
            }
            this = dcode[hold & dmask];
          dodist:
            op = (unsigned)(this.bits);
            hold >>= op;
            bits -= op;
            op = (unsigned)(this.op);
            if (op & 16) {                      /* distance base */
                dist = (unsigned)(this.val);
                op &= 15;                       /* number of extra bits */
                if (bits < op) {
                    hold += (unsigned long)(*in++) << bits;
                    bits += 8;
                    if (bits < op) {
                        hold += (unsigned long)(*in++) << bits;
                        bits += 8;
                    }
                }
                dist += (unsigned)hold & ((1U << op) - 1);
#ifdef INFLATE_STRICT
                if (dist > dmax) {
                    strm->msg = (char *)"invalid distance too far back";
                    state->mode = BAD;
                    break;
                }
#endif
                hold >>= op;
                bits -= op;
                op = (unsigned)(out - beg);     /* max distance in output */
                if (dist > op) {                /* see if copy from window */
                    op = dist - op;             /* distance back in window */
                    if (op > whave) {
                        strm->msg = (char *)"invalid distance too far back";
                        state->mode = BAD;
                        break;
                    }
                    from = window;
                    if (write == 0) {           /* very common case */
                        from += wsize - op;
                        if (op < len) {         /* some from window */
                            len -= op;
                            do {
                                *out++ = *from++;
                            } while (--op);
                            from = out - dist;  /* rest from output */
                        }
                    }
                    else if (write < op) {      /* wrap around window */
                        from += wsize + write - op;
                        op -= write;
                        if (op < len) {         /* some from end of window */
                            len -= op;
                            do {
                                *out++ = *from++;
                            } while (--op);
                            from = window;
                            if (write < len) {  /* some from start of window */
                                op = write;
                                len -= op;
                                do {
                                    *out++ = *from++;
                                } while (--op);
                                from = out - dist;      /* rest from output */
                            }
                        }
                    }
                    else {                      /* contiguous in window */
                        from += write - op;
                        if (op < len) {         /* some from window */
                            len -= op;
                            do {
                                *out++ = *from++;
                            } while (--op);
                            from = out - dist;  /* rest from output */
                        }
                    }
                    while (len > 2) {
                        *out++ = *from++;
                        *out++ = *from++;
                        *out++ = *from++;
                        len -= 3;
                    }
                    if (len) {
                        *out++ = *from++;
                        if (len > 1)
                            *out++ = *from++;
                    }
                }
                else {
		    unsigned short *sout;
		    unsigned long loops;

                    from = out - dist;          /* copy direct from output */
		    /* minimum length is three */
		    /* Align out addr */
		    if (!((long)(out - 1) & 1)) {
			*out++ = *from++;
			len--;
		    }
		    sout = (unsigned short *)(out);
		    if (dist > 2) {
			unsigned short *sfrom;

			sfrom = (unsigned short *)(from);
			loops = len >> 1;
			do {
			    if (IS_ENABLED(CONFIG_HAVE_EFFICIENT_UNALIGNED_ACCESS))
				*sout++ = *sfrom++;
			    else
				*sout++ = get_unaligned16(sfrom++);
			} while (--loops);
			out = (unsigned char *)sout;
			from = (unsigned char *)sfrom;
		    } else { /* dist == 1 or dist == 2 */
			unsigned short pat16;

			pat16 = *(sout-1);
			if (dist == 1) {
				union uu mm;
				/* copy one char pattern to both bytes */
				mm.us = pat16;
				mm.b[0] = mm.b[1];
				pat16 = mm.us;
			}
			loops = len >> 1;
			do
			    *sout++ = pat16;
			while (--loops);
			out = (unsigned char *)sout;
		    }
		    if (len & 1)
			*out++ = *from++;
                }
            }
            else if ((op & 64) == 0) {          /* 2nd level distance code */
                this = dcode[this.val + (hold & ((1U << op) - 1))];
                goto dodist;
            }
            else {
                strm->msg = (char *)"invalid distance code";
                state->mode = BAD;
                break;
            }
        }
        else if ((op & 64) == 0) {              /* 2nd level length code */
            this = lcode[this.val + (hold & ((1U << op) - 1))];
            goto dolen;
        }
        else if (op & 32) {                     /* end-of-block */
            state->mode = TYPE;
            break;
        }
        else {
            strm->msg = (char *)"invalid literal/length code";
            state->mode = BAD;
            break;
        }
    } while (in < last && out < end);

    /* return unused bytes (on entry, bits < 8, so in won't go too far back) */
    len = bits >> 3;
    in -= len;
    bits -= len << 3;
    hold &= (1U << bits) - 1;

    /* update state and return */
    strm->next_in = in;
    strm->next_out = out;
    strm->avail_in = (unsigned)(in < last ? 5 + (last - in) : 5 - (in - last));
    strm->avail_out = (unsigned)(out < end ?
                                 257 + (end - out) : 257 - (out - end));
    state->hold = hold;
    state->bits = bits;
    return;
}

/*
   inflate_fast() speedups that turned out slower (on a PowerPC G3 750CXe):
   - Using bit fields for code structure
   - Different op definition to avoid & for extra bits (do & for table bits)
   - Three separate decoding do-loops for direct, window, and write == 0
   - Special case for distance > 1 copies to do overlapped load and store copy
   - Explicit branch predictions (based on measured branch probabilities)
   - Deferring match copy and interspersed it with decoding subsequent codes
   - Swapping literal/length else
   - Swapping window/direct else
   - Larger unrolled copy loops (three is about right)
   - Moving len -= 3 statement into middle of loop
 */

#endif /* !ASMINF */

#ifdef ZLIB_INFLATE_VARIANTS
/*
 * Chunked match copies for the inflate_fast() variants (inffast_chunk.h).
 * COPY_FAR(out, from, len): len >= 1 bytes from a source that is in another
 * buffer or at least one chunk behind out; writes and reads whole chunks.
 * COPY_NEAR(out, dist, len): len >= 1 bytes from out - dist; returns out + len.
 */
#include <linux/unaligned.h>

static __always_inline void inffast_copy_far8(unsigned char *out,
		const unsigned char *from, unsigned len)
{
    do {
        u32 a = get_unaligned((const u32 *)from);
        u32 b = get_unaligned((const u32 *)(from + 4));

        put_unaligned(a, (u32 *)out);
        put_unaligned(b, (u32 *)(out + 4));
        out += 8;
        from += 8;
    } while (len > 8 ? (len -= 8, 1) : 0);
}

static __always_inline unsigned char *inffast_copy_near8(unsigned char *out,
		unsigned dist, unsigned len)
{
    unsigned char *end = out + len;
    u32 v;

    if (dist >= 8) {
        inffast_copy_far8(out, out - dist, len);
        return end;
    }
    switch (dist) {
    case 1:
        v = out[-1] * 0x01010101u;
        break;
    case 2:
        v = get_unaligned((const u16 *)(out - 2)) * 0x00010001u;
        break;
    case 4:
        v = get_unaligned((const u32 *)(out - 4));
        break;
    default: {
        const unsigned char *from = out - dist;

        do {
            *out++ = *from++;
        } while (--len);
        return end;
    }
    }
    do {
        put_unaligned(v, (u32 *)out);
        put_unaligned(v, (u32 *)(out + 4));
        out += 8;
    } while (out < end);
    return end;
}

#ifdef ZLIB_INFLATE_MSA
#define INFFAST_MSA_BEGIN ".set push\n\t.set reorder\n\t.set noat\n\t" \
                          ".set hardfloat\n\t.set fp=64\n\t.set msa\n\t"
#define INFFAST_MSA_END ".set pop\n\t"

static __always_inline void inffast_copy_far16(unsigned char *out,
		const unsigned char *from, unsigned len)
{
    unsigned char *end = out + len;

    __asm__ __volatile__(
        INFFAST_MSA_BEGIN
        "1:\n\t"
        "ld.b $w0, 0(%[from])\n\t"
        "addiu %[from], %[from], 16\n\t"
        "st.b $w0, 0(%[out])\n\t"
        "addiu %[out], %[out], 16\n\t"
        "sltu $1, %[out], %[end]\n\t"
        "bnez $1, 1b\n\t"
        INFFAST_MSA_END
        : [out] "+r" (out), [from] "+r" (from)
        : [end] "r" (end)
        : "$1", "memory");
}

/* vshf.b control: element i takes source byte i % dist */
static const unsigned char inffast_pattern[16][16] __aligned(16) = {
#define P(d) { 0 % d, 1 % d, 2 % d, 3 % d, 4 % d, 5 % d, 6 % d, 7 % d, \
               8 % d, 9 % d, 10 % d, 11 % d, 12 % d, 13 % d, 14 % d, 15 % d }
    { 0 }, P(1), P(2), P(3), P(4), P(5), P(6), P(7),
    P(8), P(9), P(10), P(11), P(12), P(13), P(14), P(15)
#undef P
};

static __always_inline unsigned char *inffast_copy_near16(unsigned char *out,
		unsigned dist, unsigned len)
{
    unsigned char *end = out + len;
    unsigned adv;

    if (dist >= 16) {
        inffast_copy_far16(out, out - dist, len);
        return end;
    }
    /* one 16-byte pattern of period dist, stored every 16 - 16 % dist bytes */
    adv = 16 - 16 % dist;
    __asm__ __volatile__(
        INFFAST_MSA_BEGIN
        "ld.b $w0, 0(%[src])\n\t"
        "ld.b $w1, 0(%[ctl])\n\t"
        "vshf.b $w1, $w0, $w0\n\t"
        "1:\n\t"
        "st.b $w1, 0(%[out])\n\t"
        "addu %[out], %[out], %[adv]\n\t"
        "sltu $1, %[out], %[end]\n\t"
        "bnez $1, 1b\n\t"
        INFFAST_MSA_END
        : [out] "+r" (out)
        : [src] "r" (out - dist), [ctl] "r" (inffast_pattern[dist]),
          [adv] "r" (adv), [end] "r" (end)
        : "$1", "memory");
    return end;
}
#endif /* ZLIB_INFLATE_MSA */

#define INFLATE_FAST_NAME inflate_fast_chunk8
#define CHUNK_MSA 0
#define CHUNK_TAIL 0
#include "inffast_chunk.h"

#define INFLATE_FAST_NAME inflate_fast_chunk8_tail
#define CHUNK_MSA 0
#define CHUNK_TAIL 1
#include "inffast_chunk.h"

#ifdef ZLIB_INFLATE_MSA
#define INFLATE_FAST_NAME inflate_fast_msa16
#define CHUNK_MSA 1
#define CHUNK_TAIL 0
#include "inffast_chunk.h"

#define INFLATE_FAST_NAME inflate_fast_msa16_tail
#define CHUNK_MSA 1
#define CHUNK_TAIL 1
#include "inffast_chunk.h"

/*
 * Adler-32 with MSA, 32 bytes per step: for a run of n bytes after which
 * the sums were s1, s2,
 *   s1' = s1 + sum(b),   s2' = s2 + n * s1 + sum((n - i) * b[i]).
 * Per 32-byte step j with byte sum S_j and W_j = sum((32 - i) * b[32j + i]):
 *   sum((n - i) * b[i]) = 32 * sum over j of (S_0 + ... + S_{j-1}) + sum(W_j).
 * Vector lanes hold S (w3), the running prefix sum PS (w4) and W (w5) over
 * at most ADLER_MSA_RUN bytes, small enough that no u32 lane overflows; the
 * scalar code reduces modulo 65521 after each run, as zlib_adler32() does.
 */
#define ADLER_BASE 65521U
#define ADLER_MSA_RUN 4096U

static const unsigned char adler_weights[32] __aligned(16) = {
    32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17,
    16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1
};

uLong zlib_adler32_msa(uLong adler, const Byte *buf, uInt len)
{
    unsigned long s1 = adler & 0xffff;
    unsigned long s2 = (adler >> 16) & 0xffff;

    if (buf == NULL)
        return 1L;

    while (len >= 32) {
        unsigned n = len < ADLER_MSA_RUN ? len & ~31U : ADLER_MSA_RUN;
        const Byte *p = buf, *end = buf + n;
        unsigned long s, ps, w;
        u32 l0, l1, l2, l3, m0, m1, m2, m3, n0, n1, n2, n3;

        __asm__ __volatile__(
            INFFAST_MSA_BEGIN
            "ld.b $w6, 0(%[wt])\n\t"
            "ld.b $w7, 16(%[wt])\n\t"
            "ldi.b $w3, 0\n\t"
            "ldi.b $w4, 0\n\t"
            "ldi.b $w5, 0\n\t"
            "1:\n\t"
            "ld.b $w0, 0(%[p])\n\t"
            "ld.b $w1, 16(%[p])\n\t"
            "addv.w $w4, $w4, $w3\n\t"
            "hadd_u.h $w8, $w0, $w0\n\t"
            "hadd_u.h $w9, $w1, $w1\n\t"
            "dotp_u.h $w10, $w0, $w6\n\t"
            "dotp_u.h $w11, $w1, $w7\n\t"
            "addv.h $w8, $w8, $w9\n\t"
            "addv.h $w10, $w10, $w11\n\t"
            "hadd_u.w $w8, $w8, $w8\n\t"
            "hadd_u.w $w10, $w10, $w10\n\t"
            "addiu %[p], %[p], 32\n\t"
            "addv.w $w3, $w3, $w8\n\t"
            "addv.w $w5, $w5, $w10\n\t"
            "bne %[p], %[end], 1b\n\t"
            "copy_s.w %[l0], $w3[0]\n\t"
            "copy_s.w %[l1], $w3[1]\n\t"
            "copy_s.w %[l2], $w3[2]\n\t"
            "copy_s.w %[l3], $w3[3]\n\t"
            "copy_s.w %[m0], $w4[0]\n\t"
            "copy_s.w %[m1], $w4[1]\n\t"
            "copy_s.w %[m2], $w4[2]\n\t"
            "copy_s.w %[m3], $w4[3]\n\t"
            "copy_s.w %[n0], $w5[0]\n\t"
            "copy_s.w %[n1], $w5[1]\n\t"
            "copy_s.w %[n2], $w5[2]\n\t"
            "copy_s.w %[n3], $w5[3]\n\t"
            INFFAST_MSA_END
            : [p] "+r" (p), [l0] "=&r" (l0), [l1] "=&r" (l1), [l2] "=&r" (l2),
              [l3] "=&r" (l3), [m0] "=&r" (m0), [m1] "=&r" (m1), [m2] "=&r" (m2),
              [m3] "=&r" (m3), [n0] "=&r" (n0), [n1] "=&r" (n1), [n2] "=&r" (n2),
              [n3] "=&r" (n3)
            : [end] "r" (end), [wt] "r" (adler_weights)
            : "memory");
        s = l0 + l1 + l2 + l3;
        ps = (unsigned long)(m0 + m1 + m2 + m3) % ADLER_BASE;
        w = n0 + n1 + n2 + n3;
        s2 = (s2 + n * s1 + 32 * ps + w) % ADLER_BASE;
        s1 = (s1 + s) % ADLER_BASE;
        buf += n;
        len -= n;
    }
    while (len--) {
        s1 += *buf++;
        s2 += s1;
    }
    s1 %= ADLER_BASE;
    s2 %= ADLER_BASE;
    return (s2 << 16) | s1;
}
#endif /* ZLIB_INFLATE_MSA */
#endif /* ZLIB_INFLATE_VARIANTS */

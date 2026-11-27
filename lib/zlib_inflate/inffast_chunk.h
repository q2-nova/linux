/* SPDX-License-Identifier: Zlib */
/*
 * inflate_fast() with chunked match copies, included by inffast.c once per
 * variant. Before inclusion define:
 *
 *   INFLATE_FAST_NAME  the function's name
 *   CHUNK_MSA          1: 16-byte MSA copies (caller holds kernel_msa_begin()),
 *                      0: 8-byte copies with unaligned 32-bit loads and stores
 *   CHUNK_TAIL         1: decode until the output buffer is full instead of
 *                      stopping 257 bytes before its end
 *
 * The decoding is inflate_fast()'s, unchanged. What differs:
 *
 * - A match is copied in whole chunks when the output buffer has room for the
 *   match plus one chunk. A chunked copy may then write up to CHUNK - 1 bytes
 *   past the end of the match; those bytes lie inside the buffer the caller
 *   passed to this inflate() call and are overwritten by the following output
 *   or, at the end of the stream, left beyond the returned length. Reads may
 *   likewise extend up to CHUNK - 1 bytes past the bytes copied: in the output
 *   buffer (same bound) and in the window (inflate_workspace pads it).
 * - Every byte at and before the current output position is final when the
 *   next copy starts, so a copy from the output at distance >= CHUNK reads
 *   only final bytes, and a copy at distance < CHUNK replicates the first
 *   dist bytes, read once.
 * - Matches that do not fit with that margin are copied exactly, with
 *   inflate_fast()'s own loops.
 * - CHUNK_TAIL: the loop runs while out < the buffer's end. A match longer
 *   than the remaining space is left to inflate()'s MATCH state
 *   (state->length, state->offset), which copies what fits and resumes in
 *   the next call, as it does when the slow path decodes a match.
 */

#if CHUNK_MSA
#define CHUNK 16
#define COPY_FAR inffast_copy_far16
#define COPY_NEAR inffast_copy_near16
#else
#define CHUNK 8
#define COPY_FAR inffast_copy_far8
#define COPY_NEAR inffast_copy_near8
#endif

void INFLATE_FAST_NAME(z_streamp strm, unsigned start)
{
    struct inflate_state *state;
    const unsigned char *in;    /* local strm->next_in */
    const unsigned char *last;  /* while in < last, enough input available */
    unsigned char *out;         /* local strm->next_out */
    unsigned char *beg;         /* inflate()'s initial strm->next_out */
    unsigned char *end;         /* while out < end, keep decoding */
    unsigned char *limit;       /* end of the output buffer */
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
    limit = out + strm->avail_out;
#if CHUNK_TAIL
    end = limit;
#else
    end = out + (strm->avail_out - 257);
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
                }
#if CHUNK_TAIL
                if (len > (unsigned)(limit - out)) {
                    /* finish in inflate()'s MATCH state */
                    state->length = len;
                    state->offset = dist;
                    state->mode = MATCH;
                    break;
                }
#endif
                if (len + CHUNK <= (unsigned)(limit - out)) {
                    if (dist > (unsigned)(out - beg)) {
                        /* op: distance back in window */
                        from = window;
                        if (write == 0) {
                            from += wsize - op;
                        }
                        else if (write < op) {  /* wrap around window */
                            from += wsize + write - op;
                            op -= write;
                            if (op < len) {     /* some from end of window */
                                COPY_FAR(out, from, op);
                                out += op;
                                len -= op;
                                from = window;
                                op = write;
                            }
                        }
                        else {                  /* contiguous in window */
                            from += write - op;
                        }
                        if (op < len) {         /* some from window */
                            COPY_FAR(out, from, op);
                            out += op;
                            len -= op;
                            out = COPY_NEAR(out, dist, len);
                        }
                        else {
                            COPY_FAR(out, from, len);
                            out += len;
                        }
                    }
                    else
                        out = COPY_NEAR(out, dist, len);
                }
                else if (dist > (unsigned)(out - beg)) {
                    /* exact copy from the window: inflate_fast()'s loops */
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
                    /* exact copy from the output, byte by byte */
                    from = out - dist;
                    do {
                        *out++ = *from++;
                    } while (--len);
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
    strm->avail_out = (unsigned)(limit - out);
    state->hold = hold;
    state->bits = bits;
    return;
}

#undef CHUNK
#undef COPY_FAR
#undef COPY_NEAR
#undef INFLATE_FAST_NAME
#undef CHUNK_MSA
#undef CHUNK_TAIL

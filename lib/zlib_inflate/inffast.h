/* inffast.h -- header to use inffast.c
 * Copyright (C) 1995-2003 Mark Adler
 * For conditions of distribution and use, see copyright notice in zlib.h
 */

/* WARNING: this file should *not* be used by applications. It is
   part of the implementation of the compression library and is
   subject to change. Applications should only use zlib.h.
 */

void inflate_fast (z_streamp strm, unsigned start);

#ifdef ZLIB_INFLATE_VARIANTS
/* inflate_fast() with chunked copies (inffast_chunk.h); see inflate.c */
void inflate_fast_chunk8(z_streamp strm, unsigned start);
void inflate_fast_chunk8_tail(z_streamp strm, unsigned start);
#ifdef ZLIB_INFLATE_MSA
void inflate_fast_msa16(z_streamp strm, unsigned start);
void inflate_fast_msa16_tail(z_streamp strm, unsigned start);
uLong zlib_adler32_msa(uLong adler, const Byte *buf, uInt len);
#endif
#endif

# SPDX-License-Identifier: GPL-2.0-only
#
# This is a modified version of zlib, which does all memory
# allocation ahead of time.
#
# This is only the decompression, see zlib_deflate for the
# the compression
#
# Decompression needs to be serialized for each memory
# allocation.
#
# (The upsides of the simplification is that you can't get in
# any nasty situations wrt memory management, and that the
# uncompression can be done without blocking on allocation).
#

obj-$(CONFIG_ZLIB_INFLATE) += zlib_inflate.o

zlib_inflate-objs := inffast.o inflate.o infutil.o \
		     inftrees.o inflate_syms.o

# Q2: the decompressor at -O2 instead of CONFIG_CC_OPTIMIZE_FOR_SIZE (the
# last -O wins); measured in tests/perf/decomp.
ccflags-y += -O2

# SPDX-License-Identifier: GPL-2.0-only
lzo_compress-objs := lzo1x_compress.o lzo1x_compress_safe.o
lzo_decompress-objs := lzo1x_decompress_safe.o

obj-$(CONFIG_LZO_COMPRESS) += lzo_compress.o
obj-$(CONFIG_LZO_DECOMPRESS) += lzo_decompress.o

# Q2: the decompressor at -O2 instead of CONFIG_CC_OPTIMIZE_FOR_SIZE (the
# last -O wins); measured in tests/perf/decomp.
CFLAGS_lzo1x_decompress_safe.o += -O2

# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the linux squashfs routines.
#

obj-$(CONFIG_SQUASHFS) += squashfs.o
squashfs-y += block.o cache.o dir.o export.o file.o fragment.o id.o inode.o
squashfs-y += namei.o super.o symlink.o decompressor.o page_actor.o
squashfs-$(CONFIG_SQUASHFS_FILE_CACHE) += file_cache.o
squashfs-$(CONFIG_SQUASHFS_FILE_DIRECT) += file_direct.o
squashfs-$(CONFIG_SQUASHFS_DECOMP_SINGLE) += decompressor_single.o
squashfs-$(CONFIG_SQUASHFS_DECOMP_MULTI) += decompressor_multi.o
squashfs-$(CONFIG_SQUASHFS_DECOMP_MULTI_PERCPU) += decompressor_multi_percpu.o
squashfs-$(CONFIG_SQUASHFS_XATTR) += xattr.o xattr_id.o
squashfs-$(CONFIG_SQUASHFS_LZ4) += lz4_wrapper.o
squashfs-$(CONFIG_SQUASHFS_LZO) += lzo_wrapper.o
squashfs-$(CONFIG_SQUASHFS_XZ) += xz_wrapper.o
squashfs-$(CONFIG_SQUASHFS_ZLIB) += zlib_wrapper.o
squashfs-$(CONFIG_SQUASHFS_ZSTD) += zstd_wrapper.o

# Q2: the decompressor wrappers at -O2 instead of CONFIG_CC_OPTIMIZE_FOR_SIZE
# (the last -O wins); measured in tests/perf/decomp.
CFLAGS_lzo_wrapper.o += -O2
CFLAGS_xz_wrapper.o += -O2
CFLAGS_zlib_wrapper.o += -O2
CFLAGS_zstd_wrapper.o += -O2

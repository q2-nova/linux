.. SPDX-License-Identifier: GPL-2.0

=============================
Shanling Q2 production kernel
=============================

The Q2 production profile is based on Linux v7.3. It carries the X2000
platform, the Shanling board description and drivers for the player's audio,
display, controls, touchscreen, storage and radio interfaces. The kernel,
device tree and modules must come from one build. System suspend and wake
experiments are kept on a separate branch with their own configuration.

Configurations
==============

``arch/mips/configs/shanling_q2_production_defconfig``
  The resolved production configuration. Its ``CONFIG_INITRAMFS_SOURCE`` is
  empty; the build supplies the rescue initramfs.

``arch/mips/configs/shanling_q2_defconfig``
  The original board configuration from the development series. It is not
  the production configuration.

The panel touchscreen driver, ``CONFIG_INPUT_SHANLING_Q2_TOUCHSCREEN``, is
built as the ``shanling-q2-touchscreen`` module. It binds the
``shanling,q2-touchscreen`` controller at address 0x15 on I2C3, with reset on
PA14, interrupt on PA15 and the ALDO3 supply. The controller is powered and
scanned only while its input device is open and the system is awake. The
touch wheel is a separate device and driver, ``shanling-q2-wheel``.

Build inputs
============

The production configuration was resolved with Debian GCC 14.2.0 and GNU
binutils 2.44 for the little-endian MIPS target. Install the Linux build
dependencies, the ``mipsel-linux-gnu-`` cross toolchain and U-Boot
``mkimage``. Provide the rescue initramfs from the selected Q2 OS build as a
path. For a compile-only check, an uncompressed synthetic ``newc`` archive
is sufficient. A device release requires the matching OS rescue build.

Building
========

From the source root, use a fresh output directory::

    : "${Q2_RESCUE:?set Q2_RESCUE to the rescue archive}"
    test -f "$Q2_RESCUE"
    Q2_RESCUE=$(realpath "$Q2_RESCUE")
    out=$(mktemp -d)
    cp arch/mips/configs/shanling_q2_production_defconfig "$out/.config"
    scripts/config --file "$out/.config" --set-str INITRAMFS_SOURCE "$Q2_RESCUE"
    export KBUILD_BUILD_USER=q2 KBUILD_BUILD_HOST=q2-build KBUILD_BUILD_VERSION=1
    export KBUILD_BUILD_TIMESTAMP='Thu Jan 1 00:00:00 UTC 1970'
    make O="$out" ARCH=mips CROSS_COMPILE=mipsel-linux-gnu- -j2 olddefconfig
    grep '^CONFIG_LTO_NONE=y$' "$out/.config"
    make O="$out" ARCH=mips CROSS_COMPILE=mipsel-linux-gnu- -j2 vmlinux dtbs modules
    SOURCE_DATE_EPOCH=0 make O="$out" ARCH=mips CROSS_COMPILE=mipsel-linux-gnu- -j2 uzImage.bin

``olddefconfig`` should change only ``CONFIG_INITRAMFS_SOURCE``. Compare the
resolved ``$out/.config`` with the production defconfig before using the
build.

Outputs
=======

The build produces:

- ``$out/vmlinux``, release ``7.3.0-q2`` with a ``+`` suffix when built from
  an untagged revision;
- ``$out/arch/mips/boot/dts/ingenic/shanling-q2.dtb``;
- ``$out/arch/mips/boot/uzImage.bin``, the U-Boot wrapped kernel;
- 49 in-tree modules, including
  ``drivers/input/misc/shanling-q2-touchscreen.ko``.

Check the module set with::

    find "$out" -name '*.ko' | wc -l

Use the release value from ``$out/include/config/kernel.release`` for module
installation. The OS and release tools take the kernel wrapper, DTB and
module set of one build for an install image.

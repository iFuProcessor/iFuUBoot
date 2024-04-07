#!/bin/sh
# NOTE: please read BUILD_NOTE.md before using this script
export ARCH=la32r
export CROSS_COMPILE=../loongson-gnu-toolchain-8.3-x86_64-loongarch32r-linux-gnusf-v2.0/bin/loongarch32r-linux-gnusf-
clear && make -j8 && loongarch32r-linux-gnusf-objdump -lS u-boot > u-boot.S
loongarch32r-linux-gnusf-objcopy ./u-boot -O binary u-boot.bin
loongarch32r-linux-gnusf-objdump -dS u-boot > u-boot.s

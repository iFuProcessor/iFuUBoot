#!/bin/sh
export DEVICE_TREE=la32rmega_demo
export ARCH=la32r
export CROSS_COMPILE=../loongson-gnu-toolchain-8.3-x86_64-loongarch32r-linux-gnusf-v2.0/bin/loongarch32r-linux-gnusf-
#clear && make la32rmega_defconfig && make && loongarch32r-linux-gnusf-objdump -S u-boot > u-boot.S
clear && make -j8 && loongarch32r-linux-gnusf-objdump -lS u-boot > u-boot.S
loongarch32r-linux-gnusf-objcopy ./u-boot -O binary u-boot.bin
loongarch32r-linux-gnusf-objdump -dS u-boot > u-boot.s

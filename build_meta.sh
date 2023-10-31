#!/bin/sh
export DEVICE_TREE=la32rmega_demo
export ARCH=la32r
export CROSS_COMPILE=loongarch32r-linux-gnusf-
#clear && make la32rmega_defconfig && make && loongarch32r-linux-gnusf-objdump -S u-boot > u-boot.S
clear && make -j8 && loongarch32r-linux-gnusf-objdump -lS u-boot > u-boot.S
loongarch32r-linux-gnusf-objcopy ./u-boot -O binary u-boot.bin

#!/bin/sh
../qemu-system-loongarch32 -M ls3a5k32 -bios \
    ./u-boot.bin -nographic -monitor \
    tcp::4278,server,nowait -gdb tcp::1234 

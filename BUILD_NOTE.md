# BX100E SOC 手动构建命令
```sh
export ARCH=la32r
export CROSS_COMPILE=loongarch32r-linux-gnusf-
clear && make la32rmega_defconfig && make && loongarch32r-linux-gnusf-objdump -S u-boot > u-boot.S
loongarch32r-linux-gnusf-objcopy ./u-boot -O binary u-boot.bin
```

# BX100E SOC 自动构建命令
```sh
make clean
make la32rmega_defconfig
./build_meta.sh
```

# LOONGSON SOC 手动构建命令
```sh
export ARCH=la32r
export CROSS_COMPILE=loongarch32r-linux-gnusf-
clear && make la32rsoc_defconfig && make && loongarch32r-linux-gnusf-objdump -S u-boot > u-boot.S
loongarch32r-linux-gnusf-objcopy ./u-boot -O binary u-boot.bin
```

# LOONGSON SOC 自动动构建命令
```sh
make clean
make la32rsoc_defconfig
./build_meta.sh
```

# Linux SD卡加载启动命令(仅用于BAIXIN SOC)
```sh
fatload mmc 0 0xa0e00000 vmlinux
fatload mmc 0:0 0xa0e00000 vmlinux
bootelf 0xa0e00000 g console=ttyS0,115200  
bootelf 0xa0e00000 g console=ttyS0,115200 rdinit=/init 
bootelf 0xa0e00000 g console=ttyS0,115200 rdinit=/sbin/init
```

# Linux 网络加载启动命令
```sh
setenv serverip 192.168.1.123
setenv ipaddr 192.168.1.122
printenv
tftpboot 0xa0e00000 vmlinux
bootelf 0xa0e00000 console=ttyS0,115200  
bootelf 0xa0e00000 console=ttyS0,115200 rdinit=/init 
bootelf 0xa0e00000 console=ttyS0,115200 rdinit=/sbin/init
```

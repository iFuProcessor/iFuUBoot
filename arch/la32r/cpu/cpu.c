// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2003
 * Wolfgang Denk, DENX Software Engineering, <wd@denx.de>
 */

#include <common.h>
#include <command.h>
#include <linux/compiler.h>
#include <asm/cache.h>
#include <asm/la32rregs.h>
#include <asm/reboot.h>

#ifndef CONFIG_SYSRESET
void __weak _machine_restart(void)
{
    fprintf(stderr, "This function isn't implemented\n");
    while (1);
}

int do_reset(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	_machine_restart();

	return 0;
}
#endif

int print_cpuinfo(void)
{
	printf("LoongArch!\n");
	return 0;
}
int arch_cpu_init(void)
{
	la32r_cache_probe();
	return 0;
}

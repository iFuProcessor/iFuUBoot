/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (c) 2012 The Chromium OS Authors.
 */

#ifndef __ASM_LA32R_SECTIONS_H
#define __ASM_LA32R_SECTIONS_H

#include <asm-generic/sections.h>

/**
 * __rel_start: Relocation data generated by the la32r-relocs tool
 *
 * See arch/la32r/lib/reloc.c for details on the format & use of this data.
 */
extern uint8_t __rel_start[];

#endif

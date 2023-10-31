/* SPDX-License-Identifier: GPL-2.0+ */
/*
 *  Startup Code for LA32R CPU-core
 *
 *  Copyright (c) 2023 
 */

#include <asm-offsets.h>
#include <config.h>
#include <asm/asm.h>
#include <asm/regdef.h>
#include <asm/la32rregs.h>
#include <asm/addrspace.h>
#include <asm/ns16550.h>

#ifndef CONFIG_SYS_INIT_SP_ADDR
#define CONFIG_SYS_INIT_SP_ADDR	(CONFIG_SYS_SDRAM_BASE + \
				CONFIG_SYS_INIT_SP_OFFSET)
#endif

.macro setup_stack_gd
    li.w	t0, -16
    li.w	t1, CONFIG_SYS_INIT_SP_ADDR
    and		sp, t1, t0		# force 16 byte alignment
    addi.w	sp, sp, -GD_SIZE		# reserve space for gd
    and		sp, sp, t0		# force 16 byte alignment
    add.w	x0, sp, zero			# save gd pointer
#if CONFIG_VAL(SYS_MALLOC_F_LEN)
	li.w	t2, CONFIG_VAL(SYS_MALLOC_F_LEN)
	sub.w	sp, sp, t2		# reserve space for early malloc
	and		sp, sp, t0		# force 16 byte alignment
#endif
	add.w	fp, sp, zero

	/* Clear gd */
	add.w	t0, x0, zero
1:
	st.w	zero, t0, 0
	addi.w	t0, t0, 4
	blt	t0, t1, 1b

#if CONFIG_VAL(SYS_MALLOC_F_LEN)
	st.w	sp, x0, GD_MALLOC_BASE
#endif
	.endm

#define PRINTSTR(x) \                                                     
    .section .rodata;98: .asciz x; .text; la a0, 98b; bl stringserial     
#define PRINT_CSR(offset)	\
	PRINTSTR("\r\ncsr 0x");	\
	li.w	a0, offset;	\
	bl	hexserial;	\
	PRINTSTR(" ->0x");	\
	csrrd	a0, offset;	\
	bl	hexserial;	\
	PRINTSTR("\r\n");

ENTRY(_start)
	/* U-Boot entry point */
	bl	reset

uncached:
    nop 

	.org 0x1000
	/* s0 in different stage should fixup */
	la	a0, _start
	li.w	a1, PHYS_TO_UNCACHED(0x1c000000)
	sub.w	a0, a0, a1
	li.w	a1, 0xffff0000
	and	a0, a0, a1
	beq	a0, s0, 1f
	or	s0, zero, zero
1:
	and	s0, s0, a0

	PRINTSTR("\r\nCPU Trigger exception!\r\n")
	PRINT_CSR(0x4);
	PRINT_CSR(0x5);
	PRINT_CSR(0x6);
	PRINT_CSR(0x7);
	PRINT_CSR(0x8);
1:
	b	1b

reset:
    la      s0, uncached 
    sub.w   s0, ra, s0

    li.w    t0, PHYS_TO_CACHED(0x1c001000)
    csrwr   t0, csr_eentry
    li.w    t0, 0x1c001000
    csrwr   t0, csr_tlbrentry

    /* cacheed window, cache attribute is set after */
    li.w    t1, CACHED_MEMORY_ADDR | 0x9 
    csrwr   t1, csr_dmw0 
    /* uncacheed window, temporary */
    li.w    t1, UNCACHED_MEMORY_ADDR | 0x9  
    csrwr   t1, csr_dmw1

    /*spi speed up*/       
    li.w    t0, 0x1fe80000  
    li.w    t1, 0x17    
    st.b    t1, t0, 4  
     
    bl      initserial

    bl      cache_init                  
    //PRINTSTR("\r\nCache init over\r\n")
                                                                                          
    /* enable DATF */
    li.w    t1, 0x20 
    li.w    t2, 0x60 
    csrxchg t1, t2, csr_crmd 

    /*sdram_config*/
    li.w      t0, 0x1fd00000 
    /* 33M and 26M */
    li.w      t1, 0x44518a   
    st.w      t1, t0, 0x410
    /* 33M and 26M */
    li.w      t1, 0x24    
    st.w      t1, t0, 0x414
    /* 33M and 26M */
    li.w      t1, 0x224       
    st.w      t1, t0, 0x414
    //PRINTSTR("\r\nSDRAM init over\r\n")


    //PRINTSTR("\r\nstart copy all program\r\n") 
    la      t0, _start 
    la      t2, _end  
    
    add.w   t1, t0, s0 /*link addr to 0x1c000000*/

    /*copy text section*/ 
loop:
    ld.w    t3, t1, 0 
    st.w    t3, t0, 0 
    addi.w  t0, t0, 4 
    addi.w  t1, t1, 4 
    bne     t2, t0, loop

    li.w    t1, CACHED_MEMORY_ADDR | 0x19 //cache window 
    csrwr   t1, csr_dmw0 

    la      t0, c_main
    jirl    zero, t0, 0
c_main:
    //PRINTSTR("\r\nbefore set gd\r\n")
 	/* Set up initial stack and global data */
	setup_stack_gd

    /* enable address translation */
    li.w    t1, 0x10 
    li.w    t2, 0x18 
    csrwr   t1, csr_crmd 

	add.w	a0, zero, zero		# a0 <-- boot_flags = 0
	add.w	ra, zero,zero
	b	board_init_f


END(_start)

/******************************************************
 *used: a0~a4
 ******************************************************/
LEAF(cache_init)
     li.w      t1, 256  #cycle 256
     li.w      t0, 0x0
1:
     cacop   0x0, t0, 0x0  #0 way  icache
     cacop   0x0, t0, 0x1  #1 way  icache
     cacop   0x1, t0, 0x0  #0 way  dcache
     cacop   0x1, t0, 0x1  #1 way  dcache 
     addi.w  t0, t0, 1<<4
     addi.w  t1, t1, -1
     blt     zero, t1, 1b
     /* cache_init_finish */
     jirl    zero, ra, 0
END(cache_init)

/******************************************************
 *used: a0~a2
 ******************************************************/
LEAF(tgt_putchar)
    li.w    t0, COM1_BASE_ADDR
1:
    ld.b    t1, t0, NS16550_LSR
    andi    t1, t1, LSR_TXRDY
    beq     t1, zero, 1b

    st.b    a0, t0, NS16550_DATA
    jirl    zero, ra, 0
END(tgt_putchar)

LEAF(stringserial)
    move    a2, ra
    add.w   a1, a0, s0
    ld.b    a0, a1, 0
1:
    beq     a0, zero, 2f
    addi.w  a1, a1, 1
    bl      tgt_putchar
    ld.b    a0, a1, 0
    b       1b
2:
    jirl    zero, a2, 0
END(stringserial)

/*****************************************************
 *used: a0~a5
 *****************************************************/
LEAF(hexserial)
     move    a2, ra
     move    a1, a0
     li.w    a3, 7
1:
     slli.w  a4, a3, 2
     srl.w   a0, a1, a4
     andi    a0, a0, 0xf
     la      t0, hexchar
     add.w   t0, t0, s0
     add.w   t0, t0, a0
     ld.b    a0, t0, 0
     bl      tgt_putchar
     addi.w  a3, a3, -1
     bge     a3, zero, 1b
     jirl    zero, a2, 0
END(hexserial)

LEAF(tgt_getchar)
    li.w    t0, COM1_BASE_ADDR
1:
    ld.b    t1, t0, NS16550_LSR
    andi    t1, t1, LSR_TXRDY
    beq     t1, zero, 1b
    ld.b    v0, t0, NS16550_DATA
    jirl    zero, ra, 0
END(tgt_getchar)

LEAF(initserial)
    /* COM0_BASE_ADDR:0x1fe40000 */
    li.w   t0, COM0_BASE_ADDR    
1:
    li.w   t1, FIFO_ENABLE|FIFO_RCV_RST|FIFO_XMT_RST|FIFO_TRIGGER_4
    st.b   t1, t0, NS16550_FIFO

 ##set baud rate
    li.w    t1, CFCR_DLAB
    st.b    t1, t0, NS16550_CFCR
    /* Lc modify Baut 115200 33M */
    li.w    t1, 0x12 
    st.b    t1, t0, NS16550_DATA
    srli.w  t1, t1, 8
    st.b    t1, t0, NS16550_IER
    li.w    t1, CFCR_8BITS
    st.b    t1, t0, NS16550_CFCR

    li.w    t1, CFCR_8BITS
    st.b    t1, t0, NS16550_CFCR
    li.w    t1, MCR_DTR|MCR_RTS
    st.b    t1, t0, NS16550_MCR
    li.w    t1, 0x0
    st.b    t1, t0, NS16550_IER

    /* uart int enable */
    li.w    t0, CONFREG_BASE
    li.w    t1, 0x1000
    or      t0, t0, t1
    li.w    t1, 0x1
    st.w    t1, t0, 0x44 #EN
    st.w    t1, t0, 0x50 #POL

    jirl    zero, ra, 0
END(initserial)
    .section .rodata
hexchar:
     .ascii "0123456789abcdef"


#define wp a0
#define xp a1
#define yp a2
#define tos a3
#define st a4
#define ss a5
#define rp s0
#define ip s1
#define up tp

	.equ sscomp, (1 << 0)
	.equ ssdund, (1 << 1)

	.equ addrsize, 4
	.equ stksize, 20
	.equ tasksize, 8
	.equ tibsize, 31

	.equ RAMBAK_ADDR, 0x3800
	.equ RAMBAK_SIZE, 0x800

	.equ tip, (0 * addrsize)
	.equ tss, (1 * addrsize)
	.equ tsp, (2 * addrsize)
	.equ tst, (3 * addrsize)
	.equ trp, (4 * addrsize)
	.equ tnp, (5 * addrsize)

	.global _start
_start:
	.option norvc;
	j reset
	.option rvc;

reset:
	.equ RCC_BASE, 0x40021000
	.equ RCC_CTLR, 0x00
	.equ RCC_PLLRDY, (1 << 25)
	.equ RCC_PLLON, (1 << 24)

        li wp, RCC_BASE
	lw xp, RCC_CTLR(wp)
	li yp, RCC_PLLON
	or xp, xp, yp
	sw xp, RCC_CTLR(wp)
	li yp, RCC_PLLRDY

	.macro nop2
		nop
		nop
	.endm

	.macro nop4
		nop2
		nop2
	.endm

	.macro nop8
		nop4
		nop4
	.endm

	.macro nop16
		nop8
		nop8
	.endm

1:
	nop16
	lw xp, RCC_CTLR(wp)
	and xp, xp, yp
	beqz xp, 1b


	.equ RCC_CFGR0, 0x04
	.equ RCC_SW_MASK, (0x3 << 0)
	.equ RCC_SW_PLL, (0x2 << 0)
	lw xp, RCC_CFGR0(wp)
	li yp, RCC_SW_MASK
	xori yp, yp, -1
	and xp, xp, yp
	ori xp, xp, RCC_SW_PLL
	sw xp, RCC_CFGR0(wp)

        .equ RCC_HPRE_MASK, (0xF << 4)
        .equ RCC_HPRE_NODIV, (0 << 4)
        lw xp, RCC_CFGR0(wp)
        li yp, RCC_HPRE_MASK
        xori yp, yp, -1
        and xp, xp, yp
        ori xp, xp, RCC_HPRE_NODIV
        sw xp, RCC_CFGR0(wp)

        .equ RCC_RSTSCKR, 0x24
        .equ RCC_LSION, (1 << 0)
        .equ RCC_LSIRDY, (1 << 1)
        lw xp, RCC_RSTSCKR(wp)
        or xp, xp, RCC_LSION
        sw xp, RCC_RSTSCKR(wp)

1:
        lw xp, RCC_RSTSCKR(wp)
        andi xp, xp, RCC_LSIRDY
        beqz xp, 1b

	# SYSCLK: 48Mhz
	# HCLK:   48Mhz
	# HSI:    128Khz

	.equ RCC_APB2PCENR, 0x18
	.equ RCC_AFIOEN, (1 << 0)
	.equ RCC_IOPAEN, (1 << 2)
	.equ RCC_IOPDEN, (1 << 5)
	.equ RCC_USART1EN, (1 << 14)
	lw xp, RCC_APB2PCENR(wp)
	li yp, RCC_AFIOEN | RCC_IOPAEN | RCC_IOPDEN | RCC_USART1EN
	or xp, xp, yp
	sw xp, RCC_APB2PCENR(wp)

	.equ AFIO_BASE, 0x40010000
	.equ AFIO_PCFR1, 0x04
	.equ AFIO_UART_MASK, (1 << 21) | (1 << 2)
	.equ AFIO_UART_D6TX, (1 << 21) | (0 << 2)
	li wp, AFIO_BASE
	lw xp, AFIO_PCFR1(wp)
	li yp, AFIO_UART_MASK
	xori yp, yp, -1
	and xp, xp, yp
	li yp, AFIO_UART_D6TX
	or xp, xp, yp
	sw xp, AFIO_PCFR1(wp)

	.equ GPIOA_BASE, 0x40010800
	.equ GPIO_CFGLR, 0x00
	.equ GPIOA2_CFGMASK, (0xF << 8)
	.equ GPIOA2_PPOUT_30M, (0x3 << 8)
	.equ GPIO_OUTDR, 0x0C

	li wp, GPIOA_BASE
	lw xp, GPIO_CFGLR(wp)
	li yp, GPIOA2_CFGMASK
	xori yp, yp, -1
	and xp, xp, yp
	ori xp, xp, GPIOA2_PPOUT_30M
	sw xp, GPIO_CFGLR(wp)

	.equ GPIOD_BASE, 0x40011400
	.equ GPIOD6_CFGMASK, (0xF << 24)
	.equ GPIOD6_ODOUT_MU_30M, (0xF << 24)
	li wp, GPIOD_BASE
	lw xp, GPIO_CFGLR(wp)
	li yp, GPIOD6_CFGMASK
	xori yp, yp, -1
	and xp, xp, yp
	li yp, GPIOD6_ODOUT_MU_30M
	or xp, xp, yp
	sw xp, GPIO_CFGLR(wp)

	// wchlinke max baudrate is 921600
	// ch32v003 max baudrate is 3000000

	// UART:
	// HCLK / (16 * UARTDIV) = BAUDRATE
	// 48Mhz / (16 * 3.255) = 921658
	// UARTDIV = DIV_M + (DIV_F / 16)
	// 3.25 = 3 + (4 / 16)
	// DIV_M = 3
	// DIV_F = 4
	.equ UART1_BASE, 0x40013800
	.equ UART_DATAR, 0x04
	.equ UART_BRR, 0x08
	.equ UART_CTLR1, 0x0C
	.equ UART_UE, (1 << 13)
	.equ UART_TE, (1 << 3)
	.equ UART_RE, (1 << 2)
	.equ UART_CTLR3, 0x14
	.equ UART_HDSEL, (1 << 3)

	.equ BAUD_921600, (3 << 4) | (4 << 0)
	.equ BAUD_3000000, (1 << 4) | (0 << 0)

	li wp, UART1_BASE
	li xp, BAUD_921600
	sw xp, UART_BRR(wp)
	lw xp, UART_CTLR1(wp)
	li yp, UART_UE | UART_TE | UART_RE
	or xp, xp, yp
	sw xp, UART_CTLR1(wp)
	lw xp, UART_CTLR3(wp)
	ori xp, xp, UART_HDSEL
	sw xp, UART_CTLR3(wp)

	// IWDG
	.equ IWDG_BASE, 0x40003000
	.equ IWDG_CTLR_R16, 0x00
	.equ IWDG_PSCR_R16, 0x04
	.equ IWDG_RLDR_R16, 0x08
	.equ IWDG_STATR_R16, 0xC
	.equ IWDG_RVU, (1 << 1)
	.equ IWDG_PVU, (1 << 0)
	.equ IWDG_KEYFEED, 0xAAAA
	.equ IWDG_KEYUNLOCK, 0x5555
	.equ IWDG_KEYON, 0xCCCC
	li wp, IWDG_BASE
	li xp, IWDG_KEYUNLOCK
	sh xp, IWDG_CTLR_R16(wp)

	.equ IWDG_DIVMASK, 0x7
	.equ IWDG_DIV256, 0x6
1:
	lhu xp, IWDG_STATR_R16(wp)
	andi xp, xp, IWDG_PVU
	bnez xp, 1b

	li yp, IWDG_DIVMASK
	xori yp, yp, -1
	lhu xp, IWDG_PSCR_R16(wp)
	and xp, xp, yp
	li yp, IWDG_DIV256
	or xp, xp, yp
	sh xp, IWDG_PSCR_R16(wp)

1:
	lw xp, IWDG_STATR_R16(wp)
	andi xp, xp, IWDG_RVU
	bnez xp, 1b

	li xp, -1
	sh xp, IWDG_RLDR_R16(wp)

	// set vector
	la wp, _ram_entry
	ori wp, wp, 0x2
	csrw mtvec, wp

	// enable global interrupt and configure privileged mode
	li wp, 0x1880
	csrw mstatus, wp
	// not enable hardware stack, mainline gcc compiler not suppot this
	// not enable interrupt nesting

	// PFIC
	.equ PFIC_BASE, 0xE000E000
        .equ PFIC_IENR1,0x100
        .equ IRQ_STK, 12
        li wp, PFIC_BASE
        lw xp, PFIC_IENR1(wp)
        li yp, (1 << IRQ_STK)
        or xp, xp, yp
        sw xp, PFIC_IENR1(wp)
        .equ PFIC_IENR2,0x104
        .equ IRQ_UART1, 0
        lw xp, PFIC_IENR2(wp)
        li yp, (1 << IRQ_UART1)
        or xp, xp, yp
        sw xp, PFIC_IENR2(wp)

        // systick
        .equ STK_BASE, 0xE000F000
        .equ STK_CTLR, 0x00
        .equ STK_SR,   0x04
        .equ STK_CNTL, 0x08
        .equ STK_CMPLR,0x10

        .equ STK_SWIE, (1 << 31)
        .equ STK_STRE, (1 << 3)
        .equ STK_HCLK, (1 << 2)
        .equ STK_HCLKDIV8, (0 << 2)
        .equ STK_STIE, (1 << 1)
        .equ STK_STEN, (1 << 0)

	.equ ESIG_BASE, 0x1FFFF700
	.equ ESIG_UNIID1, 0xE8
	.equ ESIG_UNIID2, 0xEC
	.equ ESIG_UNIID3, 0xF0

	.macro next
		j a_next
	.endm

	.equ sram_base, 0x20000000
	.equ sram_size, (2 * 1024)

	li wp, sram_base
	li xp, sram_size
	li yp, -1

	// fill 0xFF into sram
2:
	beqz xp, 1f
	sw yp, 0(wp)
	addi wp, wp, addrsize
	addi xp, xp, -addrsize
	j 2b
1:

	la wp, ycount
	sw zero, 0(wp)

	la wp, irqcount
	sw zero, 0(wp)

	la wp, mscountl
	sw zero, 0(wp)
	la wp, mscounth
	sw zero, 0(wp)

	la wp, _ram_entry
	la xp, irq_handler
	sw xp, 0(wp)


        // jump to forth
	la wp, forth
	csrw mepc, wp
	mret

	.set lastword, 0

	.equ link_mask, (0x20003FFC)
	.equ nlen_shift, 16
	.equ nlen_mask, (0xF << nlen_shift)
	.equ attr_immed, (1 << 0)
	.macro wdef label, name, entry, link, attr
		.p2align 2, 0xFF
	h_\label:
		.word \link + nlen_shift_\label + \attr
	f_\label:
		.word \entry
	n_\label:
		.ascii "\name"
	n_end_\label:
		.set nlen_\label, (n_end_\label - n_\label)
		.set nlen_shift_\label, (nlen_\label << nlen_shift)
		.set lastword, f_\label
		.p2align 2, 0xFF
	p_\label:
	.endm


	.equ UART_STATR, 0x00
	.equ UART_TC, (1 << 6)
early_txc:
	li wp, UART1_BASE
1:
	lw xp, UART_STATR(wp)
	andi xp, xp, UART_TC
	beqz xp, 1b
	sw tos, UART_DATAR(wp)
	ret

	wdef fail, "fail", a_fail, 0, 0
a_fail:
	li tos, 'F'
	call early_txc
	li tos, 'A'
	call early_txc
	li tos, 'I'
	call early_txc
	li tos, 'L'
	call early_txc
1:
	j 1b

	wdef okay, "okay", a_okay, f_fail, 0
a_okay:
	li tos, 'O'
	call early_txc
	li tos, 'K'
	call early_txc
1:
	j 1b

	wdef next, "next", a_next, f_okay, 0
a_next:
	lw wp, 0(ip)
	addi ip, ip, addrsize
	lw xp, 0(wp)
	jr xp

dpop:
	addi sp, sp, addrsize
	ble sp, st, 1f
	ori ss, ss, ssdund
1:
	lw tos, 0(sp)
	ret

dpush:
	sw tos, 0(sp)
	addi sp, sp, -addrsize
	ret

rpop:
	addi rp, rp, addrsize
	lw tos, 0(rp)
	ret

rpush:
	sw tos, 0(rp)
	addi rp, rp, -addrsize
	ret

	wdef call, "call", a_call, f_next, 0
a_call:
	mv tos, ip
	call rpush
	lw xp, -addrsize(wp)
	li yp, nlen_mask
	and xp, xp, yp
	srli xp, xp, nlen_shift
	addi xp, xp, ((-1) + addrsize + addrsize)
	andi xp, xp, -addrsize
	add ip, wp, xp
	next

	wdef exit, "exit", a_exit, f_call, 0
a_exit:
	call rpop
	mv ip, tos
	next

	wdef noop, "noop", a_call, f_exit, 0
	.word f_exit

	wdef branch, "branch", a_branch, f_noop, 0
a_branch:
	lw ip, 0(ip)
	next

	wdef branch0, "branch0", a_branch0, f_branch, 0
a_branch0:
	call dpop
	bnez tos, 1f
	lw ip, 0(ip)
	next
1:
	addi ip, ip, addrsize
	next

	wdef lit, "lit", a_lit, f_branch0, 0
a_lit:
	lw tos, 0(ip)
	call dpush
	addi ip, ip, addrsize
	next

tasksave:
	sw ip, tip(up)
	sw ss, tss(up)
	sw sp, tsp(up)
	sw st, tst(up)
	sw rp, trp(up)
	ret

taskload:
	lw ip, tip(up)
	lw ss, tss(up)
	lw sp, tsp(up)
	lw st, tst(up)
	lw rp, trp(up)
	ret

	wdef yield, "yield", a_yield, f_lit, 0
a_yield:
	call tasksave
	la wp, ycount
	lw xp, 0(wp)
	addi xp, xp, 1
	sw xp, 0(wp)
	lw up, tnp(up)
	call taskload
	next

	wdef txava, "tx?", a_txava, f_yield, 0
a_txava:
	li wp, UART1_BASE
	lw xp, UART_STATR(wp)
	andi xp, xp, UART_TC
	bnez xp, 1f
	mv tos, zero
	call dpush
	next
1:
	li tos, -1
	call dpush
	next

	wdef drop, "drop", a_drop, f_txava, 0
a_drop:
	call dpop
	next

	wdef dup, "dup", a_dup, f_drop, 0
a_dup:
	call dpop
	call dpush
	call dpush
	next

	wdef equ, "=", a_equ, f_dup, 0
a_equ:
	call dpop
	mv wp, tos
	call dpop
	beq wp, tos, 1f
	mv tos, zero
	call dpush
	next
1:
	li tos, -1
	call dpush
	next

	wdef 2lit, "2lit", a_2lit, f_equ, 0
a_2lit:
	lw tos, 0(ip)
	call dpush
	addi ip, ip, addrsize
	lw tos, 0(ip)
	call dpush
	addi ip, ip, addrsize
	next

	wdef failez, "failez", a_call, f_2lit,0
	.word f_lit
	.word 0
	.word f_equ
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_exit

	wdef failnz, "failnz", a_call, f_failez, 0
	.word f_lit
	.word 0
	.word f_equ
	.word f_branch0
	.word 1f
	.word f_exit
1:
	.word f_fail
	.word f_exit

	wdef txfill, "txfill", a_txfill, f_failnz, 0
a_txfill:
	li wp, UART1_BASE
	call dpop
	sw tos, UART_DATAR(wp)
	next

	wdef txwait, "txwait", a_call, f_txfill, 0
1:
	.word f_yield
	.word f_txava
	.word f_branch0
	.word 1b
	.word f_exit

	wdef txc, "txc", a_call, f_txwait, 0
	.word f_txwait
	.word f_txfill
	.word f_txwait
	.word f_exit

	wdef emit, "emit", a_call, f_txc, 0
	.word f_txc
	.word f_exit

	wdef add, "+", a_add, f_emit, 0
a_add:
	call dpop
	mv wp, tos
	call dpop
	add tos, tos, wp
	call dpush
	next

	wdef inc, "1+", a_call, f_add, 0
	.word f_lit
	.word 1
	.word f_add
	.word f_exit

	wdef sub, "-", a_sub, f_inc, 0
a_sub:
	call dpop
	mv wp, tos
	call dpop
	sub tos, tos, wp
	call dpush
	next

	wdef dec, "1-", a_call, f_sub, 0
	.word f_lit
	.word 1
	.word f_sub
	.word f_exit

	wdef cload, "c@", a_cload, f_dec, 0
a_cload:
	call dpop
	lbu tos, 0(tos)
	call dpush
	next

	wdef 2drop, "2drop", a_call, f_cload, 0
	.word f_drop
	.word f_drop
	.word f_exit

	wdef swap, "swap", a_swap, f_2drop, 0
a_swap:
	call dpop
	mv wp, tos
	call dpop
	mv xp, tos
	mv tos, wp
	call dpush
	mv tos, xp
	call dpush
	next

	wdef type, "type", a_call, f_swap, 0
2:
	.word f_dup
	.word f_branch0
	.word 1f
	.word f_dec
	.word f_swap
	.word f_dup
	.word f_cload
	.word f_emit
	.word f_inc
	.word f_swap
	.word f_branch
	.word 2b
1:
	.word f_2drop
	.word f_exit

	wdef stget, "st@", a_stget, f_type, 0
a_stget:
	mv tos, st
	call dpush
	next

	wdef spget, "sp@", a_spget, f_stget, 0
a_spget:
	mv tos, sp
	call dpush
	next

	wdef doconst, "doconst", a_doconst, f_spget, 0
a_doconst:
        lw xp, -addrsize(wp)
        li yp, nlen_mask
        and xp, xp, yp
        srli xp, xp, nlen_shift
        addi xp, xp, ((-1) + addrsize + addrsize)
        andi xp, xp, -addrsize
        add wp, wp, xp
	lw tos, 0(wp)
	call dpush
        next

	wdef cell, "cell", a_doconst, f_doconst, 0
	.word addrsize

	wdef dzchk, "dzchk", a_dzchk, f_cell, 0
a_dzchk:
	bne sp, st, a_fail
	andi ss, ss, ssdund
	bnez ss, a_fail
	next

	wdef rshift, "rshift", a_rshift, f_dzchk, 0
a_rshift:
	call dpop
	mv wp, tos
	call dpop
	srl tos, tos, wp
	call dpush
	next

	wdef 2div, "2/", a_call, f_rshift, 0
	.word f_lit
	.word 1
	.word f_rshift
	.word f_exit

	wdef celldiv, "cell/", a_call, f_2div, 0
	.word f_lit
	.word 2
	.word f_rshift
	.word f_exit

	wdef depth, "depth", a_call, f_celldiv, 0
	.word f_spget
	.word f_stget
	.word f_swap
	.word f_sub
	.word f_celldiv
	.word f_exit

	wdef and, "and", a_and, f_depth, 0
a_and:
	call dpop
	mv wp, tos
	call dpop
	and tos, tos, wp
	call dpush
	next

xdigits:
	.ascii "0123456789ABCDEF"
	.p2align 2, 0

	wdef num2hex, "num2hex", a_call, f_and, 0
	.word f_lit
	.word 0xF
	.word f_and
	.word f_lit
	.word xdigits
	.word f_add
	.word f_cload
	.word f_exit

	wdef hex4, "hex4", a_call, f_num2hex, 0
	.word f_num2hex
	.word f_emit
	.word f_exit

	wdef hex8, "hex8", a_call, f_hex4, 0
	.word f_dup
	.word f_lit
	.word 4
	.word f_rshift
	.word f_hex4
	.word f_hex4
	.word f_exit

	wdef hex16, "hex16", a_call, f_hex8, 0
	.word f_dup
	.word f_lit
	.word 8
	.word f_rshift
	.word f_hex8
	.word f_hex8
	.word f_exit

	wdef hex32, "hex32", a_call, f_hex16, 0
	.word f_dup
	.word f_lit
	.word 16
	.word f_rshift
	.word f_hex16
	.word f_hex16
	.word f_exit

	wdef load, "@", a_load, f_hex32, 0
a_load:
	call dpop
	lw tos, 0(tos)
	call dpush
	next

	wdef dsdump, ".s", a_call, f_load, 0
	.word f_depth
	.word f_lit
	.word '('
	.word f_emit
	.word f_hex8
	.word f_lit
	.word ')'
	.word f_emit

	.word f_depth
	.word f_stget
	.word f_swap
2:
	.word f_dup
	.word f_branch0
	.word 1f
	.word f_dec
	.word f_swap
	.word f_dup
	.word f_load
	.word f_hex32
	.word f_lit
	.word ' '
	.word f_emit
	.word f_cell
	.word f_sub
	.word f_swap
	.word f_branch
	.word 2b
1:
	.word f_2drop
	.word f_exit

	wdef rxava, "rx?", a_rxava, f_dsdump, 0
a_rxava:
	.equ UART_RXNE, (1 << 5)
	li wp, UART1_BASE
	lw xp, UART_STATR(wp)
	andi xp, xp, UART_RXNE
	bnez xp, 1f
	mv tos, zero
	call dpush
	next
1:
	li tos, -1
	call dpush
	next

	wdef rxwait, "rxwait", a_call, f_rxava, 0
1:
	.word f_yield
	.word f_rxava
	.word f_branch0
	.word 1b
	.word f_exit

	wdef rxread, "rxread", a_rxread, f_rxwait, 0
a_rxread:
	li wp, UART1_BASE
	lw tos, UART_DATAR(wp)
	call dpush
	next

	wdef rxc, "rxc", a_call, f_rxread, 0
	.word f_rxwait
	.word f_rxread
	.word f_exit

	wdef key, "key", a_call, f_rxc, 0
	.word f_rxc
	.word f_exit

	wdef or, "or", a_or, f_key, 0
a_or:
	call dpop
	mv wp, tos
	call dpop
	or tos, tos, wp
	call dpush
	next

	wdef isnl, "isnl", a_call, f_or, 0
	.word f_dup
	.word f_lit
	.word '\n'
	.word f_equ
	.word f_swap
	.word f_lit
	.word '\r'
	.word f_equ
	.word f_or
	.word f_exit

	wdef isdel, "isdel", a_call, f_isnl, 0
	.word f_dup
	.word f_lit
	.word '\b'
	.word f_equ
	.word f_swap
	.word f_lit
	.word 0x7F
	.word f_equ
	.word f_or
	.word f_exit

	wdef tib, "tib", a_doconst, f_isdel, 0
	.word tib

	wdef toin, ">in", a_doconst, f_tib, 0
	.word toin

	.equ ROM_BASE, 0x00000000
	.equ ROM_END,  ROM_BASE + (16 * 1024)

	wdef inrom, "inrom", a_call, f_toin, 0
	.word f_2lit
	.word ROM_BASE
	.word ROM_END
	.word f_within
	.word f_exit

	wdef store, "!", a_call, f_inrom, 0
	.word f_dup
	.word f_inrom
	.word f_branch0
	.word 1f
	.word f_rom32store
	.word f_exit
1:
	.word f_mem32store
	.word f_exit

	wdef mem32store, "mem32!", a_mem32store, f_store, 0
a_mem32store:
	call dpop
	mv wp, tos
	call dpop
	sw tos, 0(wp)
	next

	wdef toinchk, ">inchk", a_toinchk, f_store, 0
a_toinchk:
	la wp, tib
	la xp, tib_top
	sub wp, xp, wp
	la xp, toin
	lw xp, 0(xp)
	bltz xp, 1f
	bge xp, wp, 1f
	li tos, -1
	call dpush
	next
1:
	mv tos, zero
	call dpush
	next

	wdef toinrst, ">inrst", a_call, f_toinchk, 0
	.word f_lit
	.word 0
	.word f_toin
	.word f_store
	.word f_exit

	wdef toinload, ">in@", a_call, f_toinrst, 0
	.word f_toin
	.word f_load
	.word f_exit

	wdef cstore, "c!", a_cstore, f_toinload, 0
a_cstore:
	call dpop
	mv wp, tos
	call dpop
	sb tos, 0(wp)
	next

	wdef tipush, "tipush", a_call, f_cstore, 0
	.word f_toinchk
	.word f_branch0
	.word 1f
	.word f_tib
	.word f_toinload
	.word f_add
	.word f_cstore
	.word f_toinload
	.word f_inc
	.word f_toin
	.word f_store
	.word f_exit
1:
	.word f_drop
	.word f_exit

	wdef tidrop, "tidrop", a_call, f_tipush, 0
	.word f_toinchk
	.word f_branch0
	.word 1f
	.word f_toinload
	.word f_branch0
	.word 1f
	.word f_toinload
	.word f_dec
	.word f_toin
	.word f_store
1:
	.word f_exit

	wdef cr, "cr", a_call, f_tidrop, 0
	.word f_2lit
	.word '\r'
	.word '\n'
	.word f_emit
	.word f_emit
	.word f_exit

	wdef token, "token", a_call, f_cr, 0
2:
	.word f_2lit
	.word n_key
	.word 3
	.word f_find
	.word f_execute
	.word f_dup
	.word f_isdel
	.word f_branch0
	.word 1f
	.word f_drop
	.word f_tidrop
	.word f_branch
	.word 2b
1:
	.word f_dup
	.word f_isnl
	.word f_branch0
	.word 1f
	.word f_cr
	.word f_drop
	.word f_exit
1:
	.word f_dup
	.word f_lit
	.word ' '
	.word f_equ
	.word f_branch0
	.word 1f
	.word f_drop
	.word f_exit
1:
	.word f_tipush
	.word f_branch
	.word 2b

	wdef true, "true", a_doconst, f_token, 0
	.word -1

	wdef false, "false", a_doconst, f_true, 0
	.word 0

	wdef min, "min", a_min, f_false, 0
a_min:
	call dpop
	mv wp, tos
	call dpop
	blt tos, wp, 1f
	mv tos, wp
	call dpush
	next
1:
	call dpush
	next

	wdef tor, ">r", a_tor, f_min, 0
a_tor:
	call dpop
	call rpush
	next

	wdef fromr, "r>", a_fromr, f_tor, 0
a_fromr:
	call rpop
	call dpush
	next

	wdef rot, "rot", a_call, f_fromr, 0
	.word f_tor
	.word f_swap
	.word f_fromr
	.word f_swap
	.word f_exit

	wdef compare, "compare", a_call, f_rot, 0
	.word f_rot
	.word f_min
	.word f_dup
	.word f_branch0
	.word compare_fail
2:
	.word f_dup
	.word f_branch0
	.word compare_ok
	.word f_dec
	.word f_rot
	.word f_dup
	.word f_cload
	.word f_tor
	.word f_rot
	.word f_dup
	.word f_cload
	.word f_fromr
	.word f_equ
	.word f_branch0
	.word compare_fail
	.word f_inc
	.word f_swap
	.word f_inc
	.word f_rot
	.word f_branch
	.word 2b

compare_fail:
	.word f_2drop
	.word f_drop
	.word f_false
	.word f_exit

compare_ok:
	.word f_2drop
	.word f_drop
	.word f_true
	.word f_exit

	wdef latest, "latest", a_doconst, f_compare, 0
	.word latest

	wdef latestload, "latest@", a_call, f_latest, 0
	.word f_latest
	.word f_load
	.word f_exit

	wdef lateststore, "latest!", a_call, f_latestload, 0
	.word f_latest
	.word f_store
	.word f_exit

	wdef wlinkload, "wlink@", a_wlinkload, f_lateststore, 0
a_wlinkload:
	call dpop
	lw tos, -addrsize(tos)
	li wp, link_mask
	and tos, tos, wp
	call dpush
	next

	wdef wnlenload, "wnlen@", a_wnlenload, f_wlinkload, 0
a_wnlenload:
	call dpop
	lw tos, -addrsize(tos)
	li wp, nlen_mask
	and tos, tos, wp
	srli tos, tos, nlen_shift
	call dpush
	next

	wdef wnameload, "wname@", a_call, f_wnlenload, 0
	.word f_cell
	.word f_add
	.word f_exit

	wdef over, "over", a_call, f_wnameload, 0
	.word f_tor
	.word f_dup
	.word f_fromr
	.word f_swap
	.word f_exit

	wdef words, "words", a_call, f_over, 0
	.word f_latestload
2:
	.word f_dup
	.word f_branch0
	.word 1f

	.word f_dup
	.word f_wnameload
	.word f_over
	.word f_wnlenload
	.word f_type
	.word f_wlinkload
	.word f_lit
	.word ' '
	.word f_emit

	.word f_branch
	.word 2b

1:
	.word f_drop
	.word f_exit

	wdef 2dup, "2dup", a_call, f_words, 0
	.word f_over
	.word f_over
	.word f_exit

	wdef 2swap, "2swap", a_call, f_2dup, 0
	.word f_rot
	.word f_tor
	.word f_rot
	.word f_fromr
	.word f_exit

	wdef 2over, "2over", a_call, f_2swap, 0
	.word f_tor
	.word f_tor
	.word f_2dup
	.word f_fromr
	.word f_fromr
	.word f_2swap
	.word f_exit

	wdef nip, "nip", a_call, f_2over, 0
	.word f_swap
	.word f_drop
	.word f_exit

	wdef find, "find", a_call, f_nip, 0
	.word f_latestload
find_loop:
	.word f_dup
	.word f_branch0
	.word find_over

	.word f_2dup
	.word f_wnlenload
	.word f_equ
	.word f_branch0
	.word find_next

	.word f_dup
	.word f_wnameload
	.word f_2over
	.word f_dup
	.word f_rot
	.word f_swap
	.word f_compare
	.word f_branch0
	.word find_next

find_over:
	.word f_nip
	.word f_nip
	.word f_exit

find_next:
	.word f_wlinkload
	.word f_branch
	.word find_loop

	wdef execute, "execute", a_execute, f_find, 0
a_execute:
	call dpop
	mv wp, tos
	lw xp, 0(wp)
	jr xp

	wdef ssget, "ss@", a_ssget, f_execute, 0
a_ssget:
	mv tos, ss
	call dpush
	next

	wdef ssset, "ss!", a_ssset, f_ssget, 0
a_ssset:
	call dpop
	mv ss, tos
	next

	wdef ssrst, "ssrst", a_call, f_ssset, 0
	.word f_lit
	.word 0
	.word f_ssset
	.word f_exit

	wdef ssdund, "ssdund", a_doconst, f_ssrst, 0
	.word ssdund

	wdef xor, "xor", a_xor, f_ssdund, 0
a_xor:
	call dpop
	mv wp, tos
	call dpop
	xor tos, tos, wp
	call dpush
	next

	wdef invert, "invert", a_call, f_xor, 0
	.word f_lit
	.word -1
	.word f_xor
	.word f_exit

	wdef neq, "<>", a_call, f_invert, 0
	.word f_equ
	.word f_invert
	.word f_exit

	wdef eqz, "0=", a_call, f_neq, 0
	.word f_lit
	.word 0
	.word f_equ
	.word f_exit

	wdef ssdchk, "ssdchk", a_call, f_eqz, 0
	.word f_ssget
	.word f_ssdund
	.word f_and
	.word f_eqz
	.word f_exit

	wdef sprst, "sprst", a_sprst, f_ssdchk, 0
a_sprst:
	mv sp, st
	next

	wdef lt, "<", a_lt, f_sprst, 0
a_lt:
	call dpop
	mv wp, tos
	call dpop
	blt tos, wp, 1f
	mv tos, zero
	call dpush
	next
1:
	li tos, -1
	call dpush
	next

	wdef gt, ">", a_call, f_lt, 0
	.word f_swap
	.word f_lt
	.word f_exit

	wdef ge, ">=", a_call, f_gt, 0
	.word f_lt
	.word f_invert
	.word f_exit

	wdef le, "<=", a_call, f_ge, 0
	.word f_gt
	.word f_invert
	.word f_exit

	wdef within, "within", a_call, f_le, 0
	.word f_tor
	.word f_over
	.word f_le
	.word f_swap
	.word f_fromr
	.word f_lt
	.word f_and
	.word f_exit

	wdef isxdigit, "isxdigit", a_call, f_within, 0
	.word f_dup
	.word f_2lit
	.word '0'
	.word '9' + 1
	.word f_within
	.word f_swap
	.word f_2lit
	.word 'A'
	.word 'F' + 1
	.word f_within
	.word f_or
	.word f_exit

	// (addr u)
	wdef isnumber, "isnumber", a_call, f_isxdigit, 0
	.word f_dup
	.word f_lit
	.word 2
	.word f_gt
	.word f_branch0
	.word isnumber_false

	// check header "0x"
	.word f_2dup
	.word f_2lit
	.word isnumber_hdr
	.word 2
	.word f_compare
	.word f_branch0
	.word isnumber_false
	.word f_dec
	.word f_dec
	.word f_swap
	.word f_inc
	.word f_inc
	.word f_swap

1:
	.word f_dup
	.word f_branch0
	.word isnumber_true
	.word f_dec
	.word f_swap
	.word f_dup
	.word f_cload
	.word f_isxdigit
	.word f_branch0
	.word isnumber_false
	.word f_inc
	.word f_swap
	.word f_branch
	.word 1b

isnumber_true:
	.word f_2drop
	.word f_true
	.word f_exit

isnumber_false:
	.word f_2drop
	.word f_false
	.word f_exit

isnumber_hdr:
	.ascii "0x"
	.p2align 2, 0xFF

	wdef lshift, "lshift", a_lshift, f_isnumber, 0
a_lshift:
	call dpop
	mv wp, tos
	call dpop
	sll tos, tos, wp
	call dpush
	next

	wdef 4mul, "4*", a_call, f_lshift, 0
	.word f_lit
	.word 2
	.word f_lshift
	.word f_exit

	wdef 4div, "4/", a_call, f_4mul, 0
	.word f_lit
	.word 2
	.word f_rshift
	.word f_exit

	wdef hex2num, "hex2num", a_call, f_4div, 0
	.word f_dup
	.word f_lit
	.word '9'
	.word f_le
	.word f_branch0
	.word hex2num_x
	.word f_lit
	.word '0'
	.word f_sub
	.word f_exit

hex2num_x:
	.word f_lit
	.word 'A'
	.word f_sub
	.word f_lit
	.word 0xA
	.word f_add
	.word f_exit
	

	// addr u
	wdef number, "number", a_call, f_hex2num, 0
	.word f_2dup
	.word f_isnumber
	.word f_branch0
	.word number_nonum

	.word f_dec
	.word f_dec
	.word f_swap
	.word f_inc
	.word f_inc
	.word f_swap


	.word f_dup
	.word f_dec
	.word f_4mul
	.word f_lit
	.word 0
	// addr u shi out
1:
	.word f_2swap
	.word f_dup
	.word f_branch0
	.word number_over

	.word f_dec
	.word f_swap
	.word f_dup
	.word f_cload
	.word f_hex2num
	.word f_tor
	.word f_inc
	.word f_swap

	.word f_2swap
	.word f_over
	.word f_fromr
	.word f_swap
	.word f_lshift
	.word f_or

	.word f_swap
	.word f_lit
	.word 4
	.word f_sub
	.word f_swap

	.word f_branch
	.word 1b

number_over:
	.word f_2drop
	.word f_nip
	.word f_exit

number_nonum:
	.word f_2drop
	.word f_lit
	.word 0
	.word f_exit

	wdef sscomp, "sscomp", a_doconst, f_number, 0
	.word sscomp

	wdef bic, "bic", a_call, f_sscomp, 0
	.word f_invert
	.word f_and
	.word f_exit

	wdef compon, "]", a_call, f_bic, 0
	.word f_ssget
	.word f_sscomp
	.word f_or
	.word f_ssset
	.word f_exit

	wdef compoff, "[", a_call, f_compon, attr_immed
	.word f_ssget
	.word f_sscomp
	.word f_bic
	.word f_ssset
	.word f_exit

	wdef nez, "0<>", a_call, f_compoff, 0
	.word f_lit
	.word 0
	.word f_neq
	.word f_exit

	wdef compstat, "compstat", a_call, f_compoff, 0
	.word f_ssget
	.word f_sscomp
	.word f_and
	.word f_nez
	.word f_exit

	wdef wisimmd, "wisimmd", a_wisimmd, f_compstat, 0
a_wisimmd:
	call dpop
	lw xp, -addrsize(tos)
	andi xp, xp, attr_immed
	bnez xp, 1f
	mv tos, zero
	call dpush
	next
1:
	li tos, -1
	call dpush
	next

	wdef here, "here", a_doconst, f_wisimmd, 0
	.word here

	wdef romhere, "romhere", a_doconst, f_here, 0
	.word romhere

	wdef hereload, "here@", a_call, f_romhere, 0
	.word f_here
	.word f_load
	.word f_exit

	wdef herestore, "here!", a_call, f_hereload, 0
	.word f_here
	.word f_store
	.word f_exit

	wdef comma, ",", a_call, f_herestore, 0
	.word f_hereload
	.word f_store

	.word f_hereload
	.word f_cell
	.word f_add
	.word f_herestore
	.word f_exit

	wdef nlenshift, "nlenshift", a_doconst, f_comma, 0
	.word nlen_shift

	wdef cmove, "cmove", a_call, f_nlenshift, 0
2:
	.word f_dup
	.word f_branch0
	.word 1f
	.word f_dec
	.word f_tor
	.word f_over
	.word f_cload
	.word f_over
	.word f_cstore
	.word f_inc
	.word f_swap
	.word f_inc
	.word f_swap
	.word f_fromr
	.word f_branch
	.word 2b
1:
	.word f_drop
	.word f_2drop
	.word f_exit

	wdef move, "move", a_call, f_cmove, 0
2:
	.word f_dup
	.word f_branch0
	.word 1f
	.word f_dec
	.word f_tor
	.word f_over
	.word f_load
	.word f_over
	.word f_store
	.word f_cell
	.word f_add
	.word f_swap
	.word f_cell
	.word f_add
	.word f_swap
	.word f_fromr
	.word f_branch
	.word 2b
1:
	.word f_drop
	.word f_2drop
	.word f_exit

	wdef aligned, "aligned", a_aligned, f_move, 0
a_aligned:
	call dpop
	addi tos, tos, ((-1) + addrsize)
	andi tos, tos, -addrsize
	call dpush
	next

	wdef align, "align", a_call, f_aligned, 0
	.word f_hereload
	.word f_aligned
	.word f_herestore
	.word f_exit

	wdef wentrload, "wentr@", a_call, f_align, 0
	.word f_load
	.word f_exit

	// (addr u)
	wdef newword, "newword", a_call, f_wentrload, 0
	.word f_align

	// set link & nlen
	.word f_dup
	.word f_nlenshift
	.word f_lshift
	.word f_latestload
	.word f_or
	.word f_comma

	.word f_hereload
	.word f_lateststore

	// set it later entry
	.word f_lit
	.word -1
	.word f_comma

	.word f_dup
	.word f_tor

	// copy name
	.word f_hereload
	.word f_swap
	.word f_aligned
	.word f_4div
	.word f_move

	.word f_fromr
	.word f_hereload
	.word f_add
	.word f_herestore

	.word f_align
	.word f_exit

	wdef defword, "defword", a_call, f_newword, 0
	.word f_newword
	.word f_2lit
	.word n_call
	.word 4
	.word f_find
	.word f_load
	.word f_latestload
	.word f_store
	.word f_exit

	wdef defconst, "defconst", a_call, f_defword, 0
	.word f_newword
	.word f_2lit
	.word n_doconst
	.word 7
	.word f_find
	.word f_load
	.word f_latestload
	.word f_store
	.word f_exit

	wdef constant, "constant", a_call, f_defconst, 0
1:
	.word f_toinrst
	.word f_token
	.word f_toinload
	.word f_branch0
	.word 1b

	.word f_tib
	.word f_toinload
	.word f_defconst
	.word f_comma

	.word f_exit

	wdef docom, ":", a_call, f_constant, 0
1:
	.word f_toinrst
	.word f_token
	.word f_toinload
	.word f_branch0
	.word 1b

	.word f_tib
	.word f_toinload
	.word f_defword
	.word f_compon
	.word f_exit

	wdef doend, ";", a_call, f_docom, attr_immed
	.word f_2lit
	.word n_exit
	.word 4
	.word f_find
	.word f_comma
	.word f_compoff
	.word f_exit

	wdef ifnz, "if", a_call, f_doend, attr_immed
	.word f_2lit
	.word n_branch0
	.word 7
	.word f_find
	.word f_comma
	.word f_hereload
	.word f_lit
	.word -1
	.word f_comma
	.word f_exit

	wdef then, "then", a_call, f_ifnz, attr_immed
	.word f_hereload
	.word f_swap
	.word f_store
	.word f_exit

	wdef begin, "begin", a_call, f_then, attr_immed
	.word f_hereload
	.word f_exit

	wdef until, "until", a_call, f_begin, attr_immed
	.word f_2lit
	.word n_branch0
	.word 7
	.word f_find
	.word f_comma
	.word f_comma
	.word f_exit

	wdef tick, "'", a_call, f_until, 0
1:
	.word f_toinrst
	.word f_token
	.word f_toinload
	.word f_branch0
	.word 1b

	.word f_tib
	.word f_toinload
	.word f_find
	.word f_exit

	wdef dogon, "dogon", a_dogon, f_tick, 0
a_dogon:
	li wp, IWDG_BASE
	li xp, IWDG_KEYON
	sh xp, IWDG_CTLR_R16(wp)
	next

	wdef feedog, "feedog", a_feedog, f_dogon, 0
a_feedog:
	li wp, IWDG_BASE
	li xp, IWDG_KEYFEED
	sh xp, IWDG_CTLR_R16(wp)
	next

	wdef systickon, "systickon", a_systickon, f_feedog, 0
a_systickon:
	li wp, STK_BASE
	li xp, 6000 // 1ms
	sw xp, STK_CMPLR(wp)
	sw zero, STK_CNTL(wp)
	li xp, STK_STRE | STK_HCLKDIV8 | STK_STIE | STK_STEN
	sw xp, STK_CTLR(wp)
	next

	wdef systickoff, "systickoff", a_systickoff, f_systickon, 0
a_systickoff:
	li wp, STK_BASE
	sw zero, STK_CTLR(wp)
	next

	wdef mscountl, "mscountl", a_doconst, f_systickoff, 0
	.word mscountl

	wdef mscounth, "mscounth", a_doconst, f_mscountl, 0
	.word mscounth

	wdef irqcount, "irqcount", a_doconst, f_mscounth, 0
	.word irqcount

	wdef motd, "motd", a_call, f_irqcount, 0
	.word f_2lit
	.word _msg_motd
	.word _msg_end_motd - _msg_motd
	.word f_type
	.word f_exit

_msg_motd:
	.ascii "\n\r"
	.ascii " _____ _____ _____ _____ _____\n\r"
	.ascii "|   __|     | __  |_   _|  |  |\n\r"
	.ascii "|   __|  |  |    -| | | |     |\n\r"
	.ascii "|__|  |_____|__|__| |_| |__|__|\n\r"
	.ascii "\n\r"
	.ascii "ITC FORTH on CH32V003 (rv32ec)\n\r"
	.ascii "\n\r"
_msg_end_motd:
	.p2align 2, 0xFF

	wdef dot, ".", a_call, f_motd, 0
	.word f_hex32
	.word f_exit

	wdef quest, "?", a_call, f_dot, 0
	.word f_load
	.word f_dot
	.word f_exit

	wdef ycount, "ycount", a_doconst, f_quest, 0
	.word ycount

	wdef sysrst, "sysrst", a_sysrst, f_ycount, 0
a_sysrst:
	.equ PFIC_CFGR, 0x48
        .equ PFIC_KEY3, (0xBEEF << 16)
        .equ SYSRESET, (1 << 7)
        li wp, PFIC_BASE
        li xp, SYSRESET | PFIC_KEY3
        sw xp, PFIC_CFGR(wp)
1:
        j 1b

	wdef chipuid, "chipuid", a_chipuid, f_sysrst, 0
a_chipuid:
	li wp, ESIG_BASE
	lw tos, ESIG_UNIID3(wp)
	call dpush
	lw tos, ESIG_UNIID2(wp)
	call dpush
	lw tos, ESIG_UNIID1(wp)
	call dpush
	next

	wdef romunlock, "romunlock", a_romunlock, f_chipuid, 0
a_romunlock:
	.equ FLASH_BASE, 0x40022000
        .equ FLASH_ACTLR,  0x00
        .equ FLASH_KEYR,   0x04
        .equ FLASH_OBKEYR, 0x08
        .equ FLASH_STATR,  0x0C
        .equ FLASH_CTLR,   0x10
        .equ FLASH_KEY1,   0x45670123
        .equ FLASH_KEY2,   0xCDEF89AB
        li wp, FLASH_BASE
        li xp, FLASH_KEY1
        sw xp, FLASH_KEYR(wp)
        li xp, FLASH_KEY2
        sw xp, FLASH_KEYR(wp)
        next

	wdef romlock, "romlock", a_romlock, f_romunlock, 0
a_romlock:
	.equ FLASH_LOCK, (1 << 7)
	li wp, FLASH_BASE
	lw xp, FLASH_CTLR(wp)
	ori xp, xp, FLASH_LOCK
	sw xp, FLASH_CTLR(wp)
	next

	wdef FLASH_BASE, "FLASH_BASE", a_doconst, f_romlock, 0
	.word FLASH_BASE

	wdef FLASH_STATR, "FLASH_STATR", a_doconst, f_FLASH_BASE, 0
	.word FLASH_STATR

	.equ FLASH_BUSY, (1 << 0)
	wdef FLASH_BUSY, "FLASH_BUSY", a_doconst, f_FLASH_STATR, 0
	.word FLASH_BUSY

	wdef rombusy, "rombusy", a_call, f_FLASH_BUSY, 0
	.word f_FLASH_BASE
	.word f_FLASH_STATR
	.word f_add
	.word f_load
	.word f_FLASH_BUSY
	.word f_and
	.word f_nez
	.word f_exit

	wdef FLASH_CTLR, "FLASH_CTLR", a_doconst, f_rombusy, 0
	.word FLASH_CTLR

	.equ FLASH_PG, (1 << 0)
	wdef FLASH_PG, "FLASH_PG", a_doconst, f_FLASH_CTLR, 0
	.word FLASH_PG


	wdef romctlrload, "romctlr@", a_call, f_FLASH_PG, 0
	.word f_FLASH_BASE
	.word f_FLASH_CTLR
	.word f_add
	.word f_load
	.word f_exit

	wdef romctlrstore, "romctlr!", a_call, f_romctlrload, 0
	.word f_FLASH_BASE
	.word f_FLASH_CTLR
	.word f_add
	.word f_store
	.word f_exit

	wdef rompgon, "rompgon", a_call, f_romctlrstore, 0
	.word f_romctlrload
	.word f_FLASH_PG
	.word f_or
	.word f_romctlrstore
	.word f_exit

	wdef rompgoff, "rompgoff", a_call, f_rompgon, 0
	.word f_romctlrload
	.word f_FLASH_PG
	.word f_bic
	.word f_romctlrstore
	.word f_exit

	wdef 16store, "16!", a_16store, f_rompgoff, 0
a_16store:
	call dpop
	mv wp, tos
	call dpop
	sh tos, 0(wp)
	next

	wdef romwait, "romwait", a_call, f_16store, 0
1:
	.word f_yield
	.word f_rombusy
	.word f_invert
	.word f_branch0
	.word 1b
	.word f_exit

	.equ CODE_FLASH_BASE, 0x08000000
	wdef rom16store, "rom16!", a_call, f_romwait, 0
	.word f_romwait
	.word f_lit
	.word CODE_FLASH_BASE
	.word f_or
	.word f_16store
	.word f_romwait
	.word f_exit

	wdef rom32store, "rom32!", a_call, f_rom16store, 0
	.word f_2dup
	.word f_swap
	.word f_lit
	.word 16
	.word f_rshift
	.word f_swap
	.word f_lit
	.word 2
	.word f_add
	.word f_rom16store
	.word f_rom16store
	.word f_exit

	.equ FLASH_ADDR, 0x14
	wdef FLASH_ADDR, "FLASH_ADDR", a_doconst, f_rom32store, 0
	.word FLASH_ADDR

	.equ FLASH_PER, (1 << 1)
	wdef FLASH_PER, "FLASH_PER", a_doconst, f_FLASH_ADDR, 0
	.word FLASH_PER

	.equ FLASH_STRT, (1 << 6)
	wdef FLASH_STRT, "FLASH_STRT", a_doconst, f_FLASH_PER, 0
	.word FLASH_STRT

	wdef rom1kerase, "rom1kerase", a_call, f_FLASH_STRT, 0
	.word f_romwait
	.word f_romctlrload
	.word f_FLASH_PER
	.word f_or
	.word f_romctlrstore

	.word f_lit
	.word CODE_FLASH_BASE
	.word f_or
	.word f_FLASH_BASE
	.word f_FLASH_ADDR
	.word f_add
	.word f_store

	.word f_romctlrload
	.word f_FLASH_STRT
	.word f_or
	.word f_romctlrstore
	.word f_romwait

	.word f_romctlrload
	.word f_FLASH_PER
	.word f_bic
	.word f_romctlrstore
	.word f_exit

	wdef RAMBAK_ADDR, "RAMBAK_ADDR", a_doconst, f_rom1kerase, 0
	.word RAMBAK_ADDR

	wdef rambakerase, "rambakerase", a_call, f_RAMBAK_ADDR, 0
	.word f_RAMBAK_ADDR
	.word f_rom1kerase
	.word f_RAMBAK_ADDR
	.word f_lit
	.word 1024
	.word f_add
	.word f_rom1kerase
	.word f_exit

	.equ RAM_ADDR, 0x20000000
	wdef rambakload, "rambakload", a_rambakload, f_rambakerase, 0
a_rambakload:
	li wp, RAMBAK_ADDR
	li xp, RAMBAK_SIZE
	li yp, RAM_ADDR
2:
	beqz xp, 1f
	lw tos, 0(wp)
	sw tos, 0(yp)
	addi wp, wp, addrsize
	addi yp, yp, addrsize
	addi xp, xp, -addrsize
	j 2b
1:
	call taskload
	next

	wdef rambaksave, "rambaksave", a_rambaksave, f_rambakload, 0
a_rambaksave:
	call tasksave
	li wp, RAMBAK_ADDR | CODE_FLASH_BASE
	li xp, RAMBAK_SIZE
	li yp, RAM_ADDR

2:
	beqz xp, 1f
	lhu tos, 0(yp)
	sh  tos, 0(wp)
	addi wp, wp, 2
	addi yp, yp, 2
	addi xp, xp, -2
	j 2b
1:
	next

	wdef tip, "tip", a_doconst, f_rambaksave, 0
	.word tip
	wdef tss, "tss", a_doconst, f_tip, 0
	.word tss
	wdef tsp, "tsp", a_doconst, f_tss, 0
	.word tsp
	wdef tst, "tst", a_doconst, f_tsp, 0
	.word tst
	wdef trp, "trp", a_doconst, f_tst, 0
	.word trp
	wdef tnp, "tnp", a_doconst, f_trp, 0
	.word tnp

	wdef upget, "up@", a_upget, f_tnp, 0
a_upget:
	mv tos, up
	call dpush
	next

	wdef allot, "allot", a_call, f_upget, 0
	.word f_4mul
	.word f_hereload
	.word f_add
	.word f_herestore
	.word f_exit

	wdef interpret, "interpret", a_call, f_allot, 0
	.word f_toinload
	.word f_branch0
	.word interpret_nothing
	.word f_tib
	.word f_toinload
	.word f_find
	.word f_dup
	.word f_branch0
	.word interpret_number
	.word f_dup
	.word f_wisimmd
	.word f_branch0
	.word 1f
interpret_execute:
	.word f_execute
	.word f_ssdchk
	.word f_branch0
	.word interpret_stackerr
	.word f_exit
interpret_stackerr:
	.word f_2lit
	.word _msg_stkerr
	.word _msg_end_stkerr - _msg_stkerr
	.word f_type
	.word f_branch
	.word interpret_reset
1:
	.word f_compstat
	.word f_branch0
	.word interpret_execute
	.word f_comma
	.word f_exit

interpret_number:
	.word f_drop
	.word f_tib
	.word f_toinload
	.word f_isnumber
	.word f_branch0
	.word interpret_notfound
	.word f_tib
	.word f_toinload
	.word f_number
	.word f_compstat
	.word f_branch0
	.word interpret_nothing
	.word f_2lit
	.word n_lit
	.word 3
	.word f_find
	.word f_comma
	.word f_comma
	.word f_exit
interpret_notfound:
	.word f_tib
	.word f_toinload
	.word f_type
	.word f_2lit
	.word _msg_notfound
	.word _msg_end_notfound - _msg_notfound
	.word f_type
interpret_reset:
	.word f_sprst
	.word f_ssrst
interpret_nothing:
	.word f_exit

_msg_notfound:
	.ascii " not found\n\r"
_msg_end_notfound:
_msg_stkerr:
	.ascii " stack error\n\r"
_msg_end_stkerr:
	.p2align 2, 0xFF

forth:
	la up, task_back
        la wp, task_human
        sw wp, tnp(up)

        la ip, boot_back
        la rp, rstk_top_back
        la sp, dstk_top_back
	li ss, 0
	la st, dstk_top_back

        call tasksave

	la up, task_human
	la wp, task_back
	sw wp, tnp(up)

	la ip, boot_human
	la sp, dstk_top_human
	la rp, rstk_top_human
	li ss, 0
	la st, dstk_top_human

	la wp, latest
	la xp, lastword
	sw xp, 0(wp)

	la wp, here
	la xp, dict_base
	sw xp, 0(wp)

	la wp, romhere
	la xp, _rom_used
	sw xp, 0(wp)

	call tasksave

	next

irq_handler:
irqcountinc:
	la t0, irqcount
	lw t1, 0(t0)
	addi t1, t1, 1
	sw t1, 0(t0)

	csrr t0, mcause
	li t1, ~(1 << 31)
	and t0, t0, t1
	li t1, IRQ_STK
	beq t0, t1, systick_handler

	j irq_exit
systick_handler:
	li t0, STK_BASE
	sw zero, STK_SR(t0)

/*
	// debug
	li t0, GPIOA_BASE
        lw t1, GPIO_OUTDR(t0)
        xori t1, t1, (1 << 2)
        sw t1, GPIO_OUTDR(t0)
*/

	la t0, mscountl
	lw t1, 0(t0)
	li t2, -1
	beq t1, t2, 1f
	addi t1, t1, 1
	sw t1, 0(t0)
	j 2f
1:
	sw zero, 0(t0)
	la t0, mscounth
	lw t1, 0(t0)
	addi t1, t1, 1
	sw t1, 0(t0)
2:
	j irq_exit

irq_exit:
	mret

	.p2align 2, 0xFF
boot_human:
	.macro display label, string
		.section .text
		.word f_2lit
		.word _str_\label
		.word _str_end_\label - _str_\label
		.word f_type
		.section .rodata
	_str_\label:
		.ascii "\string"
	_str_end_\label:
		.section .text
	.endm

#ifdef TEST
	display test_branch, "branch."
	.word f_branch
	.word 1f
	.word f_fail
1:
	.word f_dzchk

	display test_branch0, "branch0."
	.word f_lit
	.word 0
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

	display test_yield, "yield."
	.word f_yield

	display test_drop, "drop."
	.word f_lit
	.word 0
	.word f_drop
	.word f_dzchk

	display test_dup, "dup."
	.word f_lit
	.word 0
	.word f_dup
	.word f_failnz
	.word f_failnz
	.word f_dzchk

	display test_equ, "=."
	.word f_2lit
	.word 1
	.word 1
	.word f_equ
	.word f_failez
	.word f_dzchk

	.word f_2lit
	.word 1
	.word 2
	.word f_equ
	.word f_failnz
	.word f_dzchk

	display test_add, "+."
	.word f_2lit
	.word 0
	.word 1
	.word f_add
	.word f_lit
	.word 1
	.word f_equ
	.word f_failez
	.word f_dzchk

	.word f_2lit
	.word 1
	.word 0
	.word f_add
	.word f_lit
	.word 1
	.word f_equ
	.word f_failez
	.word f_dzchk

	.word f_2lit
	.word 0
	.word 0
	.word f_add
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word 1
	.word 1
	.word f_add
	.word f_lit
	.word 2
	.word f_equ
	.word f_failez
	.word f_dzchk

	.word f_2lit
	.word -1
	.word 1
	.word f_add
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word -1
	.word -1
	.word f_add
	.word f_lit
	.word -2
	.word f_equ
	.word f_failez
	.word f_dzchk

	display test_inc, "1+."
	.word f_lit
	.word 0
	.word f_inc
	.word f_lit
	.word 1
	.word f_equ
	.word f_failez
	.word f_dzchk

	.word f_lit
	.word 1
	.word f_inc
	.word f_lit
	.word 2
	.word f_equ
	.word f_failez
	.word f_dzchk

	.word f_lit
	.word -2
	.word f_inc
	.word f_lit
	.word -1
	.word f_equ
	.word f_failez
	.word f_dzchk

	.word f_lit
	.word -1
	.word f_inc
	.word f_failnz
	.word f_dzchk

	display test_sub, "-."
	.word f_2lit
	.word 1
	.word 1
	.word f_sub
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word 2
	.word 1
	.word f_sub
	.word f_lit
	.word 1
	.word f_equ
	.word f_failez
	.word f_dzchk

	.word f_2lit
	.word 0
	.word 1
	.word f_sub
	.word f_lit
	.word -1
	.word f_equ
	.word f_failez
	.word f_dzchk

	.word f_2lit
	.word 1
	.word 0
	.word f_sub
	.word f_lit
	.word 1
	.word f_equ
	.word f_failez
	.word f_dzchk

	.word f_2lit
	.word -1
	.word 1
	.word f_sub
	.word f_lit
	.word -2
	.word f_equ
	.word f_failez
	.word f_dzchk

	display test_dec, "1-."
	.word f_lit
	.word 1
	.word f_dec
	.word f_failnz
	.word f_dzchk

	.word f_lit
	.word 2
	.word f_dec
	.word f_lit
	.word 1
	.word f_equ
	.word f_failez
	.word f_dzchk

	.word f_lit
	.word 0
	.word f_dec
	.word f_lit
	.word -1
	.word f_equ
	.word f_failez
	.word f_dzchk

	.word f_lit
	.word -1
	.word f_dec
	.word f_lit
	.word -2
	.word f_equ
	.word f_failez
	.word f_dzchk

	display test_2drop, "2drop."
	.word f_2lit
	.word 0
	.word 1
	.word f_2drop
	.word f_dzchk

	display test_swap, "swap."
	.word f_2lit
	.word 0
	.word 1
	.word f_swap
	.word f_failnz
	.word f_lit
	.word 1
	.word f_equ
	.word f_failez
	.word f_dzchk

	display test_cload, "c@."
	.word f_lit
	.word 1f
	.word f_cload
	.word f_lit
1:
	.word 0x55
	.word f_equ
	.word f_failez
	.word f_dzchk

	display test_spget_stget, "st@.sp@."
	.word f_spget
	.word f_stget
	.word f_swap
	.word f_sub
	.word f_failnz
	.word f_dzchk

	display test_cell, "cell."
	.word f_cell
	.word f_lit
	.word addrsize
	.word f_equ
	.word f_failez
	.word f_dzchk

	display test_rshift, "rshift."
	.word f_2lit
	.word 2
	.word 1
	.word f_rshift
	.word f_lit
	.word 1
	.word f_equ
	.word f_failez

	.word f_2lit
	.word 4
	.word 1
	.word f_rshift
	.word f_lit
	.word 2
	.word f_equ
	.word f_failez

	.word f_2lit
	.word 8
	.word 1
	.word f_rshift
	.word f_lit
	.word 4
	.word f_equ
	.word f_failez

	display test_2div, "2/."
	.word f_lit
	.word 2
	.word f_2div
	.word f_lit
	.word 1
	.word f_equ
	.word f_failez

	.word f_lit
	.word 4
	.word f_2div
	.word f_lit
	.word 2
	.word f_equ
	.word f_failez

	display test_celldiv, "cell/."
	.word f_cell
	.word f_celldiv
	.word f_lit
	.word 1
	.word f_equ
	.word f_failez

	display test_depth, "depth."
	.word f_depth
	.word f_failnz
	.word f_lit
	.word 0
	.word f_depth
	.word f_lit
	.word 1
	.word f_equ
	.word f_failez
	.word f_failnz
	.word f_2lit
	.word 0
	.word 1
	.word f_depth
	.word f_lit
	.word 2
	.word f_equ
	.word f_failez
	.word f_lit
	.word 1
	.word f_equ
	.word f_failez
	.word f_failnz
	.word f_dzchk

	display test_and, "and."
	.word f_2lit
	.word 0
	.word 0
	.word f_and
	.word f_failnz

	.word f_2lit
	.word 1
	.word 0
	.word f_and
	.word f_failnz

	.word f_2lit
	.word 0
	.word 1
	.word f_and
	.word f_failnz

	.word f_2lit
	.word 1
	.word 1
	.word f_and
	.word f_lit
	.word 1
	.word f_equ
	.word f_failez

	display test_num2hex, "num2hex."
	.word f_lit
	.word 0
	.word f_num2hex
	.word f_lit
	.word '0'
	.word f_equ
	.word f_failez

	.word f_lit
	.word 9
	.word f_num2hex
	.word f_lit
	.word '9'
	.word f_equ
	.word f_failez

	.word f_lit
	.word 0xA
	.word f_num2hex
	.word f_lit
	.word 'A'
	.word f_equ
	.word f_failez

	.word f_lit
	.word 0xF
	.word f_num2hex
	.word f_lit
	.word 'F'
	.word f_equ
	.word f_failez

	.word f_lit
	.word 0x10
	.word f_num2hex
	.word f_lit
	.word '0'
	.word f_equ
	.word f_failez

	display test_hex, "hex4.hex8.hex16.hex32."
	.word f_lit
	.word '['
	.word f_emit
	.word f_lit
	.word 0x0
	.word f_hex4
	.word f_dzchk
	.word f_lit
	.word 0x12
	.word f_hex8
	.word f_dzchk
	.word f_lit
	.word 0x3456
	.word f_hex16
	.word f_dzchk
	.word f_lit
	.word 0x789ABCDE
	.word f_hex32
	.word f_dzchk
	.word f_lit
	.word -1
	.word f_hex32
	.word f_dzchk
	.word f_lit
	.word ']'
	.word f_emit

	display test_load, "@."
	.word f_lit
	.word 1f
	.word f_load
	.word f_lit
1:
	.word 0x55AA77FF
	.word f_equ
	.word f_failez

	display test_dsdump, ".s."
	.word f_lit
	.word '['
	.word f_emit
	.word f_dsdump
	.word f_lit
	.word 1
	.word f_dsdump
	.word f_2lit
	.word 2
	.word 3
	.word f_dsdump
	.word f_2drop
	.word f_drop
	.word f_dsdump
	.word f_lit
	.word ']'
	.word f_emit
	.word f_dzchk

	display test_or, "or."
	.word f_2lit
	.word 0
	.word 0
	.word f_or
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word 0
	.word 1
	.word f_or
	.word f_lit
	.word 1
	.word f_equ
	.word f_failez
	.word f_dzchk

	.word f_2lit
	.word 1
	.word 0
	.word f_or
	.word f_lit
	.word 1
	.word f_equ
	.word f_failez
	.word f_dzchk

	.word f_2lit
	.word 1
	.word 1
	.word f_or
	.word f_lit
	.word 1
	.word f_equ
	.word f_failez
	.word f_dzchk

	display test_isnl, "isnl."
	.word f_lit
	.word '\n'
	.word f_isnl
	.word f_failez
	.word f_dzchk

	.word f_lit
	.word '\r'
	.word f_isnl
	.word f_failez
	.word f_dzchk
	
	.word f_lit
	.word 'a'
	.word f_isnl
	.word f_failnz
	.word f_dzchk

	display test_isdel, "isdel."
	.word f_lit
	.word '\b'
	.word f_isdel
	.word f_failez
	.word f_dzchk

	.word f_lit
	.word 0x7F
	.word f_isdel
	.word f_failez
	.word f_dzchk

	.word f_lit
	.word 'a'
	.word f_isdel
	.word f_failnz
	.word f_dzchk

	display test_tib_toin, "tib.>in."
	.word f_tib
	.word f_lit
	.word tib
	.word f_equ
	.word f_failez
	.word f_dzchk

	.word f_toin
	.word f_lit
	.word toin
	.word f_equ
	.word f_failez
	.word f_dzchk

	display test_toinchk, ">inchk."
	.word f_lit
	.word tib_top - tib
	.word f_toin
	.word f_store
	.word f_toinchk
	.word f_failnz
	.word f_dzchk

	.word f_lit
	.word -1
	.word f_toin
	.word f_store
	.word f_toinchk
	.word f_failnz
	.word f_dzchk

	.word f_lit
	.word 0
	.word f_toin
	.word f_store
	.word f_toinchk
	.word f_failez
	.word f_dzchk

	display test_toinrst, ">inrst.>in@."
	.word f_toinrst
	.word f_toinload
	.word f_failnz
	.word f_dzchk

	display test_tipush_tidrop, "tipush.tidrop."
	.word f_lit
	.word 0x55
	.word f_tipush
	.word f_dzchk
	.word f_tidrop
	.word f_dzchk
	.word f_toinload
	.word f_failnz
	.word f_tidrop
	.word f_toinload
	.word f_failnz
	.word f_tidrop
	.word f_toinload
	.word f_failnz
	.word f_lit
	.word tib_top - tib
	.word f_toin
	.word f_store
	.word f_lit
	.word 0
	.word f_tipush
	.word f_toinload
	.word f_lit
	.word tib_top - tib
	.word f_equ
	.word f_failez
	.word f_dzchk
	.word f_toinrst

	display test_true_flase, "true.false."
	.word f_true
	.word f_lit
	.word -1
	.word f_equ
	.word f_failez
	.word f_dzchk

	.word f_false
	.word f_failnz
	.word f_dzchk

	display test_min, "min."
	.word f_2lit
	.word 1
	.word 0
	.word f_min
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word 0
	.word 1
	.word f_min
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word 0
	.word 0
	.word f_min
	.word f_failnz
	.word f_dzchk

	display test_tor_fromr, ">r.r>."
	.word f_lit
	.word 0
	.word f_tor
	.word f_dzchk
	.word f_fromr
	.word f_failnz
	.word f_dzchk

	display test_rot, "rot."
	.word f_2lit
	.word 1
	.word 2
	.word f_lit
	.word 3
	.word f_rot
	.word f_lit
	.word 1
	.word f_equ
	.word f_failez
	.word f_lit
	.word 3
	.word f_equ
	.word f_failez
	.word f_lit
	.word 2
	.word f_equ
	.word f_failez
	.word f_dzchk

	display test_compare, "compare."
	.word f_2lit
	.word -1
	.word 0
	.word f_2lit
	.word -1
	.word 0
	.word f_compare
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word -1
	.word 0
	.word f_2lit
	.word -1
	.word 1
	.word f_compare
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word _cmp0
	.word 5
	.word f_2lit
	.word _cmp1
	.word 5
	.word f_compare
	.word f_failez
	.word f_dzchk

	.word f_2lit
	.word _cmp0
	.word 6
	.word f_2lit
	.word _cmp1
	.word 5
	.word f_compare
	.word f_failez
	.word f_dzchk

	.word f_2lit
	.word _cmp0
	.word 6
	.word f_2lit
	.word _cmp1
	.word 6
	.word f_compare
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word _cmp0
	.word 6
	.word f_2lit
	.word _cmp1
	.word 7
	.word f_compare
	.word f_failnz
	.word f_dzchk

	.word f_branch
	.word 1f
_cmp0:
	.ascii "HelloWorld"
_cmp1:
	.ascii "HelloRISCV"
	.p2align 2, 0xFF
1:

	display test_latest, "latest."
	.word f_latest
	.word f_load
	.word f_latestload
	.word f_equ
	.word f_failez
	.word f_dzchk

	.word f_latestload
	.word f_lit
	.word 5
	.word f_lateststore
	.word f_latestload
	.word f_lit
	.word 5
	.word f_equ
	.word f_failez
	.word f_lateststore
	.word f_dzchk

	display test_wlinkload, "wlink@."
	.word f_lit
	.word f_fail
	.word f_wlinkload
	.word f_failnz
	.word f_dzchk

	.word f_lit
	.word f_okay
	.word f_wlinkload
	.word f_lit
	.word f_fail
	.word f_equ
	.word f_failez
	.word f_dzchk

	display test_wnlenload, "wnlen@."
	.word f_lit
	.word f_okay
	.word f_wnlenload
	.word f_lit
	.word 4
	.word f_equ
	.word f_failez
	.word f_dzchk

	.word f_lit
	.word f_key
	.word f_wnlenload
	.word f_lit
	.word 3
	.word f_equ
	.word f_failez
	.word f_dzchk

	display test_wnameload, "wname@."
	.word f_lit
	.word f_okay
	.word f_wnameload
	.word f_lit
	.word n_okay
	.word f_equ
	.word f_failez
	.word f_dzchk

	.word f_lit
	.word f_emit
	.word f_wnameload
	.word f_lit
	.word n_emit
	.word f_equ
	.word f_failez
	.word f_dzchk

	display test_over, "over."
	.word f_2lit
	.word 0
	.word 1
	.word f_over
	.word f_failnz
	.word f_lit
	.word 1
	.word f_equ
	.word f_failez
	.word f_failnz
	.word f_dzchk

	display test_words, "words."
	.word f_lit
	.word '['
	.word f_emit

	.word f_words
	.word f_dzchk

	.word f_lit
	.word ']'
	.word f_emit

	display test_2dup, "2dup."
	.word f_2lit
	.word 6
	.word 4
	.word f_2dup
	.word f_lit
	.word 4
	.word f_equ
	.word f_failez

	.word f_lit
	.word 6
	.word f_equ
	.word f_failez

	.word f_lit
	.word 4
	.word f_equ
	.word f_failez

	.word f_lit
	.word 6
	.word f_equ
	.word f_failez

	.word f_dzchk

	display test_2swap, "2swap."
	.word f_2lit
	.word 8
	.word 9
	.word f_2lit
	.word 6
	.word 4
	.word f_2swap

	.word f_lit
	.word 9
	.word f_equ
	.word f_failez
	.word f_lit
	.word 8
	.word f_equ
	.word f_failez
	.word f_lit
	.word 4
	.word f_equ
	.word f_failez
	.word f_lit
	.word 6
	.word f_equ
	.word f_failez

	.word f_dzchk

	display test_2over, "2over."
	.word f_2lit
	.word 1
	.word 2
	.word f_2lit
	.word 3
	.word 4
	.word f_2over

	.word f_lit
	.word 2
	.word f_equ
	.word f_failez

	.word f_lit
	.word 1
	.word f_equ
	.word f_failez

	.word f_lit
	.word 4
	.word f_equ
	.word f_failez

	.word f_lit
	.word 3
	.word f_equ
	.word f_failez

	.word f_lit
	.word 2
	.word f_equ
	.word f_failez

	.word f_lit
	.word 1
	.word f_equ
	.word f_failez

	.word f_dzchk

	display test_nip, "nip."
	.word f_2lit
	.word 1
	.word 0
	.word f_nip
	.word f_failnz
	.word f_dzchk

	display test_find, "find."
	.word f_2lit
	.word 0
	.word 0
	.word f_find
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word _find0
	.word 4
	.word f_find
	.word f_lit
	.word f_fail
	.word f_equ
	.word f_failez
	.word f_dzchk

	.word f_2lit
	.word _find1
	.word 5
	.word f_find
	.word f_lit
	.word f_words
	.word f_equ
	.word f_failez
	.word f_dzchk

	.word f_2lit
	.word _find1
	.word 6
	.word f_find
	.word f_failnz
	.word f_dzchk

	.word f_branch
	.word 1f
_find0:
	.ascii "fail"
_find1:
	.ascii "words"
	.p2align 2, 0xFF
1:

	display test_execute, "execute."
	.word f_lit
	.word f_noop
	.word f_execute
	.word f_dzchk

	.word f_lit
	.word 5
	.word f_lit
	.word f_inc
	.word f_execute
	.word f_lit
	.word 6
	.word f_equ
	.word f_failez
	.word f_dzchk

	.word f_2lit
	.word 1
	.word 2
	.word f_lit
	.word f_add
	.word f_execute
	.word f_lit
	.word 3
	.word f_equ
	.word f_failez
	.word f_dzchk

	display test_ss, "ss@.ss!.ssrst."
	.word f_lit
	.word 999
	.word f_ssset
	.word f_ssget
	.word f_lit
	.word 999
	.word f_equ
	.word f_failez

	.word f_ssrst
	.word f_ssget
	.word f_failnz
	.word f_dzchk

	display test_xor, "xor."
	.word f_2lit
	.word 0
	.word 0
	.word f_xor
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word 1
	.word 1
	.word f_xor
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word 1
	.word 0
	.word f_xor
	.word f_lit
	.word 1
	.word f_equ
	.word f_failez
	.word f_dzchk

	.word f_2lit
	.word 0
	.word 1
	.word f_xor
	.word f_lit
	.word 1
	.word f_equ
	.word f_failez
	.word f_dzchk

	display test_invert, "invert."
	.word f_lit
	.word -1
	.word f_invert
	.word f_failnz
	.word f_dzchk

	.word f_lit
	.word 0
	.word f_invert
	.word f_lit
	.word -1
	.word f_equ
	.word f_failez
	.word f_dzchk

	.word f_lit
	.word 1
	.word f_invert
	.word f_lit
	.word -2
	.word f_equ
	.word f_failez
	.word f_dzchk

	.word f_lit
	.word -2
	.word f_invert
	.word f_lit
	.word 1
	.word f_equ
	.word f_failez
	.word f_dzchk

	display test_neq, "<>."
	.word f_2lit
	.word 0
	.word 0
	.word f_neq
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word 1
	.word 1
	.word f_neq
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word 0
	.word 1
	.word f_neq
	.word f_failez
	.word f_dzchk

	display test_eqz, "0=."
	.word f_lit
	.word 0
	.word f_eqz
	.word f_failez
	.word f_dzchk

	.word f_lit
	.word 1
	.word f_eqz
	.word f_failnz
	.word f_dzchk

	display test_ssdchk, "ssdchk."
	.word f_ssdund
	.word f_ssset
	.word f_ssdchk
	.word f_failnz

	.word f_ssrst
	.word f_ssdchk
	.word f_failez
	.word f_dzchk

	display test_sprst, "sprst."
	.word f_2lit
	.word 5
	.word 6
	.word f_sprst
	.word f_dzchk

	display test_lt, "<."
	.word f_2lit
	.word 0
	.word 0
	.word f_lt
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word 1
	.word 0
	.word f_lt
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word 0
	.word -1
	.word f_lt
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word 0
	.word 1
	.word f_lt
	.word f_failez
	.word f_dzchk

	.word f_2lit
	.word 1
	.word 2
	.word f_lt
	.word f_failez
	.word f_dzchk

	.word f_2lit
	.word -1
	.word 0
	.word f_lt
	.word f_failez
	.word f_dzchk

	display test_gt, ">."
	.word f_2lit
	.word 0
	.word 1
	.word f_gt
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word -1
	.word 0
	.word f_gt
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word 0
	.word 0
	.word f_gt
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word 1
	.word 0
	.word f_gt
	.word f_failez
	.word f_dzchk

	.word f_2lit
	.word 2
	.word 1
	.word f_gt
	.word f_failez
	.word f_dzchk

	.word f_2lit
	.word 0
	.word -1
	.word f_gt
	.word f_failez
	.word f_dzchk

	display test_ge, ">=."
	.word f_2lit
	.word 0
	.word 1
	.word f_ge
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word 1
	.word 2
	.word f_ge
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word -1
	.word 0
	.word f_ge
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word 0
	.word 0
	.word f_ge
	.word f_failez
	.word f_dzchk

	.word f_2lit
	.word 1
	.word 0
	.word f_ge
	.word f_failez
	.word f_dzchk

	.word f_2lit
	.word 2
	.word 1
	.word f_ge
	.word f_failez
	.word f_dzchk

	.word f_2lit
	.word 0
	.word -1
	.word f_ge
	.word f_failez
	.word f_dzchk

	display test_le, "<=."
	.word f_2lit
	.word 1
	.word 0
	.word f_le
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word 0
	.word -1
	.word f_le
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word 2
	.word 1
	.word f_le
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word 0
	.word 0
	.word f_le
	.word f_failez
	.word f_dzchk

	.word f_2lit
	.word -1
	.word 0
	.word f_le
	.word f_failez
	.word f_dzchk

	.word f_2lit
	.word 1
	.word 2
	.word f_le
	.word f_failez
	.word f_dzchk


	.word f_2lit
	.word 0
	.word 1
	.word f_le
	.word f_failez
	.word f_dzchk

	display test_within, "within."

	.word f_lit
	.word 0
	.word f_2lit
	.word 1
	.word 3
	.word f_within
	.word f_failnz
	.word f_dzchk

	.word f_lit
	.word 3
	.word f_2lit
	.word 1
	.word 3
	.word f_within
	.word f_failnz
	.word f_dzchk

	.word f_lit
	.word 1
	.word f_2lit
	.word 1
	.word 3
	.word f_within
	.word f_failez
	.word f_dzchk

	.word f_lit
	.word 2
	.word f_2lit
	.word 1
	.word 3
	.word f_within
	.word f_failez
	.word f_dzchk

	display test_isxdigit, "isxdigit."
	.word f_lit
	.word '0'
	.word f_isxdigit
	.word f_failez
	.word f_dzchk

	.word f_lit
	.word '9'
	.word f_isxdigit
	.word f_failez
	.word f_dzchk

	.word f_lit
	.word 'A'
	.word f_isxdigit
	.word f_failez
	.word f_dzchk

	.word f_lit
	.word 'F'
	.word f_isxdigit
	.word f_failez
	.word f_dzchk

	.word f_lit
	.word '0' - 1
	.word f_isxdigit
	.word f_failnz
	.word f_dzchk

	.word f_lit
	.word '9' + 1
	.word f_isxdigit
	.word f_failnz
	.word f_dzchk

	.word f_lit
	.word 'A' - 1
	.word f_isxdigit
	.word f_failnz
	.word f_dzchk

	.word f_lit
	.word 'F' + 1
	.word f_isxdigit
	.word f_failnz
	.word f_dzchk

	.word f_lit
	.word 'm'
	.word f_isxdigit
	.word f_failnz
	.word f_dzchk

	display test_isnumber, "isnumber."
	.word f_2lit
	.word _num0
	.word 3
	.word f_isnumber
	.word f_failez
	.word f_dzchk

	.word f_2lit
	.word _num1
	.word 4
	.word f_isnumber
	.word f_failez
	.word f_dzchk

	.word f_2lit
	.word _num2
	.word 4
	.word f_isnumber
	.word f_failnz
	.word f_dzchk


	.word f_2lit
	.word _num3
	.word 6
	.word f_isnumber
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word _num4
	.word 6
	.word f_isnumber
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word _num5
	.word 5
	.word f_isnumber
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word _num6
	.word 5
	.word f_isnumber
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word _num7
	.word 2
	.word f_isnumber
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word 0
	.word 1
	.word f_isnumber
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word 0
	.word 0
	.word f_isnumber
	.word f_failnz
	.word f_dzchk

	.word f_branch
	.word 1f
_num0:
	.ascii "0x0"
_num1:
	.ascii "0x12"
_num2:
	.ascii "1234"
_num3:
	.ascii "0x12 4"
_num4:
	.ascii "0x123 "
_num5:
	.ascii " x123"
_num6:
	.ascii "  123"
_num7:
	.ascii "0x"
	.p2align 2, 0xFF
1:

	display test_lshift, "lshift."
	.word f_2lit
	.word 1
	.word 0
	.word f_lshift
	.word f_lit
	.word 1
	.word f_equ
	.word f_failez

	.word f_2lit
	.word 1
	.word 1
	.word f_lshift
	.word f_lit
	.word 2
	.word f_equ
	.word f_failez

	.word f_2lit
	.word 1
	.word 2
	.word f_lshift
	.word f_lit
	.word 4
	.word f_equ
	.word f_failez

	display test_4mul, "4*."
	.word f_lit
	.word 1
	.word f_4mul
	.word f_lit
	.word 4
	.word f_equ
	.word f_failez

	.word f_lit
	.word 2
	.word f_4mul
	.word f_lit
	.word 8
	.word f_equ
	.word f_failez

	.word f_lit
	.word 3
	.word f_4mul
	.word f_lit
	.word 12
	.word f_equ
	.word f_failez

	display test_hex2num, "hex2num."
	.word f_lit
	.word '0'
	.word f_hex2num
	.word f_failnz

	.word f_lit
	.word '9'
	.word f_hex2num
	.word f_lit
	.word 9
	.word f_equ
	.word f_failez

	.word f_lit
	.word 'A'
	.word f_hex2num
	.word f_lit
	.word 0xA
	.word f_equ
	.word f_failez

	.word f_lit
	.word 'F'
	.word f_hex2num
	.word f_lit
	.word 0xF
	.word f_equ
	.word f_failez

	display test_number, "number."
	.word f_2lit
	.word _n0
	.word 6
	.word f_number
	.word f_lit
	.word 0x55AA
	.word f_equ
	.word f_failez
	.word f_dzchk

	.word f_2lit
	.word _num0
	.word 3
	.word f_number
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word _num1
	.word 4
	.word f_number
	.word f_lit
	.word 0x12
	.word f_equ
	.word f_failez
	.word f_dzchk

	.word f_2lit
	.word _num2
	.word 4
	.word f_number
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word _num3
	.word 6
	.word f_number
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word _num4
	.word 6
	.word f_number
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word _num5
	.word 5
	.word f_number
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word _num6
	.word 5
	.word f_number
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word _num7
	.word 2
	.word f_number
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word _n1
	.word 10
	.word f_number
	.word f_lit
	.word 0xFFFFFFFF
	.word f_equ
	.word f_failez
	.word f_dzchk

	.word f_2lit
	.word _n2
	.word 10
	.word f_number
	.word f_lit
	.word 0x5A5A5A5A
	.word f_equ
	.word f_failez
	.word f_dzchk

	.word f_branch
	.word 1f
_n0:
	.ascii "0x55AA"
_n1:
	.ascii "0xFFFFFFFF"
_n2:
	.ascii "0x5A5A5A5A"
	.align 2, 0xFF
1:

	display test_bic, "bic."
	.word f_2lit
	.word 3
	.word 1
	.word f_bic
	.word f_lit
	.word 2
	.word f_equ
	.word f_failez
	.word f_dzchk

	.word f_2lit
	.word 1
	.word 1
	.word f_bic
	.word f_failnz
	.word f_dzchk

	.word f_2lit
	.word 0xF
	.word 2
	.word f_bic
	.word f_lit
	.word 0xD
	.word f_equ
	.word f_failez
	.word f_dzchk

	display test_nez, "0<>."
	.word f_lit
	.word 0
	.word f_nez
	.word f_failnz
	.word f_dzchk

	.word f_lit
	.word 1
	.word f_nez
	.word f_failez
	.word f_dzchk

	display test_comp_on_off_stat, "[.].compstat."
	.word f_compon
	.word f_compstat
	.word f_failez
	.word f_dzchk
	.word f_compoff
	.word f_compstat
	.word f_failnz
	.word f_dzchk

	display test_wisimmd, "wisimmd."
	.word f_lit
	.word f_compoff
	.word f_wisimmd
	.word f_failez
	.word f_dzchk

	.word f_lit
	.word f_emit
	.word f_wisimmd
	.word f_failnz
	.word f_dzchk

	display test_here, "here.here@.here!.,."
	.word f_hereload
	.word f_lit
	.word 0x55AA
	.word f_comma
	.word f_hereload
	.word f_swap
	.word f_sub
	.word f_cell
	.word f_equ
	.word f_failez
	.word f_dzchk

	.word f_hereload
	.word f_cell
	.word f_sub
	.word f_load
	.word f_lit
	.word 0x55AA
	.word f_equ
	.word f_failez
	.word f_dzchk

	display test_cmove, "cmove."
	.word f_lit
	.word _cm0
	.word f_hereload
	.word f_lit
	.word 5
	.word f_cmove
	.word f_dzchk

	.word f_2lit
	.word _cm0
	.word 5
	.word f_hereload
	.word f_over
	.word f_compare
	.word f_failez
	.word f_dzchk

	.word f_branch
	.word 1f
_cm0:
	.ascii "12345"
	.p2align 2, 0xFF
1:

	display test_move, "move."
	.word f_lit
	.word _m0
	.word f_hereload
	.word f_lit
	.word 3
	.word f_move
	.word f_dzchk

	.word f_2lit
	.word _m0
	.word (3 * addrsize)
	.word f_hereload
	.word f_over
	.word f_compare
	.word f_failez
	.word f_dzchk
_m0:
	.word f_noop
	.word f_branch
	.word 1f
1:

	display test_aligned, "aligned."
	.word f_lit
	.word 4
	.word f_aligned
	.word f_lit
	.word 4
	.word f_equ
	.word f_failez
	.word f_dzchk

	.word f_lit
	.word 1
	.word f_aligned
	.word f_lit
	.word 4
	.word f_equ
	.word f_failez
	.word f_dzchk

	.word f_lit
	.word 5
	.word f_aligned
	.word f_lit
	.word 8
	.word f_equ
	.word f_failez
	.word f_dzchk

	.word f_lit
	.word 0
	.word f_aligned
	.word f_failnz
	.word f_dzchk

	display test_align, "align."
	.word f_hereload
	.word f_align
	.word f_hereload
	.word f_equ
	.word f_failez
	.word f_dzchk

	.word f_hereload
	.word f_dup
	.word f_lit
	.word 3
	.word f_add
	.word f_herestore
	.word f_align
	.word f_hereload
	.word f_swap
	.word f_sub
	.word f_cell
	.word f_equ
	.word f_failez
	.word f_dzchk

	display test_wentrload, "wentr@."
	.word f_lit
	.word f_call
	.word f_wentrload
	.word f_lit
	.word a_call
	.word f_equ
	.word f_failez
	.word f_dzchk

	display test_defword, "defword."
	.word f_hereload
	.word f_2lit
	.word _n_nop
	.word 3
	.word f_defword
	.word f_hereload
	.word f_swap
	.word f_sub
	.word f_lit
	.word addrsize * 3
	.word f_equ
	.word f_failez
	.word f_dzchk

	.word f_2lit
	.word n_exit
	.word 4
	.word f_find
	.word f_comma

	.word f_2lit
	.word _n_nop
	.word 3
	.word f_find
	.word f_execute

	.word f_branch
	.word 1f
_n_nop:
	.ascii "nop"
	.p2align 2, 0xFF
1:

	display test_token_interpret, "token.interpret."
1:
	.word f_toinrst
	.word f_token

	.word f_interpret

	.word f_branch
	.word 1b

	display test_echo, "key.emit."
1:
	.word f_key
	.word f_dup
	.word f_emit
	.word f_lit
	.word 'Q'
	.word f_equ
	.word f_branch0
	.word 1b
	.word f_dzchk
#endif

	.word f_motd

	.word f_2lit
	.word _msg_chipid
	.word _msg_end_chipid - _msg_chipid
	.word f_type
	.word f_chipuid
	.word f_hex32
	.word f_hex32
	.word f_hex32
	.word f_cr

	.word f_branch
	.word 1f
_msg_chipid:
	.ascii "CHIP UID: "
_msg_end_chipid:
	.p2align 2, 0xFF
1:

interpret_loop:
1:
	.word f_toinrst
	.word f_token

	.word f_interpret

	.word f_branch
	.word 1b

	.word f_noop
	.word f_okay
fail:
	.word f_fail

boot_back:
	.word f_yield
	.word f_feedog
	.word f_branch
	.word boot_back

	.section .bss
	.p2align 2, 0
_ram_entry:
	.fill 1, addrsize, 0
irqcount:
	.fill 1, addrsize, 0
ycount:
	.fill 1, addrsize, 0
mscountl:
	.fill 1, addrsize, 0
mscounth:
	.fill 1, addrsize, 0
latest:
	.fill 1, addrsize, 0
here:
	.fill 1, addrsize, 0
romhere:
	.fill 1, addrsize, 0
toin:
	.fill 1, addrsize, 0

	.p2align 2, 0
tib:
	.fill tibsize, 1, 0
tib_top:
	.fill 1, 1, 0
	.p2align 2, 0

dstk_human:
	.fill stksize, addrsize, 0
dstk_top_human:
	.fill 1, addrsize, 0
rstk_human:
	.fill stksize, addrsize, 0
rstk_top_human:
	.fill 1, addrsize, 0
task_human:
	.fill tasksize, addrsize, 0
dstk_back:
	.fill stksize, addrsize, 0
dstk_top_back:
	.fill 1, addrsize, 0
rstk_back:
	.fill stksize, addrsize, 0
rstk_top_back:
	.fill 1, addrsize, 0
task_back:
	.fill tasksize, addrsize, 0
dict_base:
	.fill 0, addrsize, 0

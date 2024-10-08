	.global _start
_start:
	.option norvc;
_vector:
	j reset
	.word 0
	.word nmi_handler
	.word hardfault_handler
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word systick_handler
	.word 0
	.word sw_handler
	.word 0
	.word wwdg_handler
	.word pvd_handler
	.word flash_handler
	.word rcc_handler
	.word exti70_handler
	.word awu_handler
	.word dma1c1_handler
	.word dma1c2_handler
	.word dma1c3_handler
	.word dma1c4_handler
	.word dma1c5_handler
	.word dma1c6_handler
	.word dma1c7_handler
	.word adc1_handler
	.word i2c1_ev_handler
	.word i2c1_er_handler
	.word usart1_handler
	.word spi1_handler
	.word tim1_brk_handler
	.word tim1_up_handler
	.word tim1_trgcom_handler
	.word tim1_cc_handler
	.word tim2_handler
_vector_end:
	.option rvc;

nmi_handler:
hardfault_handler:
sw_handler:
wwdg_handler:
pvd_handler:
flash_handler:
rcc_handler:
exti70_handler:
awu_handler:
dma1c1_handler:
dma1c2_handler:
dma1c3_handler:
dma1c4_handler:
dma1c5_handler:
dma1c6_handler:
dma1c7_handler:
dma1c7_handler:
adc1_handler:
i2c1_ev_handler:
i2c1_er_handler:
usart1_handler:
spi1_handler:
tim1_brk_handler:
tim1_up_handler:
tim1_trgcom_handler:
tim1_cc_handler:
tim2_handler:
	j sysrst

#define wp t0
#define xp t1
#define yp t2
#define rp ra
#define ip fp
#define ss s1
#define up tp
#define st gp

	.equ addrsize, 4

	.equ tonext, (0 * addrsize)
	.equ torp, (1 * addrsize)
	.equ tosp, (2 * addrsize)
	.equ toip, (3 * addrsize)
	.equ toss, (4 * addrsize)
	.equ tost, (5 * addrsize)
	
	.equ tasksize, 8
	.equ dstksize, 21
	.equ rstksize, 21
	.equ gapsize,  4

	.equ sscomp, (1 << 0)
	.equ ssdund, (1 << 1)

reset:
	.equ RCC_BASE, 0x40021000
	.equ RCC_APB2PCENR, 0x18
	.equ RCC_AFIOEN, (1 << 0)
	.equ RCC_IOPAEN, (1 << 2)
	.equ RCC_IOPDEN, (1 << 5)
	.equ RCC_USART1EN, (1 << 14)

	li t0, RCC_BASE
	lw t1, RCC_APB2PCENR(t0)
	li t2, RCC_AFIOEN | RCC_IOPAEN | RCC_IOPDEN | RCC_USART1EN
	or t1, t1, t2
	sw t1, RCC_APB2PCENR(t0)

	.equ RCC_RSTSCKR, 0x24
	.equ RCC_LSION, (1 << 0)
	.equ RCC_LSIRDY, (1 << 1)
	lw t1, RCC_RSTSCKR(t0)
	or t1, t1, RCC_LSION
	sw t1, RCC_RSTSCKR(t0)

1:
	lw t1, RCC_RSTSCKR(t0)
	andi t1, t1, RCC_LSIRDY
	beqz t1, 1b

	.equ AFIO_BASE, 0x40010000
	.equ AFIO_PCFR1, 0x04
	.equ AFIO_UART_MASK, (1 << 21) | (1 << 2)
	.equ AFIO_UART_D6TX, (1 << 21) | (0 << 2)
	li t0, AFIO_BASE
	lw t1, AFIO_PCFR1(t0)
	li t2, AFIO_UART_MASK
	xori t2, t2, -1
	and t1, t1, t2
	li t2, AFIO_UART_D6TX
	or t1, t1, t2
	sw t1, AFIO_PCFR1(t0)

	.equ GPIOA_BASE, 0x40010800
	.equ GPIO_CFGLR, 0x00
	.equ GPIOA2_CFGMASK, (0xF << 8)
	.equ GPIOA2_PPOUT_2M, (0x2 << 8)
	.equ GPIO_OUTDR, 0x0C

	li t0, GPIOA_BASE
	lw t1, GPIO_CFGLR(t0)
	li t2, GPIOA2_CFGMASK
	xori t2, t2, -1
	and t1, t1, t2
	ori t1, t1, GPIOA2_PPOUT_2M
	sw t1, GPIO_CFGLR(t0)

	.equ GPIOD_BASE, 0x40011400
	.equ GPIOD6_CFGMASK, (0xF << 24)
	.equ GPIOD6_ODOUT_MU_30M, (0xF << 24)
	li t0, GPIOD_BASE
	lw t1, GPIO_CFGLR(t0)
	li t2, GPIOD6_CFGMASK
	xori t2, t2, -1
	and t1, t1, t2
	li t2, GPIOD6_ODOUT_MU_30M
	or t1, t1, t2
	sw t1, GPIO_CFGLR(t0)

	// default:
	// SYSCLK: HSI 24Mhz
	// HCLK:   SYSCLK / 3 = 8Mhz

	// UART:
	// HCLK / (16 * UARTDIV) = BAUDRATE
	// 8Mhz / (16 * 4.34) = 115207
	// UARTDIV = DIV_M + (DIV_F / 16)
	// 4.34 = 4 + (5 / 16)
	// DIV_M = 4
	// DIV_F = 5
	.equ UART1_BASE, 0x40013800
	.equ UART_DATAR, 0x04
	.equ UART_BRR, 0x08
	.equ UART_CTLR1, 0x0C
	.equ UART_UE, (1 << 13)
	.equ UART_TE, (1 << 3)
	.equ UART_RE, (1 << 2)
	.equ UART_CTLR3, 0x14
	.equ UART_HDSEL, (1 << 3)

	.equ BAUD_9600,   (52 << 4) | (2 << 0)
	.equ BAUD_115200, (4 << 4) | (5 << 0)

	li t0, UART1_BASE
	li t1, BAUD_115200
	sw t1, UART_BRR(t0)
	lw t1, UART_CTLR1(t0)
	li t2, UART_UE | UART_TE | UART_RE
	or t1, t1, t2
	sw t1, UART_CTLR1(t0)
	lw t1, UART_CTLR3(t0)
	ori t1, t1, UART_HDSEL
	sw t1, UART_CTLR3(t0)

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
	li t0, IWDG_BASE
	li t1, IWDG_KEYUNLOCK
	sh t1, IWDG_CTLR_R16(t0)

	.equ IWDG_DIVMASK, 0x7
	.equ IWDG_DIV4, 0x0
	.equ IWDG_DIV8, 0x1
	.equ IWDG_DIV64, 0x4
	.equ IWDG_DIV256, 0x6
1:
	lhu t1, IWDG_STATR_R16(t0)
	andi t1, t1, IWDG_PVU
	bnez t1, 1b

	li t2, IWDG_DIVMASK
	xori t2, t2, -1
	lhu t1, IWDG_PSCR_R16(t0)
	and t1, t1, t2
	li t2, IWDG_DIV256
	or t1, t1, t2
	sh t1, IWDG_PSCR_R16(t0)

1:
	lw t1, IWDG_STATR_R16(t0)
	andi t1, t1, IWDG_RVU
	bnez t1, 1b

	li t1, -1
	sh t1, IWDG_RLDR_R16(t0)


	// IRQ
	la t0, _vector
	la t1, _ram_vector
	la t2, _vector_end

	// copy vector table
2:
	bge t0, t2, 1f
	lw a0, 0(t0)
	sw a0, 0(t1)
	addi t0, t0, addrsize
	addi t1, t1, addrsize
	j 2b
1:
	// set vector table address
	la t0, _ram_vector
	ori t0, t0, 0x3
	csrw mtvec, t0

	// enable global interrupt and configure privileged mode
	li t0, 0x1880
	csrw mstatus, t0
	// not enable hardware stack, mainline gcc compiler not suppot this
	// not enable interrupt nesting

        .equ PFIC_BASE, 0xE000E000
	.equ PFIC_IENR1,0x100
	.equ IRQ_STK, 12
	li t0, PFIC_BASE
	lw t1, PFIC_IENR1(t0)
	li t2, (1 << IRQ_STK)
	or t1, t1, t2
	sw t1, PFIC_IENR1(t0)
	.equ PFIC_IENR2,0x104
	.equ IRQ_UART1, 0
	lw t1, PFIC_IENR2(t0)
	li t2, (1 << IRQ_UART1)
	or t1, t1, t2
	sw t1, PFIC_IENR2(t0)

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

	// jump to forth
	la t0, forth
	csrw mepc, t0
	mret

systick_handler:
	li a5, STK_BASE
	sw zero, STK_SR(a5)

	la a5, systick_count
	lw a4, 0(a5)
	li a3, -1
	beq a4, a3, 1f
	addi a4, a4, 1
	sw a4, 0(a5)
	j 2f
1:
	sw zero, 0(a5)
	lw a4, addrsize(a5)
	addi a4, a4, 1
	sw a4, addrsize(a5)
2:

/*
	li a5, GPIOA_BASE
	lw a4, GPIO_OUTDR(a5)
	xori a4, a4, (1 << 2)
	sw a4, GPIO_OUTDR(a5)
*/
	mret

	// ch32v003 :
	// 0x00000000 ~ 0x00003FFF   ROM 16KiB
	// 0x20000000 ~ 0x200007FF   RAM 2KiB
	// 0x00003FFF | 0x200007FF = 0x20003FFF
	// 4byte align
	// 0x20003FFF & ~(0x3) = 0x20003FFC
	//       FEDCBA9876543210FEDCBA9876543210
	//     0b00100000000000000011111111111100
	//          |-----15------||-----12---||\
	// link             |len|              |attr|
	// entr             |off|

	.equ addrmask,  (0x20003FFC)
	.equ nlenshift, (16)
	.equ offtshift, (16)
	.equ attrmask,  (0x03)
	.equ nlenmask,  ((0x1F) << nlenshift)
	.equ offtmask,  ((0x1F) << offtshift)

	// gnu assmbler only support '+' '-' '<<' '>>' in expression
	// not supoort '&' '|', we use '+' instead '|'

	.set lastword, 0

	.equ attrnone, 0
	.equ attrimmd, (1 << 0)
	.equ attrhide, (1 << 1)

	.macro defcode label, name, link, attr
		.p2align 2, 0
	l_\label:
		.word \link + nlen_\label + \attr
	f_\label:
		.word a_\label + offt_\label
	n_\label:
		.ascii "\name"
	n_end_\label:
		.p2align 2, 0
		.set nlen_\label, ((n_end_\label - n_\label) << nlenshift)
		.set offt_\label, ((a_\label - f_\label) << offtshift)
		.set lastword, f_\label
	a_\label:
	.endm

	defcode uuu, "uuu", 0, attrnone
	li wp, UART1_BASE
	li xp, 0x55 // U
1:
	sw xp, UART_DATAR(wp)
	j 1b

	defcode fail, "fail", f_uuu, attrnone
	li wp, UART1_BASE
	li xp, 0x07 // bell
1:
	sw xp, UART_DATAR(wp)
	j 1b

	defcode next, "next", f_fail, attrnone
	li yp, addrmask
	lw wp, 0(ip)
	and wp, wp, yp
	addi ip, ip, addrsize
	lw xp, 0(wp)
	and xp, xp, yp
	jr xp

	.macro next
		j a_next
	.endm

	.macro rpush reg
		sw \reg, 0(rp)
		addi rp, rp, -addrsize
	.endm

	defcode call, "call", f_next, attrnone
	rpush ip
	li yp, offtmask
	lw xp, 0(wp)
	and ip, xp, yp
	srli ip, ip, offtshift
	add ip, ip, wp
	next

	.macro rpop reg
		addi rp, rp, addrsize
		lw \reg, 0(rp)
	.endm

	defcode exit, "exit", f_call, attrnone
	rpop ip
	next

        .macro defword label, name, link, attr
		.p2align 2, 0
	l_\label:
		.word \link + nlen_\label + \attr
        f_\label:
                .word a_call + offt_\label
        n_\label:
                .ascii "\name"
        n_end_\label:
                .p2align 2, 0
                .set nlen_\label, ((n_end_\label - n_\label) << nlenshift)
                .set offt_\label, ((w_\label - f_\label) << offtshift)
		.set lastword, f_\label
        w_\label:
        .endm

	defword noop, "noop", f_exit, attrnone
	.word f_exit

	.macro dpush reg
		sw \reg , 0(sp)
		addi sp, sp, -addrsize
	.endm

	defcode lit, "lit", f_noop, attrnone
	lw xp, 0(ip)
	dpush xp
	addi ip, ip, addrsize
	next

	defcode 2lit, "2lit", f_lit, attrnone
	lw xp, 0(ip)
	dpush xp
	addi ip, ip, addrsize
	lw xp, 0(ip)
	dpush xp
	addi ip, ip, addrsize
	next

	.macro dschk
		ble sp, st, 66f
		ori ss, ss, ssdund
	66:
	.endm

	.macro dpop reg
		addi sp, sp, addrsize
		dschk
		lw \reg, 0(sp)
	.endm

	defcode branch0, "branch0", f_2lit, attrnone
	dpop xp
	lw yp, 0(ip)
	addi ip, ip, addrsize
	bnez xp, 1f
	mv ip, yp
1:
	next

	defcode equ, "=", f_branch0, attrnone
	dpop xp
	dpop yp
	beq xp, yp, 1f
	dpush zero
	next
1:
	li xp, -1
	dpush xp
	next

	defcode doconst, "doconst", f_equ, attrnone
	li yp, offtmask
	lw xp, 0(wp)
	and xp, xp, yp
	srli xp, xp, offtshift
	add wp, wp, xp
	lw xp, 0(wp)
	dpush xp
	next

        .macro defconst, label, name, link, attr
		.p2align 2, 0
		.set attr_\label, \attr
	l_\label:
		.word \link + nlen_\label + \attr
        f_\label:
                .word a_doconst + offt_\label
        n_\label:
                .ascii "\name"
        n_end_\label:
                .p2align 2, 0
                .set nlen_\label, ((n_end_\label - n_\label) << nlenshift)
                .set offt_\label, ((v_\label - f_\label) << offtshift)
		.set lastword, f_\label
        v_\label:
        .endm

	defconst cell, "cell", f_doconst, attrnone
	.word addrsize

	defcode depth, "depth", f_cell, attrnone
	sub xp, st, sp
	srli xp, xp, 2 // div 4
	dpush xp
	next

	defword dzchk, "dzchk", f_depth, attrnone
	.word f_depth
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_exit

	.macro taskload
		lw rp, torp(up)
		lw sp, tosp(up)
		lw ip, toip(up)
		lw ss, toss(up)
		lw st, tost(up)
	.endm

	.macro tasksave
		sw rp, torp(up)
		sw sp, tosp(up)
		sw ip, toip(up)
		sw ss, toss(up)
		sw st, tost(up)
	.endm

	defcode yield, "yield", f_dzchk, attrnone
	tasksave
	lw wp, tonext(up)
	mv up, wp
	la wp, yield_count
	lw xp, 0(wp)
	addi xp, xp, 1
	sw xp, 0(wp)
	taskload
	next

	defcode txava, "tx?", f_yield, attrnone
	li wp, UART1_BASE
	.equ UART_STATR, 0x00
	.equ UART_TC, (1 << 6)
	lw xp, UART_STATR(wp)
	andi xp, xp, UART_TC
	beqz xp, 1f
	li xp, -1
	dpush xp
	next
1:
	dpush zero
	next

	defword txwait, "txwait", f_txava, attrnone
1:
	.word f_yield
	.word f_txava
	.word f_branch0
	.word 1b
	.word f_exit

	defcode txfill, "txfill", f_txwait, attrnone

	li wp, UART1_BASE
	dpop xp
	sw xp, UART_DATAR(wp)
	next

	defword tx, "tx", f_txfill, attrnone
	.word f_txwait
	.word f_txfill
	.word f_exit

	defword emit, "emit", f_tx, attrnone
	.word f_tx
	.word f_exit

	defcode rxava, "rx?", f_emit, attrnone

	li wp, UART1_BASE
	.equ UART_RXNE, (1 << 5)
	lw xp, UART_STATR(wp)
	andi xp, xp, UART_RXNE
	beqz xp, 1f
	li xp, -1
	dpush xp
	next
1:
	dpush zero
	next

	defword rxwait, "rxwait", f_rxava, attrnone
1:
	.word f_yield
	.word f_rxava
	.word f_branch0
	.word 1b
	.word f_exit

	defcode rxread, "rxread", f_rxwait, attrnone
	li wp, UART1_BASE
	lw xp, UART_DATAR(wp)
	dpush xp
	li wp, GPIOA_BASE
	next

	defword rx, "rx", f_rxread, attrnone
	.word f_rxwait
	.word f_rxread
	.word f_exit

	defcode num2hex, "num2hex", f_rx, attrnone
	dpop wp
	andi wp, wp, 0xF
	la xp, xdigits
	add xp, xp, wp
	lbu wp, 0(xp)
	dpush wp
	next

xdigits:
	.ascii "0123456789ABCDEF"
	.p2align 2, 0

	defword hex4, "hex4", f_num2hex, attrnone
	.word f_num2hex
	.word f_emit
	.word f_exit

	defcode rshift, "rshift", f_hex4, attrnone
	dpop wp
	dpop xp
	srl xp, xp, wp
	dpush xp
	next

	defcode dup, "dup", f_rshift, attrnone
	dpop wp
	dpush wp
	dpush wp
	next

	defword hex8, "hex8", f_dup, attrnone
	.word f_dup
	.word f_lit
	.word 4
	.word f_rshift
	.word f_hex4
	.word f_hex4
	.word f_exit

	defword hex16, "hex16", f_hex8, attrnone
	.word f_dup
	.word f_lit
	.word 8
	.word f_rshift
	.word f_hex8
	.word f_hex8
	.word f_exit

	defword hex32, "hex32", f_hex16, attrnone
	.word f_dup
	.word f_lit
	.word 16
	.word f_rshift
	.word f_hex16
	.word f_hex16
	.word f_exit

	defcode spget, "sp@", f_hex32, attrnone
	dpush sp
	next

	defcode stget, "st@", f_spget, attrnone
	dpush st
	next

	defcode load, "@", f_stget, attrnone
	dpop wp
	lw xp, 0(wp)
	dpush xp
	next

	defcode sub, "-", f_load, attrnone
	dpop wp
	dpop xp
	sub xp, xp, wp
	dpush xp
	next

	defcode eqz, "0=", f_sub, attrnone
	dpop wp
	beqz wp, 1f
	dpush zero
	next
1:
	li wp, -1
	dpush wp
	next

	defcode drop, "drop", f_eqz, attrnone
	dpop wp
	next

	defcode swap, "swap", f_drop, attrnone
	dpop wp
	dpop xp
	dpush wp
	dpush xp
	next

	defcode over, "over", f_swap, attrnone
	dpop wp
	dpop xp
	dpush xp
	dpush wp
	dpush xp
	next

	defcode branch, "branch", f_over, attrnone
	lw xp, 0(ip)
	mv ip, xp
	next

	defcode dec, "1-", f_branch, attrnone
	dpop xp
	addi xp, xp, -1
	dpush xp
	next

	defcode 2drop, "2drop", f_dec, attrnone
	dpop xp
	dpop xp
	next

	defword dsdump, ".s", f_2drop, attrnone
	.word f_lit
	.word '('
	.word f_emit
	.word f_depth
	.word f_hex16
	.word f_lit
	.word ')'
	.word f_emit

	.word f_depth
	.word f_stget
2:
	.word f_over
	.word f_branch0
	.word 1f

	.word f_dup
	.word f_load
	.word f_hex32
	.word f_lit
	.word ' '
	.word f_emit

	.word f_cell
	.word f_sub
	.word f_swap
	.word f_dec
	.word f_swap

	.word f_branch
	.word 2b

1:
	.word f_2drop
	.word f_exit

	defword isspace, "isspace", f_dsdump, attrnone
	.word f_lit
	.word ' '
	.word f_equ
	.word f_exit

	defcode or, "or", f_isspace, attrnone
	dpop wp
	dpop xp
	or wp, wp, xp
	dpush wp
	next

	defword isnewline, "isnewline", f_or, attrnone
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

	defconst toin, ">in", f_isnewline, attrnone
	.word toin

	defconst tib, "tib", f_toin, attrnone
	.word tib

	.equ ROM_BASE, 0x0
	defconst rombase, "rombase", f_tib, attrnone
	.word ROM_BASE

	.equ ROM_TOP, 0x4000
	defconst romtop, "romtop", f_rombase, attrnone
	.word ROM_TOP

	defcode mem32store, "mem32!", f_romtop, 0
	dpop wp
	dpop xp
	sw xp, 0(wp)
	next

	defword store, "!", f_mem32store, 0
	.word f_dup
	.word f_rombase
	.word f_romtop
	.word f_within
	.word f_branch0
	.word 1f
	.word f_rom32store
	.word f_exit
1:
	.word f_mem32store
	.word f_exit

	defword toinrst, ">inrst", f_store, attrnone
	.word f_lit
	.word 0
	.word f_toin
	.word f_store
	.word f_exit

	defcode cstore, "c!", f_toinrst, attrnone
	dpop wp
	dpop xp
	sb xp, 0(wp)
	next

	defcode cload, "c@", f_cstore, attrnone
	dpop wp
	lbu xp, 0(wp)
	dpush xp
	next

	defcode inc, "1+", f_cload, attrnone
	dpop wp
	addi wp, wp, 1
	dpush wp
	next

	defcode toinmax, ">inmax", f_inc, attrnone
	la wp, tib_end
	la xp, tib
	sub wp, wp, xp
	dpush wp
	next

	defcode lt, "<", f_toinmax, attrnone
	dpop wp
	dpop xp
	blt xp, wp, 1f
	dpush zero
	next
1:
	li xp, -1
	dpush xp
	next

	defcode gt, ">", f_lt, attrnone
	dpop wp
	dpop xp
	bgt xp, wp, 1f
	dpush zero
	next
1:
	li xp, -1
	dpush xp
	next

	defword 2dup, "2dup", f_gt, attrnone
	.word f_over
	.word f_over
	.word f_exit

	defcode rot, "rot",f_2dup, attrnone
	dpop wp
	dpop xp
	dpop yp
	dpush xp
	dpush wp
	dpush yp
	next

	defcode ge, ">=", f_rot, attrnone
	dpop wp
	dpop xp
	bge xp, wp, 1f
	dpush zero
	next
1:
	li xp, -1
	dpush xp
	next

	defcode and, "and", f_ge, attrnone
	dpop wp
	dpop xp
	and wp, wp, xp
	dpush wp
	next

	defcode le, "<=", f_rot, attrnone
	dpop wp
	dpop xp
	ble xp, wp, 1f
	dpush zero
	next
1:
	li xp, -1
	dpush xp
	next

	defcode within, "within", f_and, attrnone
	dpop wp // max
	dpop xp // min
	dpop yp // n
	blt yp, wp, 1f
	dpush zero
	next
1:
	bge yp, xp, 2f
	dpush zero
	next
2:
	li xp, -1
	dpush xp
	next

	defword toinchk, ">inchk", f_within, attrnone
	.word f_toin
	.word f_load
	.word f_lit
	.word 0
	.word f_toinmax
	.word f_within
	.word f_exit

	defcode add, "+", f_toinchk, attrnone
	dpop xp
	dpop wp
	add wp, wp, xp
	dpush wp
	next

	defword tipush, "tipush", f_add, attrnone
	.word f_tib
	.word f_toin
	.word f_load
	.word f_add
	.word f_cstore
	.word f_toin
	.word f_load
	.word f_inc
	.word f_toin
	.word f_store
	.word f_toinchk
	.word f_branch0
	.word 1f
	.word f_exit
1:
	.word f_toinrst
	.word f_exit

	defword tidrop, "tidrop", f_tipush, attrnone
	.word f_toin
	.word f_load
	.word f_dec
	.word f_toin
	.word f_store
	.word f_toinchk
	.word f_branch0
	.word 1f
	.word f_exit
1:
	.word f_toinrst
	.word f_exit

	defword key, "key", f_tidrop, attrnone
	.word f_rx
	.word f_exit

	defword isdelete, "isdelete", f_key, attrnone
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

	defword newline, "newline", f_isdelete, attrnone
	.word f_2lit
	.word '\n'
	.word '\r'
	.word f_emit
	.word f_emit
	.word f_exit

	defword token, "token", f_newline, attrnone
token_loop:
	.word f_key

	.word f_dup
	.word f_isspace
	.word f_branch0
	.word 1f
	.word f_drop
	.word f_exit
1:

	.word f_dup
	.word f_isnewline
	.word f_branch0
	.word 2f
	.word f_drop
	.word f_newline
	.word f_exit
2:

	.word f_dup
	.word f_isdelete
	.word f_branch0
	.word 3f
	.word f_drop
	.word f_tidrop
	.word f_branch
	.word token_loop
3:
	.word f_tipush
	.word f_branch
	.word token_loop

	defword type, "type", f_token, attrnone
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

	defcode min, "min", f_type, attrnone
	dpop wp
	dpop xp
	blt wp, xp, 1f
	dpush xp
	next
1:
	dpush wp
	next

	defconst true, "true", f_min, attrnone
	.word -1

	defconst false, "false", f_true, attrnone
	.word 0

	defcode tor, ">r", f_false, attrnone
	dpop wp
	rpush wp
	next

	defcode fromr, "r>", f_tor, attrnone
	rpop wp
	dpush wp
	next

	defword 2swap, "2swap", f_false, attrnone
	.word f_rot
	.word f_tor
	.word f_rot
	.word f_fromr
	.word f_exit

	defword compare, "compare", f_2swap, attrnone
	.word f_rot
	.word f_min
	.word f_dup
	.word f_branch0
	.word compare_fail
1:
	.word f_dup
	.word f_branch0
	.word compare_ok
	.word f_dec 
	.word f_rot // a2 n a1
	.word f_dup // a2 n a1 a1
	.word f_cload // a2 n a1 cload(a1)
	.word f_2swap // a1 cload(a1) a2 n
	.word f_over  // a1 cload(a1) a2 n a2
	.word f_cload // a1 cload(a1) a2 n cload(a2)
	.word f_tor   // a1 cload(a1) a2 n
	.word f_rot   // a1 a2 n cload(a1)
	.word f_fromr // a1 a2 n cload(a1) cload(a2)
	.word f_equ
	.word f_branch0
	.word compare_fail
	.word f_rot // a2 n a1
	.word f_inc
	.word f_rot // n a1 a2
	.word f_inc
	.word f_rot // a1 a2 n
	.word f_branch
	.word 1b
compare_ok:
	.word f_drop
	.word f_2drop
	.word f_true
	.word f_exit
compare_fail:
	.word f_drop
	.word f_2drop
	.word f_false
	.word f_exit

	defconst latest, "latest", f_compare, attrnone
	.word latest

	defcode wlinkget, "wlink@", f_latest, attrnone
	dpop wp
	lw xp, -addrsize(wp)
	li yp, addrmask
	and xp, xp, yp
	dpush xp
	next

	defcode wnlenget, "wnlen@", f_wlinkget, attrnone
	dpop wp
	lw xp, -addrsize(wp)
	li yp, nlenmask
	and xp, xp, yp
	srli xp, xp, nlenshift
	dpush xp
	next

	defcode wnameget, "wname@", f_wnlenget, attrnone
	dpop wp
	addi wp, wp, addrsize
	dpush wp
	next

	defword words, "words", f_wnameget, attrnone
	.word f_latest
	.word f_load
2:
	.word f_dup
	.word f_branch0
	.word 1f

	.word f_dup
	.word f_wnameget
	.word f_over
	.word f_wnlenget
	.word f_type
	.word f_lit
	.word ' '
	.word f_emit

	.word f_wlinkget
	.word f_branch
	.word 2b

1:
	.word f_drop
	.word f_exit

	defword 2over, "2over", f_words, attrnone
	.word f_tor
	.word f_tor
	.word f_2dup
	.word f_fromr
	.word f_fromr
	.word f_2swap
	.word f_exit

	defword nip, "nip", f_2over, attrnone
	.word f_swap
	.word f_drop
	.word f_exit

	defword find, "find", f_nip, attrnone
	.word f_latest
	.word f_load
1:
	.word f_dup
	.word f_branch0
	.word find_fail

	.word f_dup
	.word f_wnlenget // faddr fu link wu
	.word f_rot      // faddr link wu fu
	.word f_dup      // faddr link wu fu fu
	.word f_rot      // faddr link fu fu wu
	.word f_2swap    // faddr fu wu link fu
	.word f_swap     // faddr fu wu fu link
	.word f_2swap    // faddr fu link wu fu
	.word f_equ
	.word f_branch0
	.word find_next

	.word f_dup      // faddr fu link link
	.word f_wnameget // faddr fu link waddr
	.word f_2over    // faddr fu link waddr faddr fu
	.word f_dup      // faddr fu link waddr faddr fu fu
	.word f_rot      // faddr fu link waddr fu fu faddr
	.word f_swap     // faddr fu link waddr fu faddr fu
	.word f_compare
	.word f_branch0
	.word find_next

	.word f_nip
	.word f_nip
	.word f_exit

find_next:
	.word f_wlinkget
	.word f_branch
	.word 1b

find_fail:
	.word f_drop
	.word f_2drop
	.word f_false
	.word f_exit

	defcode execute, "execute", f_find, attrnone
	dpop wp
        li yp, addrmask
        and wp, wp, yp
        lw xp, 0(wp)
        and xp, xp, yp
        jr xp

	defword isxdigit, "isxdigit", f_execute, attrnone
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

	// (addr u) -- (bool)
	defword isnumber, "isnumber", f_isxdigit, attrnone
	.word f_dup
	.word f_branch0
	.word isnumber_false
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

isnumber_false:
	.word f_2drop
	.word f_false
	.word f_exit

isnumber_true:
	.word f_2drop
	.word f_true
	.word f_exit

	defcode lshift, "lshift", f_isnumber, attrnone
	dpop wp
	dpop xp
	sll xp, xp, wp
	dpush xp
	next

	defcode 4mul, "4*", f_lshift, attrnone
	dpop wp
	slli wp, wp, 2
	dpush wp
	next

	defword hex2num "hex2num", f_4mul, attrnone
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

	// (addr u) -- (number)
	defword number, "number", f_hex2num, attrnone
	.word f_dup
	.word f_branch0
	.word number_zero

	.word f_lit
	.word 0
	.word f_over
	.word f_dec
	.word f_4mul
	// addr u out shi

1:
	.word f_2swap
	// out shi addr u
	.word f_dup
	.word f_branch0
	.word number_end
	.word f_over  // out shi addr u addr
	.word f_cload // out shi addr u cload(addr)
	.word f_hex2num // out shi addr u hexnum
	.word f_tor     // out shi addr u
	.word f_dec
	.word f_swap    // out shi u addr
	.word f_inc
	.word f_swap    // out shi addr u

	.word f_2swap  // addr u out shi
	.word f_fromr  // addr u out shi hexnum
	.word f_over   // addr u out shi hexnum shi
	.word f_lshift // addr u out shi hexnum<<shi
	.word f_rot    // addr u shi hexnum<<shi out
	.word f_or     // addr u shi out
	.word f_swap   // addr u out shi
	.word f_lit
	.word 4
	.word f_sub
	.word f_branch
	.word 1b

number_end:
	.word f_drop
	.word f_2drop
	.word f_exit

number_zero:
	.word f_nip
	.word f_exit

	defcode ssget, "ss@", f_number, attrnone
	dpush ss
	next

	defcode ssset, "ss!", f_ssget, attrnone
	dpop ss
	next

	defconst sscomp, "sscomp", f_ssset, attrnone
	.word sscomp

	defcode bic, "bic", f_sscomp, attrnone
	dpop wp
	dpop xp
	xori wp, wp, -1
	and wp, wp, xp
	dpush wp
	next

	defword compoff, "[", f_bic, attrimmd
	.word f_ssget
	.word f_sscomp
	.word f_bic
	.word f_ssset
	.word f_exit

	defword compon, "]", f_compoff, attrnone
	.word f_ssget
	.word f_sscomp
	.word f_or
	.word f_ssset
	.word f_exit

	defword incomp, "incomp", f_compon, attrnone
	.word f_ssget
	.word f_sscomp
	.word f_and
	.word f_sscomp
	.word f_equ
	.word f_exit

	defconst ramhere, "ramhere", f_incomp, attrnone
	.word ramhere

	defword ramnext, "ramnext", f_ramhere, attrnone
	.word f_cell
	.word f_ramhere
	.word f_load
	.word f_add
	.word f_ramhere
	.word f_store
	.word f_exit

	defword rampush, "ram,", f_ramhere, attrnone
	.word f_ramhere
	.word f_load
	.word f_store
	.word f_ramnext
	.word f_exit

	defcode wisimmd, "wisimmd", f_rampush, attrnone
	dpop wp
	lw xp, -addrsize(wp)
	andi xp, xp, attrimmd
	bnez xp, 1f
	dpush zero
	next
1:
	li xp, -1
	dpush xp
	next

	defword here, "here", f_wisimmd, attrnone
	.word f_ramhere
	.word f_exit

	defword herenext, "herenext", f_here, attrnone
	.word f_ramnext
	.word f_exit

	defword herepush, ",", f_herenext, attrnone
	.word f_rampush
	.word f_exit

	defcode aligned, "aligned", f_herepush, attrnone
	dpop wp
	addi wp, wp, -1
	addi wp, wp, addrsize
	andi wp, wp, -addrsize
	dpush wp
	next

	defcode wentrget, "wentr@", f_aligned, attrnone
	dpop wp
	lw xp, 0(wp)
	li yp, addrmask
	and xp, xp, yp
	dpush xp
	next

	defword cmove, "cmove", f_wentrget, attrnone
1:
	.word f_dup
	.word f_branch0
	.word cmove_end
	.word f_dec
	.word f_tor    // saddr daddr
	.word f_over   // saddr daddr saddr
	.word f_cload  // saddr daddr cload(saddr)
	.word f_over   // saddr daddr cload(saddr) daddr
	.word f_cstore // saddr daddr
	.word f_inc
	.word f_swap
	.word f_inc
	.word f_swap
	.word f_fromr
	.word f_branch
	.word 1b

cmove_end:
	.word f_drop
	.word f_2drop
	.word f_exit

	defconst nlenshift, "nlenshift", f_cmove, attrnone
	.word nlenshift

	defconst offtshift, "offtshift", f_nlenshift, attrnone
	.word offtshift

	// (addr u) -- ()
	defword defword, "defword", f_offtshift, attrnone
	// set link
	.word f_dup
	.word f_nlenshift
	.word f_lshift
	.word f_latest
	.word f_load
	.word f_or
	.word f_herepush

	// set latest
	.word f_here
	.word f_load
	.word f_latest
	.word f_store

	// set entr
	.word f_dup
	.word f_aligned
	.word f_cell
	.word f_add
	.word f_offtshift
	.word f_lshift
	.word f_2lit
	.word _str_call
	.word 4
	.word f_find
	.word f_wentrget
	.word f_or
	.word f_herepush

	// set name
	.word f_here
	.word f_load
	.word f_swap
	.word f_dup
	.word f_tor
	.word f_cmove

	.word f_fromr
	.word f_aligned
	.word f_here
	.word f_load
	.word f_add
	.word f_here
	.word f_store

	.word f_exit
_str_call:
	.ascii "call"
	.p2align 2, 0

	defword docom, ":", f_defword, attrnone
1:
        .word f_toinrst
        .word f_token
        .word f_toin
        .word f_load
        .word f_branch0
        .word 1b

	.word f_tib
	.word f_toin
	.word f_load
	.word f_defword
	.word f_compon
	.word f_exit

	defword doend, ";", f_docom, attrimmd
	.word f_2lit
	.word _str_exit
	.word 4
	.word f_find
	.word f_herepush
	.word f_compoff
	.word f_exit

_str_exit:
	.ascii "exit"
	.p2align 2, 0

	defconst ssdund, "ssdund", f_doend, attrnone
	.word ssdund

	defcode sprst, "sprst", f_ssdund, attrnone
	addi sp, st, 0
	next

	defword ifnz, "if", f_sprst, attrimmd
	.word f_2lit
	.word _str_branch0
	.word 7
	.word f_find
	.word f_herepush
	.word f_here
	.word f_load
	.word f_dup // save address, used for 'then'
	.word f_cell
	.word f_add
	.word f_here
	.word f_store
	.word f_exit

_str_branch0:
	.ascii "branch0"
	.p2align 2, 0

	defword then, "then", f_ifnz, attrimmd
	.word f_here
	.word f_load
	.word f_swap
	.word f_store
	.word f_exit

	defword begin, "begin", f_then, attrimmd
	.word f_here
	.word f_load // save address, used for until
	.word f_exit

	defword until, "until", f_begin, attrimmd
	.word f_2lit
	.word _str_branch0
	.word 7
	.word f_find
	.word f_herepush
	.word f_herepush
	.word f_exit

	defword dot, ".", f_until, attrnone
	.word f_hex32
	.word f_exit

	defcode dogon, "dogon", f_dot, attrnone
	li wp, IWDG_BASE
	li xp, IWDG_KEYON
	sh xp, IWDG_CTLR_R16(wp)
	next

	defcode feedog, "feedog", f_dogon, attrnone
	li wp, IWDG_BASE
	li xp, IWDG_KEYFEED
	sh xp, IWDG_CTLR_R16(wp)
	next

	defconst yieldcount, "yieldcount", f_feedog, attrnone
	.word yield_count

	defcode systickon, "systickon", f_yieldcount, attrnone
        li wp, STK_BASE
        li xp, 1000 // 1ms
        sw xp, STK_CMPLR(wp)
        sw zero, STK_CNTL(wp)
        li xp, STK_STRE | STK_HCLKDIV8 | STK_STIE | STK_STEN
        sw xp, STK_CTLR(wp)
	next

	defcode systickoff, "systickoff", f_systickon, attrnone
	li wp, STK_BASE
	sw zero, STK_CTLR(wp)
	next

	defconst systickcount, "systickcount", f_systickoff, attrnone
	.word systick_count

	defconst systickcounthigh, "systickcounthigh", f_systickcount, attrnone
	.word systick_count + addrsize

	defword millis, "millis", f_systickcounthigh, attrnone
	.word f_systickcount
	.word f_load
	.word f_exit

	defword quest, "?", f_millis, attrnone
	.word f_load
	.word f_dot
	.word f_exit

	defcode neq, "<>", f_quest, attrnone
	dpop wp
	dpop xp
	bne wp, xp, 1f
	dpush zero
	next
1:
	li xp, -1
	dpush xp
	next

	defword delay1ms, "delay1ms", f_neq, attrnone
	.word f_millis
1:
	.word f_yield
	.word f_dup
	.word f_millis
	.word f_neq
	.word f_branch0
	.word 1b
	.word f_drop
	.word f_exit

	defword delayms, "delayms", f_delay1ms, attrnone
2:
	.word f_dup
	.word f_branch0
	.word 1f
	.word f_delay1ms
	.word f_dec
	.word f_branch
	.word 2b
1:
	.word f_drop
	.word f_exit

	defword motd, "motd", f_delayms, attrnone
	.word f_2lit
	.word _str_motd
	.word _str_motd_end - _str_motd
	.word f_type
	.word f_2lit
	.word _str_flash_info
	.word _str_flash_info_end - _str_flash_info
	.word f_type
	.word f_romsize
	.word f_hex16
	.word f_newline
	.word f_2lit
	.word _str_uid
	.word _str_uid_end - _str_uid
	.word f_type
	.word f_uid1
	.word f_hex32
	.word f_uid2
	.word f_hex32
	.word f_uid3
	.word f_hex32
	.word f_newline
	.word f_exit

_str_motd:
	.ascii "FORTH on CH32V003\n\r"
_str_motd_end:
_str_flash_info:
	.ascii "FLASH SIZE: 0x"
_str_flash_info_end:
_str_uid:
	.ascii "CHIP UID: "
_str_uid_end:
	.p2align 2, 0

	defword tick, "'", f_motd, attrnone
1:
        .word f_toinrst
        .word f_token
        .word f_toin
        .word f_load
        .word f_branch0
        .word 1b

	.word f_tib
	.word f_toin
	.word f_load
	.word f_find
	.word f_exit

	defword allot, "allot", f_tick, attrnone
1:
	.word f_dup
	.word f_branch0
	.word 2f
	.word f_dec
	.word f_herenext
	.word f_branch
	.word 1b
2:
	.word f_drop
	.word f_exit

	defconst tasksize, "tasksize", f_allot, attrnone
	.word tasksize

	defconst dstksize, "dstksize", f_tasksize, attrnone
	.word dstksize

	defconst rstksize, "rstksize", f_dstksize, attrnone
	.word rstksize

	defconst gapsize, "gapsize", f_rstksize, attrnone
	.word gapsize

	defconst tonext, "tonext", f_gapsize, attrnone
	.word tonext

	defword tnextget, "tnext@", f_tonext, attrnone
	.word f_tonext
	.word f_add
	.word f_load
	.word f_exit

        defword tnextset, "tnext!", f_tnextget, attrnone
        .word f_tonext
        .word f_add
        .word f_store
        .word f_exit

        defconst torp, "torp", f_tnextset, attrnone
        .word torp

        defword trpget, "trp@", f_torp, attrnone
        .word f_torp
        .word f_add
        .word f_load
        .word f_exit

        defword trpset, "trp!", f_trpget, attrnone
        .word f_torp
        .word f_add
        .word f_store
        .word f_exit

        defconst tosp, "tosp", f_trpset, attrnone
        .word tosp

        defword tspget, "tsp@", f_tosp, attrnone
        .word f_tosp
        .word f_add
        .word f_load
        .word f_exit

        defword tspset, "tsp!", f_tspget, attrnone
        .word f_tosp
        .word f_add
        .word f_store
        .word f_exit

        defconst toip, "toip", f_tspset, attrnone
        .word toip

        defword tipget, "tip@", f_toip, attrnone
        .word f_toip
        .word f_add
        .word f_load
        .word f_exit

        defword tipset, "tip!", f_tipget, attrnone
        .word f_toip
        .word f_add
        .word f_store
        .word f_exit

	defconst toss, "toss", f_tipset, attrnone
        .word toss

        defword tssget, "tss@", f_toss, attrnone
        .word f_toss
        .word f_add
        .word f_load
        .word f_exit

        defword tssset, "tss!", f_tssget, attrnone
        .word f_toss
        .word f_add
        .word f_store
        .word f_exit

	defconst tost, "tost", f_tssset, attrnone
        .word tost

        defword tstget, "tst@", f_tost, attrnone
        .word f_tost
        .word f_add
        .word f_load
        .word f_exit

        defword tstset, "tst!", f_tstget, attrnone
        .word f_tost
        .word f_add
        .word f_store
        .word f_exit

	defcode upget, "up@", f_tstset, attrnone
	dpush up
	next

	// (entry)
	defword tasknew, "tasknew", f_upget, attrnone
	.word f_here
	.word f_load // entry taskaddr
	.word f_tasksize
	.word f_allot
	.word f_swap // taskaddr entry
	.word f_over // taskaddr entry taskaddr
	.word f_tipset // taskaddr

	.word f_rstksize
	.word f_allot
	.word f_here
	.word f_load // taskaddr rstktop
	.word f_over // taskaddr rstktop taskaddr
	.word f_trpset // taskaddr
	.word f_lit
	.word 1
	.word f_allot

	.word f_dstksize
	.word f_allot
	.word f_here
	.word f_load // taskaddr dstktop
	.word f_over // taskaddr dstktop taskaddr
	.word f_2dup // taskaddr dstktop taskaddr dstktop taskaddr
	.word f_tspset 
	.word f_tstset
	.word f_lit
	.word 1
	.word f_allot

	.word f_lit
	.word 0
	.word f_over  // taskaddr 0 taskaddr
	.word f_tssset

	.word f_upget
	.word f_tnextget // taskaddr upnext
	.word f_over     // taskaddr upnext taskaddr
	.word f_tnextset // taskaddr

	.word f_upget
	.word f_tnextset

	.word f_exit

	defcode wbodyget, "wbody@",  f_tasknew, attrnone
	dpop wp
	lw xp, 0(wp)
	li yp, offtmask
	and xp, xp, yp
	srli xp, xp, offtshift
	add xp, xp, wp
	dpush xp
	next

	defword dotask, "task;", f_wbodyget, attrimmd
	.word f_doend
	.word f_latest
	.word f_load
	.word f_wbodyget
	.word f_tasknew
	.word f_exit

	defcode reboot, "reboot", f_dotask, attrnone
sysrst:
        .equ PFIC_CFGR, 0x48
        .equ PFIC_KEY3, (0xBEEF << 16)
        .equ SYSRESET, (1 << 7)
        li wp, PFIC_BASE
        li xp, SYSRESET | PFIC_KEY3
        sw xp, PFIC_CFGR(wp)
1:
        j 1b

	defcode uid1, "uid1", f_reboot, attrnone
	.equ ESIG_BASE, 0x1FFFF700
	.equ ESIG_UNIID1, 0xE8
	li wp, ESIG_BASE
	lw xp, ESIG_UNIID1(wp)
	dpush xp
	next

	defcode uid2, "uid2", f_uid1, attrnone
	.equ ESIG_UNIID2, 0xEC
	li wp, ESIG_BASE
	lw xp, ESIG_UNIID2(wp)
	dpush xp
	next

	defcode uid3, "uid3", f_uid2, attrnone
	.equ ESIG_UNIID3, 0xF0
	li wp, ESIG_BASE
	lw xp, ESIG_UNIID3(wp)
	dpush xp
	next

	defcode romsize, "romsize", f_uid3, attrnone
	.equ ESIG_FLACAP, 0xE0
	li wp, ESIG_BASE
	lhu xp, ESIG_FLACAP(wp)
	dpush xp
	next

	// TODO, impl div
	defword sysclk, "sysclk", f_romsize, attrnone
	.word f_lit
	.word 24
	.word f_exit

	defword hclk, "hclk", f_sysclk, attrnone
	.word f_lit
	.word 8
	.word f_exit

	defcode baudset, "baudset", f_hclk, attrnone
	li wp, UART1_BASE
	dpop xp
	li yp, 9600
	beq xp, yp, baud9600
	li yp, 115200
	beq xp, yp, baud115200
	next

baud9600:
	li xp, BAUD_9600
	sw xp, UART_BRR(wp)
	next

baud115200:
	li xp, BAUD_115200
	sw xp, UART_BRR(wp)
	next

	defcode paoutget, "paout@", f_hclk, attrnone
	li wp, GPIOA_BASE
	lw xp, GPIO_OUTDR(wp)
	dpush xp
	next

	defcode paoutset, "paout!", f_paoutget, attrnone
	li wp, GPIOA_BASE
	dpop xp
	sw xp, GPIO_OUTDR(wp)
	next

	defcode romunlock, "romunlock", f_paoutset, attrnone
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

	defcode romlock, "romlock", f_romunlock, attrnone
	.equ FLASH_LOCK, (1 << 7)
	li wp, FLASH_BASE
	lw xp, FLASH_CTLR(wp)
	ori xp, xp, FLASH_LOCK
	sw xp, FLASH_CTLR(wp)
	next

	defcode rom16store, "rom16!", f_romlock, attrnone
	dpop wp // addr
	dpop xp // value
	li yp, 0xFFFF
	and xp, xp, yp
rom16w_loop:
	lhu yp, 0(wp)
	beq xp, yp, rom16w_done
	dpush xp
start_rom_write:
	.equ FLASH_PG, (1 << 0)
	la yp, FLASH_BASE
	lw xp, FLASH_CTLR(yp)
	ori xp, xp, FLASH_PG
	sw xp, FLASH_CTLR(yp)
	dpop xp
	.equ CODE_FLASH_BASE, 0x08000000
	li yp, CODE_FLASH_BASE
	dpush wp
	or wp, wp, yp
	sh xp, 0(wp)
	dpop wp
wait_rom_write:
	dpush xp
	.equ FLASH_BUSY, (1 << 0)
	la yp, FLASH_BASE
1:
	lw xp, FLASH_STATR(yp)
	andi xp, xp, FLASH_BUSY
	bnez xp, 1b
end_rom_write:
	lw xp, FLASH_CTLR(yp)
	xori xp, xp, FLASH_PG
	sw xp, FLASH_CTLR(yp)
	dpop xp
	j rom16w_loop
rom16w_done:
	next

	defword rom32store, "rom32!", f_rom16store, attrnone
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

	defword interpret, "interpret", f_rom32store, attrnone
interpret_start:
	.word f_toinrst
	.word f_token
	.word f_toin
	.word f_load
	.word f_branch0
	.word interpret_start

	.word f_tib
	.word f_toin
	.word f_load
	.word f_find
	.word f_dup
	.word f_branch0
	.word interpret_noword
	.word f_dup
	.word f_wisimmd
	.word f_branch0
	.word 2f
	.word f_branch
	.word interpret_execute
2:
	.word f_incomp
	.word f_branch0
	.word interpret_execute
	.word f_herepush
	.word f_branch
	.word interpret_start

interpret_execute:
	.word f_execute
	.word f_ssget
	.word f_ssdund
	.word f_and
	.word f_ssdund
	.word f_equ
	.word f_branch0
	.word 1f
	.word f_2lit
	.word _str_stk_err
	.word _str_stk_err_end - _str_stk_err
	.word f_type
	.word f_sprst
	.word f_ssget
	.word f_ssdund
	.word f_bic
	.word f_ssset
1:
	.word f_branch
	.word interpret_start

_str_stk_err:
	.ascii " stack error \n\r"
_str_stk_err_end:
	.p2align 2, 0


interpret_noword:
	.word f_drop
	.word f_tib
	.word f_toin
	.word f_load
	.word f_isnumber
	.word f_branch0
	.word interpret_nonum
	.word f_tib
	.word f_toin
	.word f_load
	.word f_number
	.word f_incomp
	.word f_branch0
	.word interpret_nothing
	.word f_2lit
	.word _str_lit
	.word 3
	.word f_find
	.word f_herepush
	.word f_herepush
	.word f_branch
	.word interpret_start

_str_lit:
	.ascii "lit"
	.p2align 2, 0

interpret_nothing:
	.word f_branch
	.word interpret_start

interpret_nonum:
	.word f_drop
	.word f_tib
	.word f_toin
	.word f_load
	.word f_type
	.word f_2lit
	.word str_notfound
	.word str_end_notfound - str_notfound
	.word f_type
	.word f_sprst

	.word f_branch
	.word interpret_start

str_notfound:
	.ascii " not found\n\r"
str_end_notfound:
	.p2align 2, 0

boot_human:
	.word f_noop

#ifdef TEST
test_lit_branch0:
	.word f_lit
	.word 0
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_lit
	.word 1
	.word f_branch0
	.word fail
	.word f_dzchk

test_equ:
	.word f_lit
	.word 1
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word 0
	.word f_lit
	.word 0
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word 1
	.word f_lit
	.word 0
	.word f_equ
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

test_2lit:
        .word f_2lit
        .word 0
        .word 1
        .word f_lit
        .word 1
        .word f_equ
	.word f_branch0
	.word fail
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

test_cell:
	.word f_cell
	.word f_lit
	.word addrsize
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

test_depth:
	.word f_depth
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_depth
	.word f_depth
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

test_num2hex:
	.word f_lit
	.word 0
	.word f_num2hex
	.word f_lit
	.word '0'
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word 9
	.word f_num2hex
	.word f_lit
	.word '9'
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word 0xA
	.word f_num2hex
	.word f_lit
	.word 'A'
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word 0xF
	.word f_num2hex
	.word f_lit
	.word 'F'
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk
test_rshift:
	.word f_lit
	.word 1
	.word f_lit
	.word 0
	.word f_rshift
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word 2
	.word f_lit
	.word 1
	.word f_rshift
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word 4
	.word f_lit
	.word 2
	.word f_rshift
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

test_dup:
	.word f_lit
	.word 5
	.word f_dup
	.word f_lit
	.word 5
	.word f_equ
	.word f_branch0
	.word fail
	.word f_lit
	.word 5
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

test_spstget:
	.word f_spget
	.word f_stget
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

test_load:
	.word f_lit
	.word 1f
	.word f_load
	.word f_lit
1:
	.word 0x55AA4477
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

test_sub:
	.word f_lit
	.word 1
	.word f_lit
	.word 0
	.word f_sub
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

        .word f_lit
        .word 1
        .word f_lit
        .word 1
        .word f_sub
        .word f_lit
        .word 0
        .word f_equ
        .word f_branch0
        .word fail
        .word f_dzchk

        .word f_lit
        .word 0
        .word f_lit
        .word 1
        .word f_sub
        .word f_lit
        .word -1
        .word f_equ
        .word f_branch0
        .word fail
        .word f_dzchk

test_eqz:
	.word f_lit
	.word 0
	.word f_eqz
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word 1
	.word f_eqz
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

test_drop:
	.word f_lit
	.word 1
	.word f_drop
	.word f_dzchk

test_swap:
	.word f_lit
	.word 0
	.word f_lit
	.word 1
	.word f_swap
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

test_over:
	.word f_lit
	.word 0
	.word f_lit
	.word 1
	.word f_over
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail

	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

test_branch:
	.word f_branch
	.word 1f
	.word fail
1:
	.word f_dzchk

test_dec:
	.word f_lit
	.word 1
	.word f_dec
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

	.word f_lit
	.word 2
	.word f_dec
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk
	
	.word f_lit
	.word 0
	.word f_dec
	.word f_lit
	.word -1
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

test_isspace:
	.word f_lit
	.word ' '
	.word f_isspace
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word 'a'
	.word f_isspace
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

test_or:
	.word f_lit
	.word 0
	.word f_lit
	.word 1
	.word f_or
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word 1
	.word f_lit
	.word 0
	.word f_or
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word 1
	.word f_lit
	.word 1
	.word f_or
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word 0
	.word f_lit
	.word 0
	.word f_or
	.word f_lit
	.word 0
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

test_isnewline:
	.word f_lit
	.word '\n'
	.word f_isnewline
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word '\r'
	.word f_isnewline
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word 'a'
	.word f_isnewline
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

test_isdelete:
	.word f_lit
	.word '\b'
	.word f_isdelete
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word 0x7F
	.word f_isdelete
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word 0
	.word f_isdelete
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

test_store:
	.word f_lit
	.word 5
	.word f_toin
	.word f_store
	.word f_toin
	.word f_load
	.word f_lit
	.word 5
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_toinrst
	.word f_toin
	.word f_load
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

test_inc:
	.word f_lit
	.word 0
	.word f_inc
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word 1
	.word f_inc
	.word f_lit
	.word 2
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word -1
	.word f_inc
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

test_cstore:
	.word f_lit
	.word 0x55
	.word f_tib
	.word f_cstore
	.word f_dzchk
	.word f_lit
	.word 0xAA
	.word f_tib
	.word f_inc
	.word f_cstore
	.word f_dzchk

	.word f_tib
	.word f_cload
	.word f_lit
	.word 0x55
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_tib
	.word f_inc
	.word f_cload
	.word f_lit
	.word 0xAA
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

test_lt:
	.word f_lit
	.word 0
	.word f_lit
	.word 1
	.word f_lt
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word 1
	.word f_lit
	.word 2
	.word f_lt
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word -1
	.word f_lit
	.word 0
	.word f_lt
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word -2
	.word f_lit
	.word -1
	.word f_lt
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word 0
	.word f_lit
	.word 0
	.word f_lt
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

	.word f_lit
	.word 1
	.word f_lit
	.word 0
	.word f_lt
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

	.word f_lit
	.word -1
	.word f_lit
	.word -2
	.word f_lt
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

test_gt:
	.word f_lit
	.word 1
	.word f_lit
	.word 0
	.word f_gt
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word 0
	.word f_lit
	.word -1
	.word f_gt
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word 2
	.word f_lit
	.word 1
	.word f_gt
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word -1
	.word f_lit
	.word -2
	.word f_gt
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word 0
	.word f_lit
	.word 0
	.word f_gt
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

	.word f_lit
	.word 0
	.word f_lit
	.word 0
	.word f_gt
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

	.word f_lit
	.word 0
	.word f_lit
	.word 1
	.word f_gt
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

	.word f_lit
	.word -1
	.word f_lit
	.word 0
	.word f_gt
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

	.word f_lit
	.word -2
	.word f_lit
	.word -1
	.word f_gt
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

test_2dup:
	.word f_lit
	.word 0
	.word f_lit
	.word 1
	.word f_2dup
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail
	.word f_lit
	.word 0
	.word f_equ
	.word f_branch0
	.word fail
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail
	.word f_lit
	.word 0
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

test_rot:
	.word f_lit
	.word 0
	.word f_lit
	.word 1
	.word f_lit
	.word 2
	.word f_rot
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_lit
	.word 2
	.word f_equ
	.word f_branch0
	.word fail
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

test_ge:
	.word f_lit
	.word 0
	.word f_lit
	.word 0
	.word f_ge
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word 1
	.word f_lit
	.word 0
	.word f_ge
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word 2
	.word f_lit
	.word 1
	.word f_ge
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word 0
	.word f_lit
	.word -1
	.word f_ge
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word 0
	.word f_lit
	.word 1
	.word f_ge
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

	.word f_lit
	.word -1
	.word f_lit
	.word 0
	.word f_ge
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

	.word f_lit
	.word -2
	.word f_lit
	.word -1
	.word f_ge
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

test_le:
	.word f_lit
	.word 1
	.word f_lit
	.word 1
	.word f_le
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word 0
	.word f_lit
	.word 1
	.word f_le
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word -1
	.word f_lit
	.word 0
	.word f_le
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word -2
	.word f_lit
	.word -1
	.word f_le
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word 1
	.word f_lit
	.word 0
	.word f_le
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

test_and:
	.word f_lit
	.word 1
	.word f_lit
	.word 0
	.word f_and
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

	.word f_lit
	.word 0
	.word f_lit
	.word 1
	.word f_and
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

	.word f_lit
	.word 0
	.word f_lit
	.word 0
	.word f_and
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

	.word f_lit
	.word 1
	.word f_lit
	.word 1
	.word f_and
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

test_within:
	.word f_lit
	.word 1
	.word f_lit
	.word 2
	.word f_lit
	.word 3
	.word f_within
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

	.word f_lit
	.word 3
	.word f_lit
	.word 2
	.word f_lit
	.word 3
	.word f_within
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

	.word f_lit
	.word 4
	.word f_lit
	.word 2
	.word f_lit
	.word 3
	.word f_within
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

	.word f_lit
	.word 1
	.word f_lit
	.word 1
	.word f_lit
	.word 3
	.word f_within
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word 2
	.word f_lit
	.word 1
	.word f_lit
	.word 3
	.word f_within
	.word f_branch0
	.word fail
	.word f_dzchk

test_add:
	.word f_lit
	.word 1
	.word f_lit
	.word 1
	.word f_add
	.word f_lit
	.word 2
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

        .word f_lit
        .word 0
        .word f_lit
        .word 1
        .word f_add
        .word f_lit
        .word 1
        .word f_equ
        .word f_branch0
        .word fail
        .word f_dzchk

        .word f_lit
        .word -1
        .word f_lit
        .word 1
        .word f_add
        .word f_branch0
        .word 1f
	.word f_fail
1:
        .word f_dzchk

test_tipush:
	.word f_lit
	.word 0xF5
	.word f_tipush
	.word f_dzchk
	.word f_toin
	.word f_load
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail
	.word f_tib
	.word f_cload
	.word f_lit
	.word 0xF5
	.word f_equ
	.word f_branch0
	.word fail

	.word f_toinmax
	.word f_dec
	.word f_toin
	.word f_store
	.word f_lit
	.word 0x0
	.word f_tipush
	.word f_toin
	.word f_load
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

test_tidrop:
	.word f_lit
	.word 0xFA
	.word f_tipush
	.word f_tidrop
	.word f_toin
	.word f_load
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

	.word f_tidrop
	.word f_toin
	.word f_load
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

test_min:
	.word f_lit
	.word 0
	.word f_lit
	.word 1
	.word f_min
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

	.word f_lit
	.word 1
	.word f_lit
	.word 0
	.word f_min
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

	.word f_lit
	.word -1
	.word f_lit
	.word 0
	.word f_min
	.word f_lit
	.word -1
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word -2
	.word f_lit
	.word -1
	.word f_min
	.word f_lit
	.word -2
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

test_tor_fromr:
	.word f_lit
	.word 0x5A
	.word f_tor
	.word f_dzchk
	.word f_fromr
	.word f_lit
	.word 0x5A
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

test_2swap:
	.word f_lit
	.word 1
	.word f_lit
	.word 2
	.word f_lit
	.word 3
	.word f_lit
	.word 4
	.word f_2swap
	.word f_lit
	.word 2
	.word f_equ
	.word f_branch0
	.word fail
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail
	.word f_lit
	.word 4
	.word f_equ
	.word f_branch0
	.word fail
	.word f_lit
	.word 3
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

test_compare:
	.word f_lit
	.word 0
	.word f_dup
	.word f_dup
	.word f_dup
	.word f_compare
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

	.word f_lit
	.word test_compare
	.word f_lit
	.word 9
	.word f_2dup
	.word f_compare
	.word f_branch0
	.word fail
	.word f_dzchk


	.word f_lit	
	.word _cmp0
	.word f_lit
	.word 6
	.word f_lit
	.word _cmp1
	.word f_lit
	.word 7
	.word f_compare
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word _cmp0
	.word f_lit
	.word 7
	.word f_lit
	.word _cmp1
	.word f_lit
	.word 7
	.word f_compare
	.word f_branch0
	.word 1f
	.word f_fail
_cmp0:
	.ascii "Hello Alice"
_cmp1:
	.ascii "Hello Jack"
	.p2align 2, 0
1:
	.word f_dzchk

test_wlinkget:
	.word f_lit
	.word f_uuu
	.word f_wlinkget
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

	.word f_lit
	.word f_fail
	.word f_wlinkget
	.word f_lit
	.word f_uuu
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

test_wnlenget:
	.word f_lit
	.word f_uuu
	.word f_wnlenget
	.word f_lit
	.word 3
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word f_fail
	.word f_wnlenget
	.word f_lit
	.word 4
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

test_wnameget:
	.word f_lit
	.word f_uuu
	.word f_wnameget
	.word f_lit
	.word n_uuu
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

        .word f_lit
        .word f_fail
        .word f_wnameget
        .word f_lit
        .word n_fail
        .word f_equ
        .word f_branch0
        .word fail
        .word f_dzchk

test_2over:
	.word f_lit
	.word 5
	.word f_lit
	.word 6
	.word f_lit
	.word 7
	.word f_lit
	.word 8
	.word f_2over
	.word f_lit
	.word 6
	.word f_equ
	.word f_branch0
	.word fail
	.word f_lit
	.word 5
	.word f_equ
	.word f_branch0
	.word fail
	.word f_lit
	.word 8
	.word f_equ
	.word f_branch0
	.word fail
	.word f_lit
	.word 7
	.word f_equ
	.word f_branch0
	.word fail
	.word f_lit
	.word 6
	.word f_equ
	.word f_branch0
	.word fail
	.word f_lit
	.word 5
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

test_nip:
	.word f_lit
	.word 0
	.word f_lit
	.word 1
	.word f_nip
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

test_find:
	.word f_lit
	.word _f0
	.word f_lit
	.word 3
	.word f_find
	.word f_lit
	.word f_uuu
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word _f1
	.word f_lit
	.word 4
	.word f_find
	.word f_branch0
	.word 1f
	.word f_fail
_f0:
	.ascii "uuu"
_f1:
	.ascii "asdf"
	.p2align 2, 0
1:
	.word f_dzchk

test_execute:
	.word f_lit
	.word f_false
	.word f_execute
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

test_isxdigit:
	.word f_lit
	.word '0'
	.word f_isxdigit
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word '9'
	.word f_isxdigit
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word 'A'
	.word f_isxdigit
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word 'F'
	.word f_isxdigit
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word 'n'
	.word f_isxdigit
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

	.word f_lit
	.word '0' - 1
	.word f_isxdigit
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

	.word f_lit
	.word '0' - 1
	.word f_isxdigit
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

	.word f_lit
	.word '9' + 1
	.word f_isxdigit
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

	.word f_lit
	.word 'A' - 1
	.word f_isxdigit
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

	.word f_lit
	.word 'F' + 1
	.word f_isxdigit
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

test_isnumber:
	.word f_lit
	.word 0
	.word f_lit
	.word 0
	.word f_isnumber
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

	.word f_lit
	.word _num0
	.word f_lit
	.word 5
	.word f_isnumber
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word _num1
	.word f_lit
	.word 5
	.word f_isnumber
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word _num2
	.word f_lit
	.word 5
	.word f_isnumber
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

	.word f_lit
	.word _num3
	.word f_lit
	.word 5
	.word f_isnumber
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

	.word f_lit
	.word _num4
	.word f_lit
	.word 5
	.word f_isnumber
	.word f_branch0
	.word 1f
	.word f_fail
1:

	.word f_branch
	.word 1f
_num0:
	.ascii "AB123"
_num1:
	.ascii "DEF11"
_num2:
	.ascii " EF11"
_num3:
	.ascii "EF11 "
_num4:
	.ascii "EF 11"
	.p2align 2, 0
1:
	.word f_dzchk

test_lshift:
	.word f_lit
	.word 1
	.word f_lit
	.word 0
	.word f_lshift
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

        .word f_lit
        .word 1
        .word f_lit
        .word 1
        .word f_lshift
        .word f_lit
        .word 2
        .word f_equ
        .word f_branch0
        .word fail
        .word f_dzchk

        .word f_lit
        .word 1
        .word f_lit
        .word 2
        .word f_lshift
        .word f_lit
        .word 4
        .word f_equ
        .word f_branch0
        .word fail
        .word f_dzchk

        .word f_lit
        .word 1
        .word f_lit
        .word 3
        .word f_lshift
        .word f_lit
        .word 8
        .word f_equ
        .word f_branch0
        .word fail
        .word f_dzchk

test_4mul:
	.word f_lit
	.word 0
	.word f_4mul
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk
	.word f_lit
	.word 1
	.word f_4mul
	.word f_lit
	.word 4
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk
	.word f_lit
	.word 2
	.word f_4mul
	.word f_lit
	.word 8
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

test_hex2num:
	.word f_lit
	.word '0'
	.word f_hex2num
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

	.word f_lit
	.word '9'
	.word f_hex2num
	.word f_lit
	.word 9
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word 'A'
	.word f_hex2num
	.word f_lit
	.word 0xA
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word 'F'
	.word f_hex2num
	.word f_lit
	.word 0xF
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

test_number:
	.word f_lit
	.word 0
	.word f_lit
	.word 0
	.word f_number
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

	.word f_lit
	.word _num0
	.word f_lit
	.word 5
	.word f_number
	.word f_lit
	.word 0xAB123
	.word f_equ
	.word f_branch0
	.word f_fail
	.word f_dzchk

        .word f_lit
        .word _num1
        .word f_lit
        .word 5
        .word f_number
        .word f_lit
        .word 0xDEF11
        .word f_equ
        .word f_branch0
        .word f_fail
        .word f_dzchk

	.word f_lit
	.word _numfull
	.word f_lit
	.word 8
	.word f_number
	.word f_lit
	.word 0xA5A55A5A
	.word f_equ
	.word f_branch0
	.word f_fail
	.word f_dzchk

	.word f_branch
	.word 1f
_numfull:
	.ascii "A5A55A5A"
	.p2align 2, 0
1:

test_bic:
	.word f_lit
	.word 1
	.word f_lit
	.word 1
	.word f_bic
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

	.word f_lit
	.word 0
	.word f_lit
	.word 1
	.word f_bic
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

	.word f_lit
	.word 1
	.word f_lit
	.word 0
	.word f_bic
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

	.word f_lit
	.word 3
	.word f_lit
	.word 1
	.word f_bic
	.word f_lit
	.word 2
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

test_comp:
	.word f_compon
	.word f_incomp
	.word f_branch0
	.word fail
	.word f_compoff
	.word f_incomp
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

test_wisimmd:
	.word f_lit
	.word f_uuu
	.word f_wisimmd
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

	.word f_lit
	.word f_compoff
	.word f_wisimmd
	.word f_branch0
	.word fail
	.word f_dzchk

test_aligned:
	.word f_lit
	.word 0
	.word f_aligned
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk

        .word f_lit
        .word 1
        .word f_aligned
	.word f_lit
	.word addrsize
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

        .word f_lit
        .word 2
        .word f_aligned
        .word f_lit
        .word addrsize
        .word f_equ
        .word f_branch0
        .word fail
        .word f_dzchk

        .word f_lit
        .word 3
        .word f_aligned
        .word f_lit
        .word addrsize
        .word f_equ
        .word f_branch0
        .word fail
        .word f_dzchk

        .word f_lit
        .word addrsize
        .word f_aligned
        .word f_lit
        .word addrsize
        .word f_equ
        .word f_branch0
        .word fail
        .word f_dzchk

        .word f_lit
        .word addrsize + 1
        .word f_aligned
        .word f_lit
        .word addrsize * 2
        .word f_equ
        .word f_branch0
        .word fail
        .word f_dzchk

test_wentrget:
	.word f_lit
	.word f_call
	.word f_wentrget
	.word f_lit
	.word a_call
	.word f_equ
	.word f_branch0
	.word fail
	.word f_dzchk

test_defword:
	.word f_lit
	.word _str_hi
	.word f_lit
	.word 2
	.word f_defword

	.word f_dzchk

	.word f_lit
	.word f_lit
	.word f_herepush

	.word f_lit
	.word 'H'
	.word f_herepush

	.word f_lit
	.word f_emit
	.word f_herepush

	.word f_lit
	.word f_lit
	.word f_herepush

	.word f_lit
	.word 'i'
	.word f_herepush

	.word f_lit
	.word f_emit
	.word f_herepush

	.word f_lit
	.word f_exit
	.word f_herepush

	.word f_branch
	.word 1f
_str_hi:
	.ascii "hi"
	.p2align 2, 0
1:
	.word f_dzchk

test_tx:
	.word f_lit
	.word 'F'
	.word f_emit
	.word f_dzchk
	.word f_lit
	.word 'O'
	.word f_emit
	.word f_dzchk
	.word f_lit
	.word 'R'
	.word f_emit
	.word f_dzchk
	.word f_lit
	.word 'T'
	.word f_emit
	.word f_dzchk
	.word f_lit
	.word 'H'
	.word f_emit
	.word f_dzchk

test_toinchk:
	.word f_toinchk
	.word f_branch0
	.word fail
	.word f_dzchk
	.word f_toinmax
	.word f_toin
	.word f_store
	.word f_toinchk
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk
	.word f_lit
	.word -1
	.word f_toin
	.word f_store
	.word f_toinchk
	.word f_branch0
	.word 1f
	.word f_fail
1:
	.word f_dzchk
	.word f_toinrst
	.word f_toinchk
	.word f_branch0
	.word fail
	.word f_dzchk

test_hex:
	.word f_lit
	.word 0
	.word f_hex4
	.word f_dzchk
	.word f_lit
	.word 9
	.word f_hex4
	.word f_dzchk
	.word f_lit
	.word 0xA
	.word f_hex4
	.word f_dzchk
	.word f_lit
	.word 0xF
	.word f_hex4
	.word f_dzchk

	.word f_lit
	.word 0x08
	.word f_hex8
	.word f_dzchk
	.word f_lit
	.word 0x09
	.word f_hex8
	.word f_dzchk
	.word f_lit
	.word 0xAF
	.word f_hex8
	.word f_dzchk

	.word f_lit
	.word 0x0010
	.word f_hex16
	.word f_dzchk
	.word f_lit
	.word 0x09AF
	.word f_hex16
	.word f_dzchk

	.word f_lit
	.word 0x00000020
	.word f_hex32
	.word f_dzchk
	.word f_lit
	.word 0x01234567
	.word f_hex32
	.word f_dzchk
	.word f_lit
	.word 0x89ABCDEF
	.word f_hex32
	.word f_dzchk
	.word f_lit
	.word 0xFEDCBA98
	.word f_hex32
	.word f_dzchk
	.word f_lit
	.word 0x76543210
	.word f_hex32
	.word f_dzchk

	.word f_spget
	.word f_hex32
	.word f_dzchk
	.word f_stget
	.word f_hex32
	.word f_dzchk

test_toinmax:
	.word f_lit
	.word 'T'
	.word f_emit
	.word f_toinmax
	.word f_hex8
	.word f_dzchk

test_dsdump:
	.word f_dsdump
	.word f_dzchk
	.word f_lit
	.word 0x55
	.word f_lit
	.word 0xAA
	.word f_dsdump
	.word f_drop
	.word f_drop
	.word f_dsdump
	.word f_dzchk

test_words:
	.word f_words
	.word f_dzchk

test_type:
	.word f_lit
	.word 1f
	.word f_lit
	.word 7
	.word f_type
	.word f_dzchk
	.word f_branch
	.word 2f
1:
	.ascii "FORTH\n\r"
	.p2align 2, 0
2:

test_interpret:
	.word f_interpret

test_token:
1:
	.word f_toinrst
	.word f_token
	.word f_lit
	.word '['
	.word f_emit
	.word f_tib
	.word f_toin
	.word f_load
	.word f_type
	.word f_lit
	.word ']'
	.word f_emit
	.word f_branch
	.word 1b

test_echo:
	.word f_key
	.word f_emit
	.word f_lit
	.word 0
	.word f_branch0
	.word test_echo
#endif

	.word f_motd
	.word f_interpret
	.word f_uuu
fail:
	.word f_fail

boot_dog:
	.word f_yield
	.word f_feedog
	.word f_branch
	.word boot_dog

forth:
	li ss, 0
	la ip, boot_human
	la rp, rstk_human_end
	la st, dstk_human_end
	mv sp, st
	la up, task_human
	la wp, task_dog
	sw wp, tonext(up)
	tasksave

        li ss, 0
        la ip, boot_dog
        la rp, rstk_dog_end
        la st, dstk_dog_end
        mv sp, st
        la up, task_dog
	la wp, task_human
        sw wp, tonext(up)
        tasksave

	la wp, yield_count
	sw zero, 0(wp)

	la wp, toin
	sw zero, 0(wp)
	la wp, lastword
	la xp, latest
	sw wp, 0(xp)
	la wp, _ram_dict_start
	la xp, ramhere
	sw wp, 0(xp)

	la wp, systick_count
	sw zero, 0(wp)
	sw zero, addrsize(wp)

	next

	// flash min erase size is 1K
	.p2align 10, 0xFF  // align with 1024

	.section .bss
_ram_vector:
	.fill _vector_end - _vector, 1, 0
_ram_vector_end:
dstk_human:
        .fill dstksize, addrsize, 0
dstk_human_end:
        .fill 1, addrsize, 0
dgap_human:
        .fill gapsize, addrsize, 0
rstk_human:
        .fill rstksize, addrsize, 0
rstk_human_end:
        .fill 1, addrsize, 0
systick_count:
	.fill 2, addrsize, 0
yield_count:
	.fill 1, addrsize, 0
tib:
	.fill 31, 1, 0
tib_end:
	.fill 1, 1, 0
toin:
	.fill 1, addrsize, 0
latest:
	.fill 1, addrsize, 0
ramhere:
	.fill 1, addrsize, 0
task_human:
	.fill tasksize, 1, 0
dstk_dog:
        .fill dstksize, addrsize, 0
dstk_dog_end:
        .fill 1, addrsize, 0
rstk_dog:
        .fill rstksize, addrsize, 0
rstk_dog_end:
        .fill 1, addrsize, 0
task_dog:
	.fill tasksize, addrsize, 0

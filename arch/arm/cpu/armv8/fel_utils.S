/*
 * Utility functions for FEL mode, when running SPL in AArch64.
 *
 * Copyright (c) 2017 Arm Ltd.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm-offsets.h>
#include <config.h>
#include <asm/system.h>
#include <linux/linkage.h>
#include <asm/arch/cpu.h>

/*
 * Where the H713 FEL trap door lives (see return_to_fel below). DRAM, clear of
 * BL31 at 0x40000000 and of U-Boot at CONFIG_TEXT_BASE. VBAR_EL3 needs 2 KiB
 * alignment; the vector slot and the mailbox are fixed offsets into it.
 */
#define H713_FEL_DOOR_BASE	0x48000000
#define H713_FEL_DOOR_VEC	0x600		/* sync, lower EL, AArch32 */
#define H713_FEL_DOOR_MBOX	0x700		/* jump target, 64-bit */

/*
 * We don't overwrite save_boot_params() here, to save the FEL state upon
 * entry, since this would run *after* the RMR reset, which clobbers that
 * state.
 * Instead we store the state _very_ early in the boot0 hook, *before*
 * resetting to AArch64.
 */

/*
 * The FEL routines in BROM run in AArch32.
 * Reset back into 32-bit mode here and restore the saved FEL state
 * afterwards.
 * Resetting back into AArch32/EL3 using the RMR always enters the BROM,
 * but we can use the CPU hotplug mechanism to branch back to our code
 * immediately.
 */
ENTRY(return_to_fel)
	/*
	 * the RMR reset will clear all registers, so save the arguments
	 * (LR and SP) in the fel_stash structure, which we read anyways later
	 */
	adr	x2, fel_stash
	str	w0, [x2]
	str	w1, [x2, #4]

	adr	x1, fel_stash_addr	// to find the fel_stash address in AA32
	str	w2, [x1]

#if defined(CONFIG_MACH_SUN50I_H713)
	/*
	 * H713 cannot return via the H616/H6 RMR hotplug mailbox; it
	 * exception-returns directly into the AArch32 FEL restore stub.
	 *
	 * That return leaves EL3 in AArch64, so the AArch32 RMR register is
	 * out of reach for whatever runs next: the BROM's FEL loop resumes at
	 * EL1, where "mcr p15, 0, rX, c12, c0, 2" is undefined. Any later
	 * attempt to enter AArch64 the usual way - sunxi-fel's "uboot" and
	 * "reset64", for instance - therefore wedges the core instead of
	 * booting it.
	 *
	 * Leave a door open on the way out. EL3 being AArch64 is exactly what
	 * makes an "smc" from AArch32 a trap door back into AArch64/EL3, so
	 * install a minimal EL3 vector table whose "synchronous exception from
	 * a lower EL in AArch32" slot jumps to an address deposited in a
	 * mailbox. FEL can then re-enter AArch64 with a plain "smc #0", with
	 * no RMR, no warm reset and no help from the BROM.
	 *
	 * The table lives in DRAM, which is up by the time the FEL boot path
	 * calls us (this is only reached for BOOT_DEVICE_BOARD). A mailbox of
	 * zero parks the core rather than jumping into nowhere.
	 */
	mrs	x0, CurrentEL
	cmp	x0, #0xc
	b.ne	4f				// only EL3 owns VBAR_EL3

	movz	x0, #(H713_FEL_DOOR_BASE >> 16), lsl #16
	adr	x1, h713_fel_door_stub
	add	x2, x0, #H713_FEL_DOOR_VEC
	ldp	x3, x4, [x1]
	stp	x3, x4, [x2]
	ldr	x3, [x1, #16]
	str	x3, [x2, #16]
	str	xzr, [x0, #H713_FEL_DOOR_MBOX]	// no target yet: park

	/* publish the freshly written code to the instruction side */
	add	x4, x2, #16
	dc	cvau, x2
	dc	cvau, x4
	dsb	ish
	ic	ivau, x2
	ic	ivau, x4
	dsb	ish
	isb

	msr	VBAR_EL3, x0
	isb
4:
	b	return_to_fel_eret

	/*
	 * The stub is copied to H713_FEL_DOOR_BASE + 0x600, so its literal
	 * offsets have to be relative to *that* address, not to this one.
	 * Hand-encoded for that reason:
	 *
	 *   ldr  x0, [pc, #0x100]   ; mailbox at H713_FEL_DOOR_BASE + 0x700
	 *   cbz  x0, .+8            ; empty mailbox: park instead of jumping
	 *   br   x0
	 *   wfi
	 *   b    .-4
	 */
	.align	3
h713_fel_door_stub:
	.word	0x58000800
	.word	0xb4000040
	.word	0xd61f0000
	.word	0xd503207f
	.word	0x17ffffff
	.word	0xd503207f			// pad to a 24-byte copy
#else
	ldr	w0, =0xfa50392f		// CPU hotplug magic
#if defined(CONFIG_MACH_SUN50I_H616) || defined(CONFIG_MACH_SUN50I_A133) || \
    defined(CONFIG_MACH_SUN55I_A523)
	ldr	w2, =(SUNXI_R_CPUCFG_BASE + 0x1c0)
	str	w0, [x2], #0x4
#elif defined(CONFIG_MACH_SUN50I_H6)
	ldr	w2, =(SUNXI_RTC_BASE + 0x1b8)	// BOOT_CPU_HP_FLAG_REG
	str	w0, [x2], #0x4
#else
	ldr	w2, =(SUNXI_CPUCFG_BASE + 0x1a4) // offset for CPU hotplug base
	str	w0, [x2, #0x8]
#endif
	adr	x0, back_in_32
	str	w0, [x2]

	dsb	sy
	isb	sy
	mov	x0, #2			// RMR reset into AArch32
	dsb	sy
	msr	RMR_EL3, x0
	isb	sy
1:	wfi
	b	1b
#endif

ENTRY(return_to_fel_eret)
	adr	x2, fel_stash
	adr	x1, fel_stash_addr	// to find the fel_stash address in AA32
	str	w2, [x1]

	ldr	w0, [x2, #8]		// saved AArch32 CPSR
	adr	x3, back_in_32
	dsb	sy
	isb	sy

	mrs	x1, CurrentEL
	cmp	x1, #0xc
	b.eq	3f
	cmp	x1, #0x8
	b.eq	2f

	msr	SPSR_EL1, x0
	msr	ELR_EL1, x3
	isb	sy
	eret

2:	mrs	x4, HCR_EL2
	bic	x4, x4, #(1 << 31)	// lower EL runs AArch32
	msr	HCR_EL2, x4
	msr	SPSR_EL2, x0
	msr	ELR_EL2, x3
	isb	sy
	eret

3:	mrs	x4, SCR_EL3
	bic	x4, x4, #0xf		// restore secure AArch32-style routing
	bic	x4, x4, #(1 << 10)	// lower EL runs AArch32
	msr	SCR_EL3, x4
	msr	SPSR_EL3, x0
	msr	ELR_EL3, x3
	isb	sy
	eret
1:	wfi
	b	1b
ENDPROC(return_to_fel_eret)

fel_stash_addr:			// must immediately precede back_in_32:
	.word   0x00000000	// receives fel_stash addr, by AA64 code above

/* AArch32 code to restore the state from fel_stash and return back to FEL. */
back_in_32:
	.word	0xe51f000c	// ldr	r0, [pc, #-12]  ; load fel_stash address
	.word	0xe5901008	// ldr	r1, [r0, #8]
	.word	0xe129f001	// msr	CPSR_fc, r1
	.word	0xf57ff06f	// isb
	.word	0xe590d000	// ldr	sp, [r0]
	.word	0xe590e004	// ldr	lr, [r0, #4]
	.word	0xe5901014	// ldr  r1, [r0, #20]
	.word	0xe121f301	// msr  SP_irq, r1
	.word	0xe5901010	// ldr	r1, [r0, #16]
	.word	0xee0c1f10	// mcr	15, 0, r1, cr12, cr0, {0} ; VBAR
	.word	0xe590100c	// ldr	r1, [r0, #12]
	.word	0xee011f10	// mcr	15, 0, r1, cr1, cr0, {0}  ; SCTLR
	.word	0xf57ff06f	// isb
#ifdef CONFIG_MACH_SUN55I_A523
	.word	0xe5901018	// ldr  r1, [r0, #24]
	.word	0xee041f16	// mcr  15, 0, r1, cr4, cr6, {0}; ICC_PMR
	.word	0xe590101c	// ldr  r1, [r0, #28]
	.word	0xee0c1ffc	// mcr  15, 0, r1, cr12, cr12, {7}; ICC_IGRPEN1
#endif

	.word	0xe12fff1e	// bx	lr		; return to FEL
ENDPROC(return_to_fel)

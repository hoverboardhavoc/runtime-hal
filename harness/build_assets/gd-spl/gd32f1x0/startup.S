/* Minimal startup for the GD32F1x0 (Cortex-M3) SPL snippet harness.
 *
 * The extractor enters at regcmp_test with LR = sentinel, so this _start path
 * is not normally executed (it exists only to give the ELF a valid vector
 * table / entry). SystemInit and main are never called: only the snippet body's
 * MMIO is captured. Written fresh for this harness.
 */

    .syntax unified
    .cpu cortex-m3
    .thumb

    .section .vectors, "a", %progbits
    .align 2
    .global _vectors
_vectors:
    .word _estack
    .word _start + 1
    .word _default_handler + 1   /* NMI */
    .word _default_handler + 1   /* HardFault */
    .word _default_handler + 1   /* MemManage */
    .word _default_handler + 1   /* BusFault */
    .word _default_handler + 1   /* UsageFault */
    .word 0
    .word 0
    .word 0
    .word 0
    .word _default_handler + 1   /* SVCall */
    .word _default_handler + 1   /* DebugMon */
    .word 0
    .word _default_handler + 1   /* PendSV */
    .word _default_handler + 1   /* SysTick */

    .section .text._default_handler, "ax", %progbits
    .thumb_func
    .global _default_handler
_default_handler:
    b _default_handler

    .section .text._start, "ax", %progbits
    .thumb_func
    .global _start
_start:
    ldr  r0, =_sentinel + 1
    mov  lr, r0
    bl   regcmp_test
    b    _sentinel

    .section .text._sentinel, "ax", %progbits
    .thumb_func
    .global _sentinel
_sentinel:
    bkpt #0x55

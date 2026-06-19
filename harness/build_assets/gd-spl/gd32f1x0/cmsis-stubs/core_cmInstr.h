/* Minimal CMSIS-Core 4.x instruction-access shim for GD's core_cm3.h.
 *
 * The GD32F1x0 CMSIS dir ships only core_cm3.h, which #includes the CMSIS-Core
 * 4.x split-out instruction header it does not provide. For snippet emulation
 * we only need these intrinsics to compile; they are thin wrappers around
 * standard GCC builtins / inline asm. Written fresh for this harness (no ARM
 * CMSIS content). On the include path before the SPL trees.
 */

#ifndef __CORE_CMINSTR_H
#define __CORE_CMINSTR_H

#include <stdint.h>

#define __NOP()         __asm volatile ("nop")
#define __WFI()         __asm volatile ("wfi")
#define __WFE()         __asm volatile ("wfe")
#define __SEV()         __asm volatile ("sev")
#define __ISB()         __asm volatile ("isb 0xF":::"memory")
#define __DSB()         __asm volatile ("dsb 0xF":::"memory")
#define __DMB()         __asm volatile ("dmb 0xF":::"memory")
#define __CLREX()       __asm volatile ("clrex" ::: "memory")

#define __REV(v)        __builtin_bswap32(v)
#define __REV16(v)      __builtin_bswap16(v)
#define __REVSH(v)      ((int16_t)__builtin_bswap16(v))

__attribute__((always_inline)) static inline uint32_t __RBIT(uint32_t v)
{
    uint32_t r;
    __asm volatile ("rbit %0, %1" : "=r" (r) : "r" (v));
    return r;
}

__attribute__((always_inline)) static inline uint8_t __CLZ(uint32_t v)
{
    return (uint8_t)__builtin_clz(v);
}

#define __LDREXB(p)     (*(volatile uint8_t *)(p))
#define __LDREXH(p)     (*(volatile uint16_t *)(p))
#define __LDREXW(p)     (*(volatile uint32_t *)(p))
#define __STREXB(v, p)  ((*(volatile uint8_t *)(p) = (v)), 0)
#define __STREXH(v, p)  ((*(volatile uint16_t *)(p) = (v)), 0)
#define __STREXW(v, p)  ((*(volatile uint32_t *)(p) = (v)), 0)

#endif /* __CORE_CMINSTR_H */

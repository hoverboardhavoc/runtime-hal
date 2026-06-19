/* Minimal CMSIS-Core 4.x function-access shim for GD's core_cm3.h.
 *
 * The GD32F1x0 CMSIS dir ships only core_cm3.h, which #includes this split-out
 * function-access header it does not provide. These core-register accessors are
 * thin inline-asm wrappers; for snippet emulation they only need to compile (the
 * GPIO path never executes them). Written fresh for this harness (no ARM CMSIS
 * content). On the include path before the SPL trees.
 */

#ifndef __CORE_CMFUNC_H
#define __CORE_CMFUNC_H

#include <stdint.h>

#ifndef __STATIC_INLINE
#define __STATIC_INLINE static inline
#endif

__attribute__((always_inline)) __STATIC_INLINE void __enable_irq(void)  { __asm volatile ("cpsie i" ::: "memory"); }
__attribute__((always_inline)) __STATIC_INLINE void __disable_irq(void) { __asm volatile ("cpsid i" ::: "memory"); }

#define _CMFUNC_GET(reg)  \
    ({ uint32_t __r; __asm volatile ("MRS %0, " #reg : "=r" (__r)); __r; })
#define _CMFUNC_SET(reg, v) \
    do { __asm volatile ("MSR " #reg ", %0" :: "r" (v) : "memory"); } while (0)

__attribute__((always_inline)) __STATIC_INLINE uint32_t __get_CONTROL(void)        { return _CMFUNC_GET(control); }
__attribute__((always_inline)) __STATIC_INLINE void     __set_CONTROL(uint32_t v)  { _CMFUNC_SET(control, v); }
__attribute__((always_inline)) __STATIC_INLINE uint32_t __get_IPSR(void)           { return _CMFUNC_GET(ipsr); }
__attribute__((always_inline)) __STATIC_INLINE uint32_t __get_APSR(void)           { return _CMFUNC_GET(apsr); }
__attribute__((always_inline)) __STATIC_INLINE uint32_t __get_xPSR(void)           { return _CMFUNC_GET(xpsr); }
__attribute__((always_inline)) __STATIC_INLINE uint32_t __get_PSP(void)            { return _CMFUNC_GET(psp); }
__attribute__((always_inline)) __STATIC_INLINE void     __set_PSP(uint32_t v)      { _CMFUNC_SET(psp, v); }
__attribute__((always_inline)) __STATIC_INLINE uint32_t __get_MSP(void)            { return _CMFUNC_GET(msp); }
__attribute__((always_inline)) __STATIC_INLINE void     __set_MSP(uint32_t v)      { _CMFUNC_SET(msp, v); }
__attribute__((always_inline)) __STATIC_INLINE uint32_t __get_PRIMASK(void)        { return _CMFUNC_GET(primask); }
__attribute__((always_inline)) __STATIC_INLINE void     __set_PRIMASK(uint32_t v)  { _CMFUNC_SET(primask, v); }
__attribute__((always_inline)) __STATIC_INLINE void     __enable_fault_irq(void)   { __asm volatile ("cpsie f" ::: "memory"); }
__attribute__((always_inline)) __STATIC_INLINE void     __disable_fault_irq(void)  { __asm volatile ("cpsid f" ::: "memory"); }
__attribute__((always_inline)) __STATIC_INLINE uint32_t __get_BASEPRI(void)        { return _CMFUNC_GET(basepri); }
__attribute__((always_inline)) __STATIC_INLINE void     __set_BASEPRI(uint32_t v)  { _CMFUNC_SET(basepri, v); }
__attribute__((always_inline)) __STATIC_INLINE uint32_t __get_FAULTMASK(void)      { return _CMFUNC_GET(faultmask); }
__attribute__((always_inline)) __STATIC_INLINE void     __set_FAULTMASK(uint32_t v){ _CMFUNC_SET(faultmask, v); }

#endif /* __CORE_CMFUNC_H */

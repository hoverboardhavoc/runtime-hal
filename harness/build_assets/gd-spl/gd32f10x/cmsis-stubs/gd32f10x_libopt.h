/* Minimal gd32f10x_libopt.h: selects which SPL peripheral headers gd32f10x.h
 * pulls in. The vendor SPL expects the user to supply this; for the harness we
 * include the peripherals the vectors exercise. Written fresh for this harness.
 *
 * Unlike the F1x0 CMSIS dir (which shipped only core_cm3.h, so the harness had
 * to vendor core_cmInstr.h / core_cmFunc.h too), the F10x CMSIS dir already
 * ships core_cmInstr.h and core_cmFunc.h, so only this libopt stub is needed.
 */

#ifndef GD32F10X_LIBOPT_H
#define GD32F10X_LIBOPT_H

#include "gd32f10x_rcu.h"
#include "gd32f10x_gpio.h"
#include "gd32f10x_usart.h"

#endif /* GD32F10X_LIBOPT_H */

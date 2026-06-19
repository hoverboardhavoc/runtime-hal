/* Minimal gd32f1x0_libopt.h: selects which SPL peripheral headers gd32f1x0.h
 * pulls in. The vendor SPL expects the user to supply this; for the harness we
 * include the peripherals the vectors exercise. Written fresh for this harness.
 */

#ifndef GD32F1X0_LIBOPT_H
#define GD32F1X0_LIBOPT_H

#include "gd32f1x0_rcu.h"
#include "gd32f1x0_gpio.h"
#include "gd32f1x0_usart.h"

#endif /* GD32F1X0_LIBOPT_H */

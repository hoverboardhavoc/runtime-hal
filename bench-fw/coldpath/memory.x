/* GD32F130C8: 64 KiB flash @ 0x0800_0000, 8 KiB RAM @ 0x2000_0000.
 *
 * Stock cortex-m-rt layout: it places .vector_table at FLASH origin, then .text/.rodata in FLASH,
 * and .data/.bss/.uninit + the stack in RAM (with _stack_start = end of RAM). The SWD-readable
 * result struct (M2_ANCHORS) is an ordinary zero-initialised `static mut` in .bss; the SWD reader
 * gets its address from `arm-none-eabi-nm`, so it does not need a fixed address.
 */
MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 64K
    RAM   : ORIGIN = 0x20000000, LENGTH = 8K
}

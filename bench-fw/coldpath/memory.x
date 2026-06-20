/* GD32F130C8: 64 KiB flash @ 0x0800_0000, 8 KiB RAM @ 0x2000_0000.
 *
 * Stock cortex-m-rt layout: it places .vector_table at FLASH origin, then .text/.rodata in FLASH,
 * and .data/.bss/.uninit + the stack in RAM. The RAM length here is the full 8 KiB MINUS a reserved
 * 256-byte tail: cortex-m-rt lays out .data/.bss and the stack within the shrunk region (top =
 * _stack_start = 0x2000_1F00), so it never touches the tail. The firmware writes its result struct to
 * a FIXED address at the start of that tail (RESULT_ADDR = 0x2000_1F00) and the SWD reader reads that
 * constant directly, no `arm-none-eabi-nm` symbol resolution needed (the size-optimised release ELF
 * drops the .symtab nm reads). An earlier attempt to pin a fixed RAM-ORIGIN section collided with
 * cortex-m-rt; reserving the TAIL avoids that.
 */
MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 64K
    RAM   : ORIGIN = 0x20000000, LENGTH = 0x1F00 /* 8 KiB - 256 B result tail @ 0x2000_1F00 */
}

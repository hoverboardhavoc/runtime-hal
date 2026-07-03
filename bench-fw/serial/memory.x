/* ONE binary for BOTH bench parts, so it must fit the SMALLER RAM: the GD32F130C8 has 8 KiB RAM
 * (the GD32F103 master has more and simply uses less). 64 KiB flash @ 0x0800_0000, 8 KiB RAM @
 * 0x2000_0000.
 *
 * Stock cortex-m-rt layout: .vector_table at FLASH origin, .text/.rodata in FLASH, .data/.bss/.uninit
 * + the stack in RAM. The RAM length here is the full 8 KiB MINUS a reserved 256-byte tail:
 * cortex-m-rt lays out .data/.bss and the stack within the shrunk region (top = _stack_start =
 * 0x2000_1F00), so it never touches the tail. The firmware writes its result struct to a FIXED
 * address at the start of that tail (RESULT_ADDR = 0x2000_1F00) and the SWD reader reads that
 * constant directly, no symbol resolution needed (the size-optimised release ELF drops .symtab).
 * Reserving the TAIL avoids the cortex-m-rt collision a pinned RAM-ORIGIN section hit.
 */
MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 64K
    RAM   : ORIGIN = 0x20000000, LENGTH = 0x1F00 /* 8 KiB - 256 B result tail @ 0x2000_1F00 */
}

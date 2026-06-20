/* Shared conservative layout for BOTH bench boards (and the 12-FET F10x).
 *
 * The GD32F130C8 has 64 KiB flash @ 0x0800_0000 and 8 KiB RAM @ 0x2000_0000; the GD32F103C8 has
 * 64 KiB flash and 20 KiB RAM. Using the SMALLER (F130) RAM size here makes ONE .bin valid on both
 * parts (it never places anything past the F130's 8 KiB, which the F103 also has). A high-density
 * F10x (the 12-FET board) has at least this much RAM too, so the same image runs there.
 *
 * The RAM length is the full 8 KiB MINUS a reserved 256-byte tail: cortex-m-rt lays out .data/.bss
 * and the stack within the shrunk region (top = _stack_start = 0x2000_1F00), so it never touches the
 * tail. The firmware writes its result struct to a FIXED address at the start of that tail
 * (RESULT_ADDR = 0x2000_1F00) and the SWD reader reads that constant directly, no `arm-none-eabi-nm`
 * symbol resolution needed (the size-optimised release ELF drops the .symtab nm reads). An earlier
 * attempt to pin a fixed RAM-ORIGIN section collided with cortex-m-rt; reserving the TAIL avoids that.
 */
MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 64K
    RAM   : ORIGIN = 0x20000000, LENGTH = 0x1F00 /* 8 KiB - 256 B result tail @ 0x2000_1F00 */
}

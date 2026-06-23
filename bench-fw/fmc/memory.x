/* Conservative layout valid on every fleet part (F103C8 / F130C8 / 12-FET F103RC).
 *
 * FLASH 64 KiB and RAM 8 KiB are the SMALLEST fleet sizes (the C8s), so one image links for all parts;
 * the 12-FET has more of both but using the small sizes never places anything past what every part
 * has. NOTE: this only constrains where the LINKER puts THIS firmware's own code/data - the FMC driver
 * under test computes its scratch page + out-of-flash bound from flash_size_bytes()/page_size() at
 * RUNTIME, so on the 12-FET it correctly erases/programs high flash (~254 KiB) even though this script
 * says 64 KiB.
 *
 * RAM length is 8 KiB MINUS a reserved 256-byte tail: cortex-m-rt lays out .data/.bss + stack within
 * the shrunk region, so it never touches the tail. The firmware writes its result struct to the fixed
 * RESULT_ADDR = 0x2000_1F00 (start of the tail) and the SWD reader reads that constant directly, no
 * nm symbol resolution (the size-optimised release ELF drops .symtab).
 */
MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 64K
    RAM   : ORIGIN = 0x20000000, LENGTH = 0x1F00 /* 8 KiB - 256 B result tail @ 0x2000_1F00 */
}

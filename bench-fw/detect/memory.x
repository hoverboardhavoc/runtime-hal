/* Shared conservative layout for BOTH bench boards (DF-T6).
 *
 * The GD32F130C8 has 64 KiB flash @ 0x0800_0000 and 8 KiB RAM @ 0x2000_0000; the GD32F103C8 has
 * 64 KiB flash and 20 KiB RAM. Using the SMALLER (F130) RAM size here makes ONE .bin valid on both
 * parts (it never places anything past the F130's 8 KiB, which the F103 also has). The result struct
 * (DETECT_RESULT) is an ordinary zero-initialised `static mut` in .bss; the SWD reader resolves its
 * address with `arm-none-eabi-nm`, so it needs no fixed address (pinning a fixed RAM-origin section
 * collided with cortex-m-rt's own RAM allocation, so do NOT do that).
 */
MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 64K
    RAM   : ORIGIN = 0x20000000, LENGTH = 8K
}

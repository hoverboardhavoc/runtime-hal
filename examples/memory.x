/* One linker layout valid on BOTH supported boards (the chip is detected at runtime, not selected
 * at build time, so one image must link for either part).
 *
 * The GD32F130C8 has 64 KiB flash @ 0x0800_0000 and 8 KiB RAM @ 0x2000_0000; the GD32F103C8 has
 * 64 KiB flash and 20 KiB RAM. Using the SMALLER (F130) 8 KiB RAM here keeps ONE image valid on both
 * parts: it never places anything past the F130's 8 KiB, which the F103 also has. The examples need
 * no fixed-origin RAM section, so do NOT pin one (that collides with cortex-m-rt's own RAM layout).
 */
MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 64K
    RAM   : ORIGIN = 0x20000000, LENGTH = 8K
}

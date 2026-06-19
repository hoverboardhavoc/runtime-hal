/* Shared conservative layout for BOTH bench boards.
 *
 * The GD32F130C8 has 64 KiB flash @ 0x0800_0000 and 8 KiB RAM @ 0x2000_0000; the GD32F103C8 has
 * 64 KiB flash and 20 KiB RAM. Using the SMALLER (F130) RAM size makes ONE .bin valid on both parts
 * (it never places anything past the F130's 8 KiB, which the F103 also has).
 */
MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 64K
    RAM   : ORIGIN = 0x20000000, LENGTH = 8K
}

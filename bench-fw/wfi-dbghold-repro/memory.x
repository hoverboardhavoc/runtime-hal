/* GD32F130C8: 64 KiB flash @ 0x0800_0000, 8 KiB RAM @ 0x2000_0000. RAM length is the full 8 KiB MINUS
 * a 256-byte tail; cortex-m-rt lays out .data/.bss + stack within the shrunk region (top = 0x2000_1F00)
 * so it never touches the tail. The marker word is written to the start of that tail (0x2000_1F00). */
MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 64K
    RAM   : ORIGIN = 0x20000000, LENGTH = 0x1F00 /* 8 KiB - 256 B marker tail @ 0x2000_1F00 */
}

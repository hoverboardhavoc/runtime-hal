/* Minimal linker script for the runtime-hal snippet harness.
 *
 * Links the startup + the runtime-hal staticlib. KEEP(regcmp_test) +
 * KEEP(.vectors) survive --gc-sections. Real-target memory sizes are
 * irrelevant: Unicorn maps whatever ranges the extractor declares. Written
 * fresh for this harness.
 */

ENTRY(_start)

MEMORY
{
    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 256K
    SRAM  (rwx): ORIGIN = 0x20000000, LENGTH = 32K
}

_estack = ORIGIN(SRAM) + LENGTH(SRAM);

SECTIONS
{
    .vectors : {
        KEEP(*(.vectors))
    } > FLASH

    .text : {
        KEEP(*(.text.regcmp_test))
        *(.text._start)
        *(.text._sentinel)
        *(.text._default_handler)
        *(.text*)
        *(.rodata*)
    } > FLASH

    .data : {
        _sdata = .;
        *(.data*)
        _edata = .;
    } > SRAM AT > FLASH

    .bss : {
        _sbss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } > SRAM

    /DISCARD/ : {
        *(.ARM.exidx*)
        *(.ARM.attributes)
    }
}

ENTRY(ResetISR)

MEMORY
{
  SRAM (rwx) : ORIGIN = 0x20280000, LENGTH = 0x00280000
}

__base_SRAM = ORIGIN(SRAM);
__top_SRAM = ORIGIN(SRAM) + LENGTH(SRAM);

__stack_size__ = 0x4000;
__heap_size__ = 0x4000;

SECTIONS
{
  .interrupts :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } > SRAM

  .text :
  {
    . = ALIGN(4);
    *(.after_vectors*)
    *(.text*)
    *(CodeQuickAccess)
    *(CodeQuickAccess*)
    *(.rodata*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame*)
    KEEP(*(.init))
    KEEP(*(.fini))
    . = ALIGN(4);
  } > SRAM

  .ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } > SRAM

  .ARM.exidx :
  {
    __exidx_start = .;
    *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    __exidx_end = .;
  } > SRAM

  .preinit_array :
  {
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
  } > SRAM

  .init_array :
  {
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .);
  } > SRAM

  .fini_array :
  {
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .);
  } > SRAM

  .section_table :
  {
    . = ALIGN(4);
    __data_section_table = .;
    LONG(LOADADDR(.data));
    LONG(ADDR(.data));
    LONG(SIZEOF(.data));
    LONG(LOADADDR(.quickaccess_data));
    LONG(ADDR(.quickaccess_data));
    LONG(SIZEOF(.quickaccess_data));
    __data_section_table_end = .;

    __bss_section_table = .;
    LONG(ADDR(.bss));
    LONG(SIZEOF(.bss));
    __bss_section_table_end = .;
  } > SRAM

  .data :
  {
    . = ALIGN(4);
    __data_start__ = .;
    *(.data*)
    . = ALIGN(4);
    __data_end__ = .;
  } > SRAM

  .quickaccess_data :
  {
    . = ALIGN(4);
    *(DataQuickAccess)
    *(DataQuickAccess*)
    . = ALIGN(4);
  } > SRAM

  .smartdma :
  {
    . = ALIGN(4);
    __smartdma_start__ = .;
    KEEP(*(.smartdma_data))
    KEEP(*(.smartdma_code))
    . = ALIGN(4);
    __smartdma_end__ = .;
  } > SRAM

  .bss (NOLOAD) :
  {
    . = ALIGN(4);
    __bss_start__ = .;
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    __bss_end__ = .;
  } > SRAM

  .heap (NOLOAD) :
  {
    . = ALIGN(8);
    __end__ = .;
    end = .;
    _end = .;
    __HeapBase = .;
    . += __heap_size__;
    __HeapLimit = .;
  } > SRAM

  .stack_dummy (NOLOAD) :
  {
    . = ALIGN(8);
    . += __stack_size__;
  } > SRAM

  _vStackTop = ORIGIN(SRAM) + LENGTH(SRAM);
  _vStackBase = _vStackTop - SIZEOF(.stack_dummy);
  PROVIDE(__stack = _vStackTop);
  _image_size = SIZEOF(.interrupts) + SIZEOF(.text) + SIZEOF(.ARM.extab) + SIZEOF(.ARM.exidx) + SIZEOF(.preinit_array) + SIZEOF(.init_array) + SIZEOF(.fini_array) + SIZEOF(.section_table) + SIZEOF(.data) + SIZEOF(.quickaccess_data);

  /DISCARD/ :
  {
    *(.note.GNU-stack)
    *(.gnu_debuglink)
    *(.comment)
  }
}
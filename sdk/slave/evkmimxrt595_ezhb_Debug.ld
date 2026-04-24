ENTRY(ResetISR)

MEMORY
{
  FLASH (rx)         : ORIGIN = 0x08000000, LENGTH = 0x04000000
  SRAM (rwx)         : ORIGIN = 0x20280000, LENGTH = 0x00280000
  USB_RAM (rwx)      : ORIGIN = 0x40140000, LENGTH = 0x00004000
  RAM_EZH_CODE (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00040000
}

__base_FLASH = ORIGIN(FLASH);
__top_FLASH = ORIGIN(FLASH) + LENGTH(FLASH);
__base_SRAM = ORIGIN(SRAM);
__top_SRAM = ORIGIN(SRAM) + LENGTH(SRAM);
__base_USB_RAM = ORIGIN(USB_RAM);
__top_USB_RAM = ORIGIN(USB_RAM) + LENGTH(USB_RAM);
__base_RAM_EZH_CODE = ORIGIN(RAM_EZH_CODE);
__top_RAM_EZH_CODE = ORIGIN(RAM_EZH_CODE) + LENGTH(RAM_EZH_CODE);

__stack_size__ = 0x4000;
__heap_size__ = 0x4000;

SECTIONS
{
  .interrupts 0x08001000 :
  {
    FILL(0xff)
    __vectors_start__ = ABSOLUTE(.);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
    __vectors_end__ = ABSOLUTE(.);
  } > FLASH

  .flash_conf 0x08000400 :
  {
    . = ALIGN(4);
    KEEP(*(.flash_conf))
    . = ALIGN(4);
  } > FLASH

  .text 0x08001200 :
  {
    . = ALIGN(4);
    *(.after_vectors*)
    *(.text*)
    *(.rodata*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame*)
    KEEP(*(.init))
    KEEP(*(.fini))
    . = ALIGN(4);
  } > FLASH

  .ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } > FLASH

  .ARM.exidx :
  {
    __exidx_start = .;
    *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    __exidx_end = .;
  } > FLASH

  .ctors :
  {
    __CTOR_LIST__ = .;
    KEEP(*(.ctors))
    KEEP(*(SORT(.ctors.*)))
    __CTOR_END__ = .;
  } > FLASH

  .dtors :
  {
    __DTOR_LIST__ = .;
    KEEP(*(.dtors))
    KEEP(*(SORT(.dtors.*)))
    __DTOR_END__ = .;
  } > FLASH

  .preinit_array :
  {
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
  } > FLASH

  .init_array :
  {
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .);
  } > FLASH

  .fini_array :
  {
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .);
  } > FLASH

  . = ALIGN(4);
  __etext = .;

  .section_table :
  {
    . = ALIGN(4);
    __section_table_start = .;
    __data_section_table = .;
    LONG(LOADADDR(.data));
    LONG(ADDR(.data));
    LONG(SIZEOF(.data));
    LONG(LOADADDR(.noncacheable_init));
    LONG(ADDR(.noncacheable_init));
    LONG(SIZEOF(.noncacheable_init));
    LONG(LOADADDR(.quickaccess_data));
    LONG(ADDR(.quickaccess_data));
    LONG(SIZEOF(.quickaccess_data));
    LONG(LOADADDR(.smartdma));
    LONG(ADDR(.smartdma));
    LONG(SIZEOF(.smartdma));
    __data_section_table_end = .;

    __bss_section_table = .;
    LONG(ADDR(.bss));
    LONG(SIZEOF(.bss));
    LONG(ADDR(.noncacheable));
    LONG(SIZEOF(.noncacheable));
    __bss_section_table_end = .;
    __section_table_end = .;
  } > FLASH

  .data : AT(LOADADDR(.section_table) + SIZEOF(.section_table))
  {
    . = ALIGN(4);
    __data_start__ = .;
    *(.data*)
    *(.ramfunc*)
    *(.data.$RAM2*)
    . = ALIGN(4);
    __data_end__ = .;
  } > SRAM

  __data_load__ = LOADADDR(.data);

  .bss (NOLOAD) :
  {
    . = ALIGN(4);
    __bss_start__ = .;
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    __bss_end__ = .;
  } > SRAM

  .noncacheable_init : AT(LOADADDR(.data) + SIZEOF(.data))
  {
    . = ALIGN(4);
    __noncacheable_init_start__ = .;
    *(.NonCacheable.init)
    *(.NonCacheable.init*)
    *(.CacheLineData.init)
    *(.CacheLineData.init*)
    . = ALIGN(4);
    __noncacheable_init_end__ = .;
  } > SRAM

  __noncacheable_init_load__ = LOADADDR(.noncacheable_init);

  .noncacheable (NOLOAD) :
  {
    . = ALIGN(32);
    __noncacheable_start__ = .;
    *(.NonCacheable)
    *(.NonCacheable*)
    *(.bss.NonCacheable*)
    *(.CacheLineData)
    *(.CacheLineData*)
    . = ALIGN(32);
    __noncacheable_end__ = .;
  } > SRAM

  .quickaccess_data : AT(LOADADDR(.noncacheable_init) + SIZEOF(.noncacheable_init))
  {
    . = ALIGN(4);
    *(CodeQuickAccess)
    *(CodeQuickAccess*)
    *(DataQuickAccess)
    *(DataQuickAccess*)
    . = ALIGN(4);
  } > SRAM

  .usb_ram (NOLOAD) :
  {
    . = ALIGN(4);
    *(.usb_ram*)
    . = ALIGN(4);
  } > USB_RAM

  .smartdma : AT(LOADADDR(.quickaccess_data) + SIZEOF(.quickaccess_data))
  {
    . = ALIGN(4);
    __smartdma_start__ = .;
    KEEP(*(.smartdma_data))
    KEEP(*(.smartdma_code))
    . = ALIGN(4);
    __smartdma_end__ = .;
  } > RAM_EZH_CODE

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

  _image_size = LOADADDR(.smartdma) + SIZEOF(.smartdma) - ORIGIN(FLASH);

  /DISCARD/ :
  {
    *(.note.GNU-stack)
    *(.gnu_debuglink)
    *(.comment)
  }
}
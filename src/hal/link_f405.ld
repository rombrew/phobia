MEMORY
{
	FLASH (rx)	: ORIGIN = 0x08000000, LENGTH = 512K
	RAM1 (rwx)	: ORIGIN = 0x20000000, LENGTH = 112K
	RAM2 (rwx)	: ORIGIN = 0x2001C000, LENGTH = 16K
	CCM (rw)	: ORIGIN = 0x10000000, LENGTH = 64K
}

ld_stack = ORIGIN(CCM) + LENGTH(CCM);

ENTRY(irq_Reset);

SECTIONS
{
	.text : ALIGN(8)
	{
		ld_text_begin = . ;

		KEEP(*(.vtab))
		. = ALIGN(8);

		*(.text)
		*(.text.*)

		*(.rodata)
		*(.rodata.*)

		. = ALIGN(8);
		ld_text_end = . ;

	} > FLASH

	.ramfunc : ALIGN(8)
	{
		ld_ramfunc_load = LOADADDR(.ramfunc) ;
		ld_ramfunc_begin = . ;

		*(.ramfunc)
		*(.ramfunc.*)

		. = ALIGN(8);
		ld_ramfunc_end = . ;

	} > RAM1 AT > FLASH

	.data : ALIGN(8)
	{
		ld_data_load = LOADADDR(.data) ;
		ld_data_begin = . ;

		*(.data)
		*(.data.*)

		. = ALIGN(8);
		ld_data_end = . ;

	} > RAM1 AT > FLASH

	.endstub (NOLOAD) : ALIGN(8)
	{
		ld_crc32_stub = . ;

	} > FLASH

	.bss (NOLOAD) : ALIGN(8)
	{
		ld_bss_begin = . ;

		*(.bss)
		*(.bss.*)
		*(COMMON)

		. = ALIGN(8);
		ld_bss_end = . ;

	} > RAM1

	.noinit (NOLOAD) : ALIGN(8)
	{
		ld_noinit_begin = . ;

		*(.noinit)
		*(.noinit.*)

		. = ALIGN(8);
		ld_noinit_end = . ;

	} > RAM2

	.ccram (NOLOAD) : ALIGN(8)
	{
		ld_ccm_begin = . ;

		*(.ccram)
		*(.ccram.*)

		. = ALIGN(8);
		ld_ccm_end = . ;

	} > CCM

	/DISCARD/ : { *(.ARM.exidx*) }
}


MEMORY
{
	ITCM_FLASH (rx)	: ORIGIN = 0x00200000, LENGTH = 512K
	ITCM_RAM1 (rwx)	: ORIGIN = 0x00000000, LENGTH = 16K
	RAM1 (rwx)	: ORIGIN = 0x20010000, LENGTH = 176K
	RAM2 (rwx)	: ORIGIN = 0x2003C000, LENGTH = 16K
	DTCM_RAM1 (rw)	: ORIGIN = 0x20000000, LENGTH = 64K
}

ld_stack = ORIGIN(DTCM_RAM1) + LENGTH(DTCM_RAM1);

ENTRY(irq_Reset);

SECTIONS
{
	.text : ALIGN(8)
	{
		ld_text_begin = . ;

		KEEP(*(.vectors))
		. = ALIGN(8);

		*(.text)
		*(.text.*)

		*(.rodata)
		*(.rodata.*)

		. = ALIGN(8);
		ld_text_end = . ;

	} > ITCM_FLASH

	.ramfunc : ALIGN(8)
	{
		ld_ramfunc_load = LOADADDR(.ramfunc) ;
		ld_ramfunc_begin = . ;

		*(.ramfunc)
		*(.ramfunc.*)

		. = ALIGN(8);
		ld_ramfunc_end = . ;

	} > ITCM_RAM1 AT > ITCM_FLASH

	.data : ALIGN(8)
	{
		ld_data_load = LOADADDR(.data) ;
		ld_data_begin = . ;

		*(.data)
		*(.data.*)

		. = ALIGN(8);
		ld_data_end = . ;

	} > RAM1 AT > ITCM_FLASH

	.endstub (NOLOAD) : ALIGN(8)
	{
		ld_end = . ;

	} > ITCM_FLASH

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

	} > DTCM_RAM1

	/DISCARD/ : { *(.ARM.exidx*) }
}


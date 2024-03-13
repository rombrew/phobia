MEMORY
{
	FLASH (rx)	: ORIGIN = 0x08000000, LENGTH = 512K
	RAM1 (rwx)	: ORIGIN = 0x20010000, LENGTH = 176K
	RAM2 (rwx)	: ORIGIN = 0x2003C000, LENGTH = 16K
	ITCM_RAM1 (rwx)	: ORIGIN = 0x00000000, LENGTH = 16K
	ITCM_FLASH (rx)	: ORIGIN = 0x00200000, LENGTH = 512K
	DTCM_RAM1 (rw)	: ORIGIN = 0x20000000, LENGTH = 64K
}

ld_stack = ORIGIN(DTCM_RAM1) + LENGTH(DTCM_RAM1);

ENTRY(irq_Reset);

SECTIONS
{
	.text : ALIGN(8)
	{
		ld_begin_text = . ;

		KEEP(*(.vectors))
		. = ALIGN(8);

		*(.text)
		*(.text.*)

		*(.rodata)
		*(.rodata.*)

		. = ALIGN(8);
		ld_end_text = . ;
		ld_end = ld_end_text + SIZEOF(.data) ;

	} > ITCM_FLASH

	.data : AT (ld_end_text) ALIGN(8)
	{
		ld_begin_data = . ;

		*(.ramfunc)
		*(.ramfunc.*)

		*(.data)
		*(.data.*)

		. = ALIGN(8);
		ld_end_data = . ;

	} > RAM1

	.bss (NOLOAD) : ALIGN(8)
	{
		ld_begin_bss = . ;

		*(.bss)
		*(.bss.*)
		*(COMMON)

		. = ALIGN(8);
		ld_end_bss = . ;

	} > RAM1

	.noinit (NOLOAD) : ALIGN(8)
	{
		ld_begin_noinit = . ;

		*(.noinit)
		*(.noinit.*)

		. = ALIGN(8);
		ld_end_noinit = . ;

	} > RAM2

	.ccram (NOLOAD) : ALIGN(8)
	{
		ld_begin_ccm = . ;

		*(.ccram)
		*(.ccram.*)

		. = ALIGN(8);
		ld_end_ccm = . ;

	} > DTCM_RAM1

	/DISCARD/ : { *(.ARM.exidx*) }
}


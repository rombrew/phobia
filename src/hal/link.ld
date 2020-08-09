MEMORY
{
	FLASH (rx)	: ORIGIN = 0x08000000, LENGTH = 512K
	RAM (rwx)	: ORIGIN = 0x20000000, LENGTH = 128K
	CCM (rw)	: ORIGIN = 0x10000000, LENGTH = 64K
}

ld_stack = ORIGIN(CCM) + LENGTH(CCM);

ENTRY(irq_Reset);

SECTIONS
{
	.text : ALIGN(4)
	{
		ld_begin_vectors = . ;

		KEEP(*(.vectors))
		. = ALIGN(4);

		*(.text)
		*(.text.*)

		*(.rodata)
		*(.rodata.*)

		. = ALIGN(4);
		ld_end_text = . ;
		ld_end_flash = . + SIZEOF(.data) ;

	} > FLASH

	.data : AT (ld_end_text) ALIGN(4)
	{
		ld_begin_data = . ;

		*(.ramfunc)
		*(.ramfunc.*)

		*(.data)
		*(.data.*)

		. = ALIGN(4);
		ld_end_data = . ;

	} > RAM

	.bss (NOLOAD) : ALIGN(4)
	{
		ld_begin_bss = . ;

		*(.bss)
		*(.bss.*)
		*(COMMON)

		. = ALIGN(4);
		ld_end_bss = . ;

	} > RAM

	.noinit (NOLOAD) : ALIGN(4)
	{
		ld_begin_noinit = . ;

		*(.noinit)
		*(.noinit.*)

		. = ALIGN(4);
		ld_end_noinit = . ;

	} > RAM

	.ccmram (NOLOAD) : ALIGN(4)
	{
		ld_begin_ccm = . ;

		*(.ccmram)
		*(.ccmram.*)

		. = ALIGN(4);
		ld_end_ccm = . ;

	} > CCM

	/DISCARD/ : { *(.ARM.exidx*) }
}


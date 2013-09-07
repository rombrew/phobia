
MEMORY
{
	RAM (rwx)	: ORIGIN = 0x20000000, LENGTH = 128K
	CCM (rw)	: ORIGIN = 0x10000000, LENGTH = 64K
	FLASH (rx)	: ORIGIN = 0x08000000, LENGTH = 1024K
}

ld_stack = 0x10000000 + 64K;

SECTIONS
{
	.text : ALIGN(4)
	{
		ld_svectors = . ;

		KEEP(*(.vectors))

		. = ALIGN(4);

		*(.text)
		*(.text.*)
		*(.rodata)
		*(.rodata.*)
 
		. = ALIGN(4);
		ld_etext = . ;

	} > FLASH

	.data : AT (ld_etext) ALIGN(4)
	{
		ld_sdata = . ;

		*(.data)
		*(.data.*)

		. = ALIGN(4);
		ld_edata = . ;

	} > CCM

	.bss : ALIGN(4)
	{
		ld_sbss = . ;

		*(.bss)
		*(.bss.*)
		*(COMMON)

		. = ALIGN(4);
		ld_ebss = . ;

	} > CCM
}


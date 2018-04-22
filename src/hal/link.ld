
MEMORY
{
	RAM (rwx)	: ORIGIN = 0x20000000, LENGTH = 128K
	CCM (rw)	: ORIGIN = 0x10000000, LENGTH = 64K
	FLASH (rx)	: ORIGIN = 0x08000000, LENGTH = 1024K
}

ld_Stack = 0x10000000 + 64K;

SECTIONS
{
	.text : ALIGN(4)
	{
		ld_Svectors = . ;

		KEEP(*(.vectors))
		. = ALIGN(4);

		*(.text)
		*(.text.*)
		*(.rodata)
		*(.rodata.*)

		. = ALIGN(4);
		ld_Etext = . ;

	} > FLASH

	.data : AT (ld_Etext) ALIGN(4)
	{
		ld_Sdata = . ;

		*(.data)
		*(.data.*)

		. = ALIGN(4);
		ld_Edata = . ;

	} > RAM

	.bss : ALIGN(4)
	{
		ld_Sbss = . ;

		*(.bss)
		*(.bss.*)
		*(COMMON)

		. = ALIGN(4);
		ld_Ebss = . ;

	} > RAM

	.ccm : ALIGN(4)
	{
		ld_Sccm = . ;

		*(.ccm)
		*(.ccm.*)

		. = ALIGN(4);
		ld_Eccm = . ;

	} > CCM
}


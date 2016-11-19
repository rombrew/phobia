
MEMORY
{
	RAM (rwx)	: ORIGIN = 0x20000000, LENGTH = 128K
	CCM (rw)	: ORIGIN = 0x10000000, LENGTH = 64K
	FLASH (rx)	: ORIGIN = 0x08000000, LENGTH = 1024K
}

ldStack = 0x10000000 + 64K;

SECTIONS
{
	.text : ALIGN(4)
	{
		ldSvectors = . ;

		KEEP(*(.vectors))
		. = ALIGN(4);

		*(.text)
		*(.text.*)
		*(.rodata)
		*(.rodata.*)

		. = ALIGN(4);
		ldEtext = . ;

	} > FLASH

	.data : AT (ldEtext) ALIGN(4)
	{
		ldSdata = . ;

		*(.data)
		*(.data.*)

		. = ALIGN(4);
		ldEdata = . ;

	} > RAM

	.bss : ALIGN(4)
	{
		ldSbss = . ;

		*(.bss)
		*(.bss.*)
		*(COMMON)

		. = ALIGN(4);
		ldEbss = . ;

	} > RAM

	.ccm : ALIGN(4)
	{
		ldSccm = . ;

		*(.ccm)
		*(.ccm.*)

		. = ALIGN(4);
		ldEccm = . ;

	} > CCM
}


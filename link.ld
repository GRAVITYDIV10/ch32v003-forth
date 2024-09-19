ENTRY(_start)

MEMORY
{
	ROM (rx) : ORIGIN = 0x00000000, LENGTH = 16K 
	RAM (rwx): ORIGIN = 0x20000000, LENGTH = 2K
}

SECTIONS
{
	.text :
	{
		*(.text)
		*(.text*)
		*(.rodata)
		*(.rodata*)
		. = ALIGN(4);
		
	} >ROM AT>ROM

	.bss (NOLOAD) :
	{
		. = ALIGN(4);
		*(.bss)
		. = ALIGN(4);
		PROVIDE(_ram_dict_start = .);
		. = ALIGN(4);
	} >RAM AT>ROM
}

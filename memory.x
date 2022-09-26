MEMORY {
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x500
    FLASH : ORIGIN = 0x10000500, LENGTH = 2048K - 0x500
    RAM   : ORIGIN = 0x20000000, LENGTH = 256K
}

SECTIONS {
    /* ### Boot loader */
    .boot2 ORIGIN(BOOT2) :
    {
        KEEP(*(.boot2));
    } > BOOT2
} INSERT BEFORE .text;

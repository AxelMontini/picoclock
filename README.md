# Spi issues

Issues in rp-hal's SPI implementation:

## Blocks

`spi.write()` blocks after a word: [code](https://docs.rs/embedded-hal/0.2.6/src/embedded_hal/blocking/spi.rs.html#71).
This inserts a pause in the output, making the last written bit persist longer.

## Motorola format

Motorola SPI format inserts a bit after a word or something.
Setting FRF register to `0b01` sets the format to TI, in which all bits are
contiguous. 


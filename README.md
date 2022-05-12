[![Rp Pico build thumbv6m](https://github.com/AxelMontini/picoclock/actions/workflows/rust.yml/badge.svg)](https://github.com/AxelMontini/picoclock/actions/workflows/rust.yml)

# Picoclock

A scuffed wall clock displaying stuff on a 32x16 led matrix (ws2812b). Also has an LCD display to display things like settings.

## Implementation

- Uses a PIO to control the LEDs (input queue receives 24bit colors and sends them to the LEDS with the appropriate protocol).
- Uses RTIC for concurrency (recurrent update/render cycle, input debouncing, ...); Has drawbacks.

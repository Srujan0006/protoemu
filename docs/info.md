<!---
This file is used to generate your project datasheet. Please fill in the sections below:
-->

## How it works

The DTC is a reprogrammable protocol emulator. It has three parts.

- **PIO-style core:** two state machines that run 16-bit pin-control instructions from a shared 64-word instruction memory. Every instruction takes exactly one tick plus its delay field, and each machine has its own clock divider, 4-word TX/RX FIFOs and shift registers. The instruction set follows the RP2040 PIO, with extensions: a page-local JMP plus a far jump (LJMP), a masked pattern WAIT, and a non-blocking pattern jump (PJMP).
- **Timestamp Capture Unit:** logs edges on masked pins into a 16-entry FIFO, each entry holding a 32-bit cycle timestamp and the 8 pin values. It works as a small logic analyzer.
- **Autonomous Trigger Unit:** waits for an edge-qualified pin pattern, then a programmable delay, then forces chosen pins for a programmable time. Its override beats both state machines.

A host MCU programs everything through a serial config port (SPI target, mode 0). After every select, reload the program.

## How to test

The host talks to the config port: CS_N, SCK and MOSI on `ui[0..2]`, MISO on `uo[0]`. Keep SCK at or below clk/8 (5.25 MHz at the 42 MHz design clock).

- **Header:** 16 bits, MSB first: RW[15] (1 = read), AINC[14] (auto-increment address), reserved[13:8], addr[7:0].
- **Write:** after the header, send 32-bit words.
- **Read:** after the header, each word takes 8 turnaround clocks and then 32 data bits on MISO.
- **Errors:** a partial header or a partial write word sets STATUS.FRAME_ERR (bit 24).

Quick check:

1. Read address 0x00. The ID is `0xD7C0_0201` (NUM_SM = 2, version 1).
2. Write a program to IMEM (0x80 to 0xBF), configure machine 0 (CLKDIV 0x04, EXECCTRL 0x05, SHIFTCTRL 0x06, PINCTRL 0x07), push data to TXF (0x08), then set CTRL bit 0 (0x01) to start machine 0.

`test/test.py` does exactly this with a UART transmitter program and checks the 8N1 waveform on `uio[0]`.

## External hardware

A host MCU (for example the RP2040 on the demo board) on the config-port pins. Protocol peripherals connect to `uio[7:0]`, for example on a Pmod.

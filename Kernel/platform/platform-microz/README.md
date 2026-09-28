# Fuzix for Bill Shen's MicroZ

The MicroZ is the follow up to the Micro80. It uses the Z84C15 rather
differently in order to get a more friendly memory map. Fuzix for this
machine is designed to run from flash as this allows two processes in main
memory plus the kernel mostly in ROM.

## Supported Hardware

- MicroZ
- Onboard CF adapter
- Onboard SIO

## Memory Mapping

The hardware is wired so that the ROM is enably by CS0 low but unusually the
second CS line is wired to A16 on the 128K RAM. This allows the use of the
CSBR and MCR registers to place either RAM bank in the low or high area or
to map ROM space.

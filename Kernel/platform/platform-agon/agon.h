#ifndef __AGON_H
#define __AGON_H

#define TMR0_CTL	0x80

#define UART0_THR	0xC0
#define UART0_RBR	0xC0
#define UART0_LSR	0xC5
#define UART0_MSR	0xC6

extern void spi_select_port(uint8_t n);
#define spi_select_none()	spi_select_port(0)

#define spi_send	sd_spi_tx_byte
#define spi_recv	sd_spi_rx_byte

#endif

#ifndef __LCD_H
#define __LCD_H

#include <stdint.h>
#include "types.h"

int lcd_open(uint_fast8_t minor, uint16_t flag);
int lcd_close(uint_fast8_t minor);
int lcd_read(uint_fast8_t minor, uint_fast8_t rawflag, uint_fast8_t flag);
int lcd_write(uint_fast8_t minor, uint_fast8_t rawflag, uint_fast8_t flag);
int lcd_ioctl(uint_fast8_t minor, uarg_t request, char *data);

void lcddev_init(void);

#endif /* __LCD_H */

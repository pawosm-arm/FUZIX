#include <kernel.h>
#include <kdata.h>
#include <stdint.h>
#include "types.h"
#include "config.h"
#include "tm4c129x.h"
#include "gpio.h"

#include "lcd.h"

#define LCD_WIDTH  320U
#define LCD_HEIGHT 240U

/* When LCD_CLK_DIV is 0, pixel clock equals system clock */
#define LCD_CLK_DIV 0U
#define LCD_CLK_SHIFT 8U
#define LCD_CLK_CLEAR_BITS (0b11111111U << (LCD_CLK_SHIFT))

#define CYCLES_FROM_NS(ns) \
  ((!(ns)) ? 0U : ((((((SYS_CLOCK) / 1000000U) * ((ns) - 1U)) / 1000U)) + 1U))

#define BITS_SHIFT(val, bits, shift) (((val) & (bits)) << (shift))

#define SYSCON_RCGCLCD  0x400fe690

#define LCD_CLKEN       0x4405006c
#define LCD_CTL         0x44050004
#define LCD_DMACTL      0x44050040
#define LCD_LIDDCTL     0x4405000c
#define LCD_LIDDCS0CFG  0x44050010
#define LCD_LIDDCS0ADDR 0x44050014
#define LCD_LIDDCS0DATA 0x44050018
#define LCD_LIDDCS1CFG  0x4405001c
#define LCD_LIDDCS1ADDR 0x44050020
#define LCD_LIDDCS1DATA 0x44050024

#define LCD_CLKEN_CLEAR_BITS 0b1111U
#define LCD_CLKEN_CORE  1U
#define LCD_CLKEN_LIDD  2U
#define LCD_CLKEN_DMA   4U
#define LCD_CLKEN_MAIN  8U

#define LCD_MODE_CLEAR_BITS 0b1U
#define LCD_MODE_LIDD   0U
#define LCD_MODE_RASTER 1U

#define LCD_DMA_CLEAR_BITS 0b11101111011U
#define LCD_DMA_FIFORDY_8WORDS       0U
#define LCD_DMA_FIFORDY_16WORDS    256U
#define LCD_DMA_FIFORDY_32WORDS    512U
#define LCD_DMA_FIFORDY_64WORDS    768U
#define LCD_DMA_FIFORDY_128WORDS  1024U
#define LCD_DMA_FIFORDY_256WORDS  1280U
#define LCD_DMA_FIFORDY_512WORDS  1536U
#define LCD_DMA_BURST1              16U
#define LCD_DMA_BURST2              16U
#define LCD_DMA_BURST4              32U
#define LCD_DMA_BURST8              48U
#define LCD_DMA_BURST16             64U
#define LCD_DMA_BYTE_ORDER_0123      0U
#define LCD_DMA_BYTE_ORDER_1023      8U
#define LCD_DMA_BYTE_ORDER_3210      2U
#define LCD_DMA_BYTE_ORDER_2301     10U
#define LCD_DMA_PING_PONG            1U

#define LCD_LIDD_CONFIG_CLEAR_BITS 0b1111111111U
#define LCD_LIDD_CONFIG_SYNC_MPU68        0U
#define LCD_LIDD_CONFIG_ASYNC_MPU68       1U
#define LCD_LIDD_CONFIG_SYNC_MPU80        2U
#define LCD_LIDD_CONFIG_ASYNC_MPU80       3U
#define LCD_LIDD_CONFIG_ASYNC_HITACHI     4U
#define LCD_LIDD_CONFIG_INVERT_ALE        8U
#define LCD_LIDD_CONFIG_INVERT_RS_EN     16U
#define LCD_LIDD_CONFIG_INVERT_WS_DIR    32U
#define LCD_LIDD_CONFIG_INVERT_CS0       64U
#define LCD_LIDD_CONFIG_INVERT_CS1      128U

#define LCD_LIDD_CS0CFG_CLEAR_BITS 0xffffffffU
#define LCD_LIDD_CS0CFG_WRSU_BITS    0b11111U
#define LCD_LIDD_CS0CFG_WRSU_SHIFT        27U
#define LCD_LIDD_CS0CFG_WRDUR_BITS  0b111111U
#define LCD_LIDD_CS0CFG_WRDUR_SHIFT       21U
#define LCD_LIDD_CS0CFG_WRHOLD_BITS   0b1111U
#define LCD_LIDD_CS0CFG_WRHOLD_SHIFT      17U
#define LCD_LIDD_CS0CFG_RDSU_BITS    0b11111U
#define LCD_LIDD_CS0CFG_RDSU_SHIFT        12U
#define LCD_LIDD_CS0CFG_RDDUR_BITS  0b111111U
#define LCD_LIDD_CS0CFG_RDDUR_SHIFT        6U
#define LCD_LIDD_CS0CFG_RDHOLD_BITS   0b1111U
#define LCD_LIDD_CS0CFG_RDHOLD_SHIFT       2U
#define LCD_LIDD_CS0CFG_GAP_BITS        0b11U
#define LCD_LIDD_CS0CFG_GAP_SHIFT          0U

#define LCD_LIDD_CS0ADDR_CLEAR_BITS 0xffffU
#define LCD_LIDD_CS0DATA_CLEAR_BITS 0xffffU

#define LCD_SSD2119_DEVICE_CODE_READ_REG   0U
#define LCD_SSD2119_OSC_START_REG          0U
#define LCD_SSD2119_OUTPUT_CTRL_REG        1U
#define LCD_SSD2119_LCD_DRIVE_AC_CTRL_REG  2U
#define LCD_SSD2119_PWR_CTRL_1_REG         3U
#define LCD_SSD2119_DISPLAY_CTRL_REG       7U
#define LCD_SSD2119_FRAME_CYCLE_CTRL_REG  11U
#define LCD_SSD2119_PWR_CTRL_2_REG        12U
#define LCD_SSD2119_PWR_CTRL_3_REG        13U
#define LCD_SSD2119_PWR_CTRL_4_REG        14U
#define LCD_SSD2119_GATE_SCAN_START_REG   15U
#define LCD_SSD2119_SLEEP_MODE_1_REG      16U
#define LCD_SSD2119_ENTRY_MODE_REG        17U
#define LCD_SSD2119_SLEEP_MODE_2_REG      18U
#define LCD_SSD2119_GEN_IF_CTRL_REG       21U
#define LCD_SSD2119_PWR_CTRL_5_REG        30U
#define LCD_SSD2119_RAM_DATA_REG          34U
#define LCD_SSD2119_FRAME_FREQ_REG        37U
#define LCD_SSD2119_ANALOG_SET_REG        38U
#define LCD_SSD2119_VCOM_OTP_1_REG        40U
#define LCD_SSD2119_VCOM_OTP_2_REG        41U
#define LCD_SSD2119_GAMMA_CTRL_1_REG      48U
#define LCD_SSD2119_GAMMA_CTRL_2_REG      49U
#define LCD_SSD2119_GAMMA_CTRL_3_REG      50U
#define LCD_SSD2119_GAMMA_CTRL_4_REG      51U
#define LCD_SSD2119_GAMMA_CTRL_5_REG      52U
#define LCD_SSD2119_GAMMA_CTRL_6_REG      53U
#define LCD_SSD2119_GAMMA_CTRL_7_REG      54U
#define LCD_SSD2119_GAMMA_CTRL_8_REG      55U
#define LCD_SSD2119_GAMMA_CTRL_9_REG      58U
#define LCD_SSD2119_GAMMA_CTRL_10_REG     59U
#define LCD_SSD2119_V_RAM_POS_REG         68U
#define LCD_SSD2119_H_RAM_START_REG       69U
#define LCD_SSD2119_H_RAM_END_REG         70U
#define LCD_SSD2119_X_RAM_ADDR_REG        78U
#define LCD_SSD2119_Y_RAM_ADDR_REG        79U

#ifndef CONFIG_EK
static __attribute__((noinline)) void lcd_mdelay(unsigned ms)
{
  volatile unsigned i, j;

  for (i = 0U; i < ms; i++) {
    for (j = 0U; j < BOARD_LOOPSPERMSEC; j++)
      asm volatile("");
  }
}
#endif

int lcd_open(uint_fast8_t minor, uint16_t flag)
{
  if (minor) {
    udata.u_error = ENXIO;
    return -1;
  }
  return 0;
}

int lcd_close(uint_fast8_t minor)
{
  if (minor) {
    udata.u_error = ENXIO;
    return -1;
  }
  return 0;
}

int lcd_read(uint_fast8_t minor, uint_fast8_t rawflag, uint_fast8_t flag)
{
#ifndef CONFIG_EK
  // TODO: ensure that it delays at least 450ns or more between each read or the read timings will be violated
  return 0; // TODO: implement reading the touchscreen coordinates
#else
  return 0;
#endif
}

int lcd_write(uint_fast8_t minor, uint_fast8_t rawflag, uint_fast8_t flag)
{
#ifndef CONFIG_EK
  return 0; // TODO: implement raw write to the device
#else
  return 0;
#endif
}

int lcd_ioctl(uint_fast8_t minor, uarg_t request, char *data)
{
#ifndef CONFIG_EK
  return 0; // TODO: implement draw primitives: fill rectangle, draw sprite
#else
  return 0;
#endif
}

#ifndef CONFIG_EK
static void lcddev_liddcs0_cmd_hilo(uint32_t addr, uint32_t hi, uint32_t lo)
{
  uint32_t regval, dataval_lo, dataval_hi;

  regval = inl(LCD_LIDDCS0DATA);
  dataval_hi = regval;
  dataval_lo = regval;
  regval = inl(LCD_LIDDCS0ADDR);
  regval &= ~LCD_LIDD_CS0ADDR_CLEAR_BITS;
  regval |= addr;
  dataval_hi &= ~LCD_LIDD_CS0DATA_CLEAR_BITS;
  dataval_hi |= hi;
  dataval_lo &= ~LCD_LIDD_CS0DATA_CLEAR_BITS;
  dataval_lo |= lo;
  outl(LCD_LIDDCS0ADDR, regval);
  outl(LCD_LIDDCS0DATA, dataval_hi);
  outl(LCD_LIDDCS0DATA, dataval_lo);
}
#endif

#ifndef CONFIG_EK
static void lcddev_liddcs0_cmd(uint32_t addr, uint32_t data)
{
  uint32_t regval, dataval_lo, dataval_hi;

  regval = inl(LCD_LIDDCS0DATA);
  dataval_hi = regval;
  dataval_lo = regval;
  regval = inl(LCD_LIDDCS0ADDR);
  regval &= ~LCD_LIDD_CS0ADDR_CLEAR_BITS;
  regval |= addr;
  dataval_hi &= ~LCD_LIDD_CS0DATA_CLEAR_BITS;
  dataval_hi |= ((data >> 8U) & 0xffU);
  dataval_lo &= ~LCD_LIDD_CS0DATA_CLEAR_BITS;
  dataval_lo |= (data & 0xffU);
  outl(LCD_LIDDCS0ADDR, regval);
  outl(LCD_LIDDCS0DATA, dataval_hi);
  outl(LCD_LIDDCS0DATA, dataval_lo);
}
#endif

void lcddev_init(void)
{
#ifndef CONFIG_EK
  unsigned u;
  uint32_t regval, dataval;

  regval = inl(SYSCON_RCGCLCD);
  regval |= 1U;
  outl(SYSCON_RCGCLCD, regval);
  gpio_write(GPIO_PORT('F'), 6, 0);
  lcd_mdelay(128U);
  gpio_write(GPIO_PORT('F'), 6, 1);
  lcd_mdelay(128U);
  regval = inl(LCD_CLKEN);
  regval &= ~LCD_CLKEN_CLEAR_BITS;
  regval |= (LCD_CLKEN_DMA | LCD_CLKEN_CORE | LCD_CLKEN_LIDD);
  outl(LCD_CLKEN, regval);
  regval = inl(LCD_CTL);
  regval &= ~(LCD_MODE_CLEAR_BITS | LCD_CLK_CLEAR_BITS);
  regval |= (LCD_MODE_LIDD | (LCD_CLK_DIV << LCD_CLK_SHIFT));
  outl(LCD_CTL, regval);
  regval = inl(LCD_DMACTL);
  regval &= ~LCD_DMA_CLEAR_BITS;
  regval |= LCD_DMA_BURST4;
  outl(LCD_DMACTL, regval);
  regval = inl(LCD_LIDDCTL);
  regval &= ~LCD_LIDD_CONFIG_CLEAR_BITS;
  regval |= LCD_LIDD_CONFIG_ASYNC_MPU80;
  outl(LCD_LIDDCTL, regval);
  regval = inl(LCD_LIDDCS0CFG);
  regval &= ~LCD_LIDD_CS0CFG_CLEAR_BITS;
  regval |= (BITS_SHIFT(CYCLES_FROM_NS(5U), LCD_LIDD_CS0CFG_WRSU_BITS,
                        LCD_LIDD_CS0CFG_WRSU_SHIFT) |
             BITS_SHIFT(CYCLES_FROM_NS(40U), LCD_LIDD_CS0CFG_WRDUR_BITS,
                        LCD_LIDD_CS0CFG_WRDUR_SHIFT) |
             BITS_SHIFT(CYCLES_FROM_NS(5U), LCD_LIDD_CS0CFG_WRHOLD_BITS,
                        LCD_LIDD_CS0CFG_WRHOLD_SHIFT) |
             BITS_SHIFT(CYCLES_FROM_NS(0U), LCD_LIDD_CS0CFG_RDSU_BITS,
                        LCD_LIDD_CS0CFG_RDSU_SHIFT) |
             BITS_SHIFT(CYCLES_FROM_NS(500U), LCD_LIDD_CS0CFG_RDDUR_BITS,
                        LCD_LIDD_CS0CFG_RDDUR_SHIFT) |
             BITS_SHIFT(CYCLES_FROM_NS(100U), LCD_LIDD_CS0CFG_RDHOLD_BITS,
                        LCD_LIDD_CS0CFG_RDHOLD_SHIFT) |
             BITS_SHIFT(CYCLES_FROM_NS(50U), LCD_LIDD_CS0CFG_GAP_BITS,
                        LCD_LIDD_CS0CFG_GAP_SHIFT));
  outl(LCD_LIDDCS0CFG, regval);
  lcddev_liddcs0_cmd_hilo(LCD_SSD2119_SLEEP_MODE_1_REG, 0U, 1U);
  lcddev_liddcs0_cmd_hilo(LCD_SSD2119_PWR_CTRL_5_REG, 0U, 178U);
  lcddev_liddcs0_cmd_hilo(LCD_SSD2119_VCOM_OTP_1_REG, 0U, 6U);
  lcddev_liddcs0_cmd_hilo(LCD_SSD2119_OSC_START_REG, 0U, 1U);
  lcddev_liddcs0_cmd_hilo(LCD_SSD2119_OUTPUT_CTRL_REG, 48U, 239U);
  lcddev_liddcs0_cmd_hilo(LCD_SSD2119_LCD_DRIVE_AC_CTRL_REG, 6U, 0U);
  lcddev_liddcs0_cmd_hilo(LCD_SSD2119_SLEEP_MODE_1_REG, 0U, 0U);
  lcd_mdelay(64U);
  lcddev_liddcs0_cmd_hilo(LCD_SSD2119_ENTRY_MODE_REG, 104U, 48U);
  lcddev_liddcs0_cmd_hilo(LCD_SSD2119_SLEEP_MODE_2_REG, 9U, 153U);
  lcddev_liddcs0_cmd_hilo(LCD_SSD2119_ANALOG_SET_REG, 56U, 0U);
  lcddev_liddcs0_cmd_hilo(LCD_SSD2119_DISPLAY_CTRL_REG, 0U, 51U);
  lcddev_liddcs0_cmd_hilo(LCD_SSD2119_PWR_CTRL_2_REG, 0U, 5U); /* VCIX2 6.1V */
  lcddev_liddcs0_cmd_hilo(LCD_SSD2119_GAMMA_CTRL_1_REG, 0U, 0U);
  lcddev_liddcs0_cmd_hilo(LCD_SSD2119_GAMMA_CTRL_2_REG, 3U, 3U);
  lcddev_liddcs0_cmd_hilo(LCD_SSD2119_GAMMA_CTRL_3_REG, 4U, 7U);
  lcddev_liddcs0_cmd_hilo(LCD_SSD2119_GAMMA_CTRL_4_REG, 3U, 1U);
  lcddev_liddcs0_cmd_hilo(LCD_SSD2119_GAMMA_CTRL_5_REG, 3U, 1U);
  lcddev_liddcs0_cmd_hilo(LCD_SSD2119_GAMMA_CTRL_6_REG, 4U, 3U);
  lcddev_liddcs0_cmd_hilo(LCD_SSD2119_GAMMA_CTRL_7_REG, 7U, 7U);
  lcddev_liddcs0_cmd_hilo(LCD_SSD2119_GAMMA_CTRL_8_REG, 4U, 0U);
  lcddev_liddcs0_cmd_hilo(LCD_SSD2119_GAMMA_CTRL_9_REG, 10U, 0U);
  lcddev_liddcs0_cmd_hilo(LCD_SSD2119_GAMMA_CTRL_10_REG, 16U, 0U);
  lcddev_liddcs0_cmd_hilo(LCD_SSD2119_PWR_CTRL_3_REG, 0U, 10U);
  lcddev_liddcs0_cmd_hilo(LCD_SSD2119_PWR_CTRL_4_REG, 46U, 0U);
  lcddev_liddcs0_cmd(LCD_SSD2119_V_RAM_POS_REG, ((LCD_HEIGHT - 1U) << 8U));
  lcddev_liddcs0_cmd(LCD_SSD2119_H_RAM_START_REG, 0U);
  lcddev_liddcs0_cmd(LCD_SSD2119_H_RAM_END_REG, (LCD_WIDTH - 1U));
  lcddev_liddcs0_cmd(LCD_SSD2119_X_RAM_ADDR_REG, 0U);
  lcddev_liddcs0_cmd(LCD_SSD2119_Y_RAM_ADDR_REG, 0U);
  /* clear screen: */
  dataval = inl(LCD_LIDDCS0DATA);
  dataval &= ~LCD_LIDD_CS0DATA_CLEAR_BITS;
  regval = inl(LCD_LIDDCS0ADDR);
  regval &= ~LCD_LIDD_CS0ADDR_CLEAR_BITS;
  regval |= LCD_SSD2119_RAM_DATA_REG;
  outl(LCD_LIDDCS0ADDR, regval);
  for (u = 0U; u < (LCD_WIDTH * LCD_HEIGHT); u++) {
    // TODO: draw some pattern for now, to see anything, replace hi and lo with zeros when it works
    regval = dataval | 0xabU; /* hi */
    outl(LCD_LIDDCS0DATA, regval);
    regval = dataval | 0xcdU; /* lo */
    outl(LCD_LIDDCS0DATA, regval);
  }
#endif
}

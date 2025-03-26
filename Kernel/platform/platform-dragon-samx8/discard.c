#include <kernel.h>
#include <timer.h>
#include <kdata.h>
#include <printf.h>
#include <device.h>
#include <devtty.h>
#include <blkdev.h>

#define DISC __attribute__((section(".discard")))

/*
 * Map handling: We have flexible paging. Each map table consists of a set of
 * pages with the last page repeated to fill any holes.
 */

void tty_detect(void);

extern uint16_t framedet;
extern uint8_t sys_hz;

DISC void plt_copyright(void)
{
	kprintf("SAMx8 2025 Ciaran Anscomb\n");
}

DISC void pagemap_init(void)
{
	int i;

	/* Kernel: 0-3
	 * Video: 31 (bottom half)
	 * Common: 31 (top half)
	 */
	for (i = 30; i >= 4; i--)
		pagemap_add(i);
}

DISC uint8_t plt_param(char *p)
{
	if (strcmp(p, "over") == 0 || strcmp(p, "overclock") == 0) {
		*((volatile uint8_t *)0xffd7) = 0;
		return 1;
	}
	return 0;
}

DISC void map_init(void)
{
	if (framedet >= 0x0500)
		sys_hz = 5;
	else
		sys_hz = 6;

	tty_detect();
}

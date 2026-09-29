/* GIDE at 0x38 */

#define data	0x38
#define error	0x39
#define count	0x3A
#define sec	0x3B
#define cyll	0x3C
#define cylh	0x3D
#define devh	0x3E
#define cmd	0x3F
#define status	0x3F

#define IDE_REG_DATA	0x38

#define ide_read(x)	in(x)
#define ide_write(x,y)	out(x,y)

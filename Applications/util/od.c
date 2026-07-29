/* od - octal dump		   Author: Andy Tanenbaum */
/* Adapted to UZI180 by H. Peraza                         */

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <limits.h>
#include <string.h>

#define RADIX_HEX   16
#define RADIX_DEC   10
#define RADIX_OCT   8
#define RADIX_ASC   1
#define RADIX_NONE  0

//int  bflag, cflag, dflag, oflag, xflag, hflag, vflag;
int  vflag;
int  addr_radix = RADIX_OCT;
int  data_radix = RADIX_OCT;
int  word_length = 2;
int  hd;
int  linenr, width, state, ever;
int  prevwds[8];
long off;
char buf[512], buffer[BUFSIZ];
int  next;
int  bytespresent;


int  main(int argc, char *argv[]);
long offset(int argc, char *argv[], int k);
void dumpfile(void);
void wdump(short *words, int k, int radix);
void bdump(char bytes[16], int k, int c);
void byte(int val, int c);
int  getwords(short **words);
int  same(short *w1, int *w2);
void outword(int val, int radix);
void outnum(int num, int radix);
void addrout(long l);
char hexit(int k);
void usage(void);



long offset(int argc, char *argv[], int k)
{
    int dot = 0, bad = 0;
    char *p, *endp;
    long val;
    long mult;

    /* See if the offset is octal with a dot. */
    p = argv[k];
    while (*p)
        if (*p++ == '.') dot = 1;

    /* Convert offset to binary. */
    p = argv[k];
    if (dot) {
        val = strtol(p, &endp, 8);
        if (val < 0 || val == LONG_MAX || endp == p || *endp++ != '.')
            bad = 1;
    } else {
        val = strtol(p, &endp, 0);
        if (val < 0 || val == LONG_MAX || endp == p)
            bad = 1;
    }

    if (bad) {
        printf("Bad offset: %s\n", p);
        exit(1);
    }

    /* Look for multiplier */
    p = endp;
    if (!*p && k + 1 == argc - 1)
            p = argv[k + 1];

    if (!*p)
        return val;

    if (*p == 'b')
        mult = 512L;
    else if (*p == 'K') {
        if (p[1] == 'B') {
            mult = 1000L;
            p++;
        } else
            mult = 1024L;
    } else if (*p == 'M') {
        if (p[1] == 'B') {
            mult = 1000L * 1000L;
            p++;
        } else
            mult = 1024L * 1024L;
    } else
        mult = 0;

    if (mult == 0 || p[1] != '\0') {
        printf("Bad offset multiplier: %s\n", p);
        exit(1);
    }

    return mult * val;
}


void dumpfile(void)
{
    int k;
    short *words;

    while ((k = getwords(&words))) {	/* 'k' is # bytes read */
	if (!vflag) {		/* ensure 'lazy' evaluation */
	    if (k == 16 && ever == 1 && same(words, prevwds)) {
		if (state == 0) {
		    printf("*\n");
		    state = 1;
		    off += 16L;
		    continue;
		} else if (state == 1) {
		    off += 16L;
		    continue;
		}
	    }
	}
	addrout(off);
	off += (long) k;
	state = 0;
	ever = 1;
	linenr = 1;
	//if (oflag) wdump(words, k, 8);
	//if (dflag) wdump(words, k, 10);
	//if (xflag) wdump(words, k, 16);
	if(word_length == 2) wdump(words, k, data_radix);
    else if(word_length == 1) bdump((char *)words, k, data_radix);

    //if (cflag) bdump((char *)words, k, (int)'c');
	//if (bflag) bdump((char *)words, k, (int)'b');
	//if (hd)    bdump((char *)words, k, (int)'h');
	for (k = 0; k < 8; k++) prevwds[k] = words[k];
	for (k = 0; k < 8; k++) words[k] = 0;
    }
}


void wdump(short *words, int k, int radix)
{
    int i;

    if (linenr++ != 1) printf("       ");
    for (i = 0; i < (k + 1) / 2; i++)
    	outword(words[i] & 0xFFFF, radix);
    printf("\n");
}


void bdump(char bytes[16], int k, int c)
{
    int i;

    if (linenr++ != 1) printf("       ");
    for (i = 0; i < k; i++)
	byte(bytes[i] & 0377, c);
    printf("\n");
}

void byte(int val, int c)
{
    if (c == RADIX_OCT) {
	printf(" ");
	outnum(val, 7);
	return;
    } else if (c == RADIX_HEX) {
	printf(" %02x", val);
	return;
    }
    if (val == 0)
	printf("  \\0");
    else if (val == '\b')
	printf("  \\b");
    else if (val == '\f')
	printf("  \\f");
    else if (val == '\n')
	printf("  \\n");
    else if (val == '\r')
	printf("  \\r");
    else if (val == '\t')
	printf("  \\t");
    else if (val >= ' ' && val < 0177)
	printf("   %c", val);
    else {
	printf(" ");
	outnum(val, 7);
    }
}


int getwords(short **words)
{
    int count;

    if (next >= bytespresent) {
	bytespresent = read(0, buf, 512);
	next = 0;
    }
    if (next >= bytespresent) return(0);
    *words = (short *) &buf[next];
    if (next + 16 <= bytespresent)
	count = 16;
    else
	count = bytespresent - next;

    next += count;

    return(count);
}

int same(short *w1, int *w2)
{
    int i;

    i = 8;
    while (i--)
	if (*w1++ != *w2++) return(0);

    return(1);
}

void outword(int val, int radix)
{
    /* Output 'val' in 'radix' in a field of total size 'width'. */

    //int i = 4;

    //if (radix == 16) i = width - 4;
    //if (radix == 10) i = width - 5;
    //if (radix == 8)  i = width - 6;

    //if (i == 1)
	//printf(" ");
    //else if (i == 2)
	//printf("  ");
    //else if (i == 3)
	//printf("   ");
    //else if (i == 4)
	//printf("    ");
    printf(" ");
    outnum(val, radix);
}


void outnum(int num, int radix)
{
    /*  Output a number with all leading 0s present.  Octal is 6 places,
     *  decimal is 5 places, hex is 4 places.
     */
    unsigned val;

    val = (unsigned) num;
    if (radix == RADIX_OCT)
	printf ("%06o", val);
    else if (radix == RADIX_DEC)
	printf ("%05u", val);
    else if (radix == RADIX_HEX)
	printf ("%04x", val);
    else if (radix == 7) {
  	/* special case */
	printf ("%03o", val);
    }
}


void addrout(long l)
{
    switch(addr_radix) {
        case RADIX_OCT:
            printf("%07lo", l);
            break;
        case RADIX_DEC:
            printf("%07ld", l);
            break;
        case RADIX_HEX:
            printf("%07lx", l);
            break;
        default:
            break;
    }
}


void usage(void)
{
    fprintf(stderr, "Usage: od [-bcdhovx] [file] [ [+] offset [.] [b] ]\n");
    exit(1);
}

int main(int argc, char *argv[])
{
    int k, flags;
    int opt;
    char *p;

    /* single-byte hex dump */
    if (!strcmp(argv[0], "hd")) {
        data_radix = RADIX_HEX;
        addr_radix = RADIX_HEX;
        word_length = 1;
        //hd = 1;
        //hflag = 1;
    }

    /* Process flags */
    setbuf(stdout, buffer);
    //flags = 0;
    //p = argv[1];
    //if (argc > 1 && *p == '-') {
	/* Flags present. */
	//flags++;
	//p++;
	//while (*p) {
	//    switch (*p) {
	//	case 'b': bflag++; break;
	//	case 'c': cflag++; break;
	//	case 'd': dflag++; break;
	//	case 'h': hflag++; break;
	//	case 'o': oflag++; break;
	//	case 'v': vflag++; break;	
	//	case 'x': xflag++; break;
	//	default:  usage();
	//    }
	//    p++;
	//}
    //} else {
	//oflag = 1;
    //}
    
    flags = 0;
    /* Default values */
    data_radix = RADIX_OCT;
    addr_radix = RADIX_OCT;
    word_length = 2;
     
    while((opt = getopt(argc, argv, "bcdhovxq")) != -1) {
        fprintf(stderr, "Opt: %c, optind: %d\n", opt, optind);
        switch(opt) {
            case 'b':
                word_length = 1;
                //bflag++;
                //flags = 1;
                break;
            case 'c':
                data_radix = RADIX_ASC;
                word_length = 1;
                break;
            case 'd':
                data_radix = RADIX_DEC;
                break;
            case 'h':
                data_radix = RADIX_HEX;
                word_length = 1;
                break;
            case 'o':
                /* Default values */
                break;
            case 'v':
                vflag++;
                break;
            case 'x':
                data_radix = RADIX_HEX;
                break;
            case 'q':
                data_radix = RADIX_HEX;
                addr_radix = RADIX_HEX;
                word_length = 1;
                break;
            default:
                usage();
                break;
        }
    }
    
    /*
    if ((bflag | cflag | dflag | oflag | xflag) == 0) oflag = 1;
    if (hd) oflag = 0;
    k = (flags ? 2 : 1);
    if (bflag | cflag) {
	width = 8;
    } else if (oflag) {
	width = 7;
    } else if (dflag) {
	width = 6;
    } else {
	width = 5;
    }*/
   
    fprintf(stderr, "Optind file: %d\n", optind); 
    /* Process file name, if any. */
    if(optind < argc)
        p = argv[optind];

    if (optind < argc && *p != '+') {
	/* Explicit file name given. */
	fprintf(stderr, "Opening filename %s\n", argv[optind]);
    close(0);
	if (open(argv[optind], O_RDONLY) != 0) {
	    fprintf(stderr, "od: cannot open %s\n", argv[k]);
	    exit(1);
	}
	optind++;
    }

    /* Process offset, if any. */
    if (optind < argc) {
	/* Offset present. */
	off = offset(argc, argv, optind);
	lseek(0, off, SEEK_SET);
    }

    dumpfile();
    addrout(off);
    printf("\n");

    return 0;
}

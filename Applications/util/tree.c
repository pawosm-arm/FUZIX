#include <stdio.h>
#include <string.h>
#include <ftw.h>

int iterator(const char *fp, const struct stat *sb, int type, struct FTW *ftw)
{
    const char *tp = fp + ftw->base;
    unsigned n = ftw->level;
    while(n > 1) {
        printf("|   ");
        n--;
    }
    if (n)
        printf("+---");
    printf("%s\n", tp);
    return 0;
}

int main(int argc, char *argv[])
{
    char *argn = argv[0];
    while(*++argv) {
        if (nftw(*argv, iterator, 2, FTW_PHYS) < 0)
            fprintf(stderr, "%s: failed to walk '%s'.\n", argn, *argv);
    }
}
#include <kernel.h>
#include <version.h>
#include <kdata.h>
#include <printf.h>
#include <exec.h>
#include <elf.h>

#undef DEBUG

/* This is a simple relocatable ELF loader. It requires -pie -static files
 *
 * It requires -r files with no
 * program headers but with the section headers present. It's intended for the
 * no-MMU flat-memory use case with a fixed size process slot (if you have an
 * MMU, then use one of the other formats, or an EXEC format prelinked ELF file
 * with a different loader). For simplicity, it assumes that user mode memory
 * can be directly accessed. It also does very little validation of the ELF
 * file as there's no point on such systems.
 *
 * There are almost certainly incorrect assumptions in here because ELF is
 * awful. Use at your own risk.
 */

static void close_on_exec(void)
{
	/* Keep the mask separate to stop SDCC generating crap code */
	uint16_t m = 1U << (UFTSIZE - 1);
	int8_t j;

	for (j = UFTSIZE - 1; j >= 0; --j) {
		if (udata.u_cloexec & m)
			doclose(j);
		m >>= 1;
	}
	udata.u_cloexec = 0;
}

/* User's execve() call. All other flavors are library routines. */
/*******************************************
execve (name, argv, envp)        Function 23
char *name;
char *argv[];
char *envp[];
********************************************/
#define name (uint8_t *)udata.u_argn
#define argv (char **)udata.u_argn1
#define envp (char **)udata.u_argn2

arg_t _execve(void)
{
	/* We aren't re-entrant where this matters */
	inoptr ino;
	char **nargv;		/* In user space */
	char **nenvp;		/* In user space */
	Elf32_Ehdr ehdr;
	Elf32_Phdr* phdr = NULL;
	Elf32_Rel* rbuf = NULL;
	Elf32_Sym* symtab;
	struct s_argblk* abuf = NULL;
	struct s_argblk* ebuf = NULL;
	int argc;
	int relidx;
	uaddr_t section_address[8] = {0};
	uaddr_t top;
	uaddr_t lomem;
	uaddr_t himem;
	uaddr_t stacktop;
	int symsect;
	int i;

	himem = ramtop - PROGLOAD;

	#ifdef DEBUG
		kprintf("_execve(%s)\n", name);
	#endif
	if (!(ino = n_open_lock(name, NULLINOPTR))) {
		#ifdef DEBUG
			kprintf("failed: file not found\n");
		#endif
		return (-1);
	}

	if (!((getperm(ino) & OTH_EX) &&
	      (ino->c_node.i_mode & F_REG) &&
	      (ino->c_node.i_mode & (OWN_EX | OTH_EX | GRP_EX)))) {
		#ifdef DEBUG
			kprintf("failed: not accessible\n");
		#endif
		goto eacces;
	}

	setftime(ino, A_TIME);

	/* Read in the ELF file header. */

	udata.u_offset = 0;
	udata.u_count = sizeof(ehdr);
	udata.u_base = (void*) &ehdr;
	udata.u_sysio = true;

	readi(ino, 0);
	if (udata.u_done != sizeof(ehdr)) {
		#ifdef DEBUG
			kprintf("failed: could not read ELF header\n");
		#endif
		goto enoexec;
	}

	if (!IS_ELF(ehdr)) {
		#ifdef DEBUG
			kprintf("failed: not an ELF file\n");
		#endif
		goto enoexec;
	}

	/* Read in the program headers. */

	{
		uint32_t psize = sizeof(Elf32_Phdr) * ehdr.e_phnum;

		if ((psize > (1<<BLKSHIFT)) || (ehdr.e_phentsize != sizeof(Elf32_Phdr))) {
			#ifdef DEBUG
				kprintf("failed: too many phdrs / invalid phdr size\n");
			#endif
			goto enoexec;
		}

		phdr = (Elf32_Phdr*) tmpbuf();
		udata.u_offset = ehdr.e_phoff;
		udata.u_count = psize;
		udata.u_base = (void*)phdr;
		udata.u_sysio = true;

		readi(ino, 0);
		if (udata.u_done != psize) {
			#ifdef DEBUG
				kprintf("failed: could not read phdrs\n");
			#endif
			goto enoexec;
		}
	}

	/* Scan the program headers to figure out where things are. */

	top = 0;
	lomem = 0;
	relidx = -1;
	for (i=0; i<ehdr.e_phnum; i++) {
		Elf32_Phdr* ph = &phdr[i];
		uaddr_t sectop = ph->p_vaddr + (uaddr_t)ALIGNUP(ph->p_memsz);
		switch (ph->p_type)
		{
			case PT_LOAD: /* Code or initalised data */
				if (ph->p_flags && (sectop > top))
					top = sectop;
				break;

			case PT_FUZIX_BSS: /* BSS */
				if (sectop > lomem)
					lomem = sectop;
				break;

			case PT_FUZIX_REL:
				relidx = i;
				break;
		}
	}
	if (lomem == 0)
		lomem = top;
	if (top > lomem) {
		#ifdef DEBUG
			kprintf("failed: lomem (%p) > top (%p)\n", lomem, top);
		#endif
	}
	stacktop = (uaddr_t)ALIGNUP(lomem) + USERSTACK;
	if (stacktop > himem) {
		#ifdef DEBUG
			kprintf("failed: out of memory (have %p, asked for %p)\n", himem, stacktop);
		#endif
		goto enoexec;
	}

	/* We've confirmed that there's room. Now, copy the command line arguments
	 * into temporary storage because we're about to trash userland. */

	abuf = (struct s_argblk *) tmpbuf();
	ebuf = (struct s_argblk *) tmpbuf();
	if (rargs(argv, abuf) || rargs(envp, ebuf)) {
		#ifdef DEBUG
			kprintf("failed: failed to read parameters\n");
		#endif
		goto enomem;
	}
	udata.u_ptab->p_status = P_NOSLEEP;

	/* FIXME. At this point we should call pagemap_realloc(), for this to work
	 * on a variable-sized process system. This must be the last test as it
	 * makes changes if it works. */

	#if 0
	if (pagemap_realloc(NULL, ))
		goto nogood3;
	#endif

	/* At this point, we are committed to reading in and
	 * executing the program. This call must not block. */

	close_on_exec();

	/* Now, load the data. */

	for (i=0; i<ehdr.e_phnum; i++) {
		Elf32_Phdr* ph = &phdr[i];
		if ((ph->p_type == PT_LOAD) && ph->p_flags) {
			uaddr_t ssize = (uaddr_t)ALIGNUP(ph->p_filesz);
			udata.u_offset = ph->p_offset;
			udata.u_count = ssize;
			udata.u_base = (uint8_t*) (PROGLOAD + ph->p_vaddr);
			udata.u_sysio = false;

			#ifdef DEBUG
				kprintf("loading %p bytes from %p to %p\n", ssize, udata.u_offset, udata.u_base);
			#endif

			readi(ino, 0);
			if (udata.u_done < ph->p_filesz) {
				#ifdef DEBUG
					kprintf("failed: couldn't read program data\n");
				#endif
				goto fatal;
			}
		}
	}

	/* Relocate, if a relocation phdr was found. */

	if (relidx != -1)
	{
		Elf32_Phdr* relph = &phdr[relidx];
		int count = relph->p_filesz / sizeof(Elf32_Rel);
		int chunksize = (1<<BLKSHIFT) / sizeof(Elf32_Rel);
		int thischunk = chunksize;
		rbuf = (Elf32_Rel*) tmpbuf();

		udata.u_offset = relph->p_offset;

		#ifdef DEBUG
			kprintf("performing %d relocations\n", count);
		#endif
		while (count) {
			if (thischunk == chunksize) {
				int remaining = count * sizeof(Elf32_Rel);
				if (remaining > (1<<BLKSHIFT))
					remaining = 1<<BLKSHIFT;
				udata.u_count = remaining;
				udata.u_base = (void*) rbuf;
				udata.u_sysio = true;

				readi(ino, 0);
				if (udata.u_done != remaining) {
					#ifdef DEBUG
						kprintf("failed: couldn't read relocs\n");
					#endif
					goto fatal;
				}
				thischunk = 0;
			}

			Elf32_Rel* r = &rbuf[thischunk];
			if (r->r_offset >= lomem) {
				#ifdef DEBUG
					kprintf("failed: relocation not in binary\n");
				#endif
				goto fatal;
			}
			if (platform_relocate_rel(r, PROGLOAD)) {
				#ifdef DEBUG
					kprintf("failed: relocation failed\n");
				#endif
				goto fatal;
			}

			count--;
			thischunk++;
		}
	}

	#ifdef DEBUG
		kprintf("himem=%p lomem=%p stacktop=%p top=%p\n", himem, lomem, stacktop, top);
	#endif
	himem += PROGLOAD;
	lomem += PROGLOAD;
	top += PROGLOAD;
	stacktop += PROGLOAD;

	/* Core dump and ptrace permission logic. */

#ifdef CONFIG_LEVEL_2
	/* Q: should uid == 0 mean we always allow core */
	if ((!(getperm(ino) & OTH_RD)) ||
		(ino->c_node.i_mode & (SET_UID | SET_GID)))
		udata.u_flags |= U_FLAG_NOCORE;
	else
		udata.u_flags &= ~U_FLAG_NOCORE;
#endif
	udata.u_top = himem;
	udata.u_ptab->p_top = himem;

	/* setuid, setgid if the executable requires it. */

	if (ino->c_node.i_mode & SET_UID)
		udata.u_euid = ino->c_node.i_uid;

	if (ino->c_node.i_mode & SET_GID)
		udata.u_egid = ino->c_node.i_gid;

	/* Wipe the memory in the BSS and stack. We don't wipe the memory above
	   that on 8bit boxes, but defer it to brk/sbrk(). */

	uzero((uint8_t *)top, stacktop - top);

	/* Set initial break for program. */

	udata.u_break = (int)ALIGNUP(stacktop);

	/* Turn off caught signals. */

	memset(udata.u_sigvec, 0, sizeof(udata.u_sigvec));

	/* Write back the arguments and environment. */

	nargv = wargs(((char *) stacktop - sizeof(uaddr_t)), abuf, &argc);
	nenvp = wargs((char *) (nargv), ebuf, NULL);

	/* Fill in udata.u_name with program invocation name. */

	uget((void *) ugetp(nargv), udata.u_name, 8);
	memcpy(udata.u_ptab->p_name, udata.u_name, 8);

	uaddr_t entry = ehdr.e_entry + PROGLOAD;
	if (rbuf)
		tmpfree(rbuf);
	if (abuf)
		tmpfree(abuf);
	if (ebuf)
		tmpfree(ebuf);
	if (phdr)
		tmpfree(phdr);
	i_deref(ino);

	/* Shove argc and the address of argv just below envp
	   FIXME: should flip them in crt0.S of app for R2L setups
	   so we can get rid of the ifdefs */

#ifdef CONFIG_CALL_R2L	/* Arguments are stacked the 'wrong' way around */
	uputp((uaddr_t) nargv, nenvp - 2);
	uputp((uaddr_t) argc, nenvp - 1);
#else
	uputp((uaddr_t) nargv, nenvp - 1);
	uputp((uaddr_t) argc, nenvp - 2);
#endif

	/* Set stack pointer for the program */

	udata.u_isp = udata.u_sp = nenvp - 2;

	/* Start execution (never returns) */

	udata.u_ptab->p_status = P_RUNNING;
	doexec(entry);

	/* Tidy up in various failure modes. */

fatal:
	/* Must not run userspace */
	udata.u_ptab->p_status = P_RUNNING;
	ssig(udata.u_ptab, SIGKILL);
error:
	if (rbuf)
		tmpfree(rbuf);
	if (abuf)
		tmpfree(abuf);
	if (ebuf)
		tmpfree(ebuf);
	if (phdr)
		tmpfree(phdr);
	i_unlock_deref(ino);
	return -1;

enomem:
	udata.u_error = ENOMEM;
	goto error;

eacces:
	udata.u_error = EACCES;
	goto error;

enoexec:
	udata.u_error = ENOEXEC;
	goto error;
}

#undef name
#undef argv
#undef envp

/* SN    TODO      max (1024) 512 bytes for argv
               and max  512 bytes for environ
*/

bool rargs(char **userspace_argv, struct s_argblk * argbuf)
{
	char *ptr;		/* Address of base of arg strings in user space */
	char *up = (char *)userspace_argv;
	uint8_t c;
	uint8_t *bufp;

	argbuf->a_argc = 0;	/* Store argc in argbuf */
	bufp = argbuf->a_buf;

	while ((ptr = (char *) ugetp(up)) != NULL) {
		up += sizeof(uptr_t);
		++(argbuf->a_argc);	/* Store argc in argbuf. */
		do {
			*bufp++ = c = ugetc(ptr++);
			if (bufp > argbuf->a_buf + 500) {
				udata.u_error = E2BIG;
				return true;	// failed
			}
		}
		while (c);
	}
	argbuf->a_arglen = bufp - (uint8_t *)argbuf->a_buf;	/*Store total string size. */
	return false;		// success
}


char **wargs(char *ptr, struct s_argblk *argbuf, int *cnt)	// ptr is in userspace
{
	char *argv;		/* Address of users argv[], just below ptr */
	int argc, arglen;
	char *argbase;
	uint8_t *sptr;

	sptr = argbuf->a_buf;

	/* Move them into the users address space, at the very top */
	ptr -= (arglen = (int)ALIGNUP(argbuf->a_arglen));

	if (arglen) {
		uput(sptr, ptr, arglen);
	}

	/* Set argv to point below the argument strings */
	argc = argbuf->a_argc;
	argbase = argv = ptr - sizeof(uptr_t) * (argc + 1);

	if (cnt) {
		*cnt = argc;
	}

	/* Set each element of argv[] to point to its argument string */
	while (argc--) {
		uputp((uptr_t) ptr, argv);
		argv += sizeof(uptr_t);
		if (argc) {
			do
				++ptr;
			while (*sptr++);
		}
	}
	uputp(0, argv);		/*;;26Feb- Add Null Pointer to end of array */
	return (char **)argbase;
}

/*
 *	Stub the 32bit only allocator calls
 */

arg_t _memalloc(void)
{
	udata.u_error = ENOMEM;
	return -1;
}

arg_t _memfree(void)
{
	udata.u_error = ENOMEM;
	return -1;
}

#ifdef CONFIG_LEVEL_2

/*
 *	Core dump
 */

static struct coredump corehdr = {
	0xDEAD,
	0xC0DE,
	16,
};

uint8_t write_core_image(void)
{
	inoptr parent = NULLINODE;
	inoptr ino;

	udata.u_error = 0;

	/* FIXME: need to think more about the case sp is lala */
	if (uput("core", udata.u_syscall_sp - 5, 5))
		return 0;

	ino = n_open(udata.u_syscall_sp - 5, &parent);
	if (ino) {
		i_deref(parent);
		return 0;
	}
	if (parent) {
		i_lock(parent);
		if (ino = newfile(parent, "core")) {
			ino->c_node.i_mode = F_REG | 0400;
			setftime(ino, A_TIME | M_TIME | C_TIME);
			wr_inode(ino);
			f_trunc(ino);

			/* FIXME: need to add some arch specific header bits, and
			   also pull stuff like the true sp and registers out of
			   the return stack properly */

			corehdr.ch_base = MAPBASE;
			corehdr.ch_break = udata.u_break;
			corehdr.ch_sp = udata.u_syscall_sp;
			corehdr.ch_top = PROGTOP;
			udata.u_offset = 0;
			udata.u_base = (uint8_t *)&corehdr;
			udata.u_sysio = true;
			udata.u_count = sizeof(corehdr);
			writei(ino, 0);
			udata.u_sysio = false;
			udata.u_base = MAPBASE;
			udata.u_count = udata.u_break - MAPBASE;
			writei(ino, 0);
			udata.u_base = udata.u_sp;
			udata.u_count = PROGTOP - (uint16_t)udata.u_sp;
			writei(ino, 0);
			i_unlock_deref(ino);
			return W_COREDUMP;
		}
	}
	return 0;
}
#endif

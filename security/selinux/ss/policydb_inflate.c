/* Derived from drivers/block/rd.c. */

#undef malloc
#undef malloc_sleep
#undef free
#undef exit

unsigned char *ss_fread_inbuf;
unsigned ss_fread_insize;
int ss_fread_use_inbuf;

/*
 * gzip declarations
 */

#define OF(args)  args

#ifndef memzero
#define memzero(s, n)     memset ((s), 0, (n))
#endif

typedef unsigned char  uch;
typedef unsigned short ush;
typedef unsigned long  ulg;

#define INBUFSIZ 4096
#define WSIZE 0x8000    /* window size--must be a power of two, and */
			/*  at least 32K for zip's deflate method */

static uch *inbuf;
static uch *window;

static unsigned insize;  /* valid bytes in inbuf */
static unsigned inptr;   /* index of next byte to be processed in inbuf */
static unsigned outcnt;  /* bytes in output buffer */
static int exit_code;
static long bytes_out;
static struct file *inflate_infp;
static uch *inflate_outbuf;
static unsigned inflate_outptr = 0;
#define OUTBUFINITSIZE (1024*1024)
#define OUTBUFGROWSIZE (512*1024)
static unsigned inflate_outsize = OUTBUFINITSIZE;

#define get_byte()  (inptr < insize ? inbuf[inptr++] : fill_inbuf())
		
/* Diagnostic functions (stubbed out) */
#define Assert(cond,msg)
#define Trace(x)
#define Tracev(x)
#define Tracevv(x)
#define Tracec(c,x)
#define Tracecv(c,x)

#define STATIC static

static int  fill_inbuf(void);
static void flush_window(void);
static void *malloc(int size);
static void free(void *where);
static void error(char *m);
static void gzip_mark(void **);
static void gzip_release(void **);

#include "../../lib/inflate.c"

static void *malloc(int size)
{
	return kmalloc(size, GFP_KERNEL);
}

static void free(void *where)
{
	kfree(where);
}

static void gzip_mark(void **ptr)
{
}

static void gzip_release(void **ptr)
{
}


/* ===========================================================================
 * Fill the input buffer. This is called only when the buffer is empty
 * and at least one byte is really needed.
 */
static int fill_inbuf(void)
{
	if (exit_code) return -1;
	
	insize = fread(inbuf, 1, INBUFSIZ, inflate_infp);

	if (insize == 0) return -1;

	inptr = 1;

	return inbuf[0];
}

/* ===========================================================================
 * Write the output window window[0..outcnt-1] and update crc and bytes_out.
 * (Used for the decompressed data only.)
 */
static void flush_window(void)
{
    ulg c = crc;         /* temporary variable */
    unsigned n;
    uch *in, ch;

    if ((inflate_outptr + outcnt) > inflate_outsize) {
	    uch *new_outbuf;
	    unsigned new_outsize;

	    if (outcnt > OUTBUFGROWSIZE) 
		    new_outsize = inflate_outsize + outcnt;
	    else
		    new_outsize = inflate_outsize + OUTBUFGROWSIZE;
	    new_outbuf = vmalloc(new_outsize);
	    if (!new_outbuf) {
		    error("security: Couldn't grow inflate output buffer\n");
		    return;
	    }
	    memcpy(new_outbuf, inflate_outbuf, inflate_outsize);
	    memset(new_outbuf + inflate_outsize, 0, new_outsize - inflate_outsize);
	    vfree(inflate_outbuf);
	    inflate_outbuf = new_outbuf;
	    inflate_outsize = new_outsize;
    }

    for (n = 0; n < outcnt; n++) {
	    if (inflate_outbuf[inflate_outptr]) {
		    error("security:  corrupted output buffer\n");
		    return;
	    }
	    inflate_outbuf[inflate_outptr++] = window[n];
    }

    in = window;
    for (n = 0; n < outcnt; n++) {
	    ch = *in++;
	    c = crc_32_tab[((int)c ^ ch) & 0xff] ^ (c >> 8);
    }
    crc = c;
    bytes_out += (ulg)outcnt;
    outcnt = 0;
}

static void error(char *x)
{
	printk(KERN_ERR "%s", x);
	exit_code = 1;
}

int policydb_inflate(struct file *fp)
{
	uch buf[2];
	int result;

	ss_fread_use_inbuf = 0;
	ss_fread_insize = 0;
	ss_fread_inbuf = NULL;
	buf[0] = buf[1] = 0;
	result = fread(buf, 1, 2, fp);
	if (result != 2) {
		printk(KERN_ERR "security: Couldn't read policydb, rc=%d.\n", -result);
		return -EINVAL;
	}
	fp->f_pos = 0;
	if (buf[0] != 037 || ((buf[1] != 0213) && (buf[1] != 0236))) {
		/* Not compressed. */
		return 0;
	}
	
	printk("security:  policydb is compressed, decompressing...\n");

	insize = 0;		/* valid bytes in inbuf */
	inptr = 0;		/* index of next byte to be processed in inbuf */
	outcnt = 0;		/* bytes in output buffer */
	exit_code = 0;
	bytes_out = 0;
	crc = (ulg)0xffffffffL; /* shift register contents */

	inflate_outsize = OUTBUFINITSIZE;
	inflate_outbuf = vmalloc(inflate_outsize);
	if (inflate_outbuf == 0) {
		printk(KERN_ERR "security: Couldn't allocate inflate output buffer\n");
		return -1;
	}
	memset(inflate_outbuf, 0, inflate_outsize);
	inflate_outptr = 0;

	inflate_infp = fp;

	inbuf = kmalloc(INBUFSIZ, GFP_KERNEL);
	if (inbuf == 0) {
		printk(KERN_ERR "security: Couldn't allocate inflate input buffer\n");
		vfree(inflate_outbuf);
		return -1;
	}
	window = kmalloc(WSIZE, GFP_KERNEL);
	if (window == 0) {
		printk(KERN_ERR "security: Couldn't allocate inflate window\n");
		kfree(inbuf);
		vfree(inflate_outbuf);
		return -1;
	}
	makecrc();
	result = gunzip();
	kfree(inbuf);
	kfree(window);
	if (result < 0 || exit_code) {
		printk("security:  decompression failed\n");
		vfree(inflate_outbuf);
	} else {
		ss_fread_inbuf = inflate_outbuf;
		ss_fread_insize = inflate_outptr;
		ss_fread_use_inbuf = 1;
		fp->f_pos = 0;
		printk("security:  decompressed %d bytes\n", ss_fread_insize);
	}
	return result;
}

void policydb_inflate_release(struct file *fp)
{
	if (ss_fread_inbuf)
		vfree(ss_fread_inbuf);
	ss_fread_use_inbuf = 0;
	ss_fread_insize = 0;
	ss_fread_inbuf = NULL;
}


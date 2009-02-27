#ifndef _LINUX_SCCB_H
#define _LINUX_SCCB_H

typedef struct {
	int subaddr;
	char* buf;
	int len;
	int write_verify;
} sccb_ioctl_t;

/* Use 's' as magic number */
#define SCCB_MAGIC  's'
#define SCCB_READ  _IOR(SCCB_MAGIC, 0, sccb_ioctl_t)
#define SCCB_WRITE _IOW(SCCB_MAGIC, 1, sccb_ioctl_t)

#endif // _LINUX_SCCB_H

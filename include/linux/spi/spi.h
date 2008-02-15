/*
 *  linux/include/linux/spi/spi.h
 *
 *  Copyright (C) 2001 Russell King, All Rights Reserved.
 *  Copyright (C) 2002 Compaq Computer Corporation, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * Derived from l3.h by Jamey Hicks
 */
#ifndef SPI_H
#define SPI_H

struct spi_msg {
	unsigned char	addr;	/* slave address	*/
	unsigned char	flags;		
#define SPI_M_RD		0x01
#define SPI_M_NOADDR	0x02
	unsigned short	len;	/* msg length		*/
	unsigned char	*buf;	/* pointer to msg data	*/
};

#ifdef __KERNEL__

#include <linux/types.h>
#include <linux/list.h>

struct spi_client;

struct spi_ops {
	int	(*open)(struct spi_client *);
	int	(*command)(struct spi_client *, int cmd, void *arg);
	void	(*close)(struct spi_client *);
};

/*
 * A driver is capable of handling one or more physical devices present on
 * SPI adapters. This information is used to inform the driver of adapter
 * events.
 */
struct spi_driver {
	/*
	 * This name is used to uniquely identify the driver.
	 * It should be the same as the module name.
	 */
	char			name[32];

	/*
	 * Notifies the driver that a new client wishes to use its
	 * services.  Note that the module use count will be increased
	 * prior to this function being called.  In addition, the
	 * clients driver and adapter fields will have been setup.
	 */
	int			(*attach_client)(struct spi_client *);

	/*
	 * Notifies the driver that the client has finished with its
	 * services, and any memory that it allocated for this client
	 * should be cleaned up.  In addition the chip should be
	 * shut down.
	 */
	void			(*detach_client)(struct spi_client *);

	/*
	 * Possible operations on the driver.
	 */
	struct spi_ops		*ops;

	/*
	 * Module structure, if any.	
	 */
	struct module		*owner;

	/*
	 * drivers list
	 */
	struct list_head	drivers;
};

struct spi_adapter;

struct spi_algorithm {
	/* textual description */
	char name[32];

	/* perform bus transactions */
	int (*xfer)(struct spi_adapter *, struct spi_msg msgs[], int num);
};

struct semaphore;

/*
 * spi_adapter is the structure used to identify a physical SPI bus along
 * with the access algorithms necessary to access it.
 */
struct spi_adapter {
	/*
	 * This name is used to uniquely identify the adapter.
	 * It should be the same as the module name.
	 */
	char			name[32];

	/*
	 * the algorithm to access the bus
	 */
	struct spi_algorithm	*algo;

	/*
	 * Algorithm specific data
	 */
	void			*algo_data;

	/*
	 * This may be NULL, or should point to the module struct
	 */
	struct module		*owner;

	/*
	 * private data for the adapter
	 */
	void			*data;

	/*
	 * Our lock.
	 */
	struct semaphore	lock;

	/*
	 * List of attached clients.
	 */
	struct list_head	clients;

	/*
	 * List of all adapters.
	 */
	struct list_head	adapters;
};

/*
 * spi_client identifies a single device (i.e. chip) that is connected to an 
 * SPI bus. The behaviour is defined by the routines of the driver. This
 * function is mainly used for lookup & other admin. functions.
 */
struct spi_client {
	struct spi_adapter	*adapter;	/* the adapter we sit on	*/
	struct spi_driver	*driver;	/* and our access routines	*/
	void			*driver_data;	/* private driver data		*/
	struct list_head	__adap;
};


extern int spi_add_adapter(struct spi_adapter *);
extern int spi_del_adapter(struct spi_adapter *);

extern int spi_add_driver(struct spi_driver *);
extern int spi_del_driver(struct spi_driver *);

extern int spi_attach_client(struct spi_client *, const char *, const char *);
extern int spi_detach_client(struct spi_client *);

extern int spi_transfer(struct spi_adapter *, struct spi_msg msgs[], int);
extern int spi_write(struct spi_client *, int, const char *, int);
extern int spi_read(struct spi_client *, int, char *, int);

/**
 * spi_command - send a command to a SPI device driver
 * @client: registered client structure
 * @cmd: device driver command
 * @arg: device driver arguments
 *
 * Ask the SPI device driver to perform some function.  Further information
 * should be sought from the device driver in question.
 *
 * Returns negative error code on failure.
 */
static inline int spi_command(struct spi_client *clnt, int cmd, void *arg)
{
	struct spi_ops *ops = clnt->driver->ops;
	int ret = -EINVAL;

	if (ops && ops->command)
		ret = ops->command(clnt, cmd, arg);

	return ret;
}

static inline int spi_open(struct spi_client *clnt)
{
	struct spi_ops *ops = clnt->driver->ops;
	int ret = 0;

	if (ops && ops->open)
		ret = ops->open(clnt);
	return ret;
}

static inline void spi_close(struct spi_client *clnt)
{
	struct spi_ops *ops = clnt->driver->ops;
	if (ops && ops->close)
		ops->close(clnt);
}
#endif

#endif /* SPI_H */

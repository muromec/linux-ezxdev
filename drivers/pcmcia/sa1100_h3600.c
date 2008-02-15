/*
 * drivers/pcmcia/sa1100_h3600.c
 *
 * PCMCIA implementation routines for H3600
 * All true functionality is shuttled off to the
 * pcmcia implementation for the current sleeve
 */
#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/i2c.h>

#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/arch/h3600-sleeve.h>
#include "sa1100_generic.h"

static struct pcmcia_init  sa1100_h3600_pcmcia_init; // Store the interrupt handler
struct pcmcia_low_level   *sleeve_pcmcia_ops = NULL; // Initialize with no sleeve

/* Forward declaractions */
int sa1100_h3600_common_pcmcia_get_irq_info( struct pcmcia_irq_info *info );

/***********************************************/
/* Stub routines called by the PCMCIA device.  */
/***********************************************/

static int h3600_pcmcia_init(struct pcmcia_init *init)
{
        if (0) printk(__FUNCTION__ ": init=%p ops=%p\n", init, sleeve_pcmcia_ops);
	sa1100_h3600_pcmcia_init = *init;

	if ( sleeve_pcmcia_ops != NULL
	     && sleeve_pcmcia_ops->init != NULL
	     && init->handler != NULL )
		return sleeve_pcmcia_ops->init( init );

	return 2;   /* Return by default */
}

static int h3600_pcmcia_shutdown(void)
{
        if (0) printk(__FUNCTION__ ": ops=%p\n", sleeve_pcmcia_ops);
	if ( sleeve_pcmcia_ops != NULL 
	     && sleeve_pcmcia_ops->shutdown != NULL )
		sleeve_pcmcia_ops->shutdown();

	return 0;   /* Not examined */
}

static int h3600_pcmcia_socket_state(struct pcmcia_state_array *state_array)
{
	if (0) printk(__FUNCTION__ ": ops=%p\n", sleeve_pcmcia_ops);

	if ( sleeve_pcmcia_ops != NULL 
	     && sleeve_pcmcia_ops->socket_state != NULL )
		return sleeve_pcmcia_ops->socket_state( state_array );

	/* Default actions */
        if ( state_array->size < 2 ) 
		return -1;

        memset(state_array->state, 0, (state_array->size)*sizeof(struct pcmcia_state));
	return 0;
}

static int h3600_pcmcia_get_irq_info(struct pcmcia_irq_info *info)
{
	if ( sleeve_pcmcia_ops != NULL 
	     && sleeve_pcmcia_ops->get_irq_info != NULL )
		return sleeve_pcmcia_ops->get_irq_info( info );

	return
		sa1100_h3600_common_pcmcia_get_irq_info( info );
}

static int h3600_pcmcia_configure_socket(const struct pcmcia_configure *configure)
{
	if (0) printk(__FUNCTION__ ": %p\n", configure);

	if ( sleeve_pcmcia_ops != NULL 
	     && sleeve_pcmcia_ops->configure_socket != NULL )
		return sleeve_pcmcia_ops->configure_socket( configure );

        return 0;
}

static int h3600_pcmcia_socket_init(int sock)
{
	if (0) printk(__FUNCTION__ ": %d\n", sock);

	if ( sleeve_pcmcia_ops != NULL 
	     && sleeve_pcmcia_ops->socket_init != NULL )
		return sleeve_pcmcia_ops->socket_init( sock );

        return 0;
}

static int h3600_pcmcia_socket_suspend(int sock)
{
	if (0) printk(__FUNCTION__ ": %d\n", sock);

	if ( sleeve_pcmcia_ops != NULL 
	     && sleeve_pcmcia_ops->socket_suspend != NULL )
		return sleeve_pcmcia_ops->socket_suspend( sock );

        return 0;
}

static int h3600_pcmcia_socket_get_timing(unsigned int sock, unsigned int cpu_speed, unsigned int cmd_time)
{
	if ( sleeve_pcmcia_ops != NULL 
	     && sleeve_pcmcia_ops->socket_get_timing != NULL )
		return sleeve_pcmcia_ops->socket_get_timing( sock, cpu_speed, cmd_time );

        return 0;
}

struct pcmcia_low_level h3600_pcmcia_ops = { 
        init:             h3600_pcmcia_init,
        shutdown:         h3600_pcmcia_shutdown,
        socket_state:     h3600_pcmcia_socket_state,
        get_irq_info:     h3600_pcmcia_get_irq_info,
        configure_socket: h3600_pcmcia_configure_socket,
        socket_init:      h3600_pcmcia_socket_init,
        socket_suspend:   h3600_pcmcia_socket_suspend,
        socket_get_timing: h3600_pcmcia_socket_get_timing
};

/****************************************************/
/*  Swapping functions for PCMCIA operations        */
/****************************************************/

void sa1100_h3600_pcmcia_change_sleeves(struct pcmcia_low_level *ops)
{
	if ( ops != sleeve_pcmcia_ops ) {
		h3600_pcmcia_shutdown();
		sleeve_pcmcia_ops = ops;
		h3600_pcmcia_init( &sa1100_h3600_pcmcia_init );
	}
}

void sa1100_h3600_pcmcia_remove_sleeve( void )
{
	if ( sleeve_pcmcia_ops != NULL ) {
		h3600_pcmcia_shutdown();
		sleeve_pcmcia_ops = NULL;
	}
}

EXPORT_SYMBOL(sa1100_h3600_pcmcia_change_sleeves);
EXPORT_SYMBOL(sa1100_h3600_pcmcia_remove_sleeve);


/****************************************************/
/*  Common functions used by the different sleeves */
/****************************************************/

int sa1100_h3600_common_pcmcia_init( struct pcmcia_init *init )
{
        int irq, res;

	if (0) printk(__FUNCTION__ "\n");

        /* Enable PCMCIA/CF bus: */
	set_h3600_egpio(IPAQ_EGPIO_OPT_ON);
        clr_h3600_egpio(IPAQ_EGPIO_OPT_RESET);
        clr_h3600_egpio(IPAQ_EGPIO_CARD_RESET);

        /* Set transition detect */
        set_GPIO_IRQ_edge( GPIO_H3600_PCMCIA_CD0 | GPIO_H3600_PCMCIA_CD1, GPIO_BOTH_EDGES );
        set_GPIO_IRQ_edge( GPIO_H3600_PCMCIA_IRQ0| GPIO_H3600_PCMCIA_IRQ1, GPIO_FALLING_EDGE );

        /* Register interrupts */
        irq = IRQ_GPIO_H3600_PCMCIA_CD0;
        res = request_irq( irq, init->handler, SA_INTERRUPT, "PCMCIA_CD0", NULL );
        if( res < 0 ) { 
		printk( KERN_ERR __FUNCTION__ ": Request for IRQ %u failed\n", irq );
		return -1;
	}

        irq = IRQ_GPIO_H3600_PCMCIA_CD1;
        res = request_irq( irq, init->handler, SA_INTERRUPT, "PCMCIA_CD1", NULL );
        if( res < 0 ) { 
		printk( KERN_ERR __FUNCTION__ ": Request for IRQ %u failed\n", irq );
		return -1;
	}

        return 2;  /* Always allow for two PCMCIA devices */
}

int sa1100_h3600_common_pcmcia_shutdown( void )
{
	if (0) printk(__FUNCTION__ "\n");

	/* disable IRQs */
	free_irq( IRQ_GPIO_H3600_PCMCIA_CD0, NULL );
	free_irq( IRQ_GPIO_H3600_PCMCIA_CD1, NULL );
  
	/* Disable CF bus: */
	clr_h3600_egpio(IPAQ_EGPIO_OPT_ON);
	set_h3600_egpio(IPAQ_EGPIO_OPT_RESET);
	return 0;
}

int sa1100_h3600_common_pcmcia_socket_state( struct pcmcia_state_array *state_array )
{
        unsigned long levels;

        if (state_array->size < 2) 
		return -1;

        memset(state_array->state, 0, (state_array->size)*sizeof(struct pcmcia_state));
	
	levels=GPLR;
	
	state_array->state[0].detect=((levels & GPIO_H3600_PCMCIA_CD0)==0)?1:0;
	state_array->state[0].ready=(levels & GPIO_H3600_PCMCIA_IRQ0)?1:0;
	state_array->state[1].detect=((levels & GPIO_H3600_PCMCIA_CD1)==0)?1:0;
	state_array->state[1].ready=(levels & GPIO_H3600_PCMCIA_IRQ1)?1:0;

	return 0;
}

int sa1100_h3600_common_pcmcia_get_irq_info( struct pcmcia_irq_info *info )
{
        switch (info->sock) {
        case 0:
                info->irq=IRQ_GPIO_H3600_PCMCIA_IRQ0;
                break;
        case 1:
                info->irq=IRQ_GPIO_H3600_PCMCIA_IRQ1;
                break;
        default:
                return -1;
        }
        return 0;
}

int sa1100_h3600_common_pcmcia_socket_init(int sock)
{
	if (0) printk(__FUNCTION__ ": %d\n", sock);

	/* Enable CF bus: */
	set_h3600_egpio(IPAQ_EGPIO_OPT_ON);
	clr_h3600_egpio(IPAQ_EGPIO_OPT_RESET);

	set_current_state(TASK_UNINTERRUPTIBLE);
	schedule_timeout(10*HZ / 1000);

	switch (sock) {
	case 0:
		set_GPIO_IRQ_edge(GPIO_H3600_PCMCIA_CD0, GPIO_BOTH_EDGES);
		break;
	case 1:
		set_GPIO_IRQ_edge(GPIO_H3600_PCMCIA_CD1, GPIO_BOTH_EDGES);
		break;
	}

	return 0;
}

int sa1100_h3600_common_pcmcia_socket_suspend(int sock)
{
	if (0) printk(__FUNCTION__ ": %d\n", sock);

	switch (sock) {
	case 0:
		set_GPIO_IRQ_edge(GPIO_H3600_PCMCIA_CD0, GPIO_NO_EDGES);
		break;
	case 1:
		set_GPIO_IRQ_edge(GPIO_H3600_PCMCIA_CD1, GPIO_NO_EDGES);
		break;
	}

	return 0;
}

EXPORT_SYMBOL(sa1100_h3600_common_pcmcia_init);
EXPORT_SYMBOL(sa1100_h3600_common_pcmcia_shutdown);
EXPORT_SYMBOL(sa1100_h3600_common_pcmcia_socket_state);
EXPORT_SYMBOL(sa1100_h3600_common_pcmcia_get_irq_info);
EXPORT_SYMBOL(sa1100_h3600_common_pcmcia_socket_init);
EXPORT_SYMBOL(sa1100_h3600_common_pcmcia_socket_suspend);

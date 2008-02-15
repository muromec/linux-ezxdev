/*
 * Copyright 2005 Motorola, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  
 * 02111-1307, USA
 *
 */
/*!
 * @file usr_blk_dev.c
 *
 * @ingroup usr_blk_dev
 *
 * @brief This is the main file for the user block driver.
 *
 * <B>Overview:</B><BR>
 *    The user block driver is a removable media block driver intended to allow
 * for user space drivers for the media.
 *
 *    This driver operates by creating an entry on /proc and using the entry
 * to communicate commands to user space.  See ::USR_BLK_DEV_CMD_ID_T for a list
 * of these commands.  Commands are sent from a kernel thread through /proc
 * to user space.  Once the user space process has completed the command it
 * responds with a write to the proc entry.  Only one command can be processed
 * at a time by a user space process and a kernel thread.  A thread is used
 * for the commands in order to keep the request queue from blocking which is
 * not allowed.
 *
 *    At this time a single thread is used for all devices attached.  This
 * is not intended to be the final design, but was done to speed up development.
 * This will be acceptable for systems which implement only one device.  If
 * more than one device is to be supported, then multiple threads should be
 * used, one per /proc entry.  This will speed up throughput, since one part
 * will not block the others.
 *
 * There are two different levels of command processing within this driver.
 * Commands from interrupts, the request queue and general file operations
 * (open, revalidate, check_media) are placed in the command structure
 * (usr_blk_dev_cmd_s) within the information structure (::USR_BLK_DEV_INFO_T)
 * for the device.  These commands are then processed by the kernel thread
 * (usr_blk_dev_thread()) and sent to user space.  Once they are processed by
 * user space, the kernel thread copies any return data and informs the source
 * of the command that it has been completed.
 *
 *   Part detection is done via a switch in the socket which is connected to
 * GPIO on the part.  An interrupt is generated on both the rising edge and the
 * falling edge of the signal.  Once an interrupt is detected a timer is used
 * do debounce the signal.  After the signal has achieved a steady state the
 * interrupt is re-enabled and the timer stopped.
 *
 * <B>Devices Created:</B><BR>
 *    -# /proc/usr_blk_dev0: The read/write interface to the user space driver
 *       for device 0.
 *    -# /proc/usr_blk_devn: The read/write interface to the user space driver
 *       for device n.
 * <B>Devices Used:</B><BR>
 *    -# /dev/mmca0: The root for the first detected card in the first slot.
 *    -# /dev/mmca\<n\>: The partitions of the detected card (up to 8 partitions
 *       are supported).
 *    -# /dev/mmcb0: The root for the card in the second slot if detected.
 *       Additional cards continue to increment the letter (c, d, e, ...).
 *    -# /dev/mmcb\<n\>: The partitions of the card in the second slot.
 *       Additional cards continue to increment the letter (c, d, e, ...).
 *
 * <B>Command Lifecycle:</B><BR>
 *    When a command is sent to the user block thread it transitions through
 * the states described in ::USR_BLK_DEV_CMD_STATE_T.  As they enter a new
 * state they may wait on a wait queue or wake up another queue as they
 * complete.  It is necessary for commands to cycle through the states or
 * they can hold up all subsequent commands.
 *
 *    The following describes the states and the wait queues used within them.
 * The state for the commands are stored in
 * usr_blk_dev_info[dev_num].cmd[cmd_src].state
 *
 * <B>#USR_BLK_DEV_CMD_STATE_NONE:</B><BR>
 *   This state can be considered the idle state of the driver.  This value sits
 * in the state variable when the no command is active.  A transition to the
 * ::USR_BLK_DEV_CMD_STATE_NEW state starts a command executing in the user block
 * thread.<BR>
 *
 * <B>#USR_BLK_DEV_CMD_STATE_NEW:</B><BR>
 *   This is the state a requester must put in the state value in order to start
 * a command executing.  In the case of commands which come from locations which
 * can block the wait queue usr_blk_dev_info[dev_num].cmd[cmd_src].wait_for_completion
 * must be used before the new command can be added.
 *
 * <B>#USR_BLK_DEV_CMD_STATE_THREAD:</B><BR>
 *   This value is placed in the state control variable as soon as the user block
 * thread detects the command is new.  It does not wait on any queues.
 *
 * <B>#USR_BLK_DEV_CMD_STATE_USR_READ</B><BR>
 *   When a command is in this state it has woken up the
 * usr_blk_dev_info[dev_num].wait_for_usr_read queue which controls when user
 * reads of the /proc entry happen.
 *
 * <B>#USR_BLK_DEV_CMD_STATE_USR_WAIT:</B><BR>
 *   This state is transitioned to as soon as the user process reads the command
 * from the /proc entry.  The command will remain in this state until the
 * response is written to the /proc entry from the user driver.  Commands in
 * this state (as well as the ::USR_BLK_DEV_CMD_STATE_USR_READ) are waiting on
 * the usr_blk_dev_info[dev_num].wait_for_usr_write queue.
 *
 * <B>#USR_BLK_DEV_CMD_STATE_USR_DONE:</B><BR>
 *   This state is entered as soon as the usr_blk_dev_info[dev_num].wait_for_usr_write
 * queue is awakened.  At this point the user block thread continues by processing
 * any necessary data.  If the data was from the request queue
 * (::USR_BLK_DEV_CMD_SRC_REQ) or an interrupt (::USR_BLK_DEV_CMD_SRC_IRQ) the command
 * is transitioned to ::USR_BLK_DEV_CMD_STATE_NONE since no more processing is
 * necessary. If it is not from one of these sources it will wake up the
 * usr_blk_dev_info[dev_num].cmd[cmd_src].wait_for_thread queue to allow the
 * requester to process the data.
 *
 * <B>#USR_BLK_DEV_CMD_STATE_THREAD_DONE:</B><BR>
 *   Once the user block thread is done executing a command the
 * usr_blk_dev_info[dev_num].cmd[cmd_src].wait_for_thread will be awakened and the
 * requester of the command will transition to this state.  Within this state
 * the requester will act upon the date returned and set the state back to
 * #USR_BLK_DEV_CMD_STATE_NONE.
 *
 * For more data on the individual commands see the ::USR_BLK_DEV_CMD_ID_T.
 */
#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <stdbool.h>
#include <asm/arch/hardware.h>
#include <linux/blk.h>
#include <linux/blkdev.h>
#include <linux/blkpg.h>
#include <linux/fs.h>
#include <linux/genhd.h>
#include <linux/hdreg.h>
#include <linux/init.h>
#include <asm/irq.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/timer.h>
#include <asm/uaccess.h>
#include <linux/wait.h>

#include <linux/usr_blk_dev.h>

/******************************************************************************
* Local constants and macros
******************************************************************************/

/*!
 * @brief The number of successive times the sample of the device attach signal must
 * match before the device state is changed.
 */
#define USR_BLK_DEV_DEBOUNCE_CNT 4

#ifndef CONFIG_ARCH_SCMA11
/*! @brief Port to which the media card detect switch is attached. */
#define USR_BLK_DEV_DEV1_INT_GPIO 11 

/*! @brief Returns non zero if the media card is attached. */
# define USR_BLK_DEV_IS_DEV1_ATTACHED ((GPLR(USR_BLK_DEV_DEV1_INT_GPIO) & GPIO_bit(USR_BLK_DEV_DEV1_INT_GPIO)) != 0)
#endif

/*! @brief The name of the /proc entry as it will appear in the directory /proc. */
#define USR_BLK_DEV_PROC_ENTRY_STR  "usr_blk_dev"

/*! @brief The number of characters in USR_BLK_DEV_PROC_ENTRY_STR. */
#define USR_BLK_DEV_PROC_ENTRY_LEN  11
/*!
 * @brief The number of times the proc entry for a device can be opened.
 *
 * The number of times the proc entry for a device can be opened.  This is set
 * to one since only one user driver is needed to drive a device and if more
 * than one user process opens the entry, bad things would happen.
 */
#define USR_BLK_DEV_MAX_PROC_OPENS 1

/*! @brief Name of the user block device. */
#define USR_BLK_DEV_DEVICE_NAME_STR "usr_blk_dev"

/*! @brief Support 8 partitions per card. */
#define USR_BLK_DEV_SHIFT   3

/* @brief The maximum number of sectors which can be included in a single request. */
#define USR_BLK_DEV_MAX_SECTORS_PER_REQ 128

/*! @brief Used to enable debugging messages to be sent out the system log. */
/* #define USR_BLK_DEV_DEBUG */

/*! @brief Macro used to send debug messages to the system log. */
#ifdef USR_BLK_DEV_DEBUG
# define tracemsg(fmt,args...)  printk(__FILE__": %s: "fmt,__func__,##args)
#else
# define tracemsg(fmt,args...)
#endif

/*!
 * @brief Macro to extract the device number from the device node.
 * 
 * Each device has 8 minor numbers since* USR_BLK_DEV_SHIFT is currently 3.
 * This results in the device number being stored in upper bits.
 */
#define USR_BLK_DEV_DEVICE_NR(i_rdev) (MINOR(i_rdev)>>USR_BLK_DEV_SHIFT)

/*! 
 * @brief Timer used to debounce the insertion of a card.
 *
 * Only one timer is used for all of the devices.
 */
static struct timer_list usr_blk_dev_timer;

/*!
 * @brief The possible states of the media detection state machine.
 *
 * A list of the possible states of the card detection state machine which is
 * implemented in usr_blk_dev_poll_card_attach().
 *
 * @note Removal is not debounced since a media card may have its power removed
 * at the point at which it is detected as removed only once.  In this case the
 * safest thing to do is to reinitialize the card.
 */
typedef enum
{
    /*! @brief The media is currently not connected. */
    USR_BLK_DEV_DEBOUNCE_STATE_REMOVED,
    /*!
     * @brief The media card has been detected as inserted and is being debounced.
     *
     * The media card has been detected as inserted at least once.  The number of
     * states determines the number of times the card must be detected as inserted
     * before it will be reported as such.
     */
    /* @{ */
    USR_BLK_DEV_DEBOUNCE_STATE_INSERTING,
    USR_BLK_DEV_DEBOUNCE_STATE_INSERTING1,
    USR_BLK_DEV_DEBOUNCE_STATE_INSERTING2,
    USR_BLK_DEV_DEBOUNCE_STATE_INSERTING3,
    USR_BLK_DEV_DEBOUNCE_STATE_INSERTING4,
    /* @} */
    /*!
     * @brief The card has been debounced as inserted, the driver must be informed.
     *
     * Once the card is detected as inserted the rest of the driver must be informed.
     * However, this may take several iterations if the thread still has not received
     * the previous state change.  This state will wait until the thread is ready to
     * receive the indication before sending it.
     */
    USR_BLK_DEV_DEBOUNCE_STATE_UPDATE_INSERT,
    /* @brief The media is currently inserted. */
    USR_BLK_DEV_DEBOUNCE_STATE_INSERTED,
    /* Removal is not debounced, so no REMOVING state exists. */
    /*!
     * @brief The card has been debounced as removed, the driver must be informed.
     *
     * Once the card is detected as removed the rest of the driver must be informed.
     * However, this may take several iterations if the thread still has not received
     * the previous state change.  This state will wait until the thread is ready to
     * receive the indication before sending it.
     */
    USR_BLK_DEV_DEBOUNCE_STATE_UPDATE_REMOVAL
} USR_BLK_DEV_DEBOUNCE_STATE_T;

/*!
 * @brief The possible sources of commands for the kernel thread to send.
 *
 * There are three different sources of a command from the kernel:
 *   - The detect interrupt,
 *   - The request queue,
 *   - A user process calling open, close, read, write etc.
 */
typedef enum
{
    USR_BLK_DEV_CMD_SRC_IRQ,  /*!< The command came from the detect interrupt. */
    USR_BLK_DEV_CMD_SRC_REQ,  /*!< The command came from the request queue. */
    USR_BLK_DEV_CMD_SRC_GEN,  /*!< The command came from a general source which can block. */
    USR_BLK_DEV_CMD_SRC__END
} USR_BLK_DEV_CMD_SRC_T;

/*!
 * @brief The state of the command being processed.
 *
 * A typical command will have the following states.  If the command originated
 * in an interrupt or from the request queue, it will not progress though the
 * final states, since there is no need to respond to the calling layer.  Please
 * note there is a difference between a command which came from the kernel
 * request queue and once which is being handled by the RW code from within the
 * thread.  The thread based read write commands progress through all of the
 * states.
 */
typedef enum
{
    /*! No command is active. */ 
    USR_BLK_DEV_CMD_STATE_NONE
    /*! A command request has been made by one of the three sources. */,
    USR_BLK_DEV_CMD_STATE_NEW,
    /*! The thread has received the command and has begun to act upon it. */
    USR_BLK_DEV_CMD_STATE_THREAD,
    /*! The thread is waiting for the user space driver to read the command. */
    USR_BLK_DEV_CMD_STATE_USR_READ,
    /*!
     * The command has been read by the user space process and the kernel thread
     * is waiting on the response.
     */
    USR_BLK_DEV_CMD_STATE_USR_WAIT,
    /*!
     * The user space process has acted upon the command and written a response
     * to the /proc entry.
     */
    USR_BLK_DEV_CMD_STATE_USR_DONE,
     /*!
      * The thread has completed operating on the command, and passed the data
      * back to the originator of the command.
      */
    USR_BLK_DEV_CMD_STATE_THREAD_DONE
} USR_BLK_DEV_CMD_STATE_T;

/*!
 * @brief Structure which holds information on the devices attached.
 *
 * The following is used as an array indexed by the device number to store any
 * data needed by the driver.  Please see the individual entries for details on
 * what is stored.
 */
typedef struct
{
    /*!
     * @brief The current state of debouncing for the device.
     * 
     * A simple state machine is used for debouncing the insertion and removal
     * of the card.  This is the state counter for that machine.  It can also
     * be used to determine the status of the card at any time while running.
     */
    USR_BLK_DEV_DEBOUNCE_STATE_T debounce_state;
    /*!
     * @brief Used to track the connection state of the media.
     */
    USR_BLK_DEV_MEDIA_STATE_T media_state;
    /*!
     * @brief true of the device has been validated at least once since it was
     *        attached.
     */
    uint8_t validated;
    /*!
     * @brief true if the media has changed since the last time ioctl was called
     *        with IOCMMCGETCARDSTATUS.
     */
    uint8_t ioctl_media_changed;
    /*!
     * @brief Keep track of the number of times the proc entry for the device has
     *        been opened.
     */
    uint8_t proc_opens;
    /*!
     * @brief Wait queue used when the user mode process is waiting for data from
     *        the kernel block driver.
     */
    wait_queue_head_t wait_for_usr_read;
    /*!
     * @brief Wait queue used when the kernel block driver is waiting for a
     *        response from the user mode process.
     */
    wait_queue_head_t wait_for_usr_write;
    /*!
     * @brief The init data which is kept while the driver is running for
     *        convenience.
     */
    USR_BLK_DEV_INIT_DATA_T params;
    /*!
     * @brief The source of the command being sent to user space.
     *
     * Indicates which source the command currently being sent to user space is
     * from.  This allows the proc functions to copy data to and from the correct
     * buffers.
     */
    USR_BLK_DEV_CMD_SRC_T cmd_src;
    /*!
     * @brief Array which holds commands from the different sources.
     *
     * Array which holds commands from the different sources. This is necessary
     * so that the different sources which cannot block can place their commands
     * somewhere.  This is done in lieu of a queue to save memory.  It is
     * possible for the interrupt code to overwrite a command before it is
     * processed, but this is acceptable as the card can only be inserted and
     * removed by the interrupt.
     */
    struct usr_blk_dev_cmd_s
    {
        /*!
         * @brief Wait queue used to wait before a new command can be added to
         *        the data register.
         * 
         * @note This is not used for the interrupt and request queue since they cannot block.
         */
        wait_queue_head_t wait_for_completion;
        /*!
         * @brief Wait queue used to wait for the kernel thread to complete
         * processing of the command.  Once the kernel thread is done operating
         * on the command the data can be processed. 
         */
        wait_queue_head_t wait_for_thread;
        /*!
         * @brief The current state of the command.
         *
         * The current state of the command is used by the wait queue code to
         * determine when it is acceptable to awaken.
         */
        USR_BLK_DEV_CMD_STATE_T state;
        /*!
         * @brief The command currently being sent to or processed by the user
         *        mode driver.
         */
        USR_BLK_DEV_CMD_T data;
    } cmd[USR_BLK_DEV_CMD_SRC__END];
    /*!
     * @brief The request queue used by this device.
     *
     * The request queue used by this device.  In the case of multiple cards
     * each will have its own request queue to maximize performance.
     */
    request_queue_t req_queue;
} USR_BLK_DEV_INFO_T;

/******************************************************************************
* Local variables
******************************************************************************/
/*!
 * @brief Array of device information indexed by the the device number, not
 *        the minor number.
 */
USR_BLK_DEV_INFO_T usr_blk_dev_info[USR_BLK_DEV_NUM_DEVICES];

/*!
 * @brief Define the maximum number of sectors which can be sent in a single
 * read or write command.  This array is indexed by minor number.
 */
static int usr_blk_dev_max_sectors[USR_BLK_DEV_NUM_DEVICES<<USR_BLK_DEV_SHIFT];

/*!
 * @brief Used to define the block sizes of the device.  This array is indexed
 *        by minor number.
 */
static int usr_blk_dev_blksizes[USR_BLK_DEV_NUM_DEVICES<<USR_BLK_DEV_SHIFT];

/*!
 * @brief The partition information needed for gendisk.  One entry per
 *        partition.  (Each minor number is a partition.)
 */
static struct hd_struct usr_blk_dev_partitions[USR_BLK_DEV_NUM_DEVICES<<USR_BLK_DEV_SHIFT];

/*!
 * @brief The partition sizes needed by gendisk in number of sectors.
 *
 * The partition size sizes user by gendisk.  These are kept in number of
 * sectors and are filled in by the call to register_disk().
 */
static int usr_blk_dev_sizes[USR_BLK_DEV_NUM_DEVICES<<USR_BLK_DEV_SHIFT];

/* Forward declaration of the operations to allow compilation. */
extern const struct block_device_operations usr_blk_dev_ops;

/*!
 * @brief The generic disk structure which will hold the partition data for
 *        the devices.
 */
static struct gendisk usr_blk_dev_gendisk =
{
    .major        = USR_BLK_DEV_MAJOR_NUM,
    .major_name   = USR_BLK_DEV_DEVICE_NAME_STR,
    .minor_shift  = USR_BLK_DEV_SHIFT,
    .fops         = (struct block_device_operations *)&usr_blk_dev_ops,
    .max_p        = 1<<USR_BLK_DEV_SHIFT,
    .part         = usr_blk_dev_partitions,
    .sizes        = usr_blk_dev_sizes,
    .nr_real      = 0,
    .real_devices = NULL
};
/*!
 * @brief Wait queue used to wake up the thread used to send commands from
 *        interrupts.
 */
#ifndef DOXYGEN_SHOULD_SKIP_THIS
static DECLARE_WAIT_QUEUE_HEAD(usr_blk_dev_thread_wait);
#endif

/******************************************************************************
* Local functions
******************************************************************************/
#if ((!defined CONFIG_OMAP_INNOVATOR) && (defined CONFIG_HOTPLUG))
# ifndef DOXYGEN_SHOULD_SKIP_THIS
/*
 * External variables and functions needed by the hotplug code which are not
 * in any header files.
 */
extern char hotplug_path[];
extern int call_usermodehelper(char *path, char **argv, char **envp);
# endif

/*!
 * @brief Calls hotplug with parameters based on if the device is attached.
 *
 * Executes the hotplug executable passing it the needed information.  The
 * device status is determined from the parameter attached and the device
 * assumed to be mmc.  This allows it to operate using the same files as
 * the standard mmc driver.
 *
 * @param dev_num  The device number for which hotplug will be called.  This
 *                 cannot be larger than 9.
 * @param attached true if the device is attached and false it it is
 *                 detached.
 */
static void usr_blk_dev_hotplug(unsigned int dev_num,  bool attached)
{
    char slot_str[] = "SLOT= ";
    const char action_add_str[] = "ACTION=add";
    const char action_remove_str[] = "ACTION=remove";
    const char *argv[] =
    {
        hotplug_path,
        "mmc",
        NULL
    };
    const char *envp[] =
    {
        "HOME=/",
        "PATH=/sbin:/bin:/usr/sbin:/usr/bin",
        slot_str,
        "MEDIA=mmc",
        action_add_str,
        NULL
    };
    int ret;

    if ((hotplug_path[0]) && (dev_num <= 9))
    {
        /* Place the device number in the slot string. */
        slot_str[5] = '0'+dev_num;
        /* Update the action string if the device is not attached. */
        if (!attached)
        {
            envp[4] = action_remove_str;
        }
        ret = call_usermodehelper((char *)argv[0], (char **)argv, (char **)envp);
        tracemsg("%s (%s %s) returned: %d\n", argv[0], envp[2], envp[4], ret);
    }
}
#else
# define usr_blk_dev_hotplug(dev, attached)
#endif

/*!
 * @brief Sends a command to the kernel thread.
 *
 * Sends a command to the block driver thread to be sent to the user process.
 * This function may block if a command is currently in process and the source
 * of the command is not the request queue or the detect interrupt.  (In other
 * words, the function will not block for the request queue and interrupt sources.)
 * The detect interrupt and request queue code must protect against overwriting a
 * previous command which has not yet been processed.
 *
 * @param id      The command to send to the kernel thread.
 * @param dev_num The device number which the command is for.
 * @param cmd_src The source of the command 
 */
static void usr_blk_dev_send_cmd_to_thread
(
    USR_BLK_DEV_CMD_ID_T id,
    unsigned int dev_num,
    USR_BLK_DEV_CMD_SRC_T cmd_src
)
{
    struct usr_blk_dev_cmd_s *cmd_p;
    USR_BLK_DEV_INFO_T *info_p;

    info_p = &usr_blk_dev_info[dev_num];
    cmd_p = &info_p->cmd[cmd_src];
    tracemsg("id: %d cmd_src: %d cmd state: %d media_state: %d\n", id, cmd_src, cmd_p->state, info_p->media_state);
    /*
     * If this is a command from the request queue and a request command is already pending
     * simply ignore it, since it will be picked up from the request queue when the current
     * request command completes.
     */
    if ((cmd_src == USR_BLK_DEV_CMD_SRC_REQ) && (cmd_p->state > USR_BLK_DEV_CMD_STATE_NONE))
    {
        return;
    }
    /*
     * If the source is not an interrupt block if a command is pending.
     */
    if ((cmd_src != USR_BLK_DEV_CMD_SRC_IRQ) && (cmd_src != USR_BLK_DEV_CMD_SRC_REQ))
    {
        wait_event(cmd_p->wait_for_completion,
                   ((cmd_p->state == USR_BLK_DEV_CMD_STATE_NONE) ||
                    (info_p->media_state == USR_BLK_DEV_MEDIA_STATE_ERROR)));
    }
    /*
     * Set up to send the command.
     */
    cmd_p->data.id = id;
    cmd_p->data.device_num = dev_num;
    cmd_p->data.media_state = info_p->media_state;
    cmd_p->state = USR_BLK_DEV_CMD_STATE_NEW;
    wake_up(&usr_blk_dev_thread_wait);
    tracemsg("Done\n");
}

/*!
 * @brief Aborts a command and sets the media state to ERROR.
 *
 * Aborts a command by setting the media state to error, the command state to
 * none and waking up any wait queues which might be waiting on data from the
 * command.  It is the responsibility of any code waiting on a queue to check
 * the media state for an indication of the error.
 *
 * @param info_p  Pointer to the information structure for the device being
 *                operated upon.
 */
void usr_blk_dev_abort_cmd(USR_BLK_DEV_INFO_T *info_p)
{
    struct usr_blk_dev_cmd_s *cmd_p;

    cmd_p = &info_p->cmd[info_p->cmd_src];
    tracemsg("id: %d cmd_src: %d cmd state: %d media_state: %d\n",
             cmd_p->data.id, info_p->cmd_src, cmd_p->state, info_p->media_state);
    cmd_p->state = USR_BLK_DEV_CMD_STATE_NONE;
    info_p->media_state = USR_BLK_DEV_MEDIA_STATE_ERROR;
    wake_up(&cmd_p->wait_for_completion);
    wake_up(&cmd_p->wait_for_thread);
    wake_up(&info_p->wait_for_usr_read);
    wake_up(&info_p->wait_for_usr_write);
}

/*!
 * @brief Set up and send the init or remove command using the block driver thread.
 *
 * Sets up and sends the init or remove command to the block driver thread.  This
 * function does not protect against over writing a previous command sent to the
 * thread since there is no harm in updating the attached status at any time.
 *
 * @param dev_num  The number of the device which was just attached.
 * @param attached true if the device is attached, false is detached.
 *
 * @return true if the attached state was updated by the driver.<BR>
 *         false if the driver is busy and cannot update the attached state
 *         at this time
 */
static bool usr_blk_dev_set_attached_state(unsigned int dev_num, bool attached)
{
    USR_BLK_DEV_INFO_T *info_p;
    USR_BLK_DEV_CMD_SRC_T i;
    bool dev_attached;
    
    info_p = &usr_blk_dev_info[dev_num];
    tracemsg("attached: %d cmd state: %d proc_opens: %d\n",
             attached, info_p->cmd[USR_BLK_DEV_CMD_SRC_IRQ].state, info_p->proc_opens);
    dev_attached = info_p->media_state != USR_BLK_DEV_MEDIA_STATE_REMOVED;
    info_p->media_state =
        (attached != false) ? USR_BLK_DEV_MEDIA_STATE_ATTACHED : USR_BLK_DEV_MEDIA_STATE_REMOVED;
    /*
     * Update all command sources with the new command status.  This will allow the
     * commands to abort if the card has been removed.
     */
    for (i = 0; i < USR_BLK_DEV_CMD_SRC__END; i++)
    {
        info_p->cmd[i].data.media_state = info_p->media_state;
    }
    /*
     * If a previous attach state command is still being processed indicate
     * this state change must wait.  If it does not wait the previous state
     * change may be missed by the user driver.
     */
    if (info_p->cmd[USR_BLK_DEV_CMD_SRC_IRQ].state != USR_BLK_DEV_CMD_STATE_NONE)
    {
        return false;
    }
    /*
     * Only send the command to the thread if the proc entry has been opened.  If it has
     * not been opened, there is no user process to receive the command.  In this case it
     * will be sent the first time the proc entry is opened.
     */
    if (info_p->proc_opens > 0)
    {
        usr_blk_dev_send_cmd_to_thread(attached ? USR_BLK_DEV_CMD_ID_INIT : USR_BLK_DEV_CMD_ID_REMOVE,
                                       dev_num, USR_BLK_DEV_CMD_SRC_IRQ);
    }
    return true;
}

/*!
 * @brief Returns true if the device is attached.
 *
 * @param dev_num The number of the device which needs to be checked
 *
 * @return true if the device is attached
 *         false if the device is removed
 */
static bool usr_blk_dev_check_dev_attached(unsigned int dev_num)
{
    tracemsg("\n");
    /* At this time only one device is supported. */
    return (USR_BLK_DEV_IS_DEV1_ATTACHED);
}

/*!
 * @brief Enables or disables the detection interrupt for a device.
 *
 * @param dev_num The device number which must be enabled.
 * @param enable  true if the interrupt must be enabled.<BR>
 *                false if the interrupt must be disabled.
 */
static void usr_blk_dev_set_irq_state(unsigned int dev_num, bool enable)
{
    tracemsg("%d\n", enable);
    /* Currently only one device is supported. */
#ifndef CONFIG_ARCH_SCMA11
    if (enable)
    {
        enable_irq(IRQ_GPIO(USR_BLK_DEV_DEV1_INT_GPIO));
    }
    else
    {
        disable_irq(IRQ_GPIO(USR_BLK_DEV_DEV1_INT_GPIO));
    }
#endif
}

/*!
 * @brief Called each time the attach signal must be polled.
 *
 * Checks the attach state of a device and updates the debounce state based
 * on the status.  Only the attached state is debounced.  For removal, a single
 * interrupt or poll indicating removal, is all that is necessary to indicate the
 * media has been removed.  This is to insure the proper power up sequence is
 * sent to the device if power was removed.
 *
 * The debounce is implemented as a state machine in order to handle the different
 * debounce counts for removal and insertion as well as special requirements for
 * sending the media state indication to the command handling thread.  If the
 * thread has not processed a previous card attach state change, a new state
 * change cannot be sent.  For this reason the "UPDATE" states are present.  See
 * USR_BLK_DEV_DEBOUNCE_STATE_T for more details.
 *
 * @param dev_num The device number to operate on
 *
 * @return true if the device is in a steady state.<BR>
 *         false if the debouncing must continue.
 */
static bool usr_blk_dev_poll_card_attach(unsigned int dev_num)
{
    bool attached;
    bool debounce_done;
    USR_BLK_DEV_INFO_T *info_p;

    info_p = &usr_blk_dev_info[dev_num];
    attached = usr_blk_dev_check_dev_attached(dev_num);
    debounce_done = false;
    tracemsg("debounce state: %d media_state: %d attached: %d\n",
             info_p->debounce_state, info_p->media_state, attached);
    switch (info_p->debounce_state)
    {
        /*
         * The device is currently removed.
         */
        case USR_BLK_DEV_DEBOUNCE_STATE_REMOVED:
            /* Move on to the next state if the device is now inserted. */
            if (attached)
            {
                info_p->debounce_state = USR_BLK_DEV_DEBOUNCE_STATE_INSERTING;
            }
            else
            {
                debounce_done = true;
            }
            break;
        /*
         * The device has been detected as inserted but has not been debounced.
         */
        case USR_BLK_DEV_DEBOUNCE_STATE_INSERTING:
        case USR_BLK_DEV_DEBOUNCE_STATE_INSERTING1:
        case USR_BLK_DEV_DEBOUNCE_STATE_INSERTING2:
        case USR_BLK_DEV_DEBOUNCE_STATE_INSERTING3:
        case USR_BLK_DEV_DEBOUNCE_STATE_INSERTING4:
            if (attached)
            {
                info_p->debounce_state++;
            }
            else
            {
                info_p->debounce_state = USR_BLK_DEV_DEBOUNCE_STATE_REMOVED;
            }
            break;
            
        /*
         * The device has been detected as inserted and the driver needs to be
         * informed of the new state.
         */
        case USR_BLK_DEV_DEBOUNCE_STATE_UPDATE_INSERT:
            if (attached)
            {
                if (usr_blk_dev_set_attached_state(dev_num, attached))
                {
                    info_p->debounce_state = USR_BLK_DEV_DEBOUNCE_STATE_INSERTED;
                }
            }
            else
            {
                /*
                 * If the device has been removed and it has not been reported as
                 * inserted, go back to the removed state.
                 */
                info_p->debounce_state = USR_BLK_DEV_DEBOUNCE_STATE_REMOVED;
            }
            break;

        /*
         * The device is currently inserted.
         */
        case USR_BLK_DEV_DEBOUNCE_STATE_INSERTED:
            if (!attached)
            {
                info_p->debounce_state = USR_BLK_DEV_DEBOUNCE_STATE_UPDATE_REMOVAL;
            }
            else
            {
                debounce_done = true;
            }
            break;

        /*
         * The device has been detected as removed and the driver needs to be
         * informed of the new state.
         */
        case USR_BLK_DEV_DEBOUNCE_STATE_UPDATE_REMOVAL:
            if (usr_blk_dev_set_attached_state(dev_num, false))
            {
                info_p->debounce_state = USR_BLK_DEV_DEBOUNCE_STATE_REMOVED;
            }
            break;

        default:
            info_p->debounce_state = USR_BLK_DEV_DEBOUNCE_STATE_UPDATE_REMOVAL;
            break;
    }
    return debounce_done;
}

/*!
 * @brief Timer function called to handle debouncing of the card detect signal.
 *
 * Function loops through the possible devices and debounces them if the
 * corresponding bit is set in the device_msk.  If any devices require
 * further debouncing the timer is restarted.  If no more debouncing is
 * needed for a device it's interrupt is re-enabled.  If further debouncing
 * is needed the interrupt will be disabled.
 *
 * @param device_msk A bit mask which has the bits set for the devices
 *                   which must be debounced.
 */
static void usr_blk_dev_timer_poll(unsigned long device_msk)
{
    unsigned int dev_num;
    static unsigned int timer_msk = 0;
    unsigned int check_msk;

    tracemsg("\n");
    for (dev_num = 0; dev_num < USR_BLK_DEV_NUM_DEVICES; dev_num++)
    {
        check_msk = 1<<dev_num;
        if (check_msk & ((unsigned int)device_msk))
        {
            /* Update the state, and if it no longer a steady state, start polling. */
            if (!usr_blk_dev_poll_card_attach(dev_num))
            {
                /* Disable the card detect interrupt until the next steady state is reached. */
                usr_blk_dev_set_irq_state(dev_num, false);
        
                /* Set up a timer to poll for a steady state on the card detect signal. */
                timer_msk |= check_msk;
            }
            else
            {
                /* No reason to continue checking this device. */
                timer_msk &= ~check_msk;
                /* Re-enable the interrupt for the next card attach event. */
                usr_blk_dev_set_irq_state(dev_num, true);
            }
        }
    }

    /* Restart the timer if not done debouncing. */
    if (timer_msk)
    {
        tracemsg("Setting up timer for poll in 1 jiffy.\n"); 
        usr_blk_dev_timer.function = usr_blk_dev_timer_poll;
        usr_blk_dev_timer.data = (unsigned long)timer_msk;
        usr_blk_dev_timer.expires = jiffies+1;
        /* If the timer is running for more than the devices checked, simply update it. */
        if ((timer_msk & ~((unsigned int)device_msk)) != 0)
        {
            mod_timer(&usr_blk_dev_timer, usr_blk_dev_timer.expires);
        }
        else
        {
            add_timer(&usr_blk_dev_timer);
        }
    }
}

/*!
 * @brief Interrupt handler for when a card detect signal changes state.
 *
 * This function is called once a change in state is detected.  As a result of
 * this function being called the interrupt will be disabled and the debounce
 * timer will be started.  See usr_blk_dev_timer_poll() for all of the details
 * on debouncing.
 *
 * @param irq        Not used in this function, but required by the caller.
 * @param device_msk The device mask to operate on.
 * @param regs       Not used in this function, but required by the caller.
 */
static void usr_blk_dev_attach_irq(int irq, void *device_msk, struct pt_regs *regs)
{
    tracemsg("\n");
    /* Take a sample and set up a timer if not in a steady state already. */
    usr_blk_dev_timer_poll((unsigned long)device_msk);
}

/*!
 * @brief Clears out the data necessary once a command is processed.
 *
 * This function must be called once a command has been processed by the block
 * device thread and the data has been processed by the function which requested
 * the command.  For example if the request came from
 * usr_blk_dev_check_media_change(), usr_blk_dev_check_media_change() must call
 * this function to allow other commands to be processed.  Any commands requested
 * during the processing of this command will be waiting until this command is
 * processed.
 *
 * @param info_p  Pointer to the information structure for the device being
 *                operated upon.
 * @param cmd_src The source of the command.
 *
 * @todo  Only the GEN user actually needs to be woken up.  It may make more
 *        sense to move the wait queue out of the command structure.
 */
static void usr_blk_dev_cmd_data_processed
(
    USR_BLK_DEV_INFO_T *info_p,
    USR_BLK_DEV_CMD_SRC_T cmd_src
)
{
    struct usr_blk_dev_cmd_s *cmd_p;
    
    tracemsg("\n");
    cmd_p = &info_p->cmd[cmd_src];
    cmd_p->data.id = USR_BLK_DEV_CMD_ID_NONE;
    cmd_p->state = USR_BLK_DEV_CMD_STATE_NONE;
    wake_up(&cmd_p->wait_for_completion);
}

/*!
 * @brief Sends a command to the thread and waits for the kernel thread to
 *        complete processing of the command.
 *
 * Sets up a command to be sent to kernel thread and waits for the kernel thread
 * to respond to the command.  This function will block until the response is
 * received from the user space driver and processed by the kernel thread.  It
 * should not be used from the request queue or an interrupt.
 *
 * @param info_p  Pointer to the information structure for the device being
 *                operated upon.
 * @param id      The command to send.
 * @param dev_num The device which the command must be sent to.
 * @param cmd_src The source of the command.
 *
 * @note: Do not call this from a context which cannot block.  For that
 *        case usr_blk_dev_send_cmd_to_thread() must be used.
 */
static void usr_blk_dev_send_cmd_to_thread_and_wait
(
    USR_BLK_DEV_INFO_T *info_p,
    USR_BLK_DEV_CMD_ID_T id,
    unsigned int dev_num,
    USR_BLK_DEV_CMD_SRC_T cmd_src
)
{
    struct usr_blk_dev_cmd_s *cmd_p;
    
    tracemsg("id: %d cmd_src:%d\n", id, cmd_src);
    cmd_p = &info_p->cmd[cmd_src];
    usr_blk_dev_send_cmd_to_thread(id, dev_num, cmd_src);

    /* Wait for thread to respond to the command. */
    wait_event(cmd_p->wait_for_thread,
               ((cmd_p->state == USR_BLK_DEV_CMD_STATE_THREAD_DONE) ||
                (info_p->media_state == USR_BLK_DEV_MEDIA_STATE_ERROR)));
    tracemsg("Thread command response received.\n");
}

/*!
 * @brief Clears out the data necessary once a command is processed.
 *
 * This must be called once the user block thread is done processing a command.
 * It will clear out the necessary data and set the state to
 * USR_BLK_DEV_CMD_STATE_THREAD_DONE.  If the command was not from an interrupt
 * or the request queue, the sending routine will be woken up.
 *
 * @param info_p  Pointer to the information structure for the device which
 *                the command has been completed.
 *
 * @todo  Only the GEN user actually needs to be woken up.  It may make more
 *        sense to move the wait queue out of the command structure.
 */
static void usr_blk_dev_thread_cmd_done(USR_BLK_DEV_INFO_T *info_p)
{
    struct usr_blk_dev_cmd_s *cmd_p;
    
    tracemsg("id: %d cmd_src: %d\n", info_p->cmd[info_p->cmd_src].data.id, info_p->cmd_src);
    cmd_p = &info_p->cmd[info_p->cmd_src];
    cmd_p->state = USR_BLK_DEV_CMD_STATE_THREAD_DONE;
    /*
     * If the source of the command was the request queue or an interrupt, no completed
     * ack or return data is supported.
     */
    if ((info_p->cmd_src == USR_BLK_DEV_CMD_SRC_IRQ) || (info_p->cmd_src == USR_BLK_DEV_CMD_SRC_REQ))
    {
        cmd_p->data.id = USR_BLK_DEV_CMD_ID_NONE;
        cmd_p->state = USR_BLK_DEV_CMD_STATE_NONE;
        return;
    }
    wake_up(&cmd_p->wait_for_thread);
}

/*
 * @brief Sends a command to user space and waits for the user mode driver to
 *        complete processing of the command.
 *
 * Sends a command from the user block device thread to user space and waits for
 * the user space driver to respond.  This function will block until the response
 * is received from user space.
 *
 * @param info_p  Pointer to the information structure for the device the
 *                command must be sent to.
 * @param id      The command which must be sent.
 * @param dev_num The device number to which the command must be sent.
 */
static void usr_blk_dev_send_cmd_to_usr_and_wait
(
    USR_BLK_DEV_INFO_T *info_p,
    USR_BLK_DEV_CMD_ID_T id,
    unsigned int dev_num
)
{
    struct usr_blk_dev_cmd_s *cmd_p;
    
    tracemsg("id: %d cmd_src: %d\n", id, info_p->cmd_src);
    /*
     * Set up to send the command.
     */
    cmd_p = &info_p->cmd[info_p->cmd_src];
    cmd_p->data.id = id;
    cmd_p->data.media_state = info_p->media_state;
    cmd_p->data.device_num = dev_num;
    cmd_p->state = USR_BLK_DEV_CMD_STATE_USR_READ;
    wake_up(&info_p->wait_for_usr_read);

    /* Wait for the user space driver to respond to the command. */
    wait_event(info_p->wait_for_usr_write,
               ((cmd_p->state == USR_BLK_DEV_CMD_STATE_USR_DONE) ||
                (info_p->media_state == USR_BLK_DEV_MEDIA_STATE_ERROR)));
    tracemsg("User command response received.\n");
}

/*!
 * @brief Handles reads from the /proc entry from the user mode process.
 *
 * Called when a user space application attempts to read from the /proc
 * entry for the usr_blk_dev.  When read is called the currently active
 * command is copied into the callers buffer.
 *
 * @param file  Pointer to the file structure for the operation.
 * @param buf   Pointer to the user space buffer in which to place the data read.
 * @param count The maximum number of bytes which can be placed in buf.
 * @param pos   The user file position.
 */
static ssize_t usr_blk_dev_proc_read(struct file *file, char *buf, size_t count, loff_t *pos)
{
    struct usr_blk_dev_cmd_s *cmd_p;
    USR_BLK_DEV_INFO_T *info_p;

    /* Send the current thread command to the usr driver. */
    info_p = (USR_BLK_DEV_INFO_T *)file->private_data;
    cmd_p = &info_p->cmd[info_p->cmd_src];
    tracemsg("cmd_src: %d\n", info_p->cmd_src);
    if (count < sizeof(cmd_p->data))
    {
        tracemsg("Warning: Size requested (%d) is too small.\n", count);
        usr_blk_dev_abort_cmd(info_p);
        return -EINVAL;
    }
    if (copy_to_user(buf, &cmd_p->data, sizeof(cmd_p->data)))
    {
        usr_blk_dev_abort_cmd(info_p);
        return -EFAULT;
    }
    cmd_p->state = USR_BLK_DEV_CMD_STATE_USR_WAIT;
    return sizeof(cmd_p->data);
}

/*!
 * @brief Handles writes to the /proc entry from the user mode process.
 *
 * Once the user space driver has completed a command it calls write to send
 * the data back to the kernel driver.  This routine handles the call to write.
 * As a result of the call the data is copied from the user space process, and
 * the routine waiting on the data is woken up.
 *
 * @param file  Pointer to the file structure for the operation.
 * @param buf   Pointer to the user space buffer which the data must be taken from.
 * @param count The maximum number of bytes which can be read from buf.
 * @param pos   The user file position.
 */
static ssize_t usr_blk_dev_proc_write(struct file *file, const char *buf, size_t count, loff_t *pos)
{
    struct usr_blk_dev_cmd_s *cmd_p;
    USR_BLK_DEV_INFO_T *info_p;

    info_p = (USR_BLK_DEV_INFO_T *)file->private_data;
    cmd_p = &info_p->cmd[info_p->cmd_src];
    tracemsg("count %d cmd_src: %d\n", count, info_p->cmd_src);
    if (count > sizeof(USR_BLK_DEV_CMD_T))
    {
        tracemsg("Warning: count (%d) is too large.\n", count);
        count = sizeof(USR_BLK_DEV_CMD_T);
    }
    if (cmd_p->state != USR_BLK_DEV_CMD_STATE_USR_WAIT)
    {
        tracemsg("Warning: Command write done in an invalid state (%d).\n", cmd_p->state);
        return 0;
    }
    if (count == sizeof(USR_BLK_DEV_CMD_T))
    {
        if (copy_from_user(&cmd_p->data, buf, count))
        {
            printk(KERN_ERR "usr_blk_dev: Error copying data from user space.\n");
            usr_blk_dev_abort_cmd(info_p);
            return -EFAULT;
        }
        cmd_p->state = USR_BLK_DEV_CMD_STATE_USR_DONE;
        wake_up(&info_p->wait_for_usr_write);
        return count;
    }
    else
    {
        tracemsg("Warning: Not enough data %d bytes.\n", count);
    }
    return 0;
}

/*!
 * @brief Handles calls to poll or select on the /proc entry from the user mode
 *        driver.
 *
 * Called when the user mode application polls on the /proc entry.  It must
 * return 0 when the wait must happen and a positive value when data is
 * available.
 *
 * @param file  Pointer to the file structure for the operation.
 * @param wait  Points to the kernel wait table for poll.
 *
 * @return Returns 0 when no data is available or valid POLL flags when data is
 *         available for reading.
 */
static unsigned int usr_blk_dev_proc_poll(struct file *file, poll_table *wait)
{
    USR_BLK_DEV_INFO_T *info_p;

    info_p = (USR_BLK_DEV_INFO_T *)file->private_data;
    tracemsg("info_p: %p usr_blk_dev_info: %p wait: %p\n",
             info_p, usr_blk_dev_info, wait);
    poll_wait(file, &info_p->wait_for_usr_read, wait);
    if ((info_p->cmd[info_p->cmd_src].data.id != USR_BLK_DEV_CMD_ID_NONE) &&
        (info_p->cmd[info_p->cmd_src].state == USR_BLK_DEV_CMD_STATE_USR_READ))
    {
        return (POLLIN | POLLRDNORM);
    }
    /*
     * In the case of an error or sporadic wakeup the user code is required to
     * do nothing, since reading the data makes no sense.  If it does read it
     * will see the error state in the media state field.
     */
    return 0;
}

/*!
 * @brief Handles calls to open on the /proc entry from the user mode process.
 *
 * Called when the user process attempts to open the /proc entry.  The function
 * sets up a pointer to the proc data, and increments the use count.
 *
 * @param inode A pointer to the files inode.
 * @param file  Pointer to the file structure for the operation.
 *
 * @return 0 upon success.<BR>
 *         -EBUSY if too many instances are already open.
 */
static int usr_blk_dev_proc_open(struct inode *inode, struct file *file)
{
    const struct proc_dir_entry *proc_p = inode->u.generic_ip;
    USR_BLK_DEV_INFO_T *info_p;
    USR_BLK_DEV_CMD_ID_T cmd;
    unsigned int dev_num;
    unsigned long irq_flags;

    info_p = (USR_BLK_DEV_INFO_T *)proc_p->data;
    if (info_p == NULL)
    {
        return -ENODEV;
    }
    tracemsg("proc_opens: %d data: %p\n", info_p->proc_opens, proc_p->data);
    if (info_p->proc_opens >= USR_BLK_DEV_MAX_PROC_OPENS)
    {
        return -EBUSY;
    }
    file->private_data = proc_p->data;
    /*
     * If this is the first open and the device is not initialized, send
     * the init command.
     */
    if (info_p->proc_opens == 0)
    {
        /* Find the device number. */
        dev_num = 0;
        while ((info_p != &usr_blk_dev_info[dev_num]) && (dev_num < USR_BLK_DEV_NUM_DEVICES))
        {
            dev_num++;
        }
        if (dev_num < USR_BLK_DEV_NUM_DEVICES)
        {
            local_irq_save(irq_flags);
            
            cmd = USR_BLK_DEV_CMD_ID_REMOVE;
            if (info_p->media_state > USR_BLK_DEV_MEDIA_STATE_ERROR)
            {
                cmd = USR_BLK_DEV_CMD_ID_INIT;
            }
            /*
             * The code cannot wait here, since the user process which opened /proc must respond
             * to the command.  For this reason the IRQ source is used which will emulate the insert
             * or remove command coming from the interrupt.  It would have come from the interrupt
             * normally, but when the card was detected no proc entry was open, so the command
             * could not be sent.
             */
            usr_blk_dev_send_cmd_to_thread(cmd, dev_num, USR_BLK_DEV_CMD_SRC_IRQ);
        }
        else
        {
            printk(KERN_ERR "usr_blk_dev: Unable to determine device number in usr_blk_dev_proc_open.\n");
            return -ENODEV;
        }
    }
    info_p->proc_opens++;
    return 0;
}

/*!
 * @brief Handles calls to close on the /proc entry from the user mode process.
 *
 * Called when the user process attempts to close the /proc entry.  It simply
 * decrements the use counter.
 *
 * @param inode A pointer to the files inode.
 * @param file  Pointer to the file structure for the operation.
 *
 * @return 0 upon success.<BR>
 *         -ENODEV if the device is not currently open.
 */
static int usr_blk_dev_proc_release(struct inode *inode, struct file *file)
{
    USR_BLK_DEV_INFO_T *info_p;

    info_p = (USR_BLK_DEV_INFO_T *)file->private_data;
    tracemsg("proc_opens: %d", info_p->proc_opens);
    if (info_p->proc_opens > 0)
    {
        info_p->proc_opens--;
        return 0;
    }
    return -ENODEV;
}

/*!
 * @brief Handles opening the media device
 *
 * Called by the kernel block code when a user attempts to mount/open the block
 * device.
 *
 * @param inode_p A pointer to the files inode.
 * @param file_p  Pointer to the file structure for the operation.
 *
 * @return 0 upon success.<BR>
 *         -ENODEV upon failure.
 *
 * @note Only the i_rdev record of the inode_p is valid when called to mount
 *       the device.  For file_p only f_mode and f_flags are valid when mounting.
 */
static int usr_blk_dev_open(struct inode *inode_p, struct file *file_p)
{
    USR_BLK_DEV_INFO_T *info_p;
    
    tracemsg("dev: %d minor: %d\n", USR_BLK_DEV_DEVICE_NR(inode_p->i_rdev), MINOR(inode_p->i_rdev));
    info_p = &usr_blk_dev_info[USR_BLK_DEV_DEVICE_NR(inode_p->i_rdev)];
    if ((info_p->media_state > USR_BLK_DEV_MEDIA_STATE_REMOVED) &&
        (info_p->params.device_size != 0))
    {
        /* Make sure the disk has been validated before the first open. */
        if (info_p->media_state != USR_BLK_DEV_MEDIA_STATE_VALIDATED)
        {
            check_disk_change(inode_p->i_rdev);
        }
        MOD_INC_USE_COUNT;
        tracemsg("Open successful.\n");
        return 0;
    }
    return -ENODEV;
}

/*!
 * @brief Handles closing device.
 *
 * Called when the kernel block code or the user wishes to close the media
 * device.   The file system is resynced to handle the umount command.
 *
 * @param inode_p A pointer to the files inode.
 * @param file_p  Pointer to the file structure for the operation.
 *
 * @return Always returns 0.
 */
static int usr_blk_dev_release(struct inode *inode_p, struct file *file_p)
{
    tracemsg("\n");
    tracemsg("blksize: %d\n", usr_blk_dev_blksizes[1]);
    MOD_DEC_USE_COUNT;
    return 0;
}

/*!
 * @brief Handles checking for a media change for the device.
 *
 * Called by the kernel to determine if a media changes has happened.  This
 * determination is passed up to user space since it handles communication
 * with the part.  This allows for the case where a part is removed, but
 * then re-inserted.  In this case it will be reinitialized when the part is
 * detected as inserted, so we have no reason to re-initialize.  If it is
 * the same part there is no reason to inform the file system, since nothing
 * in the block cache needs to change.
 *
 * @param i_rdev A pointer to the kernel device information
 *
 * @return Returns 0 when the device has not changed, or 1 when it has.
 */
static int usr_blk_dev_check_media_change(kdev_t i_rdev)
{
    USR_BLK_DEV_INFO_T *info_p;
    unsigned int dev_num;
    int changed;

    tracemsg("\n");
    dev_num = USR_BLK_DEV_DEVICE_NR(i_rdev);
    info_p = &usr_blk_dev_info[dev_num];
    usr_blk_dev_send_cmd_to_thread_and_wait(info_p, USR_BLK_DEV_CMD_ID_MEDIA_CHANGE,
                                            dev_num, USR_BLK_DEV_CMD_SRC_GEN);
    changed = (int)(info_p->cmd[USR_BLK_DEV_CMD_SRC_GEN].data.cmd_data_from_usr.media_changed);
    if (changed)
    {
        info_p->media_state = USR_BLK_DEV_MEDIA_STATE_INITIALIZED;
    }
    tracemsg("changed: %d\n", changed);
    usr_blk_dev_cmd_data_processed(info_p, USR_BLK_DEV_CMD_SRC_GEN);
    return changed;
}

/*!
 * @brief Handles revalidation of the device.
 *
 * The file system/kernel will call this function when a part change is detected
 * (this includes the initial part detection).   It will clear out the
 * partition information and re-initialize it by calling register_disk(). Once
 * register_disk() returns the partition information will be set up.  It should
 * be noted, that the devices partition table will be read through the request
 * queue before register disk returns.  Thus, calls to this function cannot
 * lock up any resources used by the request queue mechanism or the partition
 * table will not be read and the device will lock up.
 *
 * @param i_rdev Kernel device information
 *
 * @return Always returns 0.
 */
static int usr_blk_dev_revalidate(kdev_t i_rdev)
{
    USR_BLK_DEV_INFO_T *info_p;
    unsigned int dev_num;
    unsigned int i;

    tracemsg("\n");
    dev_num = USR_BLK_DEV_DEVICE_NR(i_rdev);
    info_p = &usr_blk_dev_info[dev_num];

    if (info_p->params.device_size != 0)
    {
        /* Send the revalidate command to user space. */
        usr_blk_dev_send_cmd_to_thread_and_wait(info_p, USR_BLK_DEV_CMD_ID_REVALIDATE,
                                                dev_num, USR_BLK_DEV_CMD_SRC_GEN);
        usr_blk_dev_cmd_data_processed(info_p, USR_BLK_DEV_CMD_SRC_GEN);
        if (info_p->media_state != USR_BLK_DEV_MEDIA_STATE_ERROR)
        {
            i = dev_num<<USR_BLK_DEV_SHIFT;
            /*
             * Reset the sizes and partition data based on the data which was received from
             * device during the initialization sequence.
             */
            while (i < (dev_num+1)<<USR_BLK_DEV_SHIFT)
            {
                memset(&usr_blk_dev_partitions[i], 0, sizeof(usr_blk_dev_partitions[0]));
                usr_blk_dev_sizes[i] = 0;
                usr_blk_dev_max_sectors[i] = USR_BLK_DEV_MAX_SECTORS_PER_REQ;
                usr_blk_dev_blksizes[i] = info_p->params.read_block_len;
                usr_blk_dev_gendisk.part[i].nr_sects = 0;
                i++;
            }
            /* Get the partition data from the device which is attached. */
            register_disk(&usr_blk_dev_gendisk,
                          i_rdev,
                          dev_num<<USR_BLK_DEV_SHIFT,
                          (struct block_device_operations *)&usr_blk_dev_ops,
                          info_p->params.device_size);
            info_p->media_state = USR_BLK_DEV_MEDIA_STATE_VALIDATED;
            tracemsg("Disk registered\n");
            for (i=dev_num<<USR_BLK_DEV_SHIFT; i<(dev_num+1)<<USR_BLK_DEV_SHIFT; i++)
            {
                tracemsg("%02d: nr_sects: %010ld start_sect: %010ld blk_size: %08d part_size: %08d\n", i,
                         usr_blk_dev_gendisk.part[i].nr_sects,
                         usr_blk_dev_gendisk.part[i].start_sect,
                         usr_blk_dev_blksizes[i],
                         usr_blk_dev_sizes[i]);
            }
        }
        else
        {
            printk(KERN_ERR "usr_blk_dev: Media error encountered while attempting to revalidate device %d.\n", dev_num);
        }
    }
    else
    {
        printk(KERN_WARNING "usr_blk_dev: Attempt to revalidate an uninitialized device %d.\n", dev_num);
    }
    return 0;
}

/*!
 * @brief Handles the i/o control calls for the device.
 *
 * Handles the following ioctl requests:<BR>
 *   BLKRRPART   - Reread the partition table.<BR>
 *   HDIO_GETGEO - Return the disk geometry.  In this case the number of
 *                 cylinders is set to the number if sectors on the disk and
 *                 the start value of the geometry is set to the start sector.
 *                 See struct hd_geometry in the kernel for more information.<BR>
 *   General     - Many other requests are handled by the the general purpose
 *                 block device ioctl handler blk_ioctl().
 *
 * @param inode_p A pointer to the files inode.
 * @param file_p  Pointer to the file structure for the operation.
 * @param cmd     The IOCTL command to operate on.
 * @param arg     The argument information for cmd.
 *
 * @return General:<BR>
 *            -EFAULT upon a copy to or from user problem.<BR>
 *            -EACCES upon an user access problem.<BR>
 *            -ENOTTY if an unsupported request is made.<BR>
 *         BLKRRPART:<BR>
 *             Returns 0 upon success.<BR>
 *         HDIO_GETGEO:<BR>
 *             Returns 0 upon success.
 *             
 */
static int usr_blk_dev_ioctl(struct inode *inode_p, struct file *file_p,
                             unsigned int cmd, unsigned long arg)
{
    int minor;
    int dev_num;
    int ret_val;
    long ret;
    long tmp;
    USR_BLK_DEV_INFO_T *info_p;
    struct hd_geometry geometry;

    tracemsg("cmd: %08x\n", cmd);
    minor = MINOR(inode_p->i_rdev);
    dev_num = USR_BLK_DEV_DEVICE_NR(inode_p->i_rdev);
    info_p = &usr_blk_dev_info[dev_num];
    switch (cmd)
    {
        case BLKRRPART:
            if (!capable(CAP_SYS_ADMIN))
            {
                return -EACCES;
            }
            return usr_blk_dev_revalidate(inode_p->i_rdev);
            
        case HDIO_GETGEO:
            geometry.cylinders = usr_blk_dev_partitions[minor].nr_sects;
            geometry.heads = 1;
            geometry.sectors = 1;
            geometry.start = usr_blk_dev_partitions[minor].start_sect;
            if (copy_to_user((void *)arg, &geometry, sizeof(geometry)))
            {
                return -EFAULT;
            }
            tracemsg("geometry.cylinders: %d geometry.start: %ld\n", geometry.cylinders, geometry.start);
            return 0;

        case _IOR('I', 0x0f05, int):  /* Temporary support for old MMC defines. */
        case IOCMMCGETCARDSTATUS:
            /*
             * The existing MMC driver returns the following statuses:
             *   0x01 - Card is write protected
             *   0x02 - Card is locked
             *   0x04 - Attempt at a the last lock failed  
             *   0x08 - Locking not supported
             *
             * The following is new to this driver:
             *   0x10 - Card has changed since the last call.  To clear this bit, it must be set before
             *          ioctl is called (see comment below).
             */
            if (get_user(tmp, (unsigned long *)arg))
            {
                return -EFAULT;
            }
            ret = IOCMMC_STATUS_LOCK_NOT_SUPPORTED;
            ret |= (info_p->ioctl_media_changed ? IOCMMC_STATUS_MEDIA_CHANGED : 0);
            /* Clear the bit if it was requested. */
            if (tmp & IOCMMC_STATUS_MEDIA_CHANGED)
            {
                info_p->ioctl_media_changed = false;
            }
            return put_user(ret, (unsigned long *)arg);
            
        case _IOR('I', 0x0f06, int):  /* Temporary support for old MMC defines. */
        case IOCMMCGETCARDTYPE:
            /*
             * Return the card type as follows (this is the same as the MMC driver):
             *  0 - MMC Card
             *  1 - SD Card
             */
            /* TODO - Needs to be updated to get the information from the user driver. */
            return put_user((unsigned long)IOCMMC_TYPE_SD, (unsigned long *)arg);

        case _IOR('I', 0x0f09, int):  /* Temporary support for old MMC defines. */
        case IOCMMCGETSIZE:
            /*
             * Returns the size in bytes as opposed to block as is done by BLKGETSIZE.
             */
            return put_user(usr_blk_dev_partitions[minor].nr_sects*usr_blk_dev_blksizes[minor], (unsigned long *)arg);

        case _IOR('I', 0x0f02, int):   /* Temporary support for old MMC defines. */
        case IOCMMCGETCARDCID:
            /*
             * Returns the 16 byte CID register as read in the user space driver.  For devices
             * which do not have a CID an array of 16 0's will be returned.
             */
            memset(info_p->cmd[USR_BLK_DEV_CMD_SRC_GEN].data.cmd_data_from_usr.cid,
                   0, sizeof(info_p->cmd[USR_BLK_DEV_CMD_SRC_GEN].data.cmd_data_from_usr.cid));
            usr_blk_dev_send_cmd_to_thread_and_wait(info_p, USR_BLK_DEV_CMD_ID_READ_CID,
                                                    dev_num, USR_BLK_DEV_CMD_SRC_GEN);
            ret_val = copy_to_user((void *)arg,
                                   info_p->cmd[USR_BLK_DEV_CMD_SRC_GEN].data.cmd_data_from_usr.cid,
                                   sizeof(info_p->cmd[USR_BLK_DEV_CMD_SRC_GEN].data.cmd_data_from_usr.cid));
            usr_blk_dev_cmd_data_processed(info_p, USR_BLK_DEV_CMD_SRC_GEN);
            if (info_p->media_state != USR_BLK_DEV_MEDIA_STATE_ERROR)
            {
                return -EIO;
            }
            if (ret_val)
            {
                return -EFAULT;
            }
            return 0;
            
        default:
            return blk_ioctl(inode_p->i_rdev, cmd, arg);
    }
    return -ENOTTY;
}

/*!
 * @brief Handle media requests for the block device.
 *
 * Handles media requests which are sent to the driver via the request queue.
 * Two different types of requests can be sent, they are for reads and writes
 * to the device.  In general this driver is set up to allow multiple request
 * queues, one per physical device attached.  All of these queues go through
 * this handler function.  Since multiple queues are supported none of the
 * general purpose handlers in blk.h can be used.
 *
 * This handler is called by the kernel when it needs to read or write data
 * to the device.  This may or may not be in a user context.  As a result
 * it cannot block under any circumstances.  This is accomplished by sending
 * the request to the user block thread for handling.  Once the thread receives
 * a read or write command the queue is operated on by the function
 * usr_blk_dev_thread_rw_cmd() which will loop until all requests have been
 * removed from the queue.
 *
 * If a request is received while the user block thread is currently acting
 * on a read or write command, nothing is done.  The transfer will be
 * handled by the user block thread.
 *
 * @param req_queue_p Pointer to the head node on the request queue.
 */
static void usr_blk_dev_request(request_queue_t *req_queue_p)
{
    USR_BLK_DEV_INFO_T *info_p;
    struct request *req_p;
    unsigned int dev_num;

    tracemsg("\n");
    req_p = blkdev_entry_next_request(&req_queue_p->queue_head);
    if (req_p == NULL)
    {
        tracemsg("Error: request pointer is NULL.\n");
        return;
    }
    dev_num = USR_BLK_DEV_DEVICE_NR(req_p->rq_dev);
    info_p = &usr_blk_dev_info[dev_num];
    tracemsg("Request for device %d state: %d\n", dev_num, (int)info_p->cmd[USR_BLK_DEV_CMD_SRC_REQ].state);
    if (info_p->media_state <= USR_BLK_DEV_MEDIA_STATE_ERROR)
    {
        tracemsg("Error: Request made while device not attached.\n");
    }
    if ((info_p->cmd[USR_BLK_DEV_CMD_SRC_REQ].state == USR_BLK_DEV_CMD_STATE_NONE) &&
        (!list_empty(&req_queue_p->queue_head)))
    {
        /*
         * At this point a valid request can be sent to the user mode driver.  Since
         * this function cannot block, the usr block dev thread is used to send the
         * command and complete the request.
         */
        usr_blk_dev_send_cmd_to_thread((req_p->cmd == WRITE) ? USR_BLK_DEV_CMD_ID_WRITE : USR_BLK_DEV_CMD_ID_READ,
                                       dev_num,
                                       USR_BLK_DEV_CMD_SRC_REQ);
    }
}

/*!
 * @brief Returns a pointer to the request queue for the device.
 *
 * @param dev Kernel device information
 *
 * @return A pointer to the request queue for the device
 */
request_queue_t *usr_blk_dev_find_req_queue(kdev_t dev)
{
    unsigned int dev_num;
    static int count = 0;

    dev_num = USR_BLK_DEV_DEVICE_NR(dev);
    if (dev_num >= USR_BLK_DEV_NUM_DEVICES)
    {
        if (count < 5)
        {
            count++;
            printk(KERN_WARNING "usr_blk_dev: Request for unknown device: %d", dev_num);
        }
        return NULL;
    }
    return &usr_blk_dev_info[dev_num].req_queue;
}

/*!
 * @brief Defines the file operations supported by the user space block
 *        driver device. 
 */
static const struct block_device_operations usr_blk_dev_ops =
{
    .check_media_change = usr_blk_dev_check_media_change,
    .ioctl              = usr_blk_dev_ioctl,
    .open               = usr_blk_dev_open,
    .release            = usr_blk_dev_release,
    .revalidate         = usr_blk_dev_revalidate
};

/*!
 * @brief Defines the files operations supported by the proc entry
 *        used for communication with user space.
 */
static const struct file_operations usr_blk_dev_proc_ops =
{
    .open =    usr_blk_dev_proc_open,
    .poll =    usr_blk_dev_proc_poll,
    .read =    usr_blk_dev_proc_read,
    .release = usr_blk_dev_proc_release,
    .write =   usr_blk_dev_proc_write
};

/*!
 * @brief Handles the init command from within the user block device thread.
 *
 * Handles the init command from the user block thread.  This will pass the init
 * command on to user space and wait for a response.  Once the response is
 * received the necessary data will be copied into the devices information
 * structure and hotplug will be called.
 *
 * @param info_p  Pointer to the devices information structure
 * @param dev_num The device number (not the minor number)
 */
static void usr_blk_dev_thread_init_cmd(USR_BLK_DEV_INFO_T *info_p, unsigned int dev_num)
{
    tracemsg("\n");
    /*
     * Send the command to the user mode driver.  The version is included in order to
     * determine if the user driver and kernel driver can communicate.
     */
    info_p->cmd[info_p->cmd_src].data.cmd_data_to_usr.version = USR_BLK_DEV_VERSION;
    usr_blk_dev_send_cmd_to_usr_and_wait(info_p, USR_BLK_DEV_CMD_ID_INIT, dev_num);
    /* Process the response. */
    memcpy(&info_p->params, &info_p->cmd[info_p->cmd_src].data.cmd_data_from_usr.init, sizeof(info_p->params));
    tracemsg("Init command done.  version_match: %d min ov: %d max ov: %d rbl: %d wbl:%d size:%d changed: %d\n",
             info_p->params.version_match,
             info_p->params.min_oper_volt,
             info_p->params.max_oper_volt,
             info_p->params.read_block_len,
             info_p->params.write_block_len,
             info_p->params.device_size,
             info_p->params.media_changed);
    if ((!info_p->params.version_match) || (info_p->media_state == USR_BLK_DEV_MEDIA_STATE_ERROR))
    {
        usr_blk_dev_abort_cmd(info_p);
        return;
    }
    if (info_p->params.media_changed)
    {
        info_p->ioctl_media_changed = true;
    }
    if (info_p->params.read_block_len != info_p->params.write_block_len)
    {
        printk(KERN_ERR "usr_blk_dev: Read (%d) and write (%d) block sizes differ, using read size for both.\n",
               info_p->params.read_block_len, info_p->params.write_block_len);
    }
    usr_blk_dev_gendisk.nr_real++;
    usr_blk_dev_thread_cmd_done(info_p);
    usr_blk_dev_hotplug(dev_num, true);
}       

/*!
 * @brief Handles the remove command from the user block thread.
 *
 * Handles the remove command from the user block thread by passing the command
 * on to user space and waiting for it to respond.  Since the device has been
 * removed the count of devices within the gendisk structure is decreased and
 * hotplug is called to let applications know of the device removal.
 *
 * @param info_p  Pointer to the devices information structure
 * @param dev_num The device number (not the minor number)
 */
static void usr_blk_dev_thread_remove_cmd(USR_BLK_DEV_INFO_T *info_p, unsigned int dev_num)
{
    tracemsg("\n");
    /* Send the command to the user mode driver. */
    usr_blk_dev_send_cmd_to_usr_and_wait(info_p, USR_BLK_DEV_CMD_ID_REMOVE, dev_num);
    usr_blk_dev_gendisk.nr_real--;
    usr_blk_dev_thread_cmd_done(info_p);
    usr_blk_dev_hotplug(dev_num, false);
}

/*!
 * @brief Handles requests for reads and writes from the devices request queue.
 *
 * Handles requests for reads and writes to the part from within the devices
 * request queue.  The queue is operated upon until it is empty, since the
 * request queue handler (usr_blk_dev_request()) will not send and new commands
 * to the user block thread.
 *
 * For read and write commands the usr_blk_dev_request() function set up the
 * command only.  No data parameters were set up, since each request queue entry
 * can contain more than one series of commands to execute.
 *
 * @param info_p  Pointer to the devices information structure
 * @param dev_num The device number (not the minor number)
 *
 * @todo Need to scan for contiguous requests and make them into one user mode
 *       request or determine why the elevator code is not doing this.
 */
static void usr_blk_dev_thread_rw_cmd(USR_BLK_DEV_INFO_T *info_p, unsigned int dev_num)
{
    struct request *req_p;
    struct buffer_head *bh_p;
    static unsigned int count = 0;
    unsigned long flags;
    int status;
    int minor;
    unsigned long num_sectors;
    unsigned long start_sector;

    tracemsg("\n");
    spin_lock_irqsave(&io_request_lock, flags);
    while (!list_empty(&info_p->req_queue.queue_head))
    {
        req_p = blkdev_entry_next_request(&info_p->req_queue.queue_head);
        /* Dequeue the request so the io_request_lock is held for the minimum time. */
        blkdev_dequeue_request(req_p);
        spin_unlock_irqrestore(&io_request_lock, flags);
        do
        {
            minor = MINOR(req_p->rq_dev);
            start_sector = req_p->sector;
            num_sectors = req_p->current_nr_sectors;
            info_p->cmd[USR_BLK_DEV_CMD_SRC_REQ].data.cmd_data_to_usr.rw.buf = req_p->buffer;

            /*
             * Combine all of the clustered sectors which can be combined.  All of the sectors in the
             * current linked list will be contiguous sectors.  However, they may or may not have
             * contiguous block cache addresses.  To avoid extra copies of the data to and from
             * user space, only the blocks with adjacent block addresses are combined.
             */
            bh_p = req_p->bh->b_reqnext;
            while ((bh_p != NULL) &&
                   (bh_p->b_rsector == start_sector+num_sectors) &&
                   (bh_p->b_rsector+bh_p->b_size/info_p->params.read_block_len <= usr_blk_dev_partitions[minor].nr_sects) &&
                   (bh_p->b_data == info_p->cmd[USR_BLK_DEV_CMD_SRC_REQ].data.cmd_data_to_usr.rw.buf+
                                        num_sectors*info_p->params.read_block_len))
            {
                num_sectors += bh_p->b_size/info_p->params.read_block_len;
                bh_p = bh_p->b_reqnext;
            }
            
            info_p->cmd[USR_BLK_DEV_CMD_SRC_REQ].data.cmd_data_to_usr.rw.sector =
                (uint32_t)(usr_blk_dev_partitions[MINOR(req_p->rq_dev)].start_sect+start_sector);
            info_p->cmd[USR_BLK_DEV_CMD_SRC_REQ].data.cmd_data_to_usr.rw.num_sectors = (uint32_t)num_sectors;
            tracemsg("minor: %02d start_sector: %010ld sector: %010d nr_sectors: %010d buffer: %p usr_blk_dev_blksize: %d\n",
                     minor,
                     usr_blk_dev_partitions[minor].start_sect,
                     info_p->cmd[USR_BLK_DEV_CMD_SRC_REQ].data.cmd_data_to_usr.rw.sector,
                     info_p->cmd[USR_BLK_DEV_CMD_SRC_REQ].data.cmd_data_to_usr.rw.num_sectors,
                     info_p->cmd[USR_BLK_DEV_CMD_SRC_REQ].data.cmd_data_to_usr.rw.buf,
                     usr_blk_dev_blksizes[minor]);
            status = 0;
            /*
             * Only make the request if its to a valid sector and the device is attached and initialized.
             */
            if ((start_sector+num_sectors <= usr_blk_dev_partitions[minor].nr_sects) &&
                (info_p->media_state > USR_BLK_DEV_MEDIA_STATE_ERROR))
            {
                usr_blk_dev_send_cmd_to_usr_and_wait(info_p,
                    (req_p->cmd == WRITE) ? USR_BLK_DEV_CMD_ID_WRITE : USR_BLK_DEV_CMD_ID_READ,
                    dev_num);
                if (info_p->media_state != USR_BLK_DEV_MEDIA_STATE_ERROR)
                {
                    status = (int)info_p->cmd[USR_BLK_DEV_CMD_SRC_REQ].data.cmd_data_from_usr.rw_success;
                }
            }
            if ((!status) && (count < 5))
            {
                printk(KERN_WARNING "usr_blk_dev: Request (%lu) past end sector (%lu).\n",
                       start_sector+num_sectors, usr_blk_dev_partitions[minor].nr_sects);
                count++;
            }
            tracemsg("RW completed status: %d\n", status);
            usr_blk_dev_thread_cmd_done(info_p);
            /*
             * Let the block driver know which sectors from the request were operated upon.
             */
            do
            {
                end_that_request_first(req_p, status, USR_BLK_DEV_DEVICE_NAME_STR);
            } while (req_p->bh != bh_p);
        } while (bh_p != NULL);
        spin_lock_irqsave(&io_request_lock, flags);
        end_that_request_last(req_p);
    }
    spin_unlock_irqrestore(&io_request_lock, flags);
    tracemsg("Done\n");
}

/*!
 * @brief Returns true if a command is pending for one of the devices attached.
 *
 * @return true if a command is pending for one of the devices attached.<BR>
 *         false if no commands are pending for any of the devices attached.
 */
static bool usr_blk_dev_cmd_pending(void)
{
    unsigned int i;
    USR_BLK_DEV_CMD_SRC_T cmd_src;

    tracemsg("\n");
    for (i=0; i < USR_BLK_DEV_NUM_DEVICES; i++)
    {
        for (cmd_src = 0; cmd_src < USR_BLK_DEV_CMD_SRC__END; cmd_src++)
        {
            if (usr_blk_dev_info[i].cmd[cmd_src].state == USR_BLK_DEV_CMD_STATE_NEW)
            {
                return true;
            }   
        }
    }
    return false;
}

/*!
 * @brief Thread used to execute commands which need to be run from an
 *        interrupt context.
 *
 * This is what is referred to as the user block thread.  It is responsible for
 * sending commands to user space and waiting for the response.  At this time
 * only one instance of this thread drives all devices attached.  If more than
 * one device is supported in the future, the code should be updated to have
 * more than one instance of this thread and pass the device number in.
 *
 * Commands are read from the information structure for the active devices.  If
 * the command state is set to USR_BLK_DEV_CMD_STATE_NEW, then the execution of
 * the command is started by setting the state to USR_BLK_DEV_CMD_STATE_THREAD
 * and sending the command to user space if necessary.  At that point the
 * command waits on a wait queue for the response.  Once the response is
 * received the data is acted upon if necessary and then the requester of the
 * command is awakened by another wait queue if it is not an interrupt or
 * the request queue.  Once the command is completed, the command handlers are
 * expected to call usr_blk_dev_thread_cmd_done() to reset the state of
 * the command.
 *
 * This is repeated for all of the possible sources of the commands.  See
 * struct usr_blk_dev_cmd_s for more information on the command sources.
 *
 * @param unused Data from schedule time.
 *
 * @return Normally does not return.  If the USR_BLK_DEV_CMD_ID_EXIT command
 *         is sent 0 will be returned.
 */
static int usr_blk_dev_thread(void *unused)
{
    USR_BLK_DEV_INFO_T *info_p;
    struct usr_blk_dev_cmd_s *cmd_p;
    bool cmd_found;
    unsigned int i;
    USR_BLK_DEV_CMD_SRC_T cmd_src;
    
    /* Set up some stuff for the thread */
#ifndef DOXYGEN_SHOULD_SKIP_THIS
# if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0))
    daemonize();
    strcpy(current->comm, "kusr_blk_dev_thread");
# else
    daemonize("kusr_blk_dev_thread");
# endif
#endif
    reparent_to_init();
    tracemsg("Thread running.\n");

    while (1)
    {
        /*
         * Check to see if any commands are requested.  If they are then send the command
         * to user space and wait for the response.
         */
        do
        {
            cmd_found = false;
            for (i=0; i<USR_BLK_DEV_NUM_DEVICES; i++)
            {
                /*
                 * Check each different source for a command and act upon them if necessary.
                 */
                for (cmd_src = 0; cmd_src < USR_BLK_DEV_CMD_SRC__END; cmd_src++)
                {
                    tracemsg("Searching for a command from device %d src %d.\n", i, cmd_src);
                    info_p = &usr_blk_dev_info[i];
                    /* Indicate what the active command source is to the proc routines. */
                    info_p->cmd_src = cmd_src;
                    cmd_p = &info_p->cmd[cmd_src];
                    if (cmd_p->state == USR_BLK_DEV_CMD_STATE_NEW)
                    {
                        cmd_p->state = USR_BLK_DEV_CMD_STATE_THREAD;
                        tracemsg("Processing id: %d cmd_src: %d dev_num: %d\n", cmd_p->data.id, cmd_src, i);
                        switch (cmd_p->data.id)
                        {
                            case USR_BLK_DEV_CMD_ID_NONE:
                                usr_blk_dev_thread_cmd_done(info_p);
                                break;
                            
                            case USR_BLK_DEV_CMD_ID_INIT:
                                usr_blk_dev_thread_init_cmd(info_p, i);
                                cmd_found = true;
                                break;

                            case USR_BLK_DEV_CMD_ID_REMOVE:
                                usr_blk_dev_thread_remove_cmd(info_p, i);
                                cmd_found = true;
                                break;
                            
                            case USR_BLK_DEV_CMD_ID_READ:
                            case USR_BLK_DEV_CMD_ID_WRITE:
                                usr_blk_dev_thread_rw_cmd(info_p, i);
                                cmd_found = true;
                                break;

                            case USR_BLK_DEV_CMD_ID_MEDIA_CHANGE:
                            case USR_BLK_DEV_CMD_ID_REVALIDATE:
                            case USR_BLK_DEV_CMD_ID_READ_CID:
                                if (info_p->media_state >= USR_BLK_DEV_MEDIA_STATE_ERROR)
                                {
                                    usr_blk_dev_send_cmd_to_usr_and_wait(info_p, cmd_p->data.id, i);
                                }
                                usr_blk_dev_thread_cmd_done(info_p);
                                cmd_found = true;
                                break;

                            case USR_BLK_DEV_CMD_ID_EXIT:
                                usr_blk_dev_send_cmd_to_usr_and_wait(info_p, cmd_p->data.id, i);
                                usr_blk_dev_thread_cmd_done(info_p);
                                return 0;
                                
                            default:
                                tracemsg("Error: Command %d not supported in thread.\n", cmd_p->data.id);
                                usr_blk_dev_thread_cmd_done(info_p);
                        }
                    }
                }
            }
        } while (cmd_found);
        tracemsg("waiting for a new command.\n");
        /* Wait for a command to be sent. */
        wait_event(usr_blk_dev_thread_wait, usr_blk_dev_cmd_pending());
    }
    return 0;
}

/*!
 * @brief Initializer for the user block device.
 *
 * Initializes all of the necessary data structures and registers the device
 * entries used for the user block device.
 *
 * The following devices are registered:
 *    -# /proc/usr_blk_dev0: The read/write interface to the user space driver
 *       for device 0.
 *    -# /proc/usr_blk_devn: The read/write interface to the user space driver
 *       for device n.
 *    -# Gendisk is initialized.
 *
 * The following are initialized or started:
 *    -# All of the wait queues are initialized.
 *    -# The user block thread is started.
 *    -# The detection timer is started.
 *    -# The gendisk arrays are initialized
 *    -# The block device size arrays are initialized.
 *
 * @note: The /dev/mmc?? entries are not valid until a valid device is
 *        detected and register_disk is called from usr_blk_dev_revalidate().
 */
int __init usr_blk_dev_init(void)
{
    USR_BLK_DEV_INFO_T *info_p;
    struct proc_dir_entry *proc_p;
    char proc_entry_str[USR_BLK_DEV_PROC_ENTRY_LEN+2] = USR_BLK_DEV_PROC_ENTRY_STR;
    unsigned int i;
    unsigned int j;
    int result;

    tracemsg("\n");
    /*
     * Initialize the wait queues and Create the /proc entry which will be used to
     * communicate with the user driver.
     */
    memset(usr_blk_dev_info, 0x00, sizeof(usr_blk_dev_info));
    proc_entry_str[USR_BLK_DEV_PROC_ENTRY_LEN+1] = '\0';
    for (i = 0; i < USR_BLK_DEV_NUM_DEVICES; i++)
    {
        info_p = &usr_blk_dev_info[i];
        /*
         * Initialize the wait queues.
         */
        init_waitqueue_head(&info_p->wait_for_usr_read);
        init_waitqueue_head(&info_p->wait_for_usr_write);
        /*
         * Initialize the proc entry.
         */
        proc_entry_str[USR_BLK_DEV_PROC_ENTRY_LEN] = '0'+i;
        proc_p = create_proc_entry(proc_entry_str, 0600, NULL);
        if (proc_p == NULL)
        {
            printk(KERN_ERR "usr_blk_dev: Unable to create entry in /proc.\n");
            return -ENOMEM;
        }
        proc_p->proc_fops = (struct file_operations *)&usr_blk_dev_proc_ops;
        proc_p->data = (void *)info_p;
        /*
         * Initialize the command structure.
         */
        for (j = 0; j < USR_BLK_DEV_CMD_SRC__END; j++)
        {
            init_waitqueue_head(&info_p->cmd[j].wait_for_completion);
            init_waitqueue_head(&info_p->cmd[j].wait_for_thread);
            info_p->cmd[j].data.id = USR_BLK_DEV_CMD_ID_NONE;
            info_p->cmd[j].state = USR_BLK_DEV_CMD_STATE_NONE;
        }
        /*
         * Initialize the request queue.
         */
        blk_init_queue(&info_p->req_queue, usr_blk_dev_request);
        blk_queue_headactive(&info_p->req_queue, 0);
    }
    blk_dev[USR_BLK_DEV_MAJOR_NUM].queue = usr_blk_dev_find_req_queue;

    /*
     * Start a kernel thread which will handles all commands.  This is needed
     * since commands may need to be processed from sources which cannot block.
     * This is true for commands which come from the request queue or an interrupt.
     */
    kernel_thread(usr_blk_dev_thread, NULL, 0);

    /* Set up the interrupt for detecting the media card. */
#ifndef CONFIG_ARCH_SCMA11
    set_GPIO_mode(USR_BLK_DEV_DEV1_INT_GPIO | GPIO_IN);
    set_GPIO_IRQ_edge(USR_BLK_DEV_DEV1_INT_GPIO, GPIO_FALLING_EDGE | GPIO_RISING_EDGE);
    if (request_irq(IRQ_GPIO(USR_BLK_DEV_DEV1_INT_GPIO), usr_blk_dev_attach_irq,
                    SA_INTERRUPT | SA_SAMPLE_RANDOM, "usr_blk_dev card detect", (void *)1))
    {
        printk(KERN_ERR "usr_blk_dev: Unable to initialize card detect interrupt.\n");
        return -1;
    }
#endif
    /* Initialize the debounce timer. */
    init_timer(&usr_blk_dev_timer);

    /* Start the timer to determine if a card is inserted or not. */
    usr_blk_dev_timer_poll((unsigned long)((1<<USR_BLK_DEV_NUM_DEVICES)-1));

    /* Register the block driver. */
    result = register_blkdev(USR_BLK_DEV_MAJOR_NUM, USR_BLK_DEV_DEVICE_NAME_STR,
                             (struct block_device_operations *)&usr_blk_dev_ops);
    if (result < 0)
    {
        printk(KERN_ERR "usr_blk_dev: Unable to get major number %d.\n", USR_BLK_DEV_MAJOR_NUM);
        return result;
    }
    
    /*
     * Set up the gendisk structure, to handle partitioning.
     */
    memset(usr_blk_dev_partitions, 0x00, sizeof(usr_blk_dev_partitions));
    memset(usr_blk_dev_sizes, 0x00, sizeof(usr_blk_dev_sizes));
    add_gendisk(&usr_blk_dev_gendisk);

    /* Please note blk_size is initialized to usr_blk_dev_blksizes in add_gendisk. */
    blksize_size[USR_BLK_DEV_MAJOR_NUM] = usr_blk_dev_blksizes;
    hardsect_size[USR_BLK_DEV_MAJOR_NUM] = usr_blk_dev_blksizes;
    read_ahead[USR_BLK_DEV_MAJOR_NUM] = 8;
    max_sectors[USR_BLK_DEV_MAJOR_NUM] = (int *)usr_blk_dev_max_sectors;

    tracemsg("Init done.\n");
    return 0;
}

/*!
 * @brief Destructor for the user block device.
 *
 * Deallocates and unregisters the devices which are used by the user
 * block device.  See the documentation for usr_blk_dev_init() for more
 * details on all of the devices registered or initialized.
 */
void __exit usr_blk_dev_exit(void)
{
    char proc_entry_str[USR_BLK_DEV_PROC_ENTRY_LEN+2] = USR_BLK_DEV_PROC_ENTRY_STR;
    unsigned int i;
    
    tracemsg("\n");
    /* Remove the entry from /proc. */
    proc_entry_str[USR_BLK_DEV_PROC_ENTRY_LEN+1] = '\0';
    for (i=0; i<USR_BLK_DEV_NUM_DEVICES; i++)
    {
        proc_entry_str[USR_BLK_DEV_PROC_ENTRY_LEN] = '0'+i;
        remove_proc_entry(proc_entry_str, NULL);
    }
    /* Stop the thread. */
    usr_blk_dev_send_cmd_to_thread(USR_BLK_DEV_CMD_ID_EXIT, 0, USR_BLK_DEV_CMD_SRC_IRQ);
    unregister_blkdev(USR_BLK_DEV_MAJOR_NUM, USR_BLK_DEV_DEVICE_NAME_STR);
    
    blksize_size[USR_BLK_DEV_MAJOR_NUM] = NULL;
    hardsect_size[USR_BLK_DEV_MAJOR_NUM] = NULL;
    max_sectors[USR_BLK_DEV_MAJOR_NUM] = NULL;
    del_gendisk(&usr_blk_dev_gendisk);
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS
module_init(usr_blk_dev_init);
module_exit(usr_blk_dev_exit);

MODULE_AUTHOR("Motorola Inc");
MODULE_DESCRIPTION("User mode block device driver");
MODULE_LICENSE("GPL");
#endif

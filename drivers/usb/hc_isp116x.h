/*
 * ISP116x HCD (Host Controller Driver) for USB.
 * 
 * (C) Copyright 2001 Roman Weissgaerber <weissg@vienna.at>
 * 
 *
 */

#define GET_FRAME_NUMBER(hci) 	READ_REG32 (hci, HcFmNumber)

/*
 * Maximum number of root hub ports.  
 */
#define MAX_ROOT_PORTS	15	/* maximum OHCI root hub ports */


/* control and status registers */
#define HcRevision		0x00
#define HcControl 		0x01
#define HcCommandStatus		0x02
#define HcInterruptStatus	0x03
#define HcInterruptEnable	0x04
#define HcInterruptDisable	0x05
#define HcFmInterval		0x0D
#define HcFmRemaining		0x0E
#define HcFmNumber		0x0F
#define HcLSThreshold		0x11
#define HcRhDescriptorA		0x12
#define HcRhDescriptorB		0x13
#define HcRhStatus		0x14
#define HcRhPortStatus		0x15

#define HcHardwareConfiguration 0x20
#define HcDMAConfiguration	0x21
#define HcTransferCounter	0x22
#define HcuPInterrupt		0x24
#define HcuPInterruptEnable	0x25
#define HcChipID		0x27
#define HcScratch		0x28
#define HcSoftwareReset		0x29
#define HcITLBufferLength	0x2A
#define HcATLBufferLength	0x2B
#define HcBufferStatus		0x2C
#define HcReadBackITL0Length	0x2D
#define HcReadBackITL1Length	0x2E
#define HcITLBufferPort		0x40
#define HcATLBufferPort		0x41

/* OHCI CONTROL AND STATUS REGISTER MASKS */

/*
 * HcControl (control) register masks
 */
#define OHCI_CTRL_HCFS	(3 << 6)	/* BUS state mask */
#define OHCI_CTRL_RWC	(1 << 9)	/* remote wakeup connected */
#define OHCI_CTRL_RWE	(1 << 10)	/* remote wakeup enable */

/* pre-shifted values for HCFS */
#define OHCI_USB_RESET	(0 << 6)
#define OHCI_USB_RESUME	(1 << 6)
#define OHCI_USB_OPER	(2 << 6)
#define OHCI_USB_SUSPEND	(3 << 6)

/*
 * HcCommandStatus (cmdstatus) register masks
 */
#define OHCI_HCR	(1 << 0)	/* host controller reset */
#define OHCI_SOC  	(3 << 16)	/* scheduling overrun count */

/*
 * masks used with interrupt registers:
 * HcInterruptStatus (intrstatus)
 * HcInterruptEnable (intrenable)
 * HcInterruptDisable (intrdisable)
 */
#define OHCI_INTR_SO	(1 << 0)	/* scheduling overrun */

#define OHCI_INTR_SF	(1 << 2)	/* start frame */
#define OHCI_INTR_RD	(1 << 3)	/* resume detect */
#define OHCI_INTR_UE	(1 << 4)	/* unrecoverable error */
#define OHCI_INTR_FNO	(1 << 5)	/* frame number overflow */
#define OHCI_INTR_RHSC	(1 << 6)	/* root hub status change */
#define OHCI_INTR_ATD	(1 << 7)	/* scheduling overrun */

#define OHCI_INTR_MIE	(1 << 31)	/* master interrupt enable */

/*
 * HcHardwareConfiguration
 */
#define InterruptPinEnable 	(1 << 0)
#define InterruptPinTrigger 	(1 << 1)
#define InterruptOutputPolarity	(1 << 2)
#define DataBusWidth16		(1 << 3)
#define DREQOutputPolarity	(1 << 5)
#define DACKInputPolarity	(1 << 6)
#define EOTInputPolarity	(1 << 7)
#define DACKMode		(1 << 8)
#define AnalogOCEnable		(1 << 10)
#define SuspendClkNotStop	(1 << 11)
#define DownstreamPort15KRSel	(1 << 12)

/* 
 * HcDMAConfiguration
 */
#define DMAReadWriteSelect 	(1 << 0)
#define ITL_ATL_DataSelect	(1 << 1)
#define DMACounterSelect	(1 << 2)
#define DMAEnable		(1 << 4)
#define BurstLen_1		0
#define BurstLen_4		(1 << 5)
#define BurstLen_8		(2 << 5)


/*
 * HcuPInterrupt
 */
#define SOFITLInt		(1 << 0)
#define ATLInt			(1 << 1)
#define AllEOTInterrupt		(1 << 2)
#define OPR_Reg			(1 << 4)
#define HCSuspended		(1 << 5)
#define ClkReady		(1 << 6)

/*
 * HcBufferStatus
 */
#define ITL0BufferFull		(1 << 0)
#define ITL1BufferFull		(1 << 1)
#define ATLBufferFull		(1 << 2)
#define ITL0BufferDone		(1 << 3)
#define ITL1BufferDone		(1 << 4)
#define ATLBufferDone		(1 << 5)



/* OHCI ROOT HUB REGISTER MASKS */
 
/* roothub.portstatus [i] bits */
#define RH_PS_CCS            0x00000001   	/* current connect status */
#define RH_PS_PES            0x00000002   	/* port enable status*/
#define RH_PS_PSS            0x00000004   	/* port suspend status */
#define RH_PS_POCI           0x00000008   	/* port over current indicator */
#define RH_PS_PRS            0x00000010  	/* port reset status */
#define RH_PS_PPS            0x00000100   	/* port power status */
#define RH_PS_LSDA           0x00000200    	/* low speed device attached */
#define RH_PS_CSC            0x00010000 	/* connect status change */
#define RH_PS_PESC           0x00020000   	/* port enable status change */
#define RH_PS_PSSC           0x00040000    	/* port suspend status change */
#define RH_PS_OCIC           0x00080000    	/* over current indicator change */
#define RH_PS_PRSC           0x00100000   	/* port reset status change */

/* roothub.status bits */
#define RH_HS_LPS	     0x00000001		/* local power status */
#define RH_HS_OCI	     0x00000002		/* over current indicator */
#define RH_HS_DRWE	     0x00008000		/* device remote wakeup enable */
#define RH_HS_LPSC	     0x00010000		/* local power status change */
#define RH_HS_OCIC	     0x00020000		/* over current indicator change */
#define RH_HS_CRWE	     0x80000000		/* clear remote wakeup enable */

/* roothub.b masks */
#define RH_B_DR		0x0000ffff		/* device removable flags */
#define RH_B_PPCM	0xffff0000		/* port power control mask */

/* roothub.a masks */
#define	RH_A_NDP	(0xff << 0)		/* number of downstream ports */
#define	RH_A_PSM	(1 << 8)		/* power switching mode */
#define	RH_A_NPS	(1 << 9)		/* no power switching */
#define	RH_A_DT		(1 << 10)		/* device type (mbz) */
#define	RH_A_OCPM	(1 << 11)		/* over current protection mode */
#define	RH_A_NOCP	(1 << 12)		/* no over current protection */
#define	RH_A_POTPGT	(0xff << 24)		/* power on to power good time */

#define URB_DEL 1


typedef struct hcipriv {
	int irq;
	int disabled;			/* e.g. got a UE, we're hung */
	atomic_t resume_count;		/* defending against multiple resumes */
	struct ohci_regs * regs;	/* OHCI controller's memory */
	unsigned long hcport;
	u8 *hcvirtual;

	int intrstatus;
	__u32 hc_control;		/* copy of the hc control reg */

	int frame;

	__u8 * tl;
	int tlp;
	int atl_len;
	int atl_buffer_len;
	int itl0_len;
	int itl1_len;
	int itl_buffer_len;
	int itl_index;
	int tl_last;
	int units_left;

} hcipriv_t;
struct hci;

static inline u32 READ_REG32 (struct hci * hci, int regindex);
static inline u16 READ_REG16 (struct hci * hci, int regindex);
static inline void READ_REGn16 (struct hci * hci, int regindex, int length, __u8 * buffer);
static inline void WRITE_REG32 (struct hci * hci, u32 value, int regindex);
static inline void WRITE_REG16 (struct hci * hci, u16 value, int regindex);
static inline void WRITE_REGn16 (struct hci * hci, int regindex, int length, __u8 * buffer);


/*
 * Following from old version of hc_simple.h
 */

/*-------------------------------------------------------------------------*/
/* list of all controllers using this driver 
 * */
 
static LIST_HEAD (hci_hcd_list);


/* URB states (urb_state) */ 
/* isoc, interrupt single state */

/* bulk transfer main state and 0-length packet */
#define US_BULK 	0	 
#define US_BULK0 	1
/* three setup states */
#define US_CTRL_SETUP 	2
#define US_CTRL_DATA	1
#define US_CTRL_ACK		0


/*-------------------------------------------------------------------------*/
/* HC private part of a device descriptor
 * */

#define NUM_EDS 32

struct hci_device {
	struct urb * pipe_head [NUM_EDS];
	struct list_head urb_queue [NUM_EDS];	/* Array of all Endpoint Descriptors */
	int urb_state [NUM_EDS];
};

/*-------------------------------------------------------------------------*/
/* Virtual Root HUB 
 * */

#define usb_to_hci(usb)	((struct hci_device *)(usb)->hcpriv)

struct virt_root_hub {
	int devnum; 		/* Address of Root Hub endpoint */ 
	void * urb;			/* interrupt URB of root hub */
	int send;			/* active flag */
 	int interval;		/* intervall of roothub interrupt transfers */
	struct timer_list rh_int_timer; /* intervall timer for rh interrupt EP */
};

/* USB HUB CONSTANTS (not OHCI-specific; see hub.h and USB spec) */
 
/* destination of request */
#define RH_INTERFACE               0x01
#define RH_ENDPOINT                0x02
#define RH_OTHER                   0x03

#define RH_CLASS                   0x20
#define RH_VENDOR                  0x40

/* Requests: bRequest << 8 | bmRequestType */
#define RH_GET_STATUS           0x0080
#define RH_CLEAR_FEATURE        0x0100
#define RH_SET_FEATURE          0x0300
#define RH_SET_ADDRESS			0x0500
#define RH_GET_DESCRIPTOR		0x0680
#define RH_SET_DESCRIPTOR       0x0700
#define RH_GET_CONFIGURATION	0x0880
#define RH_SET_CONFIGURATION	0x0900
#define RH_GET_STATE            0x0280
#define RH_GET_INTERFACE        0x0A80
#define RH_SET_INTERFACE        0x0B00
#define RH_SYNC_FRAME           0x0C80
/* Our Vendor Specific Request */
#define RH_SET_EP               0x2000


/* Hub port features */
#define RH_PORT_CONNECTION         0x00
#define RH_PORT_ENABLE             0x01
#define RH_PORT_SUSPEND            0x02
#define RH_PORT_OVER_CURRENT       0x03
#define RH_PORT_RESET              0x04
#define RH_PORT_POWER              0x08
#define RH_PORT_LOW_SPEED          0x09

#define RH_C_PORT_CONNECTION       0x10
#define RH_C_PORT_ENABLE           0x11
#define RH_C_PORT_SUSPEND          0x12
#define RH_C_PORT_OVER_CURRENT     0x13
#define RH_C_PORT_RESET            0x14  

/* Hub features */
#define RH_C_HUB_LOCAL_POWER       0x00
#define RH_C_HUB_OVER_CURRENT      0x01

#define RH_DEVICE_REMOTE_WAKEUP    0x00
#define RH_ENDPOINT_STALL          0x01

/*-------------------------------------------------------------------------*/
/* struct for each HC 
 * */

#define MAX_TRANS 32

typedef struct hci {
	struct virt_root_hub rh;		/* roothub */
	wait_queue_head_t waitq;		/* deletion of URBs and devices needs a waitqueue */
	int active;						/* HC is operating */
	
	struct list_head ctrl_list;		/* set of ctrl endpoints */
	struct list_head bulk_list;		/* set of bulk endpoints */
	struct list_head iso_list;		/* set of isoc endpoints */
	struct list_head intr_list;		/* ordered (tree) set of int endpoints */
	struct list_head del_list;		/* set of entpoints to be deleted */

	struct urb * urb_array [MAX_TRANS]; 
 	struct list_head hci_hcd_list;	/* list of all hci_hcd */
  	struct usb_bus * bus;    		/* our bus */

	int trans;						/* number of transactions pending */

	int frame_no;					/* frame # */
	hcipriv_t hp;			/* individual part of hc type */

} hci_t;


/*-------------------------------------------------------------------------*/
/* condition (error) CC codes and mapping OHCI like
 * */
 
#define TD_CC_NOERROR      0x00
#define TD_CC_CRC          0x01
#define TD_CC_BITSTUFFING  0x02
#define TD_CC_DATATOGGLEM  0x03
#define TD_CC_STALL        0x04
#define TD_DEVNOTRESP      0x05
#define TD_PIDCHECKFAIL    0x06
#define TD_UNEXPECTEDPID   0x07
#define TD_DATAOVERRUN     0x08
#define TD_DATAUNDERRUN    0x09
#define TD_BUFFEROVERRUN   0x0C
#define TD_BUFFERUNDERRUN  0x0D
#define TD_NOTACCESSED     0x0F

static int cc_to_error[16] = { 

/* mapping of the OHCI CC status to error codes */ 
	/* No  Error  */               USB_ST_NOERROR,
	/* CRC Error  */               USB_ST_CRC,
	/* Bit Stuff  */               USB_ST_BITSTUFF,
	/* Data Togg  */               USB_ST_CRC,
	/* Stall      */               USB_ST_STALL,
	/* DevNotResp */               USB_ST_NORESPONSE,
	/* PIDCheck   */               USB_ST_BITSTUFF,
	/* UnExpPID   */               USB_ST_BITSTUFF,
	/* DataOver   */               USB_ST_DATAOVERRUN,
	/* DataUnder  */               USB_ST_DATAUNDERRUN,
	/* reservd    */               USB_ST_NORESPONSE,
	/* reservd    */               USB_ST_NORESPONSE,
	/* BufferOver */               USB_ST_BUFFEROVERRUN,
	/* BuffUnder  */               USB_ST_BUFFERUNDERRUN,
	/* Not Access */               USB_ST_NORESPONSE,
	/* Not Access */               USB_ST_NORESPONSE 
};


/*-------------------------------------------------------------------------*/
/* PID - packet ID 
 * */
 
#define PID_SETUP	0
#define PID_OUT		1
#define PID_IN		2

/*-------------------------------------------------------------------------*/
/* misc 
 * */

// #define min(a,b) (((a)<(b))?(a):(b))  
// #define GET_FRAME_NUMBER(hci) (hci)->frame_no

/*-------------------------------------------------------------------------*/
/* functions
 * */
 
/* urb interface functions */
static int hci_get_current_frame_number (struct usb_device *usb_dev);
static int hci_unlink_urb (struct urb * urb);

static int qu_queue_urb (hci_t * hci, struct urb * urb);


/* root hub */
static int rh_init_int_timer (struct urb * urb);
static int rh_submit_urb (struct urb *urb);
static int rh_unlink_urb (struct urb *urb);

/* schedule functions */
static int sh_add_packet (hci_t * hci, struct urb * urb);

/* hc specific functions */
static inline int hc_parse_trans (hci_t * hci, int * actbytes, void ** data, 
			int * cc, int * toggle);
// static inline void start_trans (hci_t * hci);
static inline int hc_add_trans (hci_t * hci, int len, void * data,
		int toggle, int maxps, int slow, int endpoint, int address, 
		int pid, int format);


/* debug| print the main components of an URB     
 * small: 0) header + data packets 1) just header */

static void urb_print (struct urb * urb, char * str, int small)
{
	unsigned int pipe= urb->pipe;
	int i, len;
	
	if (!urb->dev || !urb->dev->bus) {
		dbg("%s URB: no dev", str);
		return;
	}
	
	printk("%s URB:[%4x] dev:%2d,ep:%2d-%c,type:%s,flags:%4x,len:%d/%d,stat:%d(%x)\n", 
			str,
		 	hci_get_current_frame_number (urb->dev), 
		 	usb_pipedevice (pipe),
		 	usb_pipeendpoint (pipe), 
		 	usb_pipeout (pipe)? 'O': 'I',
		 	usb_pipetype (pipe) < 2? (usb_pipeint (pipe)? "INTR": "ISOC"):
		 		(usb_pipecontrol (pipe)? "CTRL": "BULK"),
		 	urb->transfer_flags, 
		 	urb->actual_length, 
		 	urb->transfer_buffer_length,
		 	urb->status, urb->status);
	if (!small) {
		if (usb_pipecontrol (pipe)) {
			printk ( __FILE__ ": cmd(8):");
			for (i = 0; i < 8 ; i++) 
				printk (" %02x", ((__u8 *) urb->setup_packet) [i]);
			printk ("\n");
		}
		if (urb->transfer_buffer_length > 0 && urb->transfer_buffer) {
			printk ( __FILE__ ": data(%d/%d):", 
				urb->actual_length, 
				urb->transfer_buffer_length);
			len = usb_pipeout (pipe)? 
						urb->transfer_buffer_length: urb->actual_length;
			for (i = 0; i < 16 && i < len; i++) 
				printk (" %02x", ((__u8 *) urb->transfer_buffer) [i]);
			printk ("%s stat:%d\n", i < len? "...": "", urb->status);
		}
	} 
}


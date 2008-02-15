#define PFX MODULE_NAME

#define CAM_DEBUG
#undef CAM_VDEBUG

#ifdef CAM_DEBUG
#define	dbg(fmt, args...) printk(KERN_DEBUG "%s: " fmt, __func__, ## args)
#else
#define	dbg(fmt, args...)
#endif

#ifdef CAM_VDEBUG
#define ENTRY(x...) dbg("enter\n");
#define EXIT(x...)  dbg("exit\n");
#else
#define ENTRY(x...)
#define EXIT(x...)
#endif

#define err(format, arg...) printk(KERN_ERR PFX ": " format , ## arg)
#define info(format, arg...) printk(KERN_INFO PFX ": " format , ## arg)
#define warn(format, arg...) printk(KERN_WARNING PFX ": " format , ## arg)
#define emerg(format, arg...) printk(KERN_EMERG PFX ": " format , ## arg)

#define DUMP_BUF(buf,num) {\
	int i;\
	for (i=0; i < (num); i+=8) {\
		dbg("%02x: %02x %02x %02x %02x %02x %02x %02x %02x\n",\
		    i, (buf)[i+0], (buf)[i+1], (buf)[i+2], (buf)[i+3],\
		    (buf)[i+4], (buf)[i+5], (buf)[i+6], (buf)[i+7]);\
	}\
}

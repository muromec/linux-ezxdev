#ifndef _LINUX_KFI_H
#define _LINUX_KFI_H

#define KFI_DEBUG

typedef struct kfi_entry {
	void *va;            /* VA of instrumented function */
	void *call_site;     /* where this func was called */
	unsigned long time;  /* function entry time since trigger start time,
				in usec */
	unsigned long delta; /* delta time from entry to exit, in usec */
	int           pid;
} kfi_entry_t;

#define INTR_CONTEXT -1

typedef enum kfi_trigger_type {
	TRIGGER_NONE = 0,
	TRIGGER_TIME,
	TRIGGER_FUNC_ENTRY,
	TRIGGER_FUNC_EXIT,
	TRIGGER_PROC,
	TRIGGER_DEV,
	TRIGGER_LOG_FULL
} kfi_trigger_type_t;

typedef struct kfi_trigger {
	enum kfi_trigger_type type;
	union {
		unsigned long time; // time since boot, in usec
		void * func_addr;
	};
	unsigned long mark; // time at which this trigger occured
} kfi_trigger_t;

#define MAX_RUN_LOG_ENTRIES 2048
#define MAX_FUNC_LIST_ENTRIES 512

typedef struct kfi_filters {
	unsigned long min_delta;
	unsigned long max_delta;
	int no_ints;
	int only_ints;
	void** func_list;
	int func_list_size;
#ifdef KFI_DEBUG
	struct {
		int delta;
		int no_ints;
		int only_ints;
		int func_list;
	} cnt;
#endif
} kfi_filters_t;

typedef struct kfi_run {
	int triggered;
	int complete;
	struct kfi_trigger start_trigger;
	struct kfi_trigger stop_trigger;
	struct kfi_filters filters;
	struct kfi_entry* log;
	int num_entries;
	int next_entry;
	int id;
	struct kfi_run * next;

#ifdef KFI_DEBUG
	int notfound;
#endif
} kfi_run_t;

/* Use 'f' as magic number */
#define KFI_MAGIC  'i'

#define KFI_RESET     _IO  (KFI_MAGIC, 0)
#define KFI_NEW_RUN   _IOWR(KFI_MAGIC, 1, struct kfi_run)
#define KFI_START     _IOR (KFI_MAGIC, 2, int)
#define KFI_STOP      _IOR (KFI_MAGIC, 3, int)
#define KFI_READ      _IOWR(KFI_MAGIC, 4, struct kfi_run)
#define KFI_READ_CURR _IOWR(KFI_MAGIC, 5, struct kfi_run)
#define KFI_READ_TIMER _IOR (KFI_MAGIC, 6, unsigned long)

#endif // _LINUX_KFI_H

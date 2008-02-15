/*
 * linux/kernel/trace.c
 *
 * (C) Copyright 1999, 2000, 2001, 2002 - Karim Yaghmour (karym@opersys.com)
 *
 * This code is distributed under the GPL license
 *
 * Tracing management
 *
 */

#include <linux/config.h>
#include <linux/init.h>     /* For __init */
#include <linux/trace.h>    /* Tracing definitions */
#include <linux/errno.h>    /* Miscellaneous error codes */
#include <linux/stddef.h>   /* NULL */
#include <linux/slab.h>     /* kmalloc() */
#include <linux/module.h>   /* EXPORT_SYMBOL */
#include <linux/sched.h>    /* pid_t */
#ifdef CONFIG_ARM
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <asm/arch/irq.h>
#else
#include <linux/irq.h>      /* irq_desc */
#endif

/* Local variables */
static int        tracer_registered = 0;   /* Is there a tracer registered */
struct tracer *   tracer = NULL;           /* The registered tracer */

/* Registration lock */
rwlock_t tracer_register_lock = RW_LOCK_UNLOCKED;

/* Trace callback table entry */
struct trace_callback_table_entry
{
  tracer_call                         callback;    /* The callback function */

  struct trace_callback_table_entry*  next;        /* Next entry */
};

/* Trace callback table */
struct trace_callback_table_entry trace_callback_table[TRACE_EV_MAX];

/* Custom event description */
struct custom_event_desc
{
  /* The event itself */
  trace_new_event              event;

  /* PID of event owner, if any */
  pid_t                        owner_pid;

  /* List links */
  struct custom_event_desc*    next;
  struct custom_event_desc*    prev;
};

/* Next event ID to be used */
int next_event_id;

/* Circular list of custom events */
struct custom_event_desc  custom_events_head;
struct custom_event_desc* custom_events;

/* Circular list lock */
rwlock_t custom_list_lock = RW_LOCK_UNLOCKED;

/****************************************************
 * Register the tracer to the kernel
 * Return values :
 *   0, all is OK
 *   -EBUSY, there already is a registered tracer
 *   -ENOMEM, couldn't allocate memory
 ****************************************************/
int register_tracer(tracer_call pm_trace_function)
{
  unsigned long    l_flags;   /* Flags for irqsave */

  /* Is there a tracer already registered */
  if(tracer_registered == 1)
    return -EBUSY;

  /* Allocate memory for the tracer */
  if((tracer = (struct tracer *) kmalloc(sizeof(struct tracer), GFP_ATOMIC)) == NULL)
    /* We couldn't allocate any memory */
    return -ENOMEM;

  /* Lock registration variables */
  write_lock_irqsave(&tracer_register_lock, l_flags);

  /* There is a tracer registered */
  tracer_registered = 1;

  /* Set the tracer to the one being passed by the caller */
  tracer->trace = pm_trace_function;

  /* Unlock registration variables */
  write_unlock_irqrestore(&tracer_register_lock, l_flags);

  /* Initialize the tracer settings */
  tracer->fetch_syscall_eip_use_bounds = 0;
  tracer->fetch_syscall_eip_use_depth  = 0;

  /* Tell the caller that everything went fine */
  return 0;
}

/***************************************************
 * Unregister the currently registered tracer
 * Return values :
 *   0, all is OK
 *   -ENOMEDIUM, there isn't a registered tracer
 *   -ENXIO, unregestering wrong tracer
 ***************************************************/
int unregister_tracer(tracer_call pm_trace_function)
{
  unsigned long    l_flags;   /* Flags for irqsave */

  /* Is there a tracer already registered */
  if(tracer_registered == 0)
    /* Nothing to unregister */
    return -ENOMEDIUM;

  /* Lock registration variables */
  write_lock_irqsave(&tracer_register_lock, l_flags);

  /* Is it the tracer that was registered */
  if(tracer->trace == pm_trace_function)
    /* There isn't any tracer in here */
    tracer_registered = 0;
  else
    {
    /* Unlock registration variables */
    write_unlock_irqrestore(&tracer_register_lock, l_flags);

    /* We're done here */
    return -ENXIO;
    }

  /* Free the memory used by the tracing structure */
  kfree(tracer);
  tracer = NULL;

  /* Unlock registration variables */
  write_unlock_irqrestore(&tracer_register_lock, l_flags);

  /* Tell the caller that everything went OK */
  return 0;
}

/*******************************************************
 * Set the tracing configuration
 * Parameters :
 *   pm_trace_function, the trace function.
 *   pm_fetch_syscall_use_depth, Use depth to fetch eip
 *   pm_fetch_syscall_use_bounds, Use bounds to fetch eip
 *   pm_syscall_eip_depth, Detph to fetch eip
 *   pm_syscall_lower_bound, Lower bound eip address
 *   pm_syscall_upper_bound, Upper bound eip address
 * Return values : 
 *   0, all is OK 
 *   -ENOMEDIUM, there isn't a registered tracer
 *   -ENXIO, wrong tracer
 *   -EINVAL, invalid configuration
 *******************************************************/
int trace_set_config(tracer_call pm_trace_function,
		     int         pm_fetch_syscall_use_depth,
		     int         pm_fetch_syscall_use_bounds,
		     int         pm_syscall_eip_depth,
		     void*       pm_syscall_lower_bound,
		     void*       pm_syscall_upper_bound)
{
  /* Is there a tracer already registered */
  if(tracer_registered == 0)
    return -ENOMEDIUM;

  /* Is it the tracer that was registered */
  if(tracer->trace != pm_trace_function)
    return -ENXIO;

  /* Is this a valid configuration */
  if((pm_fetch_syscall_use_depth && pm_fetch_syscall_use_bounds)
   ||(pm_syscall_lower_bound > pm_syscall_upper_bound)
   ||(pm_syscall_eip_depth < 0))
    return -EINVAL;

  /* Set the configuration */
  tracer->fetch_syscall_eip_use_depth  = pm_fetch_syscall_use_depth;
  tracer->fetch_syscall_eip_use_bounds = pm_fetch_syscall_use_bounds;
  tracer->syscall_eip_depth = pm_syscall_eip_depth;
  tracer->syscall_lower_eip_bound = pm_syscall_lower_bound;
  tracer->syscall_upper_eip_bound = pm_syscall_upper_bound;

  /* Tell the caller that everything was OK */
  return 0;
}

/*******************************************************
 * Get the tracing configuration
 * Parameters :
 *   pm_fetch_syscall_use_depth, Use depth to fetch eip
 *   pm_fetch_syscall_use_bounds, Use bounds to fetch eip
 *   pm_syscall_eip_depth, Detph to fetch eip
 *   pm_syscall_lower_bound, Lower bound eip address
 *   pm_syscall_upper_bound, Upper bound eip address
 * Return values :
 *   0, all is OK 
 *   -ENOMEDIUM, there isn't a registered tracer
 *******************************************************/
int trace_get_config(int*        pm_fetch_syscall_use_depth,
		     int*        pm_fetch_syscall_use_bounds,
		     int*        pm_syscall_eip_depth,
		     void**      pm_syscall_lower_bound,
		     void**      pm_syscall_upper_bound)
{
  /* Is there a tracer already registered */
  if(tracer_registered == 0)
    return -ENOMEDIUM;

  /* Get the configuration */
  *pm_fetch_syscall_use_depth  = tracer->fetch_syscall_eip_use_depth;
  *pm_fetch_syscall_use_bounds = tracer->fetch_syscall_eip_use_bounds;
  *pm_syscall_eip_depth = tracer->syscall_eip_depth;
  *pm_syscall_lower_bound = tracer->syscall_lower_eip_bound;
  *pm_syscall_upper_bound = tracer->syscall_upper_eip_bound;

  /* Tell the caller that everything was OK */
  return 0;
}

/*******************************************************
 * Register a callback function to be called on occurence
 * of given event
 * Parameters :
 *   pm_trace_function, the callback function.
 *   pm_event_id, the event ID to be monitored.
 * Return values :
 *   0, all is OK
 *   -ENOMEM, unable to allocate memory for callback
 *******************************************************/
int trace_register_callback(tracer_call pm_trace_function,
			    uint8_t     pm_event_id)
{
  struct trace_callback_table_entry*  p_tct_entry;

  /* Search for an empty entry in the callback table */
  for(p_tct_entry = &(trace_callback_table[pm_event_id - 1]);
      p_tct_entry->next != NULL;
      p_tct_entry = p_tct_entry->next);

  /* Allocate a new callback */
  if((p_tct_entry->next = kmalloc(sizeof(struct trace_callback_table_entry), GFP_ATOMIC)) == NULL)
    return -ENOMEM;

  /* Setup the new callback */
  p_tct_entry->next->callback = pm_trace_function;
  p_tct_entry->next->next     = NULL;

  /* Tell the caller everything is ok */
  return 0;
}

/*******************************************************
 * UnRegister a callback function.
 * Parameters :
 *   pm_trace_function, the callback function.
 *   pm_event_id, the event ID that had to be monitored.
 * Return values :
 *   0, all is OK
 *   -ENOMEDIUM, no such callback resigtered
 *******************************************************/
int trace_unregister_callback(tracer_call pm_trace_function,
			      uint8_t     pm_event_id)
{
  struct trace_callback_table_entry*  p_tct_entry;  /* Pointer to trace callback table entry */
  struct trace_callback_table_entry*  p_temp_entry; /* Pointer to trace callback table entry */

  /* Search for the callback in the callback table */
  for(p_tct_entry = &(trace_callback_table[pm_event_id - 1]);
      ((p_tct_entry->next != NULL) && (p_tct_entry->next->callback != pm_trace_function));
      p_tct_entry = p_tct_entry->next);

  /* Did we find anything */
  if(p_tct_entry == NULL)
    return -ENOMEDIUM;

  /* Free the callback entry */
  p_temp_entry = p_tct_entry->next->next;
  kfree(p_tct_entry->next);
  p_tct_entry->next = p_temp_entry;

  /* Tell the caller everything is ok */
  return 0;
}

/*******************************************************
 * Create a new traceable event type
 * Parameters :
 *   pm_event_type, string describing event type
 *   pm_event_desc, string used for standard formatting
 *   pm_format_type, type of formatting used to log event
 *                 data
 *   pm_format_data, data specific to format
 *   pm_owner_pid, PID of event's owner (0 if none)
 * Return values :
 *   New Event ID if all is OK
 *   -ENOMEM, Unable to allocate new event
 *******************************************************/
int _trace_create_event(char*            pm_event_type,
			char*            pm_event_desc,
			int              pm_format_type,
			char*            pm_format_data,
			pid_t            pm_owner_pid)
{
  struct custom_event_desc* p_new_event;          /* Newly created event */

  /* Create event */
  if((p_new_event = (struct custom_event_desc*) kmalloc(sizeof(struct custom_event_desc), GFP_ATOMIC)) == NULL)
    return -ENOMEM;

  /* Initialize event properties */
  p_new_event->event.type[0] = '\0';
  p_new_event->event.desc[0] = '\0';
  p_new_event->event.form[0] = '\0';

  /* Set basic event properties */
  if(pm_event_type != NULL)
    strncpy(p_new_event->event.type, pm_event_type, CUSTOM_EVENT_TYPE_STR_LEN);
  if(pm_event_desc != NULL)
    strncpy(p_new_event->event.desc, pm_event_desc, CUSTOM_EVENT_DESC_STR_LEN);
  if(pm_format_data != NULL)
    strncpy(p_new_event->event.form, pm_format_data, CUSTOM_EVENT_FORM_STR_LEN);

  /* Ensure that strings are bound */
  p_new_event->event.type[CUSTOM_EVENT_TYPE_STR_LEN - 1] = '\0';
  p_new_event->event.desc[CUSTOM_EVENT_DESC_STR_LEN - 1] = '\0';
  p_new_event->event.form[CUSTOM_EVENT_FORM_STR_LEN - 1] = '\0';

  /* Set format type */
  p_new_event->event.format_type = pm_format_type;

  /* Give the new event a unique event ID */
  p_new_event->event.id = next_event_id;
  next_event_id++;

  /* Set event's owner */
  p_new_event->owner_pid = pm_owner_pid;

  /* Insert new event in event list */
  write_lock(&custom_list_lock);
  p_new_event->next = custom_events;
  p_new_event->prev = custom_events->prev;
  custom_events->prev->next = p_new_event;
  custom_events->prev = p_new_event;
  write_unlock(&custom_list_lock);

  /* Log the event creation event */
  trace_event(TRACE_EV_NEW_EVENT, &(p_new_event->event));

  /* Return new event ID */
  return p_new_event->event.id;
}
int trace_create_event(char*            pm_event_type,
		       char*            pm_event_desc,
		       int              pm_format_type,
		       char*            pm_format_data)
{
  return _trace_create_event(pm_event_type, pm_event_desc, pm_format_type, pm_format_data, 0);
}
int trace_create_owned_event(char*            pm_event_type,
			     char*            pm_event_desc,
			     int              pm_format_type,
			     char*            pm_format_data,
			     pid_t            pm_owner_pid)
{
  return _trace_create_event(pm_event_type, pm_event_desc, pm_format_type, pm_format_data, pm_owner_pid);
}

/*******************************************************
 * Destroy a created event type
 * Parameters :
 *   pm_event_id, the Id returned by trace_create_event()
 * Return values :
 *   NONE
 *******************************************************/
void trace_destroy_event(int pm_event_id)
{
  struct custom_event_desc*   p_event_desc;   /* Generic event description pointer */

  /* Lock the table for writting */
  write_lock(&custom_list_lock);

  /* Go through the event description list */
  for(p_event_desc = custom_events->next;
      p_event_desc != custom_events;
      p_event_desc = p_event_desc->next)
    if(p_event_desc->event.id == pm_event_id)
      break;

  /* If we found something */
  if(p_event_desc != custom_events)
    {
    /* Remove the event fromt the list */
    p_event_desc->next->prev = p_event_desc->prev;
    p_event_desc->prev->next = p_event_desc->next;

    /* Free the memory used by this event */
    kfree(p_event_desc);
    }

  /* Unlock the table for writting */
  write_unlock(&custom_list_lock);
}

/*******************************************************
 * Destroy an owner's events
 * Parameters :
 *   pm_owner_pid, the PID of the owner who's events are to
 *               be deleted.
 * Return values :
 *   NONE
 *******************************************************/
void trace_destroy_owners_events(pid_t pm_owner_pid)
{
  struct custom_event_desc*   p_temp_event;   /* Temporary event */
  struct custom_event_desc*   p_event_desc;   /* Generic event description pointer */

  /* Lock the table for writting */
  write_lock(&custom_list_lock);

  /* Start at the first event in the list */
  p_event_desc = custom_events->next;

  /* Go through the event description list */
  while(p_event_desc != custom_events)
    {
    /* Keep pointer to next event */
    p_temp_event = p_event_desc->next;

    /* Does this event belong to the same owner */
    if(p_event_desc->owner_pid == pm_owner_pid)
      {
      /* Remove the event fromt the list */
      p_event_desc->next->prev = p_event_desc->prev;
      p_event_desc->prev->next = p_event_desc->next;

      /* Free the memory used by this event */
      kfree(p_event_desc);
      }

    /* Go to next event */
    p_event_desc = p_temp_event;
    }

  /* Unlock the table for writting */
  write_unlock(&custom_list_lock);
}

/*******************************************************
 * Relog the declarations of custom events. This is
 * necessary to make sure that even though the event
 * creation might not have taken place during a trace,
 * that all custom events be part of all traces. Hence,
 * if a custom event occurs during a trace, we can be
 * sure that it's definition is part of the trace.
 * Parameters :
 *    NONE
 * Return values :
 *    NONE
 *******************************************************/
void trace_reregister_custom_events(void)
{
  struct custom_event_desc*   p_event_desc;   /* Generic event description pointer */

  /* Lock the table for reading */
  read_lock(&custom_list_lock);

  /* Go through the event description list */
  for(p_event_desc = custom_events->next;
      p_event_desc != custom_events;
      p_event_desc = p_event_desc->next)
    /* Log the event creation event */
    trace_event(TRACE_EV_NEW_EVENT, &(p_event_desc->event));

  /* Unlock the table for reading */
  read_unlock(&custom_list_lock);
}

/*******************************************************
 * Trace a formatted event
 * Parameters :
 *   pm_event_id, the event Id provided upon creation
 *   ..., printf-like data that will be used to fill the
 *        event string.
 * Return values :
 *   0, all is OK
 *   -ENOMEDIUM, there isn't a registered tracer or this
 *               event doesn't exist.
 *   -EBUSY, tracing hasn't started yet
 *******************************************************/
int trace_std_formatted_event(int pm_event_id, ...)
{
  int                         l_string_size;   /* Size of the string outputed by vsprintf() */
  char                        l_string[CUSTOM_EVENT_FINAL_STR_LEN];  /* Final formatted string */
  va_list                     l_var_arg_list;  /* Variable argument list */
  trace_custom                l_custom;        /* Custom event */
  struct custom_event_desc*   p_event_desc;    /* Generic event description pointer */

  /* Lock the table for reading */
  read_lock(&custom_list_lock);

  /* Go through the event description list */
  for(p_event_desc = custom_events->next;
      p_event_desc != custom_events;
      p_event_desc = p_event_desc->next)
    if(p_event_desc->event.id == pm_event_id)
      break;

  /* If we haven't found anything */
  if(p_event_desc == custom_events)
    {
    /* Unlock the table for reading */
    read_unlock(&custom_list_lock);

    /* No such thing */
    return -ENOMEDIUM;
    }

  /* Set custom event Id */
  l_custom.id = pm_event_id;

  /* Initialize variable argument list access */
  va_start(l_var_arg_list, pm_event_id);

  /* Print the description out to the temporary buffer */
  l_string_size = vsprintf(l_string, p_event_desc->event.desc, l_var_arg_list);

  /* Unlock the table for reading */
  read_unlock(&custom_list_lock);

  /* Facilitate return to caller */
  va_end(l_var_arg_list);

  /* Set the size of the event */
  l_custom.data_size = (uint32_t) (l_string_size + 1);

  /* Set the pointer to the event data */
  l_custom.data = l_string;

  /* Log the custom event */
  return trace_event(TRACE_EV_CUSTOM, &l_custom);
}

/*******************************************************
 * Trace a raw event
 * Parameters :
 *   pm_event_id, the event Id provided upon creation
 *   pm_event_size, the size of the data provided
 *   pm_event_data, data buffer describing event
 * Return values :
 *   0, all is OK
 *   -ENOMEDIUM, there isn't a registered tracer or this
 *               event doesn't exist.
 *   -EBUSY, tracing hasn't started yet
 *******************************************************/
int trace_raw_event(int pm_event_id, int pm_event_size, void* pm_event_data)
{
  trace_custom                l_custom;        /* Custom event */
  struct custom_event_desc*   p_event_desc;    /* Generic event description pointer */

  /* Lock the table for reading */
  read_lock(&custom_list_lock);

  /* Go through the event description list */
  for(p_event_desc = custom_events->next;
      p_event_desc != custom_events;
      p_event_desc = p_event_desc->next)
    if(p_event_desc->event.id == pm_event_id)
      break;

  /* Unlock the table for reading */
  read_unlock(&custom_list_lock);

  /* If we haven't found anything */
  if(p_event_desc == custom_events)
    /* No such thing */
    return -ENOMEDIUM;

  /* Set custom event Id */
  l_custom.id = pm_event_id;

  /* Set the data size */
  if(pm_event_size <= CUSTOM_EVENT_MAX_SIZE)
    l_custom.data_size = (uint32_t) pm_event_size;
  else
    l_custom.data_size = (uint32_t) CUSTOM_EVENT_MAX_SIZE;

  /* Set the pointer to the event data */
  l_custom.data = pm_event_data;

  /* Log the custom event */
  return trace_event(TRACE_EV_CUSTOM, &l_custom);
}

/*******************************************************
 * Trace an event
 * Parameters :
 *   pm_event_id, the event's ID (check out trace.h)
 *   pm_event_struct, the structure describing the event
 * Return values :
 *   0, all is OK
 *   -ENOMEDIUM, there isn't a registered tracer
 *   -EBUSY, tracing hasn't started yet
 *******************************************************/
int trace_event(uint8_t  pm_event_id, 
       	        void*    pm_event_struct)
{
  int                                 l_ret_value; /* The return value */
  struct trace_callback_table_entry*  p_tct_entry; /* Pointer to trace callback table entry */

  /* Lock registration variables */
  read_lock(&tracer_register_lock);

  /* Is there a tracer registered */
  if(tracer_registered != 1)
    l_ret_value = -ENOMEDIUM;
  else
    /* Call the tracer */
    l_ret_value = tracer->trace(pm_event_id, pm_event_struct);

  /* Unlock registration variables */
  read_unlock(&tracer_register_lock);

  /* Is this a native event */
  if(pm_event_id <= TRACE_EV_MAX)
    {
    /* Are there any callbacks to call */
    if(trace_callback_table[pm_event_id - 1].next != NULL)
      {
      /* Call all the callbacks linked to this event */
      for(p_tct_entry = trace_callback_table[pm_event_id - 1].next;
	  p_tct_entry != NULL;
	  p_tct_entry = p_tct_entry->next)
	p_tct_entry->callback(pm_event_id, pm_event_struct);
      }
    }

  /* Give the return value */
  return l_ret_value;
}

/*******************************************************
 * Initialize trace facility
 * Parameters :
 *    NONE
 * Return values :
 *    NONE
 *******************************************************/
static int __init trace_init(void)
{
  int i;  /* Generic index */

  /* Initialize callback table */
  for(i = 0; i < TRACE_EV_MAX; i++)
    {
    trace_callback_table[i].callback = NULL;
    trace_callback_table[i].next     = NULL;
    }

  /* Next event ID to be used */
  next_event_id = TRACE_EV_MAX + 1;

  /* Initialize custom events list */
  custom_events = &custom_events_head;
  custom_events->next = custom_events;
  custom_events->prev = custom_events;

  /* Everything is OK */
  return 0;
}

module_init(trace_init);

/* Export symbols so that can be visible from outside this file */
EXPORT_SYMBOL(register_tracer);
EXPORT_SYMBOL(unregister_tracer);
EXPORT_SYMBOL(trace_set_config);
EXPORT_SYMBOL(trace_get_config);
EXPORT_SYMBOL(trace_register_callback);
EXPORT_SYMBOL(trace_unregister_callback);
EXPORT_SYMBOL(trace_create_event);
EXPORT_SYMBOL(trace_create_owned_event);
EXPORT_SYMBOL(trace_destroy_event);
EXPORT_SYMBOL(trace_destroy_owners_events);
EXPORT_SYMBOL(trace_reregister_custom_events);
EXPORT_SYMBOL(trace_std_formatted_event);
EXPORT_SYMBOL(trace_raw_event);
EXPORT_SYMBOL(trace_event);
#if !defined(CONFIG_PPC) && !defined(__sh__)
EXPORT_SYMBOL(irq_desc);
#endif

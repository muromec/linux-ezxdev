/*
 * /vobs/ezx_linux/code/linux/linux-2.4.17/drivers/power_ic/module/timing.c
 * 
 * Description - This module contains code for performing timing 
 *               analysis on various parts of the driver.
 *
 * Copyright (C) 2005 - Motorola, Inc. All Rights Reserved.
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
 * 2005-Jun-22 - Design code to support timing analysis for the driver.
 */
 
 /*!
  * @file timing.c
  *
  * @ingroup poweric_debug
  *
  * @brief Power IC timing code
  *
  * This module contains code for performing timing analysis on various parts of the driver.
  * The module is not compiled by default, but is presented for use in developer builds
  * in order to aid performance analysis.
  *
  * Typical use of the timers is as follows:
  *
  *    - Add timing.o to the Makefile to build the timing code.
  *    - Include timing.h from the modules directory.
  *    - Create the timer with TIMING_NEW_TIMER().
  *    - Initialise the timer with timing_init().
  *    - To analyse the time taken for a section of code, make calls to 
  *      timing_start_timer() and timing_stop_timer() before and after the 
  *      section of code. The timer will remember the start and stop times 
  *      for up to the initialised number of times to store for later analysis.
  *    - Output information about the timers using timing_print_info() and 
  *      timing_print_rate_info(). Because this is expensive, this should only be 
  *      done from a non-critical area of the code where the time taken to 
  *      print the information does not matter.
  *
  * @note This module will not be built for any product build. The object is not included in 
  * any Makefile by default, and no references to the timing module should exist in the actual
  * product code. As a result, we will not be inspecting this code - the expectation is that
  * it can be build requested to the baseline at any time. However, it is the build requester's
  * responsibility to ensure that their changes work - if you break this, we'll break your 
  * fingers. 'nuff said.
  *
  * @note tracemsg support must be compiled in the driver in order for the print functions to 
  * do anything useful.
  */
 
/*==================================================================================================
                                        INCLUDE FILES
==================================================================================================*/

#include <asm/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/time.h>
#include <stdbool.h>

#include "../core/os_independent.h"
#include "timing.h"

/*==================================================================================================
                                     LOCAL CONSTANTS
==================================================================================================*/
/*! Microseconds in one second. usec field of a timeval should never usefully reach or 
 *  become larger than this. */
#define CARRY_USEC   1000000
/*==================================================================================================
                                        LOCAL MACROS
==================================================================================================*/
/*! Converts a timeval to microseconds. */
#define USECS(time_val)            (((time_val).tv_sec * CARRY_USEC) + ((time_val).tv_usec))
/*==================================================================================================
                          LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==================================================================================================*/

/*==================================================================================================
                                 LOCAL FUNCTION PROTOTYPES
==================================================================================================*/

static void
add(struct timeval * time1, struct timeval * time2, struct timeval * result);

static void
subtract(struct timeval * time1, struct timeval * time2, struct timeval * result);

static void
average(struct timeval * total, short num_times, struct timeval * average);

static int
rate_per_sec(struct timeval * period);

static int
greater_than(struct timeval * time1, struct timeval * time2);

static int
less_than(struct timeval * time1, struct timeval * time2);

static unsigned long
square_root(unsigned long num);

/*==================================================================================================
                                      LOCAL VARIABLES
==================================================================================================*/

/*==================================================================================================
                                     GLOBAL VARIABLES
==================================================================================================*/

/*==================================================================================================
                                     LOCAL FUNCTIONS
==================================================================================================*/

/*!
 * @brief Adds two times.
 *
 * This function adds time 1 to time 2, storing the result separately.
 *
 * @param     time1         Time to be added.
 * @param     time2         Time to be added.
 * @param     result        Result of addition.
 *
 * @return 0 if successful
 */
static void
add(struct timeval * time1, struct timeval * time2, struct timeval * result)
{
    /* Add up the two times' secsons and microseconds. */
    result->tv_usec = time1->tv_usec + time2->tv_usec;
    result->tv_sec = time1->tv_sec + time2->tv_sec;

    /* If the seconds ticked over and the resulting usec is below zero, we need
     * to steal 1 sec from the seconds and put into the usec field. This will also
     * deal with the case where many seconds' worth of usec was in one of the timevals,
     * leaving something a little more normal. */
    while(result->tv_usec >= CARRY_USEC)
    {
        result->tv_usec -= CARRY_USEC;
        result->tv_sec++;
    }
}

/*!
 * @brief Subtracts two times.
 *
 * This function subtracts time 1 from time 2, storing the result separately.
 *
 * @param     time1         Time to be subtracted
 * @param     time2         Time that time 1 will be subtracted from.
 * @param     result        Result of time2 - time 1.
 *
 * @return 0 if successful
 */
static void
subtract(struct timeval * time1, struct timeval * time2, struct timeval * result)
{                      
    /* Figure out the difference between the two times. */
    result->tv_usec = time2->tv_usec - time1->tv_usec;
    result->tv_sec = time2->tv_sec - time1->tv_sec;

    /* For the subtraction, it makes no real sense to leave usec less than zero Correct
     * this as necessary. This will also deal with the case where many seconds' worth of
     * usec was in one of the timevals leaving something a little more normal. */
    while(result->tv_usec < 0)
    {
        result->tv_sec--;
        result->tv_usec += CARRY_USEC;
    }   
}

/*!
 * @brief Finds the average.
 *
 * Given a total time andf the number of times summed, this finds the average
 *
 * @param     total         Total time to be averaged
 * @param     num_times     Number of times summed in total.
 * @param     average       The average time will be returned here.
 *
 * @return 0 if successful
 */
static void
average(struct timeval * total, short num_times, struct timeval * average)
{
    long average_usecs;
    
    /* Find the average. */
    average_usecs = USECS(*total) / num_times;
    average->tv_sec = average_usecs / CARRY_USEC;
    average->tv_usec = average_usecs % CARRY_USEC;
}

/*!
 * @brief Finds the rate at which something occurred.
 *
 * Given a period, this function determines the rate at which an event occurred.
 *
 * @param     period        Period for rate information
 *
 * @return rate corresponding to period, in events per second (rounded down).
 */
static int
rate_per_sec(struct timeval * period)
{
    return(CARRY_USEC / USECS(*period));
}

/*!
 * @brief Compares two timers.
 *
 * This function indicates whether time 1 is greater than time 2.
 *
 * @param     time1         Time to compare
 * @param     time2         Time that time1 will be compared against.
 *
 * @return 1 if time1 > time2, 0 otherwise.
 */
static int
greater_than(struct timeval * time1, struct timeval * time2)
{
    if(USECS(*time1) > USECS(*time2))
    {
        return 1;
    }
    
    return 0;
}

/*!
 * @brief Compares two timers.
 *
 * This function indicates whether time 1 is less than time 2.
 *
 * @param     time1         Time to compare
 * @param     time2         Time that time1 will be compared against.
 *
 * @return 1 if time1 < time2, 0 otherwise.
 */
static int
less_than(struct timeval * time1, struct timeval * time2)
{   
    if(USECS(*time1) < USECS(*time2))
    {
        return 1;
    }
    
    return 0;
}

/*!
 * @brief Compares two timers.
 *
 * Since math.h seems to be AWOL and we can't do floating-point maths, 
 * this will have to do. Adapted from source  at http://www.mprv.biz/iwb
 *
 * @param     num     Get the square root of this number.
 *
 * @return Guess at the square root of num.
 *
 * @note Because we can only do integer maths, the guess will oscillate between
 * the two integers on either side of the actual square root, one of which will be
 * returned. This is as good as it gets since we can't use floats or doubles.
 * 
 */
static unsigned long
square_root(unsigned long num)
{
    unsigned long guess = 1;	/* guess of sqrt(x) */
    unsigned long one_guess_ago = 0;
    unsigned long two_guesses_ago = 0;
    
    /* Repeatedly improve the guess of the root until we see the first oscillation. 
     * (i.e. the current guess is the same as we saw two iterations ago).*/
    do 
    {
        two_guesses_ago = one_guess_ago;
        one_guess_ago = guess;
        
        /* improving the guess */
        guess = (guess + num/guess) / 2;
    }
    while(guess != two_guesses_ago);

    return guess;
}


/*==================================================================================================
                                     GLOBAL FUNCTIONS
==================================================================================================*/

/*!
 * @brief Initialises a particular timer.
 *
 * This function initialises the passed timer.
 *
 * @param     timer   The timer to be initialised.
 *
 * @return 0 if successful
 */
void timing_init(TIMING_TIMER_T * timer, int times_to_store)
{
    if(timer != NULL)
    {
        if(times_to_store <= TIMING_MAX_TIMES_STORED) 
        {
            timer->num_times_to_store = times_to_store;
        }
        else
        {
            timer->num_times_to_store = TIMING_MAX_TIMES_STORED;
        }
        timer->num_times_recorded = 0;
        timer->timer_started = false;
    }
}

/*!
 * @brief Starts the timer.
 *
 * This function starts the timer if it isn't already running.
 *
 * @param     timer   The timer to be started.
 *
 * @return 0 if successful, -EPERM if the timer was already running or
 * no more times can be stored.
 */
int timing_start_timer(TIMING_TIMER_T * timer)
{
    int retval = -EPERM;
    
    if(timer != NULL)
    {
        if((timer->timer_started == false) && 
           (timer->num_times_recorded < timer->num_times_to_store))
        {
            do_gettimeofday(&(timer->times[timer->num_times_recorded].start_time));
            timer->timer_started = true;
            retval = 0;
        }
    }
    
    return retval;
}

/*!
 * @brief Stops the timer.
 *
 * This function stops the timer if has been previously started.
 *
 * @param     timer   The timer to be stopped.
 *
 * @return 0 if successful, -EPERM if the timer was not running.
 */
int timing_stop_timer(TIMING_TIMER_T * timer)
{
    int retval = -EPERM;
    
    if(timer != NULL)
    {
        /* We assume that if the timer has been started, it's OK to record a stop time. */
        if(timer->timer_started == true)
        {
            do_gettimeofday(&(timer->times[timer->num_times_recorded].stop_time));
            timer->num_times_recorded++;
            timer->timer_started = false;
            retval = 0;
        }
    }
    
    return retval;
}

/*!
 * @brief Displays info about the timer.
 *
 * This function prints a load of information about the captured timing information 
 * to the console.
 *
 * @param     timer     The timer for which info should be shown.
 * @param     show_all  After the average, etc. show all elapsed times, 1 per line. 
 *                      Set false if only a summary of the times is needed.
 *
 * @return 0 if successful
 */
void timing_print_info(TIMING_TIMER_T * timer, bool show_all)
{
    int i;
    struct timeval lowest_elapsed_time = {100000000, 0};
    int lowest_elapsed_time_index = 0;
    struct timeval highest_elapsed_time = {0, 0};
    int highest_elapsed_time_index = 0;
    
    unsigned long std_dev = 0;
    int elapsed_usec;
    int average_usec;
    
    struct timeval elapsed_time = {0, 0};
    struct timeval total_elapsed_time = {0, 0};
    struct timeval average_elapsed_time = {0, 0};
    struct timeval delta;
    
    if(timer != NULL)
    {
        if(timer->num_times_recorded == 0)
        {
            tracemsg(_a("\nTiming: no results captured."));  
            return;
        }
        
        tracemsg(_a("Timing: number of results captured: %d"), timer->num_times_recorded);
        
        /* Go figure out some info about the timers captured. */
        for(i = 0; i < timer->num_times_recorded; i++)
        {
            /* Get elapsed_time. */
            subtract(&(timer->times[i].start_time), &(timer->times[i].stop_time), &elapsed_time);
                                   
            /* Check elapsed time against highest. */
            if(greater_than(&elapsed_time, &highest_elapsed_time))
            {
                highest_elapsed_time.tv_sec = elapsed_time.tv_sec;
                highest_elapsed_time.tv_usec = elapsed_time.tv_usec;
                highest_elapsed_time_index = i;
            }
            
            /* Check elapsed time against lowest. */
            if(less_than(&elapsed_time, &lowest_elapsed_time))
            {
                lowest_elapsed_time.tv_sec = elapsed_time.tv_sec;
                lowest_elapsed_time.tv_usec = elapsed_time.tv_usec;
                lowest_elapsed_time_index = i;
            }
            
            /* Add this elapsed time to total. */
            add(&elapsed_time, &total_elapsed_time, &total_elapsed_time);
            
            if(show_all)
            {
                tracemsg(_a("      result %4d: elapsed time %2lds, %6ld usec"), 
                       i, elapsed_time.tv_sec, elapsed_time.tv_usec);
            }
        }
        
        /* Figure out the average elapsed time. */
        average(&total_elapsed_time, timer->num_times_recorded, &average_elapsed_time);
                               
        tracemsg(_a("Average elapsed %2lds, %6ld usec"), 
                       average_elapsed_time.tv_sec, average_elapsed_time.tv_usec);
                       
        /* Figure out standard deviation for the set of elapsed times. This needs to be
         * done after the average is calculated. */
        average_usec = USECS(average_elapsed_time);
        for(i = 0; i < timer->num_times_recorded; i++)
        {
            /* Get elapsed_time between the start and stop. */
            subtract(&(timer->times[i].start_time), &(timer->times[i].stop_time), &elapsed_time);
            elapsed_usec = USECS(elapsed_time);
            std_dev += ((elapsed_usec - average_usec) * (elapsed_usec - average_usec));
        }
    
        std_dev = std_dev / i; /* Rounding error shouldn't matter too much.*/
        std_dev = square_root(std_dev);
        tracemsg(_a(" Approx std dev      %6ld usec"), std_dev);
                       
        /* For highest time recorded, figure out delta between this and average. */
        subtract(&average_elapsed_time, &highest_elapsed_time, &delta);
         
        tracemsg(_a("Highest elapsed %2lds, %6ld usec (delta from avg %lds, %6ldus, result %d)"), 
                       highest_elapsed_time.tv_sec, highest_elapsed_time.tv_usec, 
                       delta.tv_sec, delta.tv_usec, 
                       highest_elapsed_time_index);
                       
        /* Same for lowest. */
        subtract(&lowest_elapsed_time, &average_elapsed_time, &delta);
                       
        tracemsg(_a(" Lowest elapsed %2lds, %6ld usec (delta from avg %lds, %6ldus, result %d)"), 
                       lowest_elapsed_time.tv_sec, lowest_elapsed_time.tv_usec, 
                       delta.tv_sec, delta.tv_usec, 
                       lowest_elapsed_time_index);
    }
}

/*!
 * @brief Displays rate info about the timer.
 *
 * This function prints a load of information about the rate of a timer's start times 
 * to the console.
 *
 * @param     timer     The timer for which info should be shown.
 * @param     show_all  After the average, etc. show all periods, 1 per line. Set false if
 *                      only a summary of the times is needed.
 *
 * @return 0 if successful
 */
void timing_print_rate_info(TIMING_TIMER_T * timer, bool show_all)
{
    int i;
    struct timeval lowest_elapsed_time = {100000000, 0};
    int lowest_elapsed_time_index = 0;
    struct timeval highest_elapsed_time = {0, 0};
    int highest_elapsed_time_index = 0;
    int rate;
    unsigned long std_dev = 0;
    int elapsed_usec;
    int average_usec;
    
    struct timeval elapsed_time = {0, 0};
    struct timeval total_elapsed_time = {0, 0};
    struct timeval average_elapsed_time = {0, 0};
    struct timeval delta;
    
    /* Since we are looking at the start times for rate information, we need two times stored
     * for one period, three times for two periods, etc. */
    int num_periods = timer->num_times_recorded - 1;
    
    /* There's no point in continuing if there is insufficient information available
     * to show period information. */
    if(num_periods <= 0)
    {
        tracemsg(_a("Rate info: %d timer start/stops is insufficient for rate analysis."), 
                    timer->num_times_recorded);
        return;
    }
    
    tracemsg(_a("Rate info: number of periods captured: %d"), num_periods);
    
    /* Go figure out some info about the timers captured. */
    for(i = 0; i < num_periods; i++)
    {
        /* Get elapsed_time between the two start times. */
        subtract(&(timer->times[i].start_time), &(timer->times[i+1].start_time), &elapsed_time);
                                   
        /* Check elapsed time against highest. */
        if(greater_than(&elapsed_time, &highest_elapsed_time))
        {
            highest_elapsed_time.tv_sec = elapsed_time.tv_sec;
            highest_elapsed_time.tv_usec = elapsed_time.tv_usec;
            highest_elapsed_time_index = i;
        }
            
        /* Check elapsed time against lowest. */
        if(less_than(&elapsed_time, &lowest_elapsed_time))
        {
            lowest_elapsed_time.tv_sec = elapsed_time.tv_sec;
            lowest_elapsed_time.tv_usec = elapsed_time.tv_usec;
            lowest_elapsed_time_index = i;
        }
            
        /* Add this elapsed time to total. */
        add(&elapsed_time, &total_elapsed_time, &total_elapsed_time);
            
        if(false)
        {
            tracemsg(_a("      result %4d: elapsed time %2lds, %6ld usec"), 
                         i, elapsed_time.tv_sec, elapsed_time.tv_usec);
        }
    }
            
    /* Figure out the average elapsed time. */
    average(&total_elapsed_time, timer->num_times_recorded, &average_elapsed_time);
    rate = rate_per_sec(&average_elapsed_time);
            
    tracemsg(_a(" Average period %2lds, %6ld usec (%d/sec)"), 
                average_elapsed_time.tv_sec, average_elapsed_time.tv_usec, rate);
                
    /* Figure out standard deviation for the set of periods. */
    average_usec = USECS(average_elapsed_time);
    for(i = 0; i < num_periods; i++)
    {
        /* Get elapsed_time between the two start times. */
        subtract(&(timer->times[i].start_time), &(timer->times[i+1].start_time), &elapsed_time);
        elapsed_usec = USECS(elapsed_time);
        std_dev += ((elapsed_usec - average_usec) * (elapsed_usec - average_usec));
    }
    
    std_dev = std_dev / i; /* Rounding error shouldn't matter too much.*/
    std_dev = square_root(std_dev);
    tracemsg(_a(" Approx std dev      %6ld usec"), std_dev);
                       
    /* For highest time recorded, figure out delta between this and average. */
    subtract(&average_elapsed_time, &highest_elapsed_time, &delta);
    rate = rate_per_sec(&highest_elapsed_time);
    tracemsg(_a(" Highest period %2lds, %6ld usec (%3d/sec, delta from avg %lds, %6ld us, result %d)"), 
                highest_elapsed_time.tv_sec, highest_elapsed_time.tv_usec, 
                rate, delta.tv_sec, delta.tv_usec, 
                highest_elapsed_time_index);
                       
    /* Same for lowest. */
    subtract(&lowest_elapsed_time, &average_elapsed_time, &delta);
    rate = rate_per_sec(&lowest_elapsed_time);          
    tracemsg(_a("  Lowest period %2lds, %6ld usec (%3d/sec, delta from avg %lds, %6ld us, result %d)"), 
                lowest_elapsed_time.tv_sec, lowest_elapsed_time.tv_usec, 
                rate, delta.tv_sec, delta.tv_usec, 
                lowest_elapsed_time_index);
    
               
    /* If told to show everything, dump all of the values to text in units of microseconds. 
     * Put commas before and after each value to make this easier to import into excel or
     * whatever as CSV as there's some sort of timestamp prepended to every printk these days... */ 
    if(show_all)
    {
        tracemsg(_a(""));
        tracemsg(_a(" All periods recorded (usecs):")); 
        for(i = 0; i < num_periods; i++)
        {
            subtract(&(timer->times[i].start_time), &(timer->times[i+1].start_time), &elapsed_time);
            tracemsg(",%ld,\n", USECS(elapsed_time));
        }
    }
        
} /* End of rate info. */

/*!
 * @brief Gets the number of results stored for a timer.
 *
 * This function returns the number of results available for a given timer.
 *
 * @param     timer   the timer for which info should be shown.
 *
 * @return 0 if successful
 */
int timing_get_num_times_stored(TIMING_TIMER_T * timer)
{
    return timer->num_times_recorded;
}

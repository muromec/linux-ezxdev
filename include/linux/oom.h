/*
 * FILE NAME include/linux/oom.h
 *
 * BRIEF MODULE DESCRIPTION
 *
 * Author: MontaVista Software, Inc.
 *         source@mvista.com
 *
 * Copyright 2004 MontaVista Software Inc.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef _LINUX_OOM_H
#define _LINUX_OOM_H

#define PROC_NAME_SIZE 80

typedef struct oom_procs {
        struct oom_procs *next;
	int len;
        char name[PROC_NAME_SIZE];
} oom_procs_t;

extern struct oom_procs oom_killable;
extern  struct oom_procs oom_dontkill;

extern int dontkill_read_proc(char *, char **, off_t, int, int *, void *);
extern int killable_read_proc(char *, char **, off_t, int, int *, void *);

extern ssize_t dontkill_write_proc(struct file *, const char *, size_t,
    loff_t *);
extern ssize_t killable_write_proc(struct file *, const char *, size_t,
    loff_t *);
#endif /* _LINUX_OOM_H */

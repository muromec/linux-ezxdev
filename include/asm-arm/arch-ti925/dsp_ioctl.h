/*
 * dsp_ioctl.h
 *
 * BRIEF MODULE DESCRIPTION
 *   Ioctl definitions for DSP control.
 *
 * Copyright (C) 2000 RidgeRun, Inc.
 * Author: RidgeRun, Inc.
 *
 */

#include <asm/ioctl.h>

#define DSP_MAGIC_NUMBER 'S'

#define DSP_ALLOCATE  _IO(DSP_MAGIC_NUMBER, 0)
#define DSP_RELEASE   _IO(DSP_MAGIC_NUMBER, 1)
#define DSP_USE_EHPI  _IO(DSP_MAGIC_NUMBER, 2)

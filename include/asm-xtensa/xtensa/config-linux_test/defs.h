/* Definitions for Xtensa instructions, types, and protos. */

/*
 * Customer ID=40; Build=11206; Copyright (c) 2002 by Tensilica Inc.  ALL RIGHTS RESERVED.
 * These coded instructions, statements, and computer programs are the
 * copyrighted works and confidential proprietary information of Tensilica Inc.
 * They may not be modified, copied, reproduced, distributed, or disclosed to
 * third parties in any manner, medium, or form, in whole or in part, without
 * the prior written consent of Tensilica Inc.
 */

/* Do not modify. This is automatically generated.*/

#ifndef _XTENSA_BASE_HEADER
#define _XTENSA_BASE_HEADER

#ifdef __XTENSA__
#if defined(__GNUC__) && !defined(__XCC__)

#define L8UI_ASM(arr, ars, imm) { \
  __asm__ volatile("l8ui %0, %1, %2" : "=a" (arr) : "a" (ars) , "i" (imm)); \
}

#define XT_L8UI(ars, imm) \
({ \
  unsigned char _arr; \
  const unsigned char *_ars = ars; \
  L8UI_ASM(_arr, _ars, imm); \
  _arr; \
})

#define L16UI_ASM(arr, ars, imm) { \
  __asm__ volatile("l16ui %0, %1, %2" : "=a" (arr) : "a" (ars) , "i" (imm)); \
}

#define XT_L16UI(ars, imm) \
({ \
  unsigned short _arr; \
  const unsigned short *_ars = ars; \
  L16UI_ASM(_arr, _ars, imm); \
  _arr; \
})

#define L16SI_ASM(arr, ars, imm) {\
  __asm__ volatile("l16si %0, %1, %2" : "=a" (arr) : "a" (ars) , "i" (imm)); \
}

#define XT_L16SI(ars, imm) \
({ \
  signed short _arr; \
  const signed short *_ars = ars; \
  L16SI_ASM(_arr, _ars, imm); \
  _arr; \
})

#define L32I_ASM(arr, ars, imm) { \
  __asm__ volatile("l32i %0, %1, %2" : "=a" (arr) : "a" (ars) , "i" (imm)); \
}

#define XT_L32I(ars, imm) \
({ \
  unsigned _arr; \
  const unsigned *_ars = ars; \
  L32I_ASM(_arr, _ars, imm); \
  _arr; \
})

#define S8I_ASM(arr, ars, imm) {\
  __asm__ volatile("s8i %0, %1, %2" : : "a" (arr), "a" (ars) , "i" (imm) : "memory" ); \
}

#define XT_S8I(arr, ars, imm) \
({ \
  signed char _arr = arr; \
  const signed char *_ars = ars; \
  S8I_ASM(_arr, _ars, imm); \
})

#define S16I_ASM(arr, ars, imm) {\
  __asm__ volatile("s16i %0, %1, %2" : : "a" (arr), "a" (ars) , "i" (imm) : "memory" ); \
}

#define XT_S16I(arr, ars, imm) \
({ \
  signed short _arr = arr; \
  const signed short *_ars = ars; \
  S16I_ASM(_arr, _ars, imm); \
})

#define S32I_ASM(arr, ars, imm) { \
  __asm__ volatile("s32i %0, %1, %2" : : "a" (arr), "a" (ars) , "i" (imm) : "memory" ); \
}

#define XT_S32I(arr, ars, imm) \
({ \
  signed int _arr = arr; \
  const signed int *_ars = ars; \
  S32I_ASM(_arr, _ars, imm); \
})

#define ADDI_ASM(art, ars, imm) {\
   __asm__ ("addi %0, %1, %2" : "=a" (art) : "a" (ars), "i" (imm)); \
}

#define XT_ADDI(ars, imm) \
({ \
   unsigned _art; \
   unsigned _ars = ars; \
   ADDI_ASM(_art, _ars, imm); \
   _art; \
})

#define ABS_ASM(arr, art) {\
   __asm__ ("abs %0, %1" : "=a" (arr) : "a" (art)); \
}

#define XT_ABS(art) \
({ \
   unsigned _arr; \
   signed _art = art; \
   ABS_ASM(_arr, _art); \
   _arr; \
})

/* Note: In the following macros that reference SAR, the magic "state"
   register is used to capture the dependency on SAR.  This is because
   SAR is a 5-bit register and thus there are no C types that can be
   used to represent it.  It doesn't appear that the SAR register is
   even relevant to GCC, but it is marked as "clobbered" just in
   case.  */

#define SRC_ASM(arr, ars, art) {\
   register int _xt_sar __asm__ ("state"); \
   __asm__ ("src %0, %1, %2" \
	    : "=a" (arr) : "a" (ars), "a" (art), "t" (_xt_sar)); \
}

#define XT_SRC(ars, art) \
({ \
   unsigned _arr; \
   unsigned _ars = ars; \
   unsigned _art = art; \
   SRC_ASM(_arr, _ars, _art); \
   _arr; \
})

#define SSR_ASM(ars) {\
   register int _xt_sar __asm__ ("state"); \
   __asm__ ("ssr %1" : "=t" (_xt_sar) : "a" (ars) : "sar"); \
}

#define XT_SSR(ars) \
({ \
   unsigned _ars = ars; \
   SSR_ASM(_ars); \
})

#define SSL_ASM(ars) {\
   register int _xt_sar __asm__ ("state"); \
   __asm__ ("ssl %1" : "=t" (_xt_sar) : "a" (ars) : "sar"); \
}

#define XT_SSL(ars) \
({ \
   unsigned _ars = ars; \
   SSL_ASM(_ars); \
})

#define SSA8B_ASM(ars) {\
   register int _xt_sar __asm__ ("state"); \
   __asm__ ("ssa8b %1" : "=t" (_xt_sar) : "a" (ars) : "sar"); \
}

#define XT_SSA8B(ars) \
({ \
   unsigned _ars = ars; \
   SSA8B_ASM(_ars); \
})

#define SSA8L_ASM(ars) {\
   register int _xt_sar __asm__ ("state"); \
   __asm__ ("ssa8l %1" : "=t" (_xt_sar) : "a" (ars) : "sar"); \
}

#define XT_SSA8L(ars) \
({ \
   unsigned _ars = ars; \
   SSA8L_ASM(_ars); \
})

#define SSAI_ASM(imm) {\
   register int _xt_sar __asm__ ("state"); \
   __asm__ ("ssai %1" : "=t" (_xt_sar) : "i" (imm) : "sar"); \
}

#define XT_SSAI(imm) \
({ \
   SSAI_ASM(imm); \
})



#define SEXT_ASM(arr, ars, tp7) {\
  __asm__ ("sext %0, %1, %2" : "=a" (arr) : "a" (ars) , "i" (tp7)); \
}

#define XT_SEXT(ars, tp7) \
({ \
  int _arr; \
  SEXT_ASM(_arr, ars, tp7); \
  _arr; \
})



#define CLAMPS_ASM(arr, ars, tp7) {\
  __asm__ ("clamps %0, %1, %2" : "=a" (arr) : "a" (ars) , "i" (tp7)); \
}

#define XT_CLAMPS(ars, tp7) \
({ \
  int _arr; \
  CLAMPS_ASM(_arr, ars, tp7); \
  _arr; \
})



#define MIN_ASM(arr, ars, art) {\
  __asm__ ("min %0, %1, %2" : "=a" (arr) : "a" (ars) , "a" (art)); \
}

#define XT_MIN(ars, art) \
({ \
  int _arr; \
  int _ars = ars; \
  int _art = art; \
  MIN_ASM(_arr, _ars, _art); \
  _arr; \
})

#define MAX_ASM(arr, ars, art) {\
  __asm__ ("max %0, %1, %2" : "=a" (arr) : "a" (ars) , "a" (art)); \
}

#define XT_MAX(ars, art) \
({ \
  int _arr; \
  int _ars = ars; \
  int _art = art; \
  MAX_ASM(_arr, _ars, _art); \
  _arr; \
})

#define MINU_ASM(arr, ars, art) {\
  __asm__ ("minu %0, %1, %2" : "=a" (arr) : "a" (ars) , "a" (art)); \
}

#define XT_MINU(ars, art) \
({ \
  unsigned _arr; \
  unsigned _ars = ars; \
  unsigned _art = art; \
  MINU_ASM(_arr, _ars, _art); \
  _arr; \
})

#define MAXU_ASM(arr, ars, art) {\
  __asm__ ("maxu %0, %1, %2" : "=a" (arr) : "a" (ars) , "a" (art)); \
}

#define XT_MAXU(ars, art) \
({ \
  unsigned _arr; \
  unsigned _ars = ars; \
  unsigned _art = art; \
  MAXU_ASM(_arr, _ars, _art); \
  _arr; \
})




#define NSA_ASM(arr, ars) {\
  __asm__ ("nsa %0, %1" : "=a" (arr) : "a" (ars)); \
}

#define XT_NSA(ars) \
({ \
  unsigned _arr; \
  int _ars = ars; \
  NSA_ASM(_arr, _ars); \
  _arr; \
})

#define NSAU_ASM(arr, ars) {\
  __asm__ ("nsau %0, %1" : "=a" (arr) : "a" (ars)); \
}

#define XT_NSAU(ars) \
({ \
  unsigned _arr; \
  unsigned _ars = ars; \
  NSAU_ASM(_arr, _ars); \
  _arr; \
})


typedef int xtbool __attribute__ ((coprocessor (0)));
typedef int xtbool2 __attribute__ ((coprocessor (1)));
typedef int xtbool4 __attribute__ ((coprocessor (2)));
typedef int xtbool8 __attribute__ ((coprocessor (3)));
typedef int xtbool16 __attribute__ ((coprocessor (4)));

#define ANDB_ASM(br, bs, bt) { \
    __asm__ ("andb	%0,%1,%2" : "=b" (br) : "b" (bs), "b" (bt)); \
}

#define XT_ANDB(bs, bt) ({ \
    xtbool _br; \
    xtbool _bs = bs; \
    xtbool _bt = bt; \
    ANDB_ASM(_br, _bs, _bt); \
    _br; \
})

#define ANDBC_ASM(br, bs, bt) { \
    __asm__ ("andbc	%0,%1,%2" : "=b" (br) : "b" (bs), "b" (bt)); \
}

#define XT_ANDBC(bs, bt) ({ \
    xtbool _br; \
    xtbool _bs = bs; \
    xtbool _bt = bt; \
    ANDBC_ASM(_br, _bs, _bt); \
    _br; \
})

#define ORB_ASM(br, bs, bt) { \
    __asm__ ("orb	%0,%1,%2" : "=b" (br) : "b" (bs), "b" (bt)); \
}

#define XT_ORB(bs, bt) ({ \
    xtbool _br; \
    xtbool _bs = bs; \
    xtbool _bt = bt; \
    ORB_ASM(_br, _bs, _bt); \
    _br; \
})

#define ORBC_ASM(br, bs, bt) { \
    __asm__ ("orbc	%0,%1,%2" : "=b" (br) : "b" (bs), "b" (bt)); \
}

#define XT_ORBC(bs, bt) ({ \
    xtbool _br; \
    xtbool _bs = bs; \
    xtbool _bt = bt; \
    ORBC_ASM(_br, _bs, _bt); \
    _br; \
})

#define XORB_ASM(br, bs, bt) { \
    __asm__ ("xorb	%0,%1,%2" : "=b" (br) : "b" (bs), "b" (bt)); \
}

#define XT_XORB(bs, bt) ({ \
    xtbool _br; \
    xtbool _bs = bs; \
    xtbool _bt = bt; \
    XORB_ASM(_br, _bs, _bt); \
    _br; \
})

#define ANY4_ASM(bt, bs4) { \
    __asm__ ("any4	%0,%1" : "=b" (bt) : "b" (bs4)); \
}

#define XT_ANY4(bs4) ({ \
    xtbool  _bt; \
    xtbool4 _bs4 = bs4; \
    ANY4_ASM(_bt, _bs4); \
    _bt; \
})

#define ALL4_ASM(bt, bs4) { \
    __asm__ ("all4	%0,%1" : "=b" (bt) : "b" (bs4)); \
}

#define XT_ALL4(bs4) ({ \
    xtbool  _bt; \
    xtbool4 _bs4 = bs4; \
    ALL4_ASM(_bt, _bs4); \
    _bt; \
})

#define ANY8_ASM(bt, bs8) { \
    __asm__ ("any8	%0,%1" : "=b" (bt) : "b" (bs8)); \
}

#define XT_ANY8(bs8) ({ \
    xtbool  _bt; \
    xtbool8 _bs8 = bs8; \
    ANY8_ASM(_bt, _bs8); \
    _bt; \
})

#define ALL8_ASM(bt, bs8) { \
    __asm__ ("all8	%0,%1" : "=b" (bt) : "b" (bs8)); \
}

#define XT_ALL8(bs8) ({ \
    xtbool  _bt; \
    xtbool8 _bs8 = bs8; \
    ALL8_ASM(_bt, _bs8); \
    _bt; \
})

#endif /* __GNUC__ && !__XCC__ */

#ifdef __XCC__

/* Core load/store instructions */
extern unsigned char _TIE_L8UI(const unsigned char * ars, immediate imm);
extern unsigned short _TIE_L16UI(const unsigned short * ars, immediate imm);
extern signed short _TIE_L16SI(const signed short * ars, immediate imm);
extern unsigned _TIE_L32I(const unsigned * ars, immediate imm);
extern void _TIE_S8I(unsigned char arr, unsigned char * ars, immediate imm);
extern void _TIE_S16I(unsigned short arr, unsigned short * ars, immediate imm);
extern void _TIE_S32I(unsigned arr, unsigned * ars, immediate imm);

#define XT_L8UI  _TIE_L8UI
#define XT_L16UI _TIE_L16UI
#define XT_L16SI _TIE_L16SI
#define XT_L32I  _TIE_L32I
#define XT_S8I   _TIE_S8I
#define XT_S16I  _TIE_S16I
#define XT_S32I  _TIE_S32I

/* Add-immediate instruction */
extern unsigned _TIE_ADDI(unsigned ars, immediate imm);
#define XT_ADDI  _TIE_ADDI

/* Absolute value instruction */
extern unsigned _TIE_ABS(int art);
#define XT_ABS _TIE_ABS

/* funnel shift instructions */
extern unsigned _TIE_SRC(unsigned ars, unsigned art);
#define XT_SRC _TIE_SRC
extern void _TIE_SSR(unsigned ars);
#define XT_SSR _TIE_SSR
extern void _TIE_SSL(unsigned ars);
#define XT_SSL _TIE_SSL
extern void _TIE_SSA8B(unsigned ars);
#define XT_SSA8B _TIE_SSA8B
extern void _TIE_SSA8L(unsigned ars);
#define XT_SSA8L _TIE_SSA8L
extern void _TIE_SSAI(immediate imm);
#define XT_SSAI _TIE_SSAI

/* Miscellaneous instructions */
extern int _TIE_SEXT(unsigned ars, immediate tp7);
#define XT_SEXT _TIE_SEXT
extern int _TIE_CLAMPS(int ars, immediate tp7);
#define XT_CLAMPS _TIE_CLAMPS
extern int _TIE_MIN(int ars, int art);
extern int _TIE_MAX(int ars, int art);
extern unsigned _TIE_MINU(unsigned ars, unsigned art);
extern unsigned _TIE_MAXU(unsigned ars, unsigned art);
#define XT_MIN _TIE_MIN
#define XT_MAX _TIE_MAX
#define XT_MINU _TIE_MINU
#define XT_MAXU _TIE_MAXU
extern unsigned _TIE_NSA(int ars);
extern unsigned _TIE_NSAU(unsigned ars);
#define XT_NSA _TIE_NSA
#define XT_NSAU _TIE_NSAU

/* Boolean registers and related instructions */
typedef _TIE_xtbool xtbool;
typedef _TIE_xtbool2 xtbool2;
typedef _TIE_xtbool4 xtbool4;
typedef _TIE_xtbool8 xtbool8;
typedef _TIE_xtbool16 xtbool16;

extern xtbool _TIE_ANDB(xtbool bs, xtbool bt);
extern xtbool _TIE_ANDBC(xtbool bs, xtbool bt);
extern xtbool _TIE_ORB(xtbool bs, xtbool bt);
extern xtbool _TIE_ORBC(xtbool bs, xtbool bt);
extern xtbool _TIE_XORB(xtbool bs, xtbool bt);
extern xtbool _TIE_ANY4(xtbool4 bs4);
extern xtbool _TIE_ALL4(xtbool4 bs4);
extern xtbool _TIE_ANY8(xtbool8 bs8);
extern xtbool _TIE_ALL8(xtbool8 bs8);

#define XT_ANDB  _TIE_ANDB
#define XT_ANDBC _TIE_ANDBC 
#define XT_ORB   _TIE_ORB 
#define XT_ORBC  _TIE_ORBC 
#define XT_XORB  _TIE_XORB 
#define XT_ANY4  _TIE_ANY4 
#define XT_ALL4  _TIE_ALL4 
#define XT_ANY8  _TIE_ANY8 
#define XT_ALL8  _TIE_ALL8 

#endif /* __XCC__ */

#endif /* __XTENSA__ */
#endif /* !_XTENSA_BASE_HEADER */

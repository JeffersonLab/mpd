#ifndef __MPDLIB__
/******************************************************************************
*
*  mpdLib.h  
*             - Driver library header file for the MultiPurpose
*             Digitizer (MPD) using a VxWorks 5.5 (PPC) or Linux
*             2.6.18 (Intel) or later based single board computer.
*
*  Author: Bryan Moffit
*          Jefferson Lab Data Acquisition Group
*          November 2014
*
*/

#define __MPDLIB__

#define MPD_BOARD_ID       0xmpd0000
#define MPD_MAX_BOARDS             20
#define MPD_MAX_A32_MEM      0x800000   /* 8 Meg */
#define MPD_MAX_A32MB_SIZE   0x800000  /*  8 MB */
#define MPD_VME_INT_LEVEL           3     
#define MPD_VME_INT_VEC          0xMPD

#define MPD_SUPPORTED_FIRMWARE 0x23


struct mpd_struct 
{
  /* 0x0000 */ volatile unsigned int version;
};

/* Function Prototypes */
STATUS mpdInit (UINT32 addr, UINT32 addr_inc, int nadc, int iFlag);

#endif /* __MPDLIB__ */

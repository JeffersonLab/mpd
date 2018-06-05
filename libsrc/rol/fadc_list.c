/*************************************************************************
 *
 *  mpd_list.c - Library of routines for readout and buffering of 
 *                events using a JLAB Trigger Interface V3 (TI) with 
 *                a Linux VME controller.
 *
 */

/* Event Buffer definitions */
/* Default block level */
unsigned int BLOCKLEVEL=1;
#define BUFFERLEVEL 1
#define MAX_EVENT_LENGTH   32768*12*BLOCKLEVEL      /* Size in Bytes */ 
#define SSP_MAX_EVENT_LENGTH 32000*12*BLOCKLEVEL     //SSP block length
#define MAX_EVENT_POOL     50


/* Define Interrupt source and address */
#define TI_MASTER
#define TI_READOUT TI_READOUT_EXT_POLL  /* Poll for available data, external triggers */
#define TI_ADDR    (21<<19)          /* GEO slot 20 */
/* Decision on whether or not to readout the TI for each block 
   - Comment out to disable readout 
*/
#define TI_DATA_READOUT

#define FIBER_LATENCY_OFFSET 0x4A  /* measured longest fiber length */
#define TDC_ID 0
#define MAX_TDC_DATA 34

#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#define UINT32 unsigned int
#define STATUS int
#include "tiprimary_list.c" /* source required for CODA */
#include <mpdLib.h>
#include <mpdConfig.h>
#include "sspLib.h"
#include "c775Lib.h"
#include "fadcLib.h"         /* Header for FADC250 library */
#include "sdLib.h"  
#include "dmaBankTools.h"

/* FADC250 Global Definitions */
int faMode=1;
#define FADC_WINDOW_LAT      0  /* Trigger Window Latency */
#define FADC_WINDOW_WIDTH     50  /* Trigger Window Width */
#define FADC_DAC_LEVEL       3300 /* Internal DAC Level */
#define FADC_THRESHOLD       0x10 /* Threshold for data readout */
unsigned int fadcSlotMask   = 0;    /* bit=slot (starting from 0) */
extern   int fadcA32Base;           /* This will need to be reset from it's default
                                     * so that it does not overlap with the TID */
extern unsigned int sspA32Base;

extern   int nfadc;                 /* Number of FADC250s verified with the library */
extern   int fadcID[FA_MAX_BOARDS]; /* Array of slot numbers, discovered by the library */
int NFADC;                          /* The Maximum number of tries the library will
                                     * use before giving up finding FADC250s */
int FA_SLOT;                        /* We'll use this over and over again to provide
				     * us access to the current FADC slot number */ 
/* for the calculation of maximum data words in the block transfer */
unsigned int MAXFADCWORDS = 0;
unsigned int MAXTIWORDS  = 0;








/* Redefine tsCrate according to TI_MASTER or TI_SLAVE */
#ifdef TI_SLAVE
int tsCrate=0;
#else
#ifdef TI_MASTER
int tsCrate=1;
#endif
#endif

/* function prototype */
void rocTrigger(int arg);

/****************************************
 *  DOWNLOAD
 ****************************************/
void
rocDownload()
{
 remexSetCmsgServer("sbs1"); // Set this to the platform's host
  remexSetRedirect(1);
  remexInit("Thunder",1);
  
  printf("%s: Build date/time %s/%s\n", __func__, __DATE__, __TIME__);

  /* Setup Address and data modes for DMA transfers
   *   
   *  vmeDmaConfig(addrType, dataType, sstMode);
   *
   *  addrType = 0 (A16)    1 (A24)    2 (A32)
   *  dataType = 0 (D16)    1 (D32)    2 (BLK32) 3 (MBLK) 4 (2eVME) 5 (2eSST)
   *  sstMode  = 0 (SST160) 1 (SST267) 2 (SST320)
   */
  vmeDmaConfig(2,5,1); 






  /*****************
   *   TI SETUP
   *****************/
  int overall_offset=0x80;

#ifndef TI_DATA_READOUT
  /* Disable data readout */
  tiDisableDataReadout();
  /* Disable A32... where that data would have been stored on the TI */
  tiDisableA32();
#endif

  /* Set crate ID */
  tiSetCrateID(0x01); /* ROC 1 */

  tiSetTriggerSourceMask(TI_TRIGSRC_TSINPUTS | TI_TRIGSRC_PULSER | 
			 TI_TRIGSRC_LOOPBACK | TI_TRIGSRC_VME);
  /*   tiSetTriggerSource(TI_TRIGGER_TSINPUTS); */

  /* Set needed TS input bits */
  tiEnableTSInput( TI_TSINPUT_1 );

  /* Load the trigger table that associates 
     pins 21/22 | 23/24 | 25/26 : trigger1
     pins 29/30 | 31/32 | 33/34 : trigger2
  */
  tiLoadTriggerTable(0);
  // MPD 1 sample readout = 3.525 us = 8 * 480
  tiSetTriggerHoldoff(1,30,1); /* 1 trigger in 20*480ns window */
  //tiSetTriggerHoldoff(1,60,1); /* 1 trigger in 20*480ns window */
  tiSetTriggerHoldoff(2,0,0);  /* 2 trigger in don't care window */

  tiSetTriggerHoldoff(3,0,0);  /* 3 trigger in don't care window */
  tiSetTriggerHoldoff(4,20,1);  /* 4 trigger in 20*3840ns window */
  // tiSetTriggerHoldoff(4,1,1);  /* 4 trigger in 20*3840ns window */

/*   /\* Set the sync delay width to 0x40*32 = 2.048us *\/ */
  tiSetSyncDelayWidth(0x54, 0x40, 1);

  tiSetTriggerPulse(1,0,25,0);

  /* Set the busy source to non-default value (no Switch Slot B busy) */
  tiSetBusySource(TI_BUSY_LOOPBACK,1);

/*   tiSetFiberDelay(10,0xcf); */

#ifdef TI_MASTER
  /* Set number of events per block */
  tiSetBlockLevel(BLOCKLEVEL);
#endif

  tiSetEventFormat(1);

  tiSetBlockBufferLevel(BUFFERLEVEL);

  tiSetOutputPort(1,1,0,0);

  tiStatus(0);


  
 


  /*****************
   *   TDC SETUP
   *****************/

  c775Init(0x600000,0,1,0); /* A24 = 0x11, A32 = 0x0A11 */



 /***************************************
   * FADC Setup 
   ***************************************/
  /* Here, we assume that the addresses of each board were set according to their
   * geographical address (slot number):
   * Slot  3:  (3<<19) = 0x180000
   * Slot  4:  (4<<19) = 0x200000
   * ...
   * Slot 20: (20<<19) = 0xA00000
   */
  int islot;

  NFADC = 2;   /* 3 slots  */
  fadcA32Base=0x09000000;

  /* Setup the iFlag.. flags for FADC initialization */
  int iFlag=0;
  /* Sync Source */
  iFlag |= (1<<0);    /* VXS */
  /* Trigger Source */
  iFlag |= (1<<2);    /* VXS */
  /* Clock Source */
  iFlag |= (0<<5);    /* Self */
  iFlag |= FA_INIT_SKIP_FIRMWARE_CHECK; /* no check of FADC firmware for HPS version */
  vmeSetQuietFlag(1); /* skip the errors associated with BUS Errors */
  faInit((unsigned int)(17<<19),(1<<19),NFADC,iFlag);
  NFADC=nfadc;        /* Redefine our NFADC with what was found from the driver */
  vmeSetQuietFlag(0); /* Turn the error statements back on */
  
  /* Calculate the maximum number of words per block transfer (assuming Pulse mode)
   *   MAX = NFADC * BLOCKLEVEL * (EvHeader + TrigTime*2 + Pulse*2*chan) 
   *         + 2*32 (words for byte alignment) 
   */
  if(faMode = 1) /* Raw window Mode */
    //MAXFADCWORDS = NFADC * BLOCKLEVEL * (1+2+FADC_WINDOW_WIDTH*16) + 3;
    MAXFADCWORDS = NFADC * BLOCKLEVEL * (1+1+2+(FADC_WINDOW_WIDTH*16)) + 3;
  else /* Pulse mode */
    MAXFADCWORDS = NFADC * BLOCKLEVEL * (1+2+32) + 2*32;
  /* Maximum TID words is easier to calculate, but we can be conservative, since
   * it's first in the readout
   */
/*   MAXTIDWORDS = 8+(3*BLOCKLEVEL); */
  
  printf("**************************************************\n");
  printf("* Calculated MAX FADC words per block = %d\n",MAXFADCWORDS);
/*   printf("* Calculated MAX TID  words per block = %d\n",MAXTIDWORDS); */
  printf("**************************************************\n");
  /* Check these numbers, compared to our buffer size.. */
/*   if( (MAXFADCWORDS+MAXTIDWORDS)*4 > MAX_EVENT_LENGTH ) */
/*     { */
/*       printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"); */
/*       printf(" WARNING.  Event buffer size is smaller than the expected data size\n"); */
/*       printf("     Increase the size of MAX_EVENT_LENGTH and recompile!\n"); */
/*       printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"); */
/*     } */

  
  if(NFADC>1)
    faEnableMultiBlock(1);

  /* Additional Configuration for each module */
  fadcSlotMask=0;
  for(islot=0;islot<NFADC;islot++) 
    {
      FA_SLOT = fadcID[islot];      /* Grab the current module's slot number */
      fadcSlotMask |= (1<<FA_SLOT); /* Add it to the mask */

      /* Set the internal DAC level */
      faSetDAC(FA_SLOT,FADC_DAC_LEVEL,0);
      /* Set the threshold for data readout */
      faSetThreshold(FA_SLOT,FADC_THRESHOLD,0);
      faPrintThreshold(FA_SLOT);
      /*  Setup option 1 processing - RAW Window Data     <-- */
      /*        option 2            - RAW Pulse Data */
      /*        option 3            - Integral Pulse Data */
      /*  Setup 200 nsec latency (PL  = 50)  */
      /*  Setup  80 nsec Window  (PTW = 20) */
      /*  Setup Pulse widths of 36ns (NSB(3)+NSA(6) = 9)  */
      /*  Setup up to 1 pulse processed */
      /*  Setup for both ADC banks(0 - all channels 0-15) */
      /* Integral Pulse Data */
      faSetProcMode(FA_SLOT,faMode,FADC_WINDOW_LAT,FADC_WINDOW_WIDTH,3,6,1,0);
	
      /* Bus errors to terminate block transfers (preferred) */
      faEnableBusError(FA_SLOT);
      /* Set the Block level */
      faSetBlockLevel(FA_SLOT,BLOCKLEVEL);

      /* Set the individual channel pedestals for the data that is sent
       * to the CTP
       */
      int ichan;
      for(ichan=0; ichan<16; ichan++)
	{
	  faSetChannelPedestal(FA_SLOT,ichan,0);
	}

     }
  //   }//(1==-1)




  printf("rocDownload: User Download Executed\n");

}

/****************************************
 *  PRESTART
 ****************************************/
void
rocPrestart()
{
  unsigned short iflag;
  int stat;
  int islot;

  tiStatus(0);
  tiSetPrescale(1);
  tiSetOutputPort(1,1,0,0);
  c775Clear(TDC_ID);
  c775DisableBerr(TDC_ID);
/*   c775EnableBerr(TDC_ID); /\* for 32bit block transfer *\/ */
  c775CommonStop(TDC_ID);
  c775BitSet2(TDC_ID, (1<<12));

  /*  c775SetFSR(TDC_ID,520 ); */
  c775Status(TDC_ID,0,0);

  /* FADC Perform some resets, status */
  for(islot=0;islot<NFADC;islot++) 
    {
      FA_SLOT = fadcID[islot];
      faSetClockSource(FA_SLOT,2);
      faClear(FA_SLOT);
      faResetToken(FA_SLOT);
      faResetTriggerCount(FA_SLOT);
      faStatus(FA_SLOT,0);
    }
  /*  Enable FADC */
  for(islot=0;islot<NFADC;islot++) 
    {
      FA_SLOT = fadcID[islot];
      faChanDisable(FA_SLOT,0xffff);
      faSetMGTTestMode(FA_SLOT,0);
      faEnable(FA_SLOT,0,0);
    }


  printf("rocPrestart: User Prestart Executed\n");

}

/****************************************
 *  GO
 ****************************************/
void
rocGo()
{
  int islot;
  tiSetOutputPort(0,1,0,0);



  /* Enable modules, if needed, here */
  c775Enable(TDC_ID);
  for(islot=0;islot<NFADC;islot++)
    {
      FA_SLOT = fadcID[islot];
      faChanDisable(FA_SLOT,0x0);
      faSetMGTTestMode(FA_SLOT,1);
      faSetBlockLevel(FA_SLOT,BLOCKLEVEL);
    }




  /* Get the current block level */
  BLOCKLEVEL = tiGetCurrentBlockLevel();
  printf("%s: Current Block Level = %d\n",
	 __FUNCTION__,BLOCKLEVEL);
 
  /* Use this info to change block level is all modules */

  tiSetBlockLimit(0); // 0: disables block limit
  tiStatus(0);
}

/****************************************
 *  END
 ****************************************/
void
rocEnd()
{

  int islot;
  tiSetOutputPort(1,1,0,0);

  for(islot=0;islot<NFADC;islot++) 
    {
      FA_SLOT = fadcID[islot];
      faDisable(FA_SLOT,0);
      faStatus(FA_SLOT,0);
    }

  tiStatus(0);



  c775Status(TDC_ID,0,0);

  c775Disable(TDC_ID);
  printf("rocEnd: Ended after %d blocks\n",tiGetIntCount());
  tiSetBlockLimit(0);
  
}

/****************************************
 *  TRIGGER
 ****************************************/
void
rocTrigger(int arg)
{
  //usleep(1000);//for testing ssp wordcnt, needs to be removed
  int ii, islot;
  int stat, dCnt, len=0, idata;
  int ssp_timeout;
  uint32_t bc, wc, ec, blockcnt, wordcnt, eventcnt;
  uint32_t data;
  static int tcnt = 0;
  int itime, gbready;
  int itimeout=0;
  int status, dma, count;
  int roflag=1;
  //tiStatus(1);
  
  // tiSetOutputPort(1,0,0,0);

  BANKOPEN(5,BT_UI4,0);
  *dma_dabufp++ = LSWAP(tiGetIntCount());
  *dma_dabufp++ = LSWAP(0xdead);
  *dma_dabufp++ = LSWAP(0xcebaf111);
  BANKCLOSE;
  
#ifdef TI_DATA_READOUT
  BANKOPEN(4,BT_UI4,0);

  vmeDmaConfig(2,5,1); 
  dCnt = tiReadBlock(dma_dabufp,8+(3*BLOCKLEVEL),1);
  if(dCnt<=0)
    {
      printf("No data or error.  dCnt = %d\n",dCnt);
      //tiSetBlockLimit(1);
    }
  else
    {
      dma_dabufp += dCnt;
    }

  BANKCLOSE;
#endif



 /* Readout FADC */
  if(NFADC!=0)
    {
      FA_SLOT = fadcID[0];
      for(itime=0;itime<100;itime++) 
	{
	  gbready = faGBready();
	  stat = (gbready == fadcSlotMask);
	  if (stat>0) 
	    {
	      break;
	    }
	}
      if(stat>0) 
	{
	  if(NFADC>1) roflag=2; /* Use token passing scheme to readout all modules */
	  BANKOPEN(3,BT_UI4,0);
	  dCnt = faReadBlock(FA_SLOT,dma_dabufp,MAXFADCWORDS,roflag);
	  if(dCnt<=0)
	    {
	      printf("FADC%d: No data or error.  dCnt = %d\n",FA_SLOT,dCnt);
	    }
	  else
	    {
	      if(dCnt>=MAXFADCWORDS)
		{
		  printf("%s: WARNING.. faReadBlock returned dCnt >= MAXFADCWORDS (%d >= %d)\n",
			 __FUNCTION__,dCnt, MAXFADCWORDS);
		}
	      else 
		dma_dabufp += dCnt;
	    }
	  BANKCLOSE;
	} 
      else 
	{
	  printf ("FADC%d: no events   stat=%d  intcount = %d   gbready = 0x%08x  fadcSlotMask = 0x%08x\n",
		  FA_SLOT,stat,tiGetIntCount(),gbready,fadcSlotMask);
	}

      /* Reset the Token */
      if(roflag==2)
	{
	  for(islot=0; islot<NFADC; islot++)
	    {
	      FA_SLOT = fadcID[islot];
	      faResetToken(FA_SLOT);
	    }
	}
    }

 /* Readout c775 */
  BANKOPEN(6,BT_UI4,0);
  while(itimeout<1000)
    {
      itimeout++;
      status = c775Dready(TDC_ID);
      if(status>0) break;
    }
  if(status > 0) 
    {
      if(tiGetIntCount() %1000==0)
	{
	  printf("itimeout = %d\n",itimeout);
	  c775PrintEvent(TDC_ID,0);
	}
      else
	{
	  int nwords = c775ReadEvent(TDC_ID,dma_dabufp);
	  /* or use c775ReadBlock, if BERR was enabled */
/* 	  nwords = c775ReadBlock(TDC_ID,dma_dabufp,MAX_TDC_DATA); */
	  if(nwords<=0) 
	    {
	      logMsg("ERROR: TDC Read Failed - Status 0x%x\n",nwords,0,0,0,0,0);
	      *dma_dabufp++ = 0xda000bad;
	      c775Clear(TDC_ID);
	    } 
	  else 
	    {
	      dma_dabufp += nwords;
	    }
	}
    }
  else
    {
      logMsg("ERROR: NO data in TDC  datascan = 0x%x, itimeout=%d\n",status,itimeout,0,0,0,0);
      c775Status(TDC_ID,0,0);
      c775Clear(TDC_ID);
    }
  BANKCLOSE;


  // tiSetOutputPort(0,0,0,0);

}

void
rocCleanup()
{
  int islot=0;
  for(islot=0; islot<NFADC; islot++)
    {
      FA_SLOT = fadcID[islot];
      faReset(FA_SLOT,1); /* Reset, and DO NOT restore A32 settings (1) */
    }

  printf("%s: Reset all FADCs\n",__FUNCTION__);
  remexClose();

}

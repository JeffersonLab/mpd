/*************************************************************************
 *
 *  mpd_list.c - Library of routines for readout and buffering of
 *                events using a JLAB Trigger Interface V3 (TI) with
 *                a Linux VME controller.
 *
 */

/* Event Buffer definitions */
/* Default block level */
unsigned int BLOCKLEVEL = 1;
#define BUFFERLEVEL 1
#define MAX_EVENT_LENGTH   32768*6*BLOCKLEVEL	/* Size in Bytes */
#define SSP_MAX_EVENT_LENGTH 32000*6*BLOCKLEVEL	//SSP block length
#define MAX_EVENT_POOL     50


/* Define Interrupt source and address */
#define TI_MASTER
#define TI_READOUT TI_READOUT_EXT_POLL	/* Poll for available data, external triggers */
#define TI_ADDR    (21<<19)	/* GEO slot 20 */
/* Decision on whether or not to readout the TI for each block
   - Comment out to disable readout
*/
#define TI_DATA_READOUT

#define FIBER_LATENCY_OFFSET 0x4A	/* measured longest fiber length */
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#define UINT32 unsigned int
#define STATUS int
#include "tiprimary_list.c"	/* source required for CODA */
#include <mpdLib.h>
#include <mpdConfig.h>
#include "sspLib.h"
#include "dmaBankTools.h"

extern uint32_t mpdRead32(volatile uint32_t * reg);

/*MPD Definitions*/

int h, i, j, k, kk, m, evt = 0;
int fnMPD = 10;
int error_count;
int rdone, rtout;

uint16_t mfull, mempty;
uint32_t e_head, e_head0, e_size;
uint32_t e_data32[130];
uint32_t e_trai, e_eblo;

char outfile[1000];
int acq_mode = 1;
int n_event = 10;

int vint_data;
uint32_t v_data;

int mpd_evt[21];
int UseSdram, FastReadout;
int empty, full, nwords;

uint32_t datao;

DMA_MEM_ID vmeIN, vmeOUT;
DMANODE *outEvent;

extern volatile struct mpd_struct *MPDp[(MPD_MAX_BOARDS + 1)];	/* pointers to MPD memory map */

#define MPD_TIMEOUT 10

//Output file TAG
#define VERSION_TAG 0xE0000000
#define EVENT_TAG   0x10000000
#define MPD_TAG     0x20000000
#define ADC_TAG     0x30000000
#define HEADER_TAG  0x00000040
#define DATA_TAG    0x0
#define TRAILER_TAG 0x00000050

#define FILE_VERSION 0x1
// End of MPD definition

/* function prototype */
void rocTrigger(int arg);

/****************************************
 *  DOWNLOAD
 ****************************************/
void
rocDownload()
{
#ifdef WITH_REMEX
  remexSetCmsgServer("sbs1");	// Set this to the platform's host
  remexSetRedirect(1);
  remexInit("Thunder", 1);
#endif /* WITH_REMEX */

  printf("%s: Build date/time %s/%s\n", __func__, __DATE__, __TIME__);

  /* Setup Address and data modes for DMA transfers
   *
   *  vmeDmaConfig(addrType, dataType, sstMode);
   *
   *  addrType = 0 (A16)    1 (A24)    2 (A32)
   *  dataType = 0 (D16)    1 (D32)    2 (BLK32) 3 (MBLK) 4 (2eVME) 5 (2eSST)
   *  sstMode  = 0 (SST160) 1 (SST267) 2 (SST320)
   */
  vmeDmaConfig(2, 5, 1);

  /*****************
   *   TI SETUP
   *****************/
#ifndef TI_DATA_READOUT
  /* Disable data readout */
  tiDisableDataReadout();
  /* Disable A32... where that data would have been stored on the TI */
  tiDisableA32();
#endif

  /* Set crate ID */
  tiSetCrateID(0x01);		/* ROC 1 */

  tiSetTriggerSourceMask(TI_TRIGSRC_TSINPUTS | TI_TRIGSRC_PULSER |
			 TI_TRIGSRC_LOOPBACK | TI_TRIGSRC_VME);
  /*   tiSetTriggerSource(TI_TRIGGER_TSINPUTS); */

  /* Set needed TS input bits */
  tiEnableTSInput(TI_TSINPUT_1);

  /* Load the trigger table that associates
     pins 21/22 | 23/24 | 25/26 : trigger1
     pins 29/30 | 31/32 | 33/34 : trigger2
   */
  tiLoadTriggerTable(0);
  // MPD 1 sample readout = 3.525 us = 8 * 480
  tiSetTriggerHoldoff(1, 30, 1);	/* 1 trigger in 20*480ns window */
  //tiSetTriggerHoldoff(1,60,1); /* 1 trigger in 20*480ns window */
  tiSetTriggerHoldoff(2, 0, 0);	/* 2 trigger in don't care window */

  tiSetTriggerHoldoff(3, 0, 0);	/* 3 trigger in don't care window */
  tiSetTriggerHoldoff(4, 20, 1);	/* 4 trigger in 20*3840ns window */
  // tiSetTriggerHoldoff(4,1,1);  /* 4 trigger in 20*3840ns window */

#ifdef TI_MASTER
  /* Set number of events per block */
  tiSetBlockLevel(BLOCKLEVEL);
#endif
  tiSetBlockBufferLevel(BUFFERLEVEL);

  tiStatus(0);


  /*****************
   *   SSP SETUP
   *****************/
  int iFlag =
    SSP_INIT_SKIP_FIRMWARE_CHECK | SSP_INIT_MODE_VXSLOCAL | 0xFFFF0000;

  sspInit(20 << 19, 1 << 19, 1, iFlag);
  extern int nSSP;
  int issp = 0;
  sspMpdFiberReset(0);
  sspMpdFiberLinkReset(0, 0xffffffff);

  for (issp = 0; issp < nSSP; issp++)
    {
      sspCheckAddresses(sspSlot(issp));
      sspMpdDisable(sspSlot(issp), 0xffffffff);
      sspMpdEnable(sspSlot(issp), 0x1);
      sspMpdEnable(sspSlot(issp), 0x2);

      sspEnableBusError(sspSlot(issp));
      sspSetBlockLevel(sspSlot(issp), BLOCKLEVEL);
    }
  sspSoftReset(0);
  sspPrintMigStatus(0);

  sspGStatus(0);
  sspMpdPrintStatus(0);

  /*****************
   *   MPD SETUP
   *****************/

  //vmeDmaConfig(2,2,0);
  /*Read config file and fill internal variables */
  mpdConfigInit("/home/daq/ben/mpd/libsrc/rol/cfg/config_apv.txt");
  mpdConfigLoad();

  /* Init and config MPD+APV */

  // discover MPDs and initialize memory mapping


// discover MPDs and initialize memory mapping
  mpdInit(0x1, 0, 1, MPD_INIT_SSP_MODE | MPD_INIT_NO_CONFIG_FILE_CHECK);


  fnMPD = mpdGetNumberMPD();
  //fnMPD = 1;
  if (fnMPD <= 0)
    {				// test all possible vme slot ?
      printf("ERR: no MPD discovered, cannot continue\n");
      return;
    }

  printf(" MPD discovered = %d\n", fnMPD);

  // APV configuration on all active MPDs
  for (k = 0; k < fnMPD; k++)
    {				// only active mpd set
      i = mpdSlot(k);

      int try_cnt = 0;
    retry:

      printf(" Try initialize I2C mpd in slot %d\n", i);
      if (mpdI2C_Init(i) != OK)
	{
	  printf("WRN: I2C fails on MPD %d\n", i);
	}

      printf("Try APV discovery and init on MPD slot %d\n", i);
      if (mpdAPV_Scan(i) <= 0 && try_cnt < 10)
	{			// no apd found, skip next
	  try_cnt++;
	  goto retry;
	}
      if (try_cnt == 10)
	{
	  printf("CANNOT CONFIGURE APV FOR %d TIMES !!!!\n\n", try_cnt);
	}

      // board configuration (APV-ADC clocks phase)
      printf("Do DELAY setting on MPD slot %d\n", i);
      mpdDELAY25_Set(i, mpdGetAdcClockPhase(i, 0), mpdGetAdcClockPhase(i, 1));

      // apv reset----this check will never fail...see "mpdI2C_ApvReset()"
      printf("Do APV reset on MPD slot %d\n", i);
      if (mpdI2C_ApvReset(i) != OK)
	{
	  printf("ERR: apv resert faild on mpd %d\n", i);
	}

      // apv configuration
      printf("Configure single APV on MPD slot %d\n", i);
      for (j = 0; j < mpdGetNumberAPV(i); j++)
	{
	  if (mpdAPV_Config(i, j) != OK)
	    {
	      printf("ERR: config apv card %d failed in mpd %d\n", j, i);
	    }
	}

      // configure adc on MPD
      printf("Configure ADC on MPD slot %d\n", i);
      mpdADS5281_Config(i);

      // configure fir
      // not implemented yet

      // 101 reset on the APV
      printf("Do 101 Reset on MPD slot %d\n", i);
      mpdAPV_Reset101(i);

      // <- MPD+APV initialization ends here

    }				// end loop on mpds
  //END of MPD configure

  printf("rocDownload: User Download Executed\n");

}

/****************************************
 *  PRESTART
 ****************************************/
void
rocPrestart()
{

  tiStatus(0);

  printf("rocPrestart: User Prestart Executed\n");

}

/****************************************
 *  GO
 ****************************************/
void
rocGo()
{

  UseSdram = mpdGetUseSdram(mpdSlot(0));	// assume sdram and fastreadout are the same for all MPDs
  FastReadout = mpdGetFastReadout(mpdSlot(0));
  printf(" UseSDRAM= %d , FastReadout= %d\n", UseSdram, FastReadout);

  /* Enable modules, if needed, here */
/*Enable MPD*/
  for (k = 0; k < fnMPD; k++)
    {				// only active mpd set
      i = mpdSlot(k);

      // mpd latest configuration before trigger is enabled
      mpdSetAcqMode(i, "process");

      // load pedestal and thr default values
      mpdPEDTHR_Write(i);

      // enable acq
      mpdDAQ_Enable(i);

      mpdTRIG_Enable(i);
      mpd_evt[i] = 0;
    }
  /* Get the current block level */
  BLOCKLEVEL = tiGetCurrentBlockLevel();
  printf("%s: Current Block Level = %d\n", __FUNCTION__, BLOCKLEVEL);
  sspMpdPrintStatus(0);
  /* Use this info to change block level is all modules */

  // tiSetBlockLimit(1);
  tiStatus(0);
}

/****************************************
 *  END
 ****************************************/
void
rocEnd()
{
  tiStatus(0);
  //mpd close
  mpdTRIG_Disable(i);
  //mpd close
  printf("rocEnd: Ended after %d blocks\n", tiGetIntCount());
  tiSetBlockLimit(0);

}

/****************************************
 *  TRIGGER
 ****************************************/
void
rocTrigger(int arg)
{
  int dCnt;
  int ssp_timeout;
  uint32_t blockcnt, wordcnt, eventcnt;
  uint32_t data;
  static int tcnt = 0;



  tiSetOutputPort(1, 0, 0, 0);

  BANKOPEN(5, BT_UI4, 0);
  *dma_dabufp++ = LSWAP(tiGetIntCount());
  *dma_dabufp++ = LSWAP(0xdead);
  *dma_dabufp++ = LSWAP(0xcebaf111);
  BANKCLOSE;

#ifdef TI_DATA_READOUT
  BANKOPEN(4, BT_UI4, 0);

  vmeDmaConfig(2, 5, 1);
  dCnt = tiReadBlock(dma_dabufp, 8 + (3 * BLOCKLEVEL), 1);
  if (dCnt <= 0)
    {
      printf("No data or error.  dCnt = %d\n", dCnt);
    }
  else
    {
      dma_dabufp += dCnt;
    }

  BANKCLOSE;
#endif
/* Readout MPD */
  // open out file

  BANKOPEN(10, BT_UI4, 0);
  // vmeDmaConfig(2,2,0);
  rtout = 0;

  for (k = 0; k < fnMPD; k++)
    {				// only active mpd set
      i = mpdSlot(k);
      mpdArmReadout(i);		// prepare internal variables for readout

      int idata;

      ssp_timeout = 0;
      while ((sspBReady(i) == 0) && (ssp_timeout < 1000))
	{
	  ssp_timeout++;
	}
      if (ssp_timeout > 12)
	printf("ssp time = %d\n", ssp_timeout);

      if (ssp_timeout == 1000)
	{
	  printf("*** SSP TIMEOUT ***\n");
	  data = mpdRead32(&MPDp[0]->ob_status.output_buffer_flag_wc);
	  if (data & 0x20000000)	// Evt_Fifo_Full
	    printf("FIFO full\n");

	  sspGetEbStatus(0, &blockcnt, &wordcnt, &eventcnt);
	  printf("\nblockcnt = %d\nwordcnt = %d\neventcnt = %d\n",
		 blockcnt, wordcnt, eventcnt);
	}
      else
	{
	  vmeDmaConfig(2, 5, 1);
	  int dCnt =
	    sspReadBlock(0, dma_dabufp, SSP_MAX_EVENT_LENGTH >> 2, 1);
	  unsigned int *pBuf = (unsigned int *) dma_dabufp;
	  tcnt++;
	  if (!(tcnt & 0x3ff))
	    printf("tcnt = %u, EV Header: %u, MPD HDR = %u\n", tcnt & 0xFFF,
		   LSWAP(pBuf[1]) & 0xFFF, LSWAP(pBuf[5]) & 0xFFF);

	  if (dCnt <= 0)
	    {
	      printf("No data or error.  dCnt = %d\n", dCnt);
	    }
	  else
	    {
	      dma_dabufp += dCnt;
	    }



	  //printf("  dCnt = %d\n",dCnt);
	  dCnt = 0;
	  for (idata = 0; idata < dCnt; idata++)
	    {
	      if ((idata % 5) == 0)
		printf("\n\t");
	      datao = (unsigned int) LSWAP(the_event->data[idata]);
	      printf("  0x%08x ", datao);


	      //  if( (datao & 0x00E00000) == 0x00A00000 ) {
	      //      mpd_evt[i]++;
	      //      evt=mpd_evt[i];
	      //  }
	      //      evt = (evt > mpd_evt[i]) ? mpd_evt[i] : evt; // evt is the smallest number of events of an MPD
	    }
	  //printf("\n\n");
	}
    }

  BANKCLOSE;
  tiSetOutputPort(0, 0, 0, 0);

}

void
rocCleanup()
{
  printf("%s: Reset modules\n", __FUNCTION__);
#ifdef WITH_REMEX
  remexClose();
#endif /* WITH_REMEX */

}

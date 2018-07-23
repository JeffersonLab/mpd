/*************************************************************************
 *
 *  mpd_list.c - Library of routines for readout and buffering of
 *                events using a JLAB Trigger Interface V3 (TI) with
 *                a Linux VME controller.
 *
 */

/* Event Buffer definitions */
#define MAX_EVENT_POOL     100
#define MAX_EVENT_LENGTH   1024*600	/* Size in Bytes */

/* Define TI_MASTER or TI_SLAVE */
#define TI_MASTER
/* Poll for available data, external triggers */
#define TI_READOUT TI_READOUT_EXT_POLL
/* GEO slot 21 */
#define TI_ADDR    (21<<19)

#define FIBER_LATENCY_OFFSET 0x4A	/* measured longest fiber length */
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "tiprimary_list.c"	/* source required for CODA */
#include "mpdLib.h"
#include "mpdConfig.h"
#include "dmaBankTools.h"




/*MPD Definitions*/

/* int h, i, j, k, kk, m; */
int fnMPD = 10;
int error_count;
int rdone;

uint16_t mfull, mempty;
uint32_t e_head, e_head0, e_size;
uint32_t e_data32[130];
uint32_t e_trai, e_eblo;



char outfile[1000];
int acq_mode = 1;
int n_event = 10;

int vint_data;
uint32_t v_data;
#define MPD_TIMEOUT 10

pthread_mutex_t mpdMutex = PTHREAD_MUTEX_INITIALIZER;
#define MPDLOCK      if(pthread_mutex_lock(&mpdMutex)<0) perror("pthread_mutex_lock");
#define MPDUNLOCK    if(pthread_mutex_unlock(&mpdMutex)<0) perror("pthread_mutex_unlock");
extern volatile struct mpd_struct *MPDp[(MPD_MAX_BOARDS + 1)];	/* pointers to MPD memory map */
extern int mpdOutputBufferBaseAddr;	/* output buffer base address */
extern int mpdOutputBufferSpace;	/* output buffer space (8 Mbyte) */
#define DMA_BUFSIZE 80000

extern GEF_VME_BUS_HDL vmeHdl;
GEF_VME_DMA_HDL dmaHdl;
uint32_t vmeAdrs;
unsigned long physMemBase;
uint32_t *fBuffer;		// DMA data buffer

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




/* Default block level */
unsigned int BLOCKLEVEL = 1;
#define BUFFERLEVEL 1

int mpd_evt[21];
//int evt;

/* function prototype */
void rocTrigger(int arg);

/****************************************
 *  DOWNLOAD
 ****************************************/
void
rocDownload()
{

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

  /* Set crate ID */
  tiSetCrateID(0x01);		/* ROC 1 */

  tiSetTriggerSource(TI_TRIGGER_TSINPUTS);

  /* Set needed TS input bits */
  tiEnableTSInput(TI_TSINPUT_1);

  /* Load the trigger table that associates
     pins 21/22 | 23/24 | 25/26 : trigger1
     pins 29/30 | 31/32 | 33/34 : trigger2
   */
  tiLoadTriggerTable(0);

  tiSetTriggerHoldoff(1, 10, 1);
  tiSetTriggerHoldoff(2, 10, 1);

/*   /\* Set the sync delay width to 0x40*32 = 2.048us *\/ */
  tiSetSyncDelayWidth(0x54, 0x40, 1);

  /* Set the busy source to non-default value (no Switch Slot B busy) */
  tiSetBusySource(TI_BUSY_LOOPBACK, 1);

/*   tiSetFiberDelay(10,0xcf); */

#ifdef TI_MASTER
  /* Set number of events per block */
  tiSetBlockLevel(BLOCKLEVEL);
#endif

  tiSetEventFormat(1);

  tiSetBlockBufferLevel(BUFFERLEVEL);

  tiStatus(0);


  /*****************
   *   MPD SETUP
   *****************/
  int rval = OK;

  /*Read config file and fill internal variables */
  mpdConfigInit("/home/daq/ben/mpd/libsrc4.0/rol/cfg/config_apv.txt");
  mpdConfigLoad();

  /* Init and config MPD+APV */

  // discover MPDs and initialize memory mapping
  mpdInit(0x80000, 0x80000, 21, 0x0);
  fnMPD = mpdGetNumberMPD();

  if (fnMPD <= 0)
    {				// test all possible vme slot ?
      printf("ERR: no MPD discovered, cannot continue\n");
      return;
    }

  printf(" MPD discovered = %d\n", fnMPD);

  // APV configuration on all active MPDs
  int impd, iapv, id;
  for (impd = 0; impd < fnMPD; impd++)
    {				// only active mpd set
      id = mpdSlot(impd);

      rval = mpdHISTO_MemTest(id);

      printf(" Try initialize I2C mpd in slot %d\n", id);
      if (mpdI2C_Init(id) != OK)
	{
	  printf("WRN: I2C fails on MPD %d\n", id);
	}
      if (mpdI2C_ApvReset(id) != OK)
	{
	  printf("WRN: I2C ApvReset fails on MPD %d\n", id);
	}
      printf("Try APV discovery and init on MPD slot %d\n", id);
      if (mpdAPV_Scan(id) <= 0)
	{			// no apd found, skip next
	  continue;
	}

      // board configuration (APV-ADC clocks phase)
      printf("Do DELAY setting on MPD slot %d\n", id);
      mpdDELAY25_Set(id, mpdGetAdcClockPhase(id, 0), mpdGetAdcClockPhase(id, 1));

      // apv reset----this check will never fail...see "mpdI2C_ApvReset()"
      printf("Do APV reset on MPD slot %d\n", id);
      if (mpdI2C_ApvReset(id) != OK)
	{
	  printf("ERR: apv resert faild on mpd %d\n", id);
	}

      // apv configuration
      printf("Configure single APV on MPD slot %d\n", id);
      for (iapv = 0; iapv < mpdGetNumberAPV(id); iapv++)
	{
	  if (mpdAPV_Config(id, iapv) != OK)
	    {
	      printf("ERR: config apv card %d failed in mpd %d\n", iapv, id);
	    }
	}

      // configure adc on MPD
      printf("Configure ADC on MPD slot %d\n", id);
      mpdADS5281_Config(id);

      // configure fir
      // not implemented yet

      // 101 reset on the APV
      printf("Do 101 Reset on MPD slot %d\n", id);
      mpdAPV_Reset101(id);

      // <- MPD+APV initialization ends here

    }				// end loop on mpds
  //END of MPD configure

  // summary report
  printf("Configured APVs (ADC 15 ... 0)\n");
  int ibit;
  for (impd = 0; impd < fnMPD; impd++)
    {
      id = mpdSlot(impd);

      if (mpdGetApvEnableMask(id) != 0)
	{
	  printf("MPD %2d : ", id);
	  iapv = 0;
	  for (ibit = 15; ibit >= 0; ibit--)
	    {
	      if (mpdGetApvEnableMask(id) & (1 << ibit))
		{
		  printf("1");
		  iapv++;
		}
	      else
		{
		  printf(".");
		}
	    }
	  printf(" (#APV %d)\n", iapv);
	}
    }

  printf("rocDownload: User Download Executed\n");

}

/****************************************
 *  PRESTART
 ****************************************/
void
rocPrestart()
{
  int impd;

  tiStatus(0);
  vmeCheckMutexHealth(1);
  tiSetBlockLimit(0);
  printf("rocPrestart: User Prestart Executed\n");

  for (impd = 0; impd < 21; impd++)
    mpd_evt[impd] = 0;

}

/****************************************
 *  GO
 ****************************************/
void
rocGo()
{
  int impd, id;
  /* Enable modules, if needed, here */

  /*Enable MPD*/
  for (impd = 0; impd < fnMPD; impd++)
    {				// only active mpd set
      id = mpdSlot(impd);

      // mpd latest configuration before trigger is enabled
      mpdSetAcqMode(id, "process");

      // load pedestal and thr default values
      mpdPEDTHR_Write(id);

      // enable acq
      mpdOutputBufferBaseAddr = 0x08000000;
      printf("%s: INFO: mpdOutputBufferBaseAddr = 0x%08x\n",
	     __func__, mpdOutputBufferBaseAddr);
      mpdDAQ_Enable(id);
    }
  /* Get the current block level */
  BLOCKLEVEL = tiGetCurrentBlockLevel();
  printf("%s: Current Block Level = %d\n", __FUNCTION__, BLOCKLEVEL);

  /* Use this info to change block level is all modules */

}

/****************************************
 *  END
 ****************************************/
void
rocEnd()
{

  int impd;

  tiStatus(0);
  //mpd close
  for (impd = 0; impd < fnMPD; impd++)
    {
      mpdDAQ_Disable(mpdSlot(impd));
    }

  printf("rocEnd: Ended after %d blocks\n", tiGetIntCount());

}

/****************************************
 *  TRIGGER
 ****************************************/
void
rocTrigger(int arg)
{
  int dCnt;
  int word_offset = 0;
  static int timeout = 0;

  BANKOPEN(4, BT_UI4, 0);

  vmeDmaConfig(2, 5, 1);
  dCnt = tiReadBlock(dma_dabufp, 8 + (3 * BLOCKLEVEL), 1);
  if (dCnt <= 0)
    {
      printf("No data or error.  dCnt = %d\n", dCnt);
      word_offset = 0;
    }
  else
    {
      dma_dabufp += dCnt;
      word_offset = dCnt;
    }

  BANKCLOSE;

/* Readout MPD */
  // open out file

  BANKOPEN(10, BT_UI4, 0);

  int UseSdram, FastReadout;
  int empty, full, nwords, obuf_nblock;
  int nwread;
  int iw, blen;
  int verbose_level = 2;

  UseSdram = mpdGetUseSdram(mpdSlot(0));	// assume sdram and fastreadout are the same for all MPDs
  FastReadout = mpdGetFastReadout(mpdSlot(0));

  printf("\n\n ========= UseSDRAM= %d , FastReadout= %d\n", UseSdram,
	 FastReadout);

  // -> now trigger can be enabled

  int tout, impd, id;
  for (impd = 0; impd < fnMPD; impd++)
    {				// only active mpd set
      id = mpdSlot(impd);
      //   vmeSetQuietFlag(1);
      //   vmeClearException(1);
      mpdArmReadout(id);		// prepare internal variables for readout @@ use old buffer scheme, need improvement
      //blen = mpdApvGetBufferAvailable(i, 0);
      if (UseSdram)
	{
	  blen = DMA_BUFSIZE;
	}
      else
	{
	  blen = mpdApvGetBufferAvailable(id, 0);
	}
      nwread = 0;

      if (UseSdram)
	{

	  int sd_init, sd_overrun, sd_rdaddr, sd_wraddr, sd_nwords;

	  mpdSDRAM_GetParam(id, &sd_init, &sd_overrun, &sd_rdaddr, &sd_wraddr,
			    &sd_nwords);
	  if (verbose_level > 0)
	    printf
	      ("SDRAM status: init=%d, overrun=%d, rdaddr=0x%x, wraddr=0x%x, nwords=%d\n",
	       sd_init, sd_overrun, sd_rdaddr, sd_wraddr, sd_nwords);

	  tout = 0;
	  while (mpdOBUF_GetBlockCount(id) == 0 && tout < 1000)
	    {
	      usleep(10);
	      tout++;
	    }
	  if (tout == 1000)
	    {
	      timeout = 1;
	      tiSetBlockLimit(1);
	      printf
		("WARNING: *** Timeout while waiting for data in mpd %d - check MPD/APV configuration\n",
		 id);
	    }

	  obuf_nblock = mpdOBUF_GetBlockCount(id);
	  // evb_nblock = mpdGetBlockCount(i);

	  if (obuf_nblock > 0)
	    {			// read data

	      mpdOBUF_GetFlags(id, &empty, &full, &nwords);

	      if (verbose_level > 0)
		printf("OBUFF status: empty=%d, full=%d, nwords=%d\n", empty,
		       full, nwords);

	      if (FastReadout > 0)
		{		//64bit transfer
		  if (nwords < 128)
		    {
		      empty = 1;
		    }
		  else
		    {
		      nwords *= 2;
		    }
		}

	      if (full)
		{
		  printf
		    ("\n\n **** OUTPUT BUFFER FIFO is FULL in MPD %d !!! RESET EVERYTHING !!!\n\n",
		     id);
		  //exit(1);
		}
	      if (verbose_level > 0)
		printf("Data waiting to be read in obuf: %d (32b-words)\n",
		       nwords);
	      if (nwords > 0)	// was >=
		{
		  if (nwords > blen / 4)
		    {
		      nwords = blen / 4;
		    }

		  mpdOBUF_Read(id, dma_dabufp, nwords, &nwread);

		  if (verbose_level > 0)
		    printf
		      ("try to read %d 32b-words 128b-aligned from obuf, got %d 32b-words\n",
		       nwords, nwread);

		  if (nwords != nwread)
		    {
		      printf
			("ERROR: 32bit-word read count does not match %d %d\n",
			 nwords, nwread);
		    }
		  dma_dabufp += nwread;
		}
	    }
	  else
	    usleep(10);

	}
      else
	{			// if not Sdram

	  mpdFIFO_IsEmpty(id, 0, &empty);	//  read fifo channel=0 status

	  if (!empty)
	    {			// read fifo
	      nwread = blen / 4;
	      mpdFIFO_ReadSingle(id, 0, mpdApvGetBufferPointer(id, 0, 0),
				 &nwread, 20);
	      if (nwread == 0)
		{
		  printf
		    ("ERROR: word read count is 0, while some words are expected back\n");
		}
	    }

	}

      if (nwread > 0)
	{			// data need to be written on file

	  int zero_count;
	  zero_count = 0;
	  if (verbose_level > 1)
	    printf("MPD Slot: %d (dump data on screen)\n", id);	// slot

	  for (iw = 0; iw < nwread; iw++)
	    {
	      uint32_t datao;
	      if (UseSdram)
		{
		  datao = LSWAP(the_event->data[iw + word_offset]);
		}
	      else
		datao = mpdApvGetBufferElement(id, 0, iw);

	      if (datao == 0)
		{
		  zero_count++;
		}
	      if (verbose_level > 1)
		{		// && ((iw<500) || ((nwread-iw)<501))) {

		  if ((verbose_level > 2) || (iw < 16)
		      || ((nwread - iw) <= (16 + nwread % 8)))
		    {
		      if ((iw % 8) == 0)
			{
			  printf("0x%06x:", iw);
			}
		      printf(" 0x%08x", datao);

		      if (((iw % 8) == 7) || (iw == (nwread - 1)))
			{
			  printf("\n");
			}
		    }

		}
	      //      if (verbose_level > 1 && iw==500) { printf(" ....\n"); }

	      if ((datao & 0x00E00000) == 0x00A00000)
		{		// EVENT TRAILER
//          printf("\n\n   ***** EVENT TRAILER: datao = 0x%08X mpd_evt[%d] = %d\n",datao,i, mpd_evt[i]);
		  mpd_evt[id]++;
//          evt=mpd_evt[i];
		}
//        evt = (evt > mpd_evt[i]) ? mpd_evt[i] : evt; // evt is the smallest number of events of an MPD
	    }


	  printf("MPD %d: nwords=%d nwcount=%d zero_count=%d evt %d\n", id,
		 nwords, nwread, zero_count, mpd_evt[id]);
	}
    }				// active mpd loop

  BANKCLOSE;

}

void
rocCleanup()
{
  int impd = 0, ia = 0;

  printf("%s: Free single read buffers\n", __FUNCTION__);
  for (impd = 0; impd < fnMPD; impd++)
    {
      for (ia = 0; ia < 16; ia++)
	mpdApvBufferFree(mpdSlot(impd), ia);
    }

}

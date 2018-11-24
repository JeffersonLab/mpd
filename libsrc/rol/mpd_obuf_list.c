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

int fnMPD = 0;
extern int mpdOutputBufferBaseAddr;	/* output buffer base address */
#define DMA_BUFSIZE 80000

// End of MPD definition

/* Default block level */
unsigned int BLOCKLEVEL = 1;
#define BUFFERLEVEL 1

/* function prototype */
void rocTrigger(int arg);

/****************************************
 *  DOWNLOAD
 ****************************************/
void
rocDownload()
{
  int error_status = OK;

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

  /* Set the busy source to non-default value (no Switch Slot B busy) */
  tiSetBusySource(TI_BUSY_LOOPBACK, 1);

#ifdef TI_MASTER
  /* Set number of events per block */
  tiSetBlockLevel(BLOCKLEVEL);
#endif

  tiSetBlockBufferLevel(BUFFERLEVEL);

  tiStatus(0);

  /*****************
   *   MPD SETUP
   *****************/
  int rval = OK;

  /*Read config file and fill internal variables */
  mpdConfigInit("/home/moffit/work/mpd/libsrc/rol/infn_cfg/config_apv.txt");
  mpdConfigLoad();

  /* Init and config MPD+APV */

  // discover MPDs and initialize memory mapping
  mpdInit((3<<19), 0x80000, 18, 0x0);
  fnMPD = mpdGetNumberMPD();

  if (fnMPD <= 0)
    {				// test all possible vme slot ?
      printf("ERR: no MPD discovered, cannot continue\n");
      return;
    }

  printf("MPD discovered = %d\n", fnMPD);

  printf("\n");

  // APV configuration on all active MPDs
  int impd, iapv, id;
  /* for (impd = 5; impd < 6; impd++) */
  for (impd = 0; impd < fnMPD; impd++)
    {				// only active mpd set
      id = mpdSlot(impd);
      printf("MPD slot %2d config:\n", id);


      rval = mpdHISTO_MemTest(id);

      printf(" - Initialize I2C\n");
      if (mpdI2C_Init(id) != OK)
	{
	  printf(" * * FAILED\n");
	  error_status = ERROR;
	}

#if 0
      printf(" - APV Reset\n");
      if (mpdI2C_ApvReset(id) != OK)
	{
	  printf(" * * FAILED\n");
	  error_status = ERROR;
	}
#endif
      printf(" - APV discovery and init\n");

      mpdSetPrintDebug(0x0);
      if (mpdAPV_Scan(id) <= 0)
	{			// no apd found, skip next
	  printf(" * * None Found\n");
	  error_status = ERROR;
	  continue;
	}
      mpdSetPrintDebug(0);

      // apv reset
      printf(" - APV Reset\n");
      if (mpdI2C_ApvReset(id) != OK)
	{
	  printf(" * * FAILED\n");
	  error_status = ERROR;
	}

      usleep(10);
      I2C_SendStop(id);
      // board configuration (APV-ADC clocks phase)
      // (do this while APVs are resetting)
      printf(" - DELAY setting\n");
      if (mpdDELAY25_Set
	  (id, mpdGetAdcClockPhase(id, 0), mpdGetAdcClockPhase(id, 1)) != OK)
	{
	  printf(" * * FAILED\n");
	  error_status = ERROR;
	}

      // apv configuration
      mpdSetPrintDebug(0);
      printf(" - Configure Individual APVs\n");
      printf(" - - ");
      for (iapv = 0; iapv < mpdGetNumberAPV(id); iapv++)
	{
	  printf("%2d ", iapv);
	  fflush(stdout);

	  if (mpdAPV_Config(id, iapv) != OK)
	    {
	      printf(" * * FAILED for APV %2d\n", iapv);
	      printf(" - - ");
	      error_status = ERROR;
	    }
	}
      printf("\n");
      mpdSetPrintDebug(0);

      // configure adc on MPD
      printf(" - Configure ADC\n");
      if (mpdADS5281_Config(id) != OK)
	{
	  printf(" * * FAILED\n");
	  error_status = ERROR;
	}

      // configure fir
      // not implemented yet

      // RESET101 on the APV
      printf(" - Do APV RESET101\n");
      if (mpdAPV_Reset101(id) != OK)
	{
	  printf(" * * FAILED\n");
	  error_status = ERROR;
	}

      // <- MPD+APV initialization ends here
      printf("\n");
    }				// end loop on mpds
  //END of MPD configure

  mpdGStatus(1);

  // summary report
  printf("\n");
  printf("Configured APVs (ADC 15 ... 0)\n");
  int ibit;
  for (impd = 0; impd < fnMPD; impd++)
    {
      id = mpdSlot(impd);

      if (mpdGetApvEnableMask(id) != 0)
	{
	  printf("  MPD %2d : ", id);
	  iapv = 0;
	  for (ibit = 15; ibit >= 0; ibit--)
	    {
	      if (((ibit + 1) % 4) == 0)
		printf(" ");
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
  printf("\n");

  if (error_status != OK)
    daLogMsg("ERROR", "MPD initialization has errors");


  printf("rocDownload: User Download Executed\n");

}

/****************************************
 *  PRESTART
 ****************************************/
void
rocPrestart()
{

  tiStatus(0);
  vmeCheckMutexHealth(1);
  tiSetBlockLimit(0);


  printf("rocPrestart: User Prestart Executed\n");

}

/****************************************
 *  GO
 ****************************************/
void
rocGo()
{
  int impd, id;
  /* Enable modules, if needed, here */

  /*Enable MPD */
  mpdOutputBufferBaseAddr = 0x08000000;
  for (impd = 0; impd < fnMPD; impd++)
    {				// only active mpd set
      id = mpdSlot(impd);

      // mpd latest configuration before trigger is enabled
      mpdSetAcqMode(id, "process");

      // load pedestal and thr default values
      mpdPEDTHR_Write(id);

      // enable acq
      mpdDAQ_Enable(id);

      if (mpdAPV_Reset101(id) != OK)
        {
          printf("MPD Slot %2d: Reset101 FAILED\n", id);
        }
    }

  /*
     Get the current block level
     Use this info to change block level is all modules
   */

  BLOCKLEVEL = tiGetCurrentBlockLevel();
  printf("%s: Current Block Level = %d\n", __FUNCTION__, BLOCKLEVEL);

  /* Check MPDs for data */
  int sd_init, sd_overrun, sd_rdaddr, sd_wraddr, sd_nwords;
  int obuf_nblock = 0, empty = 0, full = 0, nwords = 0;
  for (impd = 0; impd < fnMPD; impd++)
    {				// only active mpd set
      id = mpdSlot(impd);
      mpdSDRAM_GetParam(id, &sd_init, &sd_overrun, &sd_rdaddr, &sd_wraddr,
			&sd_nwords);

      if ((sd_nwords != 0) || (sd_overrun == 1) || (sd_init == 0))
	{
	  printf("ERROR: Slot %2d SDRAM status: \n"
		 "init=%d, overrun=%d, rdaddr=0x%x, wraddr=0x%x, nwords=%d\n",
		 id, sd_init, sd_overrun, sd_rdaddr, sd_wraddr, sd_nwords);
	}

      obuf_nblock = mpdOBUF_GetBlockCount(id);
      mpdOBUF_GetFlags(id, &empty, &full, &nwords);

      if ((obuf_nblock != 0) || (empty == 0) || (full == 1) || (nwords != 0))
	{
	  printf("ERROR: Slot %2d OBUF status: \n"
		 "nblock = %d  empty=%d  full=%d  nwords=%d\n",
		 id, obuf_nblock, empty, full, nwords);
	}
    }

  mpdGStatus(1);

}

/****************************************
 *  END
 ****************************************/
void
rocEnd()
{

  int impd;

  tiStatus(0);

  //mpd disable
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
  int dCnt, iw;
  int ti_data_offset = 0;
  static int timeout = 0;
  int UseSdram, FastReadout;
  int empty, full, nwords, obuf_nblock;
  int nwread;
  int blen;
  int verbose_level = 2;
  int errFlag = 0;
  int mpd_data_offset = 0;

  /* Data bank (4) for TI data */
  BANKOPEN(4, BT_UI4, 0);

  ti_data_offset = ((int) (dma_dabufp) - (int) (&the_event->data[0])) >> 2;

  /* Readout TI data */
  vmeDmaConfig(2, 5, 1);
  dCnt = tiReadBlock(dma_dabufp, 8 + (3 * BLOCKLEVEL), 1);
  if (dCnt <= 0)
    {
      printf("No data or error.  dCnt = %d\n", dCnt);
      ti_data_offset = 0;
    }
  else
    {
      dma_dabufp += dCnt;
    }

  if (verbose_level > 1)
    {
      printf(" (TI dump data on screen)\n");
      if (dCnt > 0)
	{

	  for (iw = 0; iw < ((dCnt > 16) ? 16 : dCnt); iw++)
	    {
	      uint32_t datao =
		LSWAP(the_event->data[iw + ti_data_offset]);

	      if (verbose_level > 1)
		{
		  if ((iw % 8) == 0)
		    {
		      printf("0x%06x:", iw);
		    }
		  printf(" 0x%08x", datao);

		  if (((iw % 8) == 7) || (iw == (dCnt - 1)))
		    {
		      printf("\n");
		    }
		}
	    }


	  printf(" - Summary: TI data word count = %d\n\n", dCnt);
	}
    }

  BANKCLOSE;

  /* Data bank (10) for MPD data */
  BANKOPEN(10, BT_UI4, 0);

  // assume sdram and fastreadout are the same for all MPDs
  UseSdram = mpdGetUseSdram(mpdSlot(0));
  FastReadout = mpdGetFastReadout(mpdSlot(0));

  if(verbose_level > 0)
    {
      printf("\n\n ========= "
	     "UseSDRAM= %d , FastReadout= %d   Readout Event = %d\n",
	     UseSdram, FastReadout, tiGetIntCount());
    }

  // -> now trigger can be enabled

  int tout, impd, id;
  for (impd = 0; impd < fnMPD; impd++)
    {				// only active mpd set
      id = mpdSlot(impd);

      if (verbose_level > 0)
	printf("MPD %d: \n", id);

      // prepare internal variables for readout @@ use old buffer scheme, need improvement
      mpdArmReadout(id);

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
	    printf(" - SDRAM status: init=%d, overrun=%d, "
		   "rdaddr=0x%x, wraddr=0x%x, nwords=%d\n",
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

	      errFlag = 1;
	      printf("WARNING: *** Timeout while waiting for data in mpd %d"
		     " - check MPD/APV configuration\n", id);
	    }

	  obuf_nblock = mpdOBUF_GetBlockCount(id);
	  // evb_nblock = mpdGetBlockCount(i);

	  if (obuf_nblock > 0)
	    {			// read data

	      mpdOBUF_GetFlags(id, &empty, &full, &nwords);

	      if (verbose_level > 0)
		printf(" - OBUF status: empty=%d, full=%d, nwords=%d\n",
		       empty, full, nwords);

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
		  printf("\n\n **** OUTPUT BUFFER FIFO is FULL in MPD %d "
			 "!!! RESET EVERYTHING !!!\n\n", id);

		  errFlag = 1;
		}
	      if (verbose_level > 0)
		printf(" - OBUF Data Ready: %d (32b-words)\n", nwords);
	      if (nwords > 0)	// was >=
		{
		  if (nwords > blen / 4)
		    {
		      nwords = blen / 4;
		    }

		  mpd_data_offset =
		    ((int) (dma_dabufp) - (int) (&the_event->data[0])) >> 2;
		  mpdOBUF_Read(id, dma_dabufp, nwords, &nwread);

		  if (verbose_level > 0)
		    printf(" - Readout: %d (32b-words)", nwread);

		  if (nwords != nwread)
		    {
		      printf
			(" * ERROR: MPD %2d OBUF Data Ready (%d) != Data Readout (%d)\n",
			 id, nwords, nwread);
		      errFlag = 1;
		    }

		  dma_dabufp += nwread;
		}
	    }
	  else
	    usleep(10);

	}
      else
	{			// if not Sdram
	  // FIXME: THIS PROCEDURE IS CURRENTLY BROKEN
	  mpdFIFO_IsEmpty(id, 0, &empty);	//  read fifo channel=0 status

	  if (!empty)
	    {			// read fifo
	      nwread = blen / 4;
	      mpdFIFO_ReadSingle(id, 0, mpdApvGetBufferPointer(id, 0, 0),
				 &nwread, 20);
	      if (nwread == 0)
		{
		  printf(" * ERROR: word read count is 0, "
			 "while some words are expected back\n");
		  errFlag = 1;
		}
	    }

	}

      if (verbose_level > 1)
	{
	  printf(" (dump data on screen)\n");
	  if (nwread > 0)
	    {

	      for (iw = 0; iw < ((nwread > 40) ? 40 : nwread); iw++)
		{
		  uint32_t datao =
		    LSWAP(the_event->data[iw + mpd_data_offset]);

		  if (verbose_level > 1)
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


	      printf(" - Summary: nwords=%d  nwread=%d\n\n", nwords, nwread);
	    }
	}
    }				// active mpd loop

  if (errFlag)
    {
      tiStatus(1);
      mpdGStatus(1);
      printf(" * * ERRORS in Readout\n");
    }

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


/*
  Local Variables:
  compile-command: "make -k -B mpd_obuf_list.so"
  End:
 */

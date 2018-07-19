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

/* Define Interrupt source and address */
#define TI_MASTER
#define TI_READOUT TI_READOUT_EXT_POLL	/* Poll for available data, external triggers */
#define TI_ADDR    (21<<19)	/* GEO slot 21 */

/* Decision on whether or not to readout the TI for each block
   - Comment out to disable readout
*/
/* #define TI_DATA_READOUT  */

#define FIBER_LATENCY_OFFSET 0x4A	/* measured longest fiber length */
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#define UINT32 unsigned int
#define STATUS int
#include "tiprimary_list.c"	/* source required for CODA */
#include "mpdLib.h"
#include "mpdConfig.h"
#include "dmaBankTools.h"




/*MPD Definitions*/

int h, i, j, k, kk, m;
int fnMPD = 10;
int error_count;
int rdone;
#define MAX_HDATA 4096
#define MAX_SDATA 1024
uint32_t hdata[MAX_HDATA];	// histogram data buffer
uint32_t sdata[MAX_SDATA];	// samples data buffer

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

/* Redefine tsCrate according to TI_MASTER or TI_SLAVE */
#ifdef TI_SLAVE
int tsCrate = 0;
#else
#ifdef TI_MASTER
int tsCrate = 1;
#endif
#endif

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
  int overall_offset = 0x80;

#ifndef TI_DATA_READOUT
  /* Disable data readout */
  tiDisableDataReadout();
  /* Disable A32... where that data would have been stored on the TI */
  tiDisableA32();
#endif

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

  vmeDmaConfig(2, 2, 0);
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
      return -1;
    }

  printf(" MPD discovered = %d\n", fnMPD);

  // APV configuration on all active MPDs
  for (k = 0; k < fnMPD; k++)
    {				// only active mpd set
      i = mpdSlot(k);

      rval = mpdHISTO_MemTest(i);
#ifdef REPLACED
      // first test MPD histo memory write/read
      printf(" test mpd %d histo memory\n", i);

      mpdHISTO_Clear(i, 0, -1);
      mpdHISTO_Read(i, 0, hdata);
      error_count = 0;
      for (j = 0; j < MAX_HDATA; j++)
	{
	  if (hdata[j] != j)
	    {
	      //      printf("ERROR matching histo read/write ch=%d rval=%d\n",j,hdata[j]);
	      error_count++;
	    }
	}
      if (error_count)
	{
	  printf("ERROR: HISTO Test fail %d time / %d attempts\n",
		 error_count, MAX_HDATA);
	}
      else
	{
	  printf("HISTO Read/Write test SUCCESS on MPD slot %d\n", i);
	}
      // END of first test MPD histo memory write/read
#endif /* REPLACED */

      printf(" Try initialize I2C mpd in slot %d\n", i);
      if (mpdI2C_Init(i) != OK)
	{
	  printf("WRN: I2C fails on MPD %d\n", i);
	}
      if (mpdI2C_ApvReset(i) != OK)
	{
	  printf("WRN: I2C ApvReset fails on MPD %d\n", i);
	}
      printf("Try APV discovery and init on MPD slot %d\n", i);
      if (mpdAPV_Scan(i) <= 0)
	{			// no apd found, skip next
	  continue;
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

  // summary report
  printf("Configured APVs (ADC 15 ... 0)\n");
  for (k = 0; k < fnMPD; k++)
    {
      i = mpdSlot(k);

      if (mpdGetApvEnableMask(i) != 0)
	{
	  printf("MPD %2d : ", i);
	  j = 0;
	  for (kk = 15; kk >= 0; kk--)
	    {
	      if (mpdGetApvEnableMask(i) & (1 << kk))
		{
		  printf("1");
		  j++;
		}
	      else
		{
		  printf(".");
		}
	    }
	  printf(" (#APV %d)\n", j);
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
  unsigned short iflag;
  int stat;
  int islot;
  int k;

  tiStatus(0);
  vmeCheckMutexHealth(1);
  tiSetBlockLimit(0);
  printf("rocPrestart: User Prestart Executed\n");

  for (k = 0; k < 21; k++)
    mpd_evt[k] = 0;
//  evt = 0;
}

/****************************************
 *  GO
 ****************************************/
void
rocGo()
{
  int islot;
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
      mpdOutputBufferBaseAddr = 0x08000000;
      printf("%s: INFO: mpdOutputBufferBaseAddr = 0x%08x\n",
	     __func__, mpdOutputBufferBaseAddr);
      int FastReadout = mpdGetFastReadout(mpdSlot(0));
      DMA_Init(i, FastReadout);	// this will work with 1 MPD only! TO BE IMPROVED
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

  int islot;

  tiStatus(0);
  //mpd close
  for (k = 0; k < fnMPD; k++)
    {
      mpdDAQ_Disable(mpdSlot(k));
    }
  DMA_Free();
  //mpd close
  printf("rocEnd: Ended after %d blocks\n", tiGetIntCount());

}

/****************************************
 *  TRIGGER
 ****************************************/
void
rocTrigger(int arg)
{
  //  usleep(1000000);
  int ii, islot;
  int stat, dCnt, len = 0, idata;
  int word_offset = 0;
  static int timeout = 0;

  tiSetOutputPort(1, 0, 0, 0);
  tiSetOutputPort(0, 0, 0, 0);
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
      word_offset = 0;
    }
  else
    {
      dma_dabufp += dCnt;
      word_offset = dCnt;
    }

  BANKCLOSE;
#endif
  tiSetOutputPort(0, 0, 0, 0);

/* Readout MPD */
  // open out file

  BANKOPEN(10, BT_UI4, 0);

  int UseSdram, FastReadout;
  int empty, full, nwords, obuf_nblock, evb_nblock, old_nblock;;
  int nwread;
  int iw, blen;
  int verbose_level = 2;

  // open out file

  //    fprintf(fout,"%x\n", FILE_VERSION | 0xD0000000);

  UseSdram = mpdGetUseSdram(mpdSlot(0));	// assume sdram and fastreadout are the same for all MPDs
  FastReadout = mpdGetFastReadout(mpdSlot(0));

  printf("\n\n ========= UseSDRAM= %d , FastReadout= %d\n", UseSdram,
	 FastReadout);

  // -> now trigger can be enabled

  int tout;
  for (k = 0; k < fnMPD; k++)
    {				// only active mpd set
      i = mpdSlot(k);
      //   vmeSetQuietFlag(1);
      //   vmeClearException(1);
      mpdArmReadout(i);		// prepare internal variables for readout @@ use old buffer scheme, need improvement
      //blen = mpdApvGetBufferAvailable(i, 0);
      if (UseSdram)
	{
	  blen = DMA_BUFSIZE;
	}
      else
	{
	  blen = mpdApvGetBufferAvailable(i, 0);
	}
      nwread = 0;

      if (UseSdram)
	{

	  int sd_init, sd_overrun, sd_rdaddr, sd_wraddr, sd_nwords;

	  mpdSDRAM_GetParam(i, &sd_init, &sd_overrun, &sd_rdaddr, &sd_wraddr,
			    &sd_nwords);
	  if (verbose_level > 0)
	    printf
	      ("SDRAM status: init=%d, overrun=%d, rdaddr=0x%x, wraddr=0x%x, nwords=%d\n",
	       sd_init, sd_overrun, sd_rdaddr, sd_wraddr, sd_nwords);

	  tout = 0;
	  while (mpdOBUF_GetBlockCount(i) == 0 && tout < 1000)
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
		 i);
	    }

	  obuf_nblock = mpdOBUF_GetBlockCount(i);
	  // evb_nblock = mpdGetBlockCount(i);

	  if (obuf_nblock > 0)
	    {			// read data

	      mpdOBUF_GetFlags(i, &empty, &full, &nwords);

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
		     i);
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
		  //  nwords = (nwords/4)*4; // 128 bit boundary
		  //mpdOBUF_Read(i, nwords, &nwread);
		  //       mpdOBUF_GetFlags(i, &empty, &full, &nwords);        // DUMMY
		  DMA_Read(i, dma_dabufp, nwords, &nwread);
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

	  mpdFIFO_IsEmpty(i, 0, &empty);	//  read fifo channel=0 status

	  if (!empty)
	    {			// read fifo
	      nwread = blen / 4;
	      mpdFIFO_ReadSingle(i, 0, mpdApvGetBufferPointer(i, 0, 0),
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
	    printf("MPD Slot: %d (dump data on screen)\n", i);	// slot
	  // fprintf(fout,"%x\n", i | MPD_TAG);
	  for (iw = 0; iw < nwread; iw++)
	    {
	      uint32_t datao;
	      if (UseSdram)
		{
		  datao = LSWAP(the_event->data[iw + word_offset]);
/* 	      *dma_dabufp = fBuffer[iw]; */
/* 	      dma_dabufp++; */
		}
	      else
		datao = mpdApvGetBufferElement(i, 0, iw);

	      //fprintf(fout,"%x\n",datao);
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
		  mpd_evt[i]++;
//          evt=mpd_evt[i];
		}
//        evt = (evt > mpd_evt[i]) ? mpd_evt[i] : evt; // evt is the smallest number of events of an MPD
	    }


	  printf("MPD %d: nwords=%d nwcount=%d zero_count=%d evt %d\n", i,
		 nwords, nwread, zero_count, mpd_evt[i]);
	}
    }				// active mpd loop



  //if (fout != stdout)  fclose(fout);






  BANKCLOSE;
  //  tiSetOutputPort(0,0,0,0);

}

void
rocCleanup()
{
  int k = 0, ia = 0;

  printf("%s: Free single read buffers\n", __FUNCTION__);
  for (k = 0; k < fnMPD; k++)
    {
      for (ia = 0; ia < 16; ia++)
	mpdApvBufferFree(mpdSlot(k), ia);
    }

}


/* DMA routines
 */
int
DMA_Init(int id, int fast_readout)
{
  GEF_STATUS status;
  GEF_MAP_PTR mapPtr;
  GEF_VME_DMA_HDL dma_hdl;
  int fBufSize = DMA_BUFSIZE;
  uint32_t data;

  switch (fast_readout)
    {
    case 0:
      vmeDmaConfig(2, 2, 0);	// A32 BLT
      break;
    case 1:
      vmeDmaConfig(2, 3, 0);	// MBLT
      break;
    case 2:
      vmeDmaConfig(2, 4, 0);	// 2eVME
      break;
    case 3:
      vmeDmaConfig(2, 5, 0);	// 2esst160
      break;
    case 4:
      vmeDmaConfig(2, 5, 1);	// 2esst266
      break;
    case 5:
      vmeDmaConfig(2, 5, 2);	// 2esst320
      break;
    default:
      vmeDmaConfig(2, 2, 0);	// A32 BLT
      break;
    }

#ifdef olderPHYSMEM
  status = gefVmeAllocDmaBuf(vmeHdl, fBufSize, &dma_hdl, &mapPtr);
  if (status != GEF_STATUS_SUCCESS)
    {
      MPD_ERR("\n\tgefVmeAllocDmaBuf returned 0x%x\n", status);
    }
  fBuffer = (uint32_t *) mapPtr;
  physMemBase = dmaHdl_to_PhysAddr(dma_hdl);
  dmaHdl = dma_hdl;
#endif
  vmeAdrs = (uint32_t) mpdOutputBufferBaseAddr + mpdOutputBufferSpace * id;
#ifdef olderPHYSMEM
  printf
    ("Buffer allocated with word size %d\n\tdmaHdl = 0x%08x  physMemBase = 0x%08x  fBuffer = 0x%08x  vmeAdrs = 0x%08x\n",
     fBufSize, (uint32_t) dmaHdl, (uint32_t) physMemBase, (uint32_t) fBuffer,
     (uint32_t) vmeAdrs);

#endif
  data = vmeAdrs >> 2;
  MPDLOCK;
  mpdWrite32(&MPDp[id]->obuf_base_addr, data);
  MPDUNLOCK;
  printf("%s(%d,%d) : Output buffer base address = 0x%x (data = 0x%x)\n",
	 __FUNCTION__, id, fast_readout, vmeAdrs, data);
  return OK;
}

int
DMA_Read(int id, volatile uint32_t * data, int size, int *wrec)
{
  int retVal = 0;
  int dummy1, dummy2, dummy3;
  int dummy = 0;
  volatile uint32_t *laddr;

  MPDLOCK;
  if ((unsigned long) (data) & 0x7)
    {
      dummy = 1;
      *data = 0;
      laddr = (data + 1);
    }
  else
    {
      dummy = 0;
      laddr = data;
    }


  vmeAdrs = (uint32_t) mpdOutputBufferBaseAddr + mpdOutputBufferSpace * id;
//printf("DMA_Read(): before vmeDmaSendPhys: physMemBase = 0x%08x, vmeAdrs = 0x%08x\n", physMemBase, vmeAdrs);
  retVal = vmeDmaSend((unsigned long) laddr, vmeAdrs, (size << 2));
//printf("DMA_Read(): after vmeDmaSendPhys retval = %d\n", retVal);

  if (retVal != 0)
    {
      MPD_ERR("DMA transfer Initialization (returned 0x%x)\n", retVal);
/*       MPD_ERR("  id=%d apv physMemBase = 0x%08x\n", */
/*               id,(uint32_t)physMemBase); */
      MPD_ERR("  id=%d apv laddr = 0x%08x  vmeAdrs = 0x%08x  size = %d\n",
	      id, (uint32_t) laddr, vmeAdrs, size);
      *wrec = 0;
      MPDUNLOCK;
      return (retVal);
    }

  /* Wait until Done or Error */
//printf("DMA_Read(): before vmeDmaDone\n");
  retVal = vmeDmaDone();
  MPDUNLOCK;
//printf("DMA_Read(): after vmeDmaDone retval = %d\n", retVal);
  mpdOBUF_GetFlags(id, &dummy1, &dummy2, &dummy3);
//printf("DMA_Read(): after mpdOBUF_GetFlags dunny1 = %d, dummy2 = %d, dummy3 = %d\n", dummy1, dummy2, dummy3);
  if (retVal == 0)
    {
      *wrec = 0;
      MPD_ERR("vmeDmaDone returned zero word count\n");

      return ERROR;
    }
  else if (retVal == ERROR)
    {
      *wrec = 0;
      MPD_ERR("vmeDmaDone returned ERROR\n");

      return ERROR;
    }
  else
    {
      *wrec = (retVal >> 2) + dummy;
//      MPD_DBG("vmeDmaDone returned 0x%x (%d)  wrec = %d\n",
//            retVal, retVal, *wrec);
/* Byteswap here, if needed */
/*       int iword=0; */
/*       for(iword =0; iword<*wrec; iword++) */
/* 	{ */
/*             data[iword] = LSWAP(data[iword]); */
/* 	} */
    }
  if ((*wrec + dummy) != size)
    {
      MPD_DBG("Count Mismatch: %d expected %d\n", *wrec, size);
      return ERROR;
    }

  return OK;

}

int
DMA_Free()
{
  GEF_STATUS status;
  if (fBuffer != 0)
    {
#ifdef olderPHYSMEM
      status = gefVmeFreeDmaBuf(dmaHdl);
      if (status != GEF_STATUS_SUCCESS)
	{
	  MPD_ERR("gefVmeFreeDmaBuf returned 0x%x\n", status);
	}
#endif
    }
  return OK;
}

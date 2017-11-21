/*************************************************************************
 *
 *  mpd_list.c - Library of routines for readout and buffering of
 *                events using a JLAB Trigger Interface V3 (TI) with
 *                a Linux VME controller.
 *
 */

/* Event Buffer definitions */
#define MAX_EVENT_POOL     100
#define MAX_EVENT_LENGTH   1024*600      /* Size in Bytes */

/* Define Interrupt source and address */
#define TI_MASTER
#define TI_READOUT TI_READOUT_EXT_POLL  /* Poll for available data, external triggers */
#define TI_ADDR    (21<<19)          /* GEO slot 21 */

/* Decision on whether or not to readout the TI for each block
   - Comment out to disable readout
*/
#define TI_DATA_READOUT

#define FIBER_LATENCY_OFFSET 0x4A  /* measured longest fiber length */
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#define UINT32 unsigned int
#define STATUS int
#include "mpdLib.h"
#include "mpdConfig.h"
#include "dmaBankTools.h"
#include "tiprimary_list.c" /* source required for CODA */



/*MPD Definitions*/

int h,i,j,k,kk,m, evt=0;
  int fnMPD=10;
  int error_count;
  int rdone, rtout;

  uint16_t mfull,mempty;
  uint32_t e_head, e_head0, e_size;
  uint32_t e_data32[130];
  uint32_t e_trai, e_eblo;

  char outfile[1000];
  int acq_mode = 1;
  int n_event=10;

  int vint_data;
  uint32_t v_data;
#define MPD_TIMEOUT 10

//Output file TAG
#define VERSION_TAG 0xE0000000
#define EVENT_TAG   0x10000000
#define MPD_TAG     0x20000000
#define ADC_TAG     0x30000000
#define HEADER_TAG  0x40000000
#define DATA_TAG    0x0
#define TRAILER_TAG 0x50000000

#define FILE_VERSION 0x1
// End of MPD definition




/* Default block level */
unsigned int BLOCKLEVEL=1;
#define BUFFERLEVEL 1

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

  tiSetTriggerSource(TI_TRIGGER_TSINPUTS);

  /* Set needed TS input bits */
  tiEnableTSInput( TI_TSINPUT_1 );

  /* Load the trigger table that associates
     pins 21/22 | 23/24 | 25/26 : trigger1
     pins 29/30 | 31/32 | 33/34 : trigger2
  */
  tiLoadTriggerTable(0);

  tiSetTriggerHoldoff(1,10,0);
  tiSetTriggerHoldoff(2,10,0);

/*   /\* Set the sync delay width to 0x40*32 = 2.048us *\/ */
  tiSetSyncDelayWidth(0x54, 0x40, 1);

  /* Set the busy source to non-default value (no Switch Slot B busy) */
  tiSetBusySource(TI_BUSY_LOOPBACK,1);

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

   vmeDmaConfig(2,2,0);
  /*Read config file and fill internal variables*/
  mpdConfigInit("cfg/config_apv.txt");
  mpdConfigLoad();

  /* Init and config MPD+APV */

  // discover MPDs and initialize memory mapping
  mpdInit(0x80000,0x80000,21,0x0);
  fnMPD = mpdGetNumberMPD();

  if (fnMPD<=0) { // test all possible vme slot ?
    printf("ERR: no MPD discovered, cannot continue\n");
    return -1;
  }

  printf(" MPD discovered = %d\n",fnMPD);

  // APV configuration on all active MPDs
  for (k=0;k<fnMPD;k++) { // only active mpd set
    i = mpdSlot(k);

    printf(" Try initialize I2C mpd in slot %d\n",i);
    if (mpdI2C_Init(i) != OK) {
      printf("WRN: I2C fails on MPD %d\n",i);
    }

    printf("Try APV discovery and init on MPD slot %d\n",i);
    if (mpdAPV_Scan(i)<=0) { // no apd found, skip next
        continue;
    }

    // board configuration (APV-ADC clocks phase)
    printf("Do DELAY setting on MPD slot %d\n",i);
    mpdDELAY25_Set(i, mpdGetAdcClockPhase(i,0), mpdGetAdcClockPhase(i,1));

    // apv reset----this check will never fail...see "mpdI2C_ApvReset()"
    printf("Do APV reset on MPD slot %d\n",i);
    if (mpdI2C_ApvReset(i) != OK) {
      printf("ERR: apv resert faild on mpd %d\n",i);
    }

    // apv configuration
    printf("Configure single APV on MPD slot %d\n",i);
    for (j=0; j < mpdGetNumberAPV(i); j++) {
      if (mpdAPV_Config(i,j) != OK) {
	printf("ERR: config apv card %d failed in mpd %d\n",j,i);
      }
    }

    // configure adc on MPD
    printf("Configure ADC on MPD slot %d\n",i);
    mpdADS5281_Config(i);

    // configure fir
    // not implemented yet

    // 101 reset on the APV
    printf("Do 101 Reset on MPD slot %d\n",i);
    mpdAPV_Reset101(i);

    // <- MPD+APV initialization ends here

  } // end loop on mpds
  //END of MPD configure

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

  printf("rocPrestart: User Prestart Executed\n");

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
    for (k=0;k<fnMPD;k++) { // only active mpd set
      i = mpdSlot(k);

      // mpd latest configuration before trigger is enabled
      mpdSetAcqMode(i, "event");

      // load pedestal and thr default values
      mpdPEDTHR_Write(i);

      // enable acq
      mpdDAQ_Enable(i);

    }
  /* Get the current block level */
  BLOCKLEVEL = tiGetCurrentBlockLevel();
  printf("%s: Current Block Level = %d\n",
	 __FUNCTION__,BLOCKLEVEL);

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
    mpdTRIG_Disable(i);
  //mpd close
  printf("rocEnd: Ended after %d blocks\n",tiGetIntCount());

}

/****************************************
 *  TRIGGER
 ****************************************/
void
rocTrigger(int arg)
{
  int ii, islot;
  int stat, dCnt, len=0, idata;

  tiSetOutputPort(1,0,0,0);

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
    }
  else
    {
      dma_dabufp += dCnt;
    }

  BANKCLOSE;
#endif
/* Readout MPD */
    // open out file

 BANKOPEN(10,BT_UI4,0);
 vmeDmaConfig(2,2,0);
  rtout=0;

  for (k=0;k<fnMPD;k++) { // only active mpd set
    i = mpdSlot(k);
    mpdArmReadout(i); // prepare internal variables for readout
  }

  rdone = 1;

  for (kk=0;kk<fnMPD;kk++) { // only active mpd set
     i = mpdSlot(kk);
     do
       { // wait for data in MPD
     	 rdone = mpdFIFO_ReadAll(i,&rtout,&error_count);
	 //printf(" fn: %d Evt: %d Rdone/ Tout/ error = %d %d %d\n", kk,evt,rdone, rtout, error_count);
	 rtout++;
       }while((rdone!=1||error_count!=0)&&(rtout < MPD_TIMEOUT)); // timeout can be changed

  if ((error_count != 0) || (rtout > MPD_TIMEOUT)) { // reset MPD on error or timeout
	  printf("%s: ERROR in readout, clear fifo\n",__FUNCTION__);
	  mpdFIFO_ClearAll(i);
	  mpdTRIG_Enable(i);
	  }

    if(rdone==1&&error_count==0)
      { // data need to be written on file


	for (j=0; j < mpdGetNumberAPV(i); j++) // loop on APV (ADC channels)
	  {
	    //CODA buf_MPDNumber
	    *dma_dabufp=LSWAP(i |MPD_TAG);
	    dma_dabufp++;
	    //CODA buf_MPDNumber

	    //CODA buf_apvNumber
	    v_data = mpdApvGetAdc(i,j);
	    *dma_dabufp=LSWAP(v_data| ADC_TAG);
	    dma_dabufp++;
	    //CODA buf_apvNumber
	    k=0; // buffer element index
	    v_data=mpdApvGetBufferSample(i,j);//printf("number of sample:%d",v_data);
	    for (h=0; h<mpdApvGetBufferSample(i,j); h++)// loop on samples
	      { e_size=130;
		e_head0 = mpdApvGetBufferElement(i, j, k);
		k++;
		//CODA buf_header
		e_head0 = LSWAP(e_head0);
		*dma_dabufp=LSWAP(e_head0 | HEADER_TAG);
		dma_dabufp++;
		//CODA buf_header
		for (m=0;m<e_size-2;m++)
		  {
		    //CODA buf_128Channel
		    v_data=mpdApvGetBufferElement(i,j,k);
		    v_data = LSWAP(v_data) & 0xFFFF;
		    *dma_dabufp=LSWAP(v_data | DATA_TAG);
		    dma_dabufp++;
		    //CODA buf_128Channel
		    k++;
		  }
		// CODA buf_trailer_TSNumber
		v_data=mpdApvGetBufferElement(i,j,k);
		*dma_dabufp=LSWAP((v_data & 0xfff) | TRAILER_TAG);
		dma_dabufp++;
		// CODA buf_trailer_TSNumber
		k++;
	      }
	    mpdApvShiftDataBuffer(i,j,k);
	  } // end loop on apv

	}

    evt++;
    //mpdFIFO_ClearAll(i);//removed
      }
 BANKCLOSE;
  tiSetOutputPort(0,0,0,0);

}

void
rocCleanup()
{
  int islot=0;

  printf("%s: Reset all FADCs\n",__FUNCTION__);

}

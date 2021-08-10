/*
 * File:
 *    mpdLibTest.c
 *
 * Description:
 *    Simply evolving program to test the mpd library
 *
 */


#include "mpdConfig.h"

#include "jvme.h"
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/signal.h>
#include <pthread.h>
/* Include MPD definitions */
#include "mpdLib.h"

int ctrl_break=0;

/* DMA variables
 */
pthread_mutex_t   mpdMutex = PTHREAD_MUTEX_INITIALIZER;
#define MPDLOCK      if(pthread_mutex_lock(&mpdMutex)<0) perror("pthread_mutex_lock");
#define MPDUNLOCK    if(pthread_mutex_unlock(&mpdMutex)<0) perror("pthread_mutex_unlock");
extern volatile struct mpd_struct *MPDp[(MPD_MAX_BOARDS+1)]; /* pointers to MPD memory map */
extern int mpdOutputBufferBaseAddr;               /* output buffer base address */
extern int mpdOutputBufferSpace;                  /* output buffer space (8 Mbyte) */
extern int mpdPrintDebug;
extern void mpdWrite32(volatile uint32_t * reg, uint32_t val);
#define DMA_BUFSIZE 40000

extern GEF_VME_BUS_HDL vmeHdl;
GEF_VME_DMA_HDL dmaHdl;
uint32_t vmeAdrs;
unsigned long physMemBase;
uint32_t *fBuffer; // DMA data buffer

int DMA_Init(int id, int fast_readout);
int DMA_Read(int id, int size, int *wrec);
int DMA_Free();

int sig_ctrl = 0; // 0 = force quit, 1 = soft quit ... when CTRL-C is pressed
int verbose_level = 0; // 0 = quit, 1 = some output, 2 = verbose output
void sig_handler(int signo);

int
main(int argc, char *argv[])
{
  int h,i,j,k,kk,m, evt;
  int fnMPD=10;
  int error_count;
  uint32_t hcount;
  int scount, sfreq;
  int sch0, sch1;
  int rdone, rtout;

  uint16_t mfull,mempty;
  uint32_t e_head, e_head0, e_size;
  uint32_t e_data32[130];
  uint32_t e_trai, e_eblo;

#define MAX_HDATA 4096
#define MAX_SDATA 1024
  uint32_t hdata[MAX_HDATA]; // histogram data buffer
  uint32_t sdata[MAX_SDATA]; // samples data buffer

  // command line parameters
  char outfile[1000];
  int acq_mode = 1;
  int n_event=10;
  int h_gain=5;
  int softri=1; // soft trigger flag, 1 if enabled
  int waitus=10000; // us sleep time between trigger

  int clp_clock0 = 0;
  int clp_clock1 = 50;
  int clp_clockd = 2;

  int mask_mpd = 0x0; // if bit set, corresponding MPD is masked; first bit is MPD with lower slot ID
  int mpd_slot0=999; // lowest MPD address (slot number), will be determined below

  FILE *fout;

  if (argc<2) {
    printf("SYNTAX: %s out_data_file [0xacq_mode par0 par1 par2 ...]\n",argv[0]);
    //#events gain]\n",argv[0]);
    printf("         par_# depends on acq_mode (%d):\n",acq_mode);
    printf("         acq_mode_bit0 (1) : EVENT readout\n");
    printf("               par0 = number of events (%d)\n",n_event);
    printf("               par1 = software trigger flag, 1 to enable (%d)\n",softri);
    printf("               par2 = us between trigger (%d)\n",waitus);
    printf("         acq_mode_bit1 (2) : SAMPLE check\n");
    printf("               par0 = min_clock_phase  (%d)\n",clp_clock0);
    printf("               par1 = max_clock_phase  (%d)\n", clp_clock1);
    printf("               par2 = clock_phase_step (%d)\n", clp_clockd);
    printf("         acq_mode_bit2 (4) : HISTO mode\n");
    printf("               par0 = gain (%d)\n",h_gain);
    printf("               par1 = MPD mask, if bit set corresponding MPD configuration disabled (0x%x)\n",mask_mpd);

    return 0;
  }

  if (argc>1) {
    sprintf(outfile,"%s",argv[1]);
  } else {
    sprintf(outfile,"out.txt");
  }

  if (argc>2) {
    sscanf(argv[2],"%x",&acq_mode);
  }

  if (argc>3) {
    sscanf(argv[3],"%d",&n_event);
    clp_clock0 = n_event;
    h_gain = n_event;
  }

  if (argc>4) {
    sscanf(argv[4],"%d",&clp_clock1);
    softri = clp_clock1;
    if (acq_mode==0x4) { sscanf(argv[4],"%x",&mask_mpd); } // Fixed 03/21 - sscanf(argv[4],"%x",&mask_mpd);
  }

  if (argc>5) {
    sscanf(argv[5],"%d",&clp_clockd);
  }

  printf("\nMPD Library Tests\n");
  printf("----------------------------\n");
  printf(" outfile = %s\n",outfile);
  printf(" acq_mode= 0x%x\n",acq_mode);

  signal(SIGINT, sig_handler);
  signal(SIGTSTP, sig_handler);

  if(vmeOpenDefaultWindows()!=OK)
    {
      printf("ERROR opening default VME windows\n");
      goto CLOSE;
    }

  // vmeDmaConfig(2,2,0); // A32, BLT32

  /**
   * Read config file and fill internal variables
   *
   */
  mpdConfigInit("/daqfs/daq_setups/vtp-mpdro/cfg/vme_config.cfg");
  mpdConfigLoad();

  printf(" CO)NFIGURATIN DFO\n");
  /**
   * Init and config MPD+APV
   */

  // discover MPDs and initialize memory mapping
  mpdInit(0x80000,0x80000,21,0x0);

  fnMPD = mpdGetNumberMPD();

  if (fnMPD<=0) { // test all possible vme slot ?
    printf("ERR: no MPD discovered, cannot continue\n");
    return -1;
  }

  for (k=0;k<fnMPD;k++) { // get lowest slot
    i = mpdSlot(k);
    mpd_slot0 = (i<mpd_slot0) ? i : mpd_slot0;
  }

  printf(" MPD discovered = %d starting from slot %d (MPD mask 0x%x)\n",fnMPD, mpd_slot0, mask_mpd);

  // APV configuration on all active MPDs
  for (k=0;k<fnMPD;k++) { // only active mpd and not masked will be set
    i = mpdSlot(k);

    if (mask_mpd & (1<<(i-mpd_slot0))) {
      printf(" MPD in slot %d disabled \n", i);
      mpdSetApvEnableMask(i,0);
      continue;
    }

    printf(" MPD Slot %d - HW Revision: %d - Firmware Revision: %d - Firmware Revision Time: %d\n",
	   i, mpdGetHWRevision(i), mpdGetFWRevision(i), mpdGetFpgaCompileTime(i));

    // first test MPD histo memory write/read
    printf(" test mpd %d histo memory\n",i);

    mpdHISTO_Clear(i,0,-1);
    mpdHISTO_Read(i,0,hdata);
    error_count=0;
    for (j=0;j<MAX_HDATA;j++) {
      if (hdata[j]!=j) {
	//	printf("ERROR matching histo read/write ch=%d rval=%d\n",j,hdata[j]);
	error_count++;
      }
    }
    if (error_count) {
      printf("ERROR: HISTO Test fail %d time / %d attempts\n",error_count, MAX_HDATA);
    } else {
      printf("HISTO Read/Write test SUCCESS on MPD slot %d\n",i);
    }
    // END of first test MPD histo memory write/read

    printf(" Try initialize I2C mpd in slot %d\n",i);
    if (mpdI2C_Init(i) != OK) {
      printf("WRN: I2C fails on MPD %d\n",i);
    }
    if (mpdI2C_ApvReset(i) != OK) {
      printf("WRN: I2C ApvReset fails on MPD %d\n",i);
    }

    printf("Try APV discovery and init on MPD slot %d\n",i);
    if (mpdAPV_Scan(i)<=0) { // no apd found, skip next
      //  continue;
    }

    // board configuration (APV-ADC clocks phase)
    printf("Do DELAY setting on MPD slot %d\n",i);
    mpdDELAY25_Set(i, mpdGetAdcClockPhase(i,0), mpdGetAdcClockPhase(i,1));

    // apv reset
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


  // Summary Printout
  printf("Configured APVs (ADC 15 ... 0)\n");
  for (k=0;k<fnMPD;k++) {
    i = mpdSlot(k);

    if (mpdGetApvEnableMask(i) != 0) {
      printf("MPD %2d : ",i);
      j=0;
      for (kk=15;kk>=0;kk--) {
	if (mpdGetApvEnableMask(i) & (1<<kk)) {
	  printf("1");
	  j++;
	} else {
	  printf(".");
	}
      }
      printf(" (#APV %d)\n",j);
    }
  }

  /**********************************
   * SAMPLES TEST
   * sample APV output at 40 MHz
   * only for testing not for normal daq
   **********************************/
  if (acq_mode & 0x2) {

    fout = fopen(outfile,"w");
    if (fout == NULL) { fout = stdout; }

    fprintf(fout,"# SAMPLE MODE OUTPUT\n");
    fprintf(fout,"# CLOCK_RANGE: %d %d %d\n",clp_clock0, clp_clock1, clp_clockd);

    for (k=0;k<fnMPD;k++) { // only active mpd set
      i = mpdSlot(k);

      mpdSetAcqMode(i, "sample");
      mpdSetEventBuilding(i, 0);

      // load pedestal and thr default values
      mpdPEDTHR_Write(i);

      for (kk=clp_clock0;kk<clp_clock1;kk+=clp_clockd) { // loop on clock phases

	// set clock phase
	mpdDELAY25_Set(i, kk, kk);

	// enable acq
	mpdDAQ_Enable(i);

	// wait for FIFOs to get full
	mfull=0;
	mempty=1;
	printf("MPD / Clock : %d %d\n",i, kk);

	rtout = 2;
	do {
	  mpdFIFO_GetAllFlags(i,&mfull,&mempty);
	  printf("SAMPLE test wait fifo full: %x (%x) %x\n",mfull,mpdGetApvEnableMask(i),mempty);
	  sleep(1);
	  rtout--;
	} while ((mfull!=mpdGetApvEnableMask(i)) && (rtout>0));

	// read data from FIFOs and estimate synch pulse period
	for (j=0;j<16;j++) { // 16 = number of ADC channel in one MPD
	  if (mpdGetApvEnableMask(i) & (1<<j)) { // apv is enabled
	    mpdFIFO_Samples(i,j, sdata, &scount, MAX_SDATA, &error_count);

	    printf("i: %d",i);
	    if (error_count != 0) {
	      printf("ERROR returned from FIFO_Samples %d\n",error_count);
	      continue;
	    }

	    fprintf(fout,"# MPD_ADC_COUNT_CLOCK_TOUT: %d %d %d %d %d\n",i, j, scount, kk, rtout);
	    printf("MPD/APV : %d / %d, peaks around: ",i,j);
	    sch0=-1;
	    sch1=-1;
	    sfreq=0;
	    for (h=0;h<scount;h++) { // detects synch peaks
	      //    printf("%04d ", sdata[h]); // output data
	      fprintf(fout,"%d ",sdata[h]);
	      if (sdata[h]>2500) { // threshold could be lower
	     	if (sch0<0) { sch0=h; sch1=sch0;} else {
		  if (h==(sch1+1)) { sch1=h; } else {
		    printf("%d-%d (v %d) ",sch0,sch1, sdata[h]);
		    sch0=h;
		    sch1=sch0;
		    sfreq++;
		  }
		}
	      }
	    }
	    fprintf(fout,"\n");
	    printf("\n Estimated synch period = %f (us) ,expected (20MHz:1.8, 40MHz:0.9)\n",((float) scount)/sfreq/40.0);
	  }
	}

      } // end loop on clock phases

    } // end loop on mpds

    if (fout != stdout)  fclose(fout);

  } // sample check mode

  /********************************************
   * HISTO Mode (obsolete); sampled data are histogrammed
   * only for testing, non for normal daq
   ********************************************/

  if (acq_mode & 0x8) {

    fout = fopen(outfile,"w");
    if (fout == NULL) { fout = stdout; }

    fprintf(fout,"# HISTO MODE OUTPUT (mpd adc gain ch0 ... ch4095)\n");

    for (k=0;k<fnMPD;k++) { // only active mpd set
      i = mpdSlot(k);

      mpdSetAcqMode(i, "histo");

      // load pedestal and thr default values
      mpdPEDTHR_Write(i);

      // set ADC gain
      mpdADS5281_SetGain(i,0,h_gain,h_gain,h_gain,h_gain,h_gain,h_gain,h_gain,h_gain);
      mpdADS5281_SetGain(i,1,h_gain,h_gain,h_gain,h_gain,h_gain,h_gain,h_gain,h_gain);

      // loop on adc channels

      for (j=0;j<16;j++) {

	mpdHISTO_Clear(i,j,0);

	mpdHISTO_Start(i,j);

	sleep(1); // wait to get some data

	mpdHISTO_Stop(i,j);

	hcount=0;
	mpdHISTO_GetIntegral(i,j,&hcount);
	error_count = mpdHISTO_Read(i,j, hdata);

	if (error_count != OK) {
	  printf("ERROR: reading histogram data from MPD %d ADCch %d\n",i,j);
	  continue;
	}

	printf(" ### histo peaks integral MPD/ADCch = %d / %d (total Integral= %d)\n",i,j,hcount);
	sch0=-1;
	sch1=0;

	fprintf(fout,"# MPD=%d\n# ADC=%d\n# GAIN=%d\n",i,j,h_gain);
	fprintf(fout,"%d %d %d",i,j,h_gain);

	for (h=0;h<4096;h++) {

	  fprintf(fout," %d",hdata[h]);

	  if (hdata[h]>0) {
	    if (sch0<0) { printf(" first bin= %d: ", h); } else {
	      if ((sch0+1) < h) { printf( "%d last bin= %d\n first bin= %d: ",sch1,sch0,h); sch1=0; }
	    }
	    sch1 += hdata[h];
	    sch0=h;
	  }

	  //	if ((h%64) == 63) { printf("\n"); }
	}

	fprintf(fout, "\n");

	if (sch1>0) {
	  printf( "%d last bin= %d\n",sch1,sch0);
	} else { printf("\n");}

      } // end loop adc channels in histo mode

    }

    if (fout != stdout)  fclose(fout);

  } // histo mode ends

    /********************************************
     * HISTO Mode speed optimized; sampled data are histogrammed
     * only for testing, non for normal daq
     ********************************************/

  if (acq_mode & 0x4) {

    fout = fopen(outfile,"w");
    if (fout == NULL) { fout = stdout; }

    fprintf(fout,"# MODE=%d\n# OUTPUT=mpd adc gain ch0 ... ch4095\n",acq_mode);

    for (k=0;k<fnMPD;k++) { // only active mpd set
      i = mpdSlot(k);

      mpdSetAcqMode(i, "histo");

      // load pedestal and thr default values
      mpdPEDTHR_Write(i);

      // set ADC gain
      mpdADS5281_SetGain(i,0,h_gain,h_gain,h_gain,h_gain,h_gain,h_gain,h_gain,h_gain);
      mpdADS5281_SetGain(i,1,h_gain,h_gain,h_gain,h_gain,h_gain,h_gain,h_gain,h_gain);
    }

    for (j=0;j<16;j++) {  // loop on adc channels

      for (k=0;k<fnMPD;k++) { // only active mpd set
	i = mpdSlot(k);
	//
	mpdHISTO_Clear(i,j,0);

	mpdHISTO_Start(i,j);

      }

      sleep(1); // wait to get some data

      for (k=0;k<fnMPD;k++) { // only active mpd set
	i = mpdSlot(k);

	mpdHISTO_Stop(i,j);

	hcount=0;
	mpdHISTO_GetIntegral(i,j,&hcount);
	error_count = mpdHISTO_Read(i,j, hdata);

	if (error_count != OK) {
	  printf("ERROR: reading histogram data from MPD %d ADCch %d\n",i,j);
	  continue;
	}

	printf(" ### histo peaks integral MPD/ADCch = %d / %d (total Integral= %d)\n",i,j,hcount);
	sch0=-1;
	sch1=0;

	fprintf(fout,"# MPD=%d\n# ADC=%d\n# GAIN=%d\n",i,j,h_gain);
	fprintf(fout,"%d %d %d",i,j,h_gain);

	for (h=0;h<4096;h++) {

	  fprintf(fout," %d",hdata[h]);

	  if (hdata[h]>0) {
	    if (sch0<0) { printf(" first bin= %d: ", h); } else {
	      if ((sch0+1) < h) { printf( "%d last bin= %d\n first bin= %d: ",sch1,sch0,h); sch1=0; }
	    }
	    sch1 += hdata[h];
	    sch0=h;
	  }

	  //	if ((h%64) == 63) { printf("\n"); }
	}

	fprintf(fout, "\n");

	if (sch1>0) {
	  printf( "%d last bin= %d\n",sch1,sch0);
	} else { printf("\n");}

      }

    } // end loop adc channels in histo mode

    if (fout != stdout)  fclose(fout);


/*
    mpdGStatus(0);
    for (k=0;k<fnMPD;k++) {
      i = mpdSlot(k);
      mpdApvStatus(i, 0xFFFF);
    }
*/




  } // histo mode ends

    /**************************
     * "Event" readout
     *  normal DAQ starts here
     **************************/

  if ((acq_mode & 1) && (mpdGetEventBuilding(mpdSlot(0)) == 0)) {

#define MPD_TIMEOUT 100
//Output file TAG
#define VERSION_TAG 0xE0000000
#define EVENT_TAG   0x10000000
#define MPD_TAG     0x20000000
#define ADC_TAG     0x30000000
#define HEADER_TAG  0x40000000
#define DATA_TAG    0x0
#define TRAILER_TAG 0x50000000

#define FILE_VERSION 0x1
    // open out file

    fout = fopen(outfile,"w");
    if (fout == NULL) { fout = stdout; }
    fprintf(fout,"%x\n", FILE_VERSION | VERSION_TAG);

    for (k=0;k<fnMPD;k++) { // only active mpd set
      i = mpdSlot(k);

      // mpd latest configuration before trigger is enabled
      mpdSetAcqMode(i, "event");

      // load pedestal and thr default values
      mpdPEDTHR_Write(i);

      // enable acq
      mpdDAQ_Enable(i);

    }
    // -> now trigger can be enabled

    evt=0;
    vmeDmaConfig(1,2,0);

    printf(" ============ START ACQ (NO Evt Bulfer) ===========\n");

    do { // simulate loop on trigger

      if (softri) { // issue soft trigger
	for (k=0;k<fnMPD;k++) {
	  i = mpdSlot(k);
	  mpdAPV_SoftTrigger(i);
	}
      }
      usleep(waitus);

      printf(" ---- Event %d occurred -----\n",evt);
      // event occurred ... need some trigger logic
      rtout=0;

      for (k=0;k<fnMPD;k++) { // only active mpd set
	i = mpdSlot(k);
	mpdArmReadout(i); // prepare internal variables for readout
      }

      rdone = 1;

      fprintf(fout,"%x\n", evt | EVENT_TAG ); // event number
      for (kk=0;kk<fnMPD;kk++) { // only active mpd set
	i = mpdSlot(kk);
	if (mask_mpd & (1<<(i-mpd_slot0))) { continue; }
	//usleep(10000);
	mpdFIFO_ClearAll(i); // @@@ THIS SEEMS TO BE IN THE WRONG PLACE, should go above! before any event @@@
	mpdTRIG_Disable(i);
	//	  	  mpdDAQ_Enable(i);
	//mpdTRIG_PauseEnable(i,5000);

	do { // wait for data in MPD
	  mpdTRIG_PauseEnable(i,200000);

	  rdone = mpdFIFO_ReadAll(i,&rtout,&error_count);

	  printf(" Rdone/Tout/error = %d %d %d\n", rdone, rtout, error_count);
	  rtout++;

	} while ((rdone == 0) && (error_count == -1) && (rtout < MPD_TIMEOUT)); // timeout can be changed
	//	mpdTRIG_Disable(i);

	if ((error_count != 0) || (rtout > MPD_TIMEOUT)) { // reset MPD on error or timeout
	  printf("%s: ERROR in readout, clear fifo\n",__FUNCTION__);
	  mpdFIFO_ClearAll(i);
	  mpdTRIG_Enable(i);
	}

	if (rdone) { // data need to be written on file

	  printf("%d",i); // slot
	  fprintf(fout,"%x\n", i | MPD_TAG);

	  for (j=0; j < mpdGetNumberAPV(i); j++) { // loop on APV (ADC channels)

	    fprintf(fout,"%x\n", (mpdApvGetAdc(i,j) | ADC_TAG));
	    k=0; // buffer element index
	    for (h=0; h<mpdApvGetBufferSample(i,j); h++) { // loop on samples
	      e_size = mpdApvGetEventSize(i,j);
	      e_head0 = mpdApvGetBufferElement(i, j, k);
	      k++;
	      e_head = ((e_head0 & 0xfff) << 4) | (mpdApvGetAdc(i,j) & 0xf);
	      fprintf(fout,"%x\n", e_head0 | HEADER_TAG);
	      // fwrite e_head
	      for (m=0;m<e_size-2;m++) {
		e_data32[m] = 0x80000 | ((i<<12) & 0x7F000) | (mpdApvGetBufferElement(i,j,k) & 0xfff);
		fprintf(fout,"%d\n",(mpdApvGetBufferElement(i,j,k) & 0xfff) | DATA_TAG );
		k++;
	      }
	      // fwire e_data32 (e_size-2)
	      e_trai = 0x100000 | ((i & 0x1F) << 12) | (mpdApvGetBufferElement(i,j,k)& 0xfff);
	      e_eblo = 0x180000 | (e_size & 0xff);
	      fprintf(fout,"%x\n",(mpdApvGetBufferElement(i,j,k) & 0xfff) | TRAILER_TAG);
	      k++;
	      // fwrite e_trai // trailer
	      // fwrite e_eblo // end sample block
	    }
	    mpdApvShiftDataBuffer(i,j,k);
	  } // end loop on apv

	} // if rdone

      } // end mpd loop

	// e_head = 0x40000;
	// fwrite end block when loop on MPD
      evt++;
      //  mpdFIFO_ClearAll(i);
    } while (evt<n_event); // end loop on events

    if (fout != stdout)  fclose(fout);

  }

  // -----------------------------
  // EVENT_BUILDER ENABLED in MPD
  // -----------------------------

  if ((acq_mode & 1) && (mpdGetEventBuilding(mpdSlot(0)) != 0)) {

    int mpd_evt[21];
    int UseSdram, FastReadout;
    int empty, full, nwords,obuf_nblock, evb_nblock, old_nblock;
    int nwread;
    int iw, blen;

    // open out file
    FILE *fout;
    fout = fopen(outfile,"w");
    if (fout == NULL) { fout = stdout; }

    fprintf(fout,"%x\n", mpdGetApvEnableMask(mpdSlot(0)));
    fprintf(fout,"%x\n",  mpdGetTriggerNumber(mpdSlot(0))*mpdApvGetPeakMode(mpdSlot(0)));

    //    fprintf(fout,"%x\n", FILE_VERSION | 0xD0000000);

    UseSdram = mpdGetUseSdram(mpdSlot(0)); // assume sdram and fastreadout are the same for all MPDs
    FastReadout = mpdGetFastReadout(mpdSlot(0));

    printf(" UseSDRAM= %d , FastReadout= %d\n",UseSdram, FastReadout);

    for (k=0;k<fnMPD;k++) { // only active mpd set
      if (k==0) { evt = mpd_evt[i]; }
      i = mpdSlot(k);

      // mpd latest configuration before trigger is enabled
      mpdSetAcqMode(i, "process");

      // load pedestal and thr default values
      mpdPEDTHR_Write(i);

      // enable acq
      mpdDAQ_Enable(i);

      mpdTRIG_Enable(i);

      mpd_evt[i]=0;

      DMA_Init(i, FastReadout);	// this will work with 1 MPD only! TO BE IMPROVED
    }
    // -> now trigger can be enabled
    sig_ctrl = 1;
    printf(" ============ START ACQ MPD EVT_BUILDER (#mpd = %d)===========\n",fnMPD);
    printf(" CTRL-Z to cycle verbose level, CTRL-C to quit\n");


    evt=0;
    verbose_level=2;

    do { // simulate loop on trigger

      printf("    |-Buffer LEN-| |-------- SDRAM Status ---------| |----- OBUFF status -----|\n");
      printf("MPD    32bwords    init overrun rdaddr wraddr nwords empty full nwords  nblocks\n");

      if (softri) { // issue soft trigger
	for (k=0;k<fnMPD;k++) {
	  i = mpdSlot(k);
	  mpdAPV_SoftTrigger(i);
	}
      }
      usleep(waitus); // wait for "event"

      // event occurred ... need some trigger logic
      rtout=0;

      if( (evt%100) == 0 && evt != 0 )
        printf(" ---- At least %d events acquired on each active MPDs -----\n",evt);

      for (k=0;k<fnMPD;k++) { // only active mpd set
	i = mpdSlot(k);
	if (mask_mpd & (1<<(i-mpd_slot0))) { continue; }
	vmeSetQuietFlag(1);
	vmeClearException(1);
	mpdArmReadout(i); // prepare internal variables for readout @@ use old buffer scheme, need improvement
//	blen = mpdApvGetBufferAvailable(i, 0);
	blen = DMA_BUFSIZE;
	//	if( verbose_level > 1 ) printf("MPD %d: Buffer LEN = %d (bytes) -> %d (32b words)\n",i,blen,blen/4);
        if (verbose_level > 1) printf(" %2d   %10d   ",i,blen/4);
	nwread = 0;

	if ( UseSdram ) {

	  int sd_init, sd_overrun, sd_rdaddr, sd_wraddr, sd_nwords;

	  mpdSDRAM_GetParam(i, &sd_init, &sd_overrun, &sd_rdaddr, &sd_wraddr, &sd_nwords);
	  /*
	  if( verbose_level > 0 )
	    printf("MPD %2d: SDRAM status: init=%d, overrun=%d, rdaddr=0x%x, wraddr=0x%x, nwords=%d\n",
	    	   i, sd_init, sd_overrun, sd_rdaddr, sd_wraddr, sd_nwords);
	  */
	  if( verbose_level > 0 ) printf(" %1d  %1d  0x%6x 0x%6x %6d ", sd_init, sd_overrun, sd_rdaddr, sd_wraddr, sd_nwords);

	  obuf_nblock = mpdOBUF_GetBlockCount(i);
	  mpdOBUF_GetFlags(i, &empty, &full, &nwords);

	  /*
	  if( verbose_level > 0 )
	    printf("MPD %d: OBUFF status: empty=%d, full=%d, nwords=%d : nblock %d\n",i, empty, full, nwords, obuf_nblock);
	  */
	  if( verbose_level > 0 ) printf(" %d %d %d %d\n", empty, full, nwords, obuf_nblock);

	  if (FastReadout>0) { //64bit transfer
	    if ( nwords < 128 ) { empty = 1; } else { nwords *= 2; }
	  }

	  if ( obuf_nblock > 0) { // read data

	    if( verbose_level > 0 )
	      printf("MPD %d: Data waiting to be read in obuf: %d (32b-words)\n",i, nwords);

	    if (nwords > blen/4) { nwords = blen/4; }
	    nwords = (nwords/4)*4; // 128 bit boundary
//	    mpdOBUF_Read(i, nwords, &nwread);
//  	    mpdOBUF_GetFlags(i, &empty, &full, &nwords);	// DUMMY
	    DMA_Read(i, nwords, &nwread);
	    if( verbose_level > 0 )
	      printf("MPD %d: try to read %d 32b-words 128b-aligned from obuf, got %d 32b-words\n",i, nwords, nwread);

	    if (nwords != nwread ) {
	      printf("MPD %d: Error: 32bit-word read count does not match %d %d\n", i, nwords, nwread);
	    }

	  }
          else
		usleep(10);

	} else { // if not Sdram

	  mpdFIFO_IsEmpty(i, 0, &empty); //  read fifo channel=0 status

	  if (!empty) { // read fifo
	    nwread = blen/4;
	    mpdFIFO_ReadSingle(i, 0, mpdApvGetBufferPointer(i, 0, 0), &nwread, 20);
	    if (nwread == 0) {
	      printf("MPD %d: Error: word read count is 0, while some words are expected back\n",i);
	    }
	  }

	}

	if (nwread>0) { // data need to be written on file

	  int zero_count;
	  zero_count=0;
	  if( verbose_level > 1 )
	    printf("MPD Slot: %d (dump on screen first 24 and last 24 words)\n",i); // slot
	  // fprintf(fout,"%x\n", i | MPD_TAG);
	  for(iw=0; iw<nwread; iw++) {
	    uint32_t datao;
	    if( UseSdram )
	      datao = fBuffer[iw];
	    else
	      datao = mpdApvGetBufferElement(i, 0, iw);

	    fprintf(fout,"%x\n",datao);
	    if(iw == (nwread-1) )
	      fprintf(fout,"end\n");
	    if (datao ==0) {
	      zero_count++;
	    }
	    if ( verbose_level > 1 && ((iw<24) || ((nwread-iw)<25))) {

	      printf(" 0x%x",datao);

	      if (((iw %8)==7)|| (iw==(nwread-1))) {
		printf("\n");
	      }

	    }
	    if (verbose_level > 1 && iw==24) { printf(" ....\n"); }

	    if( (datao & 0x00E00000) == 0x00A00000 ) { // EVENT TRAILER
	      mpd_evt[i]++;
	      evt=mpd_evt[i];
	    }
	    evt = (evt > mpd_evt[i]) ? mpd_evt[i] : evt; // evt is the smallest number of events of an MPD
	  }

	  if( verbose_level > 0 )
	    printf("MPD %d: nwords=%d nwcount=%d zero_count=%d evt %d\n",i,nwords,nwread,zero_count,mpd_evt[i]);

	  printf(" press return to continue\n");
	  getchar();
	}
      } // active mpd loop

    } while ((evt<n_event) && (sig_ctrl < 2));  // events loop

    if (fout != stdout)  fclose(fout);

  }

 CLOSE:

  for (k=0;k<fnMPD;k++)
    {
      mpdDAQ_Disable(mpdSlot(k));
      mpdFiberEnable(mpdSlot(k));
    }
  DMA_Free();
  vmeCloseDefaultWindows();

  exit(0);

}


void sig_handler(int signo)
{
  //  int i, status;
  int k, fnMPD;

  switch (signo) {
  case SIGINT:
    printf("\nCTRL-C pressed, (level = %d)\n\n",sig_ctrl);

    if (sig_ctrl == 0) { // force quit
      fnMPD = mpdGetNumberMPD();
      for (k=0;k<fnMPD;k++)
        mpdDAQ_Disable(mpdSlot(k));
      vmeCloseDefaultWindows();
      exit(1);  /* exit if CRTL/C is issued */
    }

    if (sig_ctrl == 1) { // soft quit
      sig_ctrl = 2;
    }
    break;
  case SIGTSTP:
    verbose_level += 1;
    verbose_level = verbose_level % 3;
    printf("Verbose level = %d\n",verbose_level);
    break;

  }
  return;
}



/* DMA routines
 */
int DMA_Init(int id, int fast_readout)
{
  GEF_STATUS status;
  GEF_MAP_PTR mapPtr;
  GEF_VME_DMA_HDL dma_hdl;
  int fBufSize = DMA_BUFSIZE;
  uint32_t data;

  switch (fast_readout) {
  case 0:
    vmeDmaConfig(2,2,0); // A32 BLT
    break;
  case 1:
    vmeDmaConfig(2,3,0); // MBLT
    break;
  case 2:
    vmeDmaConfig(2,4,0); // 2eVME
    break;
  case 3:
    vmeDmaConfig(2,5,0); // 2esst160
    break;
  case 4:
    vmeDmaConfig(2,5,1); // 2esst266
    break;
  case 5:
    vmeDmaConfig(2,5,2); // 2esst320
    break;
  default:
    vmeDmaConfig(2,2,0); // A32 BLT
    break;
  }
  status = gefVmeAllocDmaBuf (vmeHdl, fBufSize, &dma_hdl,&mapPtr);
  if(status != GEF_STATUS_SUCCESS)
    {
      MPD_ERR("\n\tgefVmeAllocDmaBuf returned 0x%x\n",status);
    }
  fBuffer     = (uint32_t *)mapPtr;
  physMemBase = dmaHdl_to_PhysAddr(dma_hdl);
  dmaHdl      = dma_hdl;
  vmeAdrs = (uint32_t) mpdOutputBufferBaseAddr + mpdOutputBufferSpace * id;
  printf("Buffer allocated with word size %d\n\tdmaHdl = 0x%lx  physMemBase = 0x%08x  fBuffer = 0x%lx  vmeAdrs = 0x%08x\n",
          fBufSize,
          (uintptr_t)dmaHdl,
          (uint32_t)physMemBase,
          (uintptr_t)fBuffer, (uint32_t) vmeAdrs);

  data = vmeAdrs >> 2;
  MPDLOCK;
  mpdWrite32(&MPDp[id]->obuf_base_addr, data);
  MPDUNLOCK;
  return OK ;
}

int DMA_Read(int id, int size, int *wrec)
{
  int retVal=0;
  int dummy1, dummy2, dummy3;

  MPDLOCK;
//printf("DMA_Read(): before vmeDmaSendPhys: physMemBase = 0x%08x, vmeAdrs = 0x%08x\n", physMemBase, vmeAdrs);
  retVal = vmeDmaSendPhys(physMemBase,vmeAdrs,(size<<2));
//printf("DMA_Read(): after vmeDmaSendPhys retval = %d\n", retVal);

  if(retVal != 0)
    {
      MPD_ERR("DMA transfer Initialization (returned 0x%x)\n",retVal);
      MPD_ERR("  id=%d apv physMemBase = 0x%08x\n",
              id,(uint32_t)physMemBase);
      *wrec=0;
      MPDUNLOCK;
      return(retVal);
    }

  /* Wait until Done or Error */
//printf("DMA_Read(): before vmeDmaDone\n");
  retVal = vmeDmaDone();
  MPDUNLOCK;
//printf("DMA_Read(): after vmeDmaDone retval = %d\n", retVal);
  mpdOBUF_GetFlags(id, &dummy1, &dummy2, &dummy3);
//printf("DMA_Read(): after mpdOBUF_GetFlags dunny1 = %d, dummy2 = %d, dummy3 = %d\n", dummy1, dummy2, dummy3);
  if(retVal==0)
    {
      *wrec=0;
      MPD_ERR("vmeDmaDone returned zero word count\n");

      return ERROR;
    }
  else if(retVal==ERROR)
      {
        *wrec=0;
        MPD_ERR("vmeDmaDone returned ERROR\n");

        return ERROR;
      }
  else
    {
      *wrec   = (retVal>>2);
//      MPD_DBG("vmeDmaDone returned 0x%x (%d)  wrec = %d\n",
//            retVal, retVal, *wrec);
      int iword=0;
      for(iword =0; iword<*wrec; iword++)
        {
          /* Byte swap necessary for block transfers */
          fBuffer[iword] = LSWAP(fBuffer[iword]);
        }
    }
  if( *wrec != size ) {
    MPD_DBG("Count Mismatch: %d expected %d\n", *wrec, size);
    return ERROR;
  }

  return OK;

}

int DMA_Free()
{
  GEF_STATUS status;
  if (fBuffer != 0)
    {
      status = gefVmeFreeDmaBuf(dmaHdl);
      if(status != GEF_STATUS_SUCCESS)
        {
          MPD_ERR("gefVmeFreeDmaBuf returned 0x%x\n",status);
        }
    }
  return OK;
}

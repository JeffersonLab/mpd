/** \file
 *
 * High level Multi Purpose Digitizer object
 *
 * TO DO: use the official logger!
 *
 */

#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <iomanip>
#include <ios>
#include <iostream>

#include "MPDhi.h"
#include "GI_Common.h"

#define MAX_SAMPLES	8192


//

MPDhi::MPDhi(int debug) {
  
  fMPDcount = 0;

  fDebugLevel=debug;

  fReadMask=0;
  fTimeout = 0;

  std::cout << __FUNCTION__ << ": allocated" << std::endl;;

  fMPD.clear();

}

//

MPDhi::~MPDhi() {

  DisableBoards();
  fMPD.clear();
}

/**
 * Obsolete
 *
 */
int MPDhi::CommonLatAndInit(int lat) { // configure common latency and initialize boards for daq

  unsigned int i;
  int j;
  char logstring[512];

  std::cout << __FUNCTION__ << ": started" << std::endl;;

  for(i=0; i<fMPD.size(); i++)
    for(j=0; j<fMPD[i]->GetApvCount(); j++)
      fMPD[i]->fApv[j].Latency = lat;
  
  sprintf(logstring, "MPD board %d common latency set to %d", i, lat);
  std::cout << logstring << std::endl;;

  InitAndReset();
  
  std::cout << __FUNCTION__ << ": done" << std::endl;;

  return 0;

};

int MPDhi::InitAndReset() {  // intialize and reset FEC
  
  unsigned int i;
  //  char logstring[512];
  
  MPD_DBG("started\n");
  
  for(i=0; i<fMPD.size(); i++)
    {
      MPDlo* ff;
      ff=fMPD[i]; /// why ???
      ff->BOARD_Init();
      fMPD[i]->APV_Reset101();
      //     sprintf(logstring, "MPD board %d:%d initialized", ff->GetBus(), ff->GetSlot());
      //      std::cout << logstring << std::endl;;
      //		fMPD[i]->DAQ_Enable();
    }
  
  MPD_DBG("done\n");

  return 0;
};

/**
 * Obsolete
 */

int MPDhi::LoadPedAndThr(int ped, int thr) {
  unsigned int i,j;
    
  for(j=0; j<fMPD.size(); j++) {
    for (i=0;i<8;i++) {
      fMPD[j]->PED_Write(i, ped);
      fMPD[j]->THR_Write(i, thr);
    }
  }
  
  return 0;

};

/**
 *
 */
int MPDhi::LoadPedAndThr() {
  unsigned int j;
    
  for(j=0; j<fMPD.size(); j++) {
    fMPD[j]->PEDTHR_Write();
  }
  
  return 0;

};

/**
 *
 *
 */

int MPDhi::EnableAcq() {

  unsigned int j;
  for(j=0; j<fMPD.size(); j++) {
    fMPD[j]->DAQ_Enable();
    fBrdFull[j] = false;
  }
  
  return 0;
  
};

  // return == 0 if fifo is full, >0 if not full

int MPDhi::CheckFIFOnotFull() {

  int y;
  unsigned int j;
  uint16_t full, empty;

  for(j=0; j<fMPD.size(); j++)
    {
      fMPD[j]->FIFO_GetAllFlags(&full, &empty);
      if( full == fMPD[j]->GetApvEnableMask() )
	fBrdFull[j] = true;
    }
  y = fMPDcount;
  for(j=0; j<fMPD.size(); j++)
    if( fBrdFull[j] ) y--;
  
  return y;
  
};

//

int MPDhi::FifoClearAndTriggerEnable() {

  unsigned int j;

  for(j=0; j<fMPD.size(); j++) {
    fMPD[j]->FIFO_ClearAll();
    fMPD[j]->TRIG_Enable();
  }
  
  return 0;
};


void MPDhi::HISTO_Clear(int histo_board, int histo_ch, int val) {

  fMPD[histo_board]->HISTO_Clear(histo_ch, val);

};


void MPDhi::HISTO_Start(int histo_board, int histo_ch) {

  fMPD[histo_board]->HISTO_Start(histo_ch);
};

void MPDhi::HISTO_Stop(int histo_board, int histo_ch) {

  fMPD[histo_board]->HISTO_Stop(histo_ch);
  
};

uint32_t MPDhi::HISTO_Read(int histo_board, int histo_ch, uint32_t *data) {

  uint32_t count;

  count = 0;
  //  std::cout << __FUNCTION__ << " HistoStop Return  = " << fMPD[histo_board]->HISTO_Stop(histo_ch) << " (0=ok)" << std::endl;
  std::cout << __FUNCTION__ << " HistoRead Return  = " << fMPD[histo_board]->HISTO_Read(histo_ch, data)<< " (0=ok)" <<  std::endl;
  std::cout << __FUNCTION__ << " HistoInt. Return  = " << fMPD[histo_board]->HISTO_GetIntegral(histo_ch, &count) << " (0=ok)" << std::endl;

  return count;

};

/**
 * check synch pulse latched. useful for clock phase and digital threshold optimization 
 * NOT sure l1 is significant !
 */

int MPDhi::FIFO_Synch(int l0_min, int l0_max, int l1_min, int l1_max, int l_step, 
		      int phase_min, int phase_max, int phase_step) {

  // l0_max, l0_min l_step
  // l1_max, l1_min
  // phase_max, phase_min, phase_step

  int tcount, ccount;
  int scount[16];

  int syn;
  int l0, l1, i;
  std::cout << "FIFO_sync started with " << fMPD.size() << std::endl;

  int phase;
  int llow,lhigh;

  std::cout << "SYNCH: # clock_phase cloch_phase_ns mpd_index l0_low l1_high l_difference [16xsingle_channel_count_eff.] coinc. all_count" << std::endl;
  for (i=0;i<(int)fMPD.size();i++) {
    std::cout << "SYNCH_: " << i << " APV enable mask= 0x" << (std::hex) << fMPD[i]->GetApvEnableMask() << (std::dec) << std::endl;

    for (phase=phase_min;phase<phase_max;phase+=phase_step) { // clock phase
      fMPD[i]->DELAY25_Set(phase,phase);
      llow=99999;
      lhigh=-99999;

      tcount=0;
      ccount=0;
      for (int si=0;si<16;si++) { // count synch reset
	scount[si] = 0;
      }

      for (l0=l0_max;l0>=l0_min;l0-=l_step) {
	for (l1=l1_min;l1<=l1_max;l1+=l_step) {
	  fMPD[i]->SetZeroLevel(l0);
	  fMPD[i]->SetOneLevel(l1);
	  fMPD[i]->SetTriggerMode(1,1);
	  //	fMPD[i]->SetApvEnableMask(0xffff);

	  EnableAcq();
	  usleep(50);
	  
	  fMPD[i]->FIFO_AllSynced(&syn);
	  
	  //	std::cout << i << " " << l0 << " " << l1 << " " << (std::hex) << syn << (std::dec) << std::endl;
	
	  for (int si=0;si<16;si++) { // count synch
	    scount[si] += (syn>>si)&1;
	  }
	  tcount++;
	  if (syn == fMPD[i]->GetApvEnableMask())  {
	    llow = (l0<llow) ? l0 : llow;
	    lhigh = (l1>lhigh) ? l1 : lhigh;
	    ccount++;
	    //	  std::cout << i << " " << l0 << " " << l1 << " " << (std::hex) << syn << " " << fMPD[i]->GetApvEnableMask() << (std::dec) << std::endl;
	  }
	  //	  break;
	}
      }
      //      std::cout << "SYNCH: " << phase << " " << phase*0.5 << " " << i << " " << llow << " " << lhigh << " " << lhigh-llow << std::endl;
      GI_INF("%3d %5.1f %3d %4d %4d %4d [",phase,phase*0.5,i,llow,lhigh,lhigh-llow);
      for (int si=0;si<16;si++) {
	printf(" %3.0f",((float) scount[si])/((float) tcount)*100.);
      }
      printf("] %3.0f %d\n",((float) ccount)/((float) tcount)*100., tcount);
    }
  }

  return 0;

};

uint32_t MPDhi::getMissedTriggers(int i) { 

  uint32_t missed_triggers;
  fMPD[i]->TRIG_GetMissed(&missed_triggers);

  return missed_triggers;
};

/**
 * Discover the VME MPD and register them into the MPD structure
 */

int MPDhi::CountAndRegisterBoards() {

  unsigned int i;
  int tcnt, mcnt;
  uint32_t rev, timec;
  time_t tt;
  char logstring[512];

  if (fMPD.size()<=0) {
    std::cout << __FUNCTION__ << ": Error, no MPD allocated from configuration file, must quit" << std::endl;;
    exit(1);
  }

  std::cout << __FUNCTION__ << ": started, scanning the bus " << fBusIdx << std::endl;;

  //  fMPDcount = 0;
  tcnt = 0; // total boards in bus
  mcnt = 0; // boards in bus that match configuration

  // ... dummy scan the bus to discover MPDs, not strictly needed
  for(i=1; i<MAX_BOARDS; i++) { // @@@@ to be changed .., slot start from 1 
    if( fMPD[0]->CR_Scan(i, &rev, &timec) == BUS_OK ) {
      tt = timec;
      std::cout << __FUNCTION__ << " MPD found in bus:slot " << fBusIdx << ":" << i 
		<< " HW rev / FW rev / time= " << ((rev >> 24) & 0xff) << " / " << (rev & 0xff) << " / (" << timec << ") " << ctime(&tt);
      tcnt++;
    }
  }

  for(i=0; i<fMPD.size(); i++) { // scan only mpd defined in configuration file
    if (fMPD[i]->GetBus() != fBusIdx) { continue; }
    if( fMPD[i]->CR_Scan(-1, &rev, &timec) == BUS_OK ) {
      std::cout << __FUNCTION__ << " MPD in slot " << fMPD[i]->GetSlot() << " of bus " << fBusIdx << " match configuration settings" << std::endl;;
      fMPD[i]->SetFpgaRevision(rev);
      fMPD[i]->SetFpgaCompileTime(timec);
      fMPD[i]->Enable();
      mcnt++;
    }
  }
  
  std::cout << __FUNCTION__ << ": " << tcnt << " MPD boards detected, " << mcnt << " matching config on bus " << fBusIdx << std::endl;;

  if( mcnt == 0 ) {
      std::cout << __FUNCTION__ << ": No MPD Board Found on bus matching config ... quit!" << std::endl;;
      exit(1);
  } 

  std::vector<MPDlo *>::iterator it=fMPD.begin();
  while (it<fMPD.end()) { // @@@ can be integrated with the previous loop

    if (!(*it)->IsEnabled()) { // no MPD found on bus, remove from config
      sprintf(logstring, "Erase MPD setting in Bus %d, Slot %d", (*it)->GetBus(), (*it)->GetSlot());
      it = fMPD.erase(it);
      fMPDcount--; // @ must be removed
    } else {
      sprintf(logstring,
	      "Found MPD in Bus %d, Slot %d @ address 0x%08X, FPGA revision = %d (ctime = %d)",
	      (*it)->GetBus(), (*it)->GetSlot(), (uint32_t) (*it)->GetBaseAddress(), 
	      (*it)->GetFpgaRevision(), (*it)->GetFpgaCompileTime());
      it++;
    }
    std::cout << logstring << std::endl;;
  }
  
  if (fMPDcount >0) { fReadMask = new uint16_t[fMPDcount]; }
 
  std::cout << __FUNCTION__ << ": " << fMPDcount << " MPDs found on bus and matching settings" << std::endl;;

  return fMPDcount;

}

int MPDhi::DiscoverFEC() {

  unsigned int i;
  int j;
  char logstring[512];

  MPD_DBG("started\n");

  std::vector<MPDlo *>::iterator it;
  for (it= fMPD.begin(); it<fMPD.end(); it++) { // Scanning for APVs, loop on MPD boards
    if( (*it)->I2C_Init() != BUS_OK ) { MPD_WRN("I2C_Init fails on MPD %d\n", (*it)->GetSlot());}
    if( (*it)->I2C_ApvReset() != BUS_OK ) { MPD_WRN("I2C_ApvReset fails on MPD %d\n", (*it)->GetSlot());}
    
    (*it)->APV_Scan();

  }
  
  for(i=0; i<fMPD.size(); i++)	// Reporting
    {
      sprintf(logstring, "MPD[%d] has %d APV connected and matching settings", i, fMPD[i]->GetApvCount());
      std::cout << logstring << std::endl;;

      for(j=0; j<fMPD[i]->GetApvCount(); j++)
	{
	  sprintf(logstring, "MPD[%d] Apv[%d] I2C address =  %d",
		  i, j, fMPD[i]->fApv[j].i2c);
	  std::cout << logstring << std::endl;;
	}
    }

  MPD_DBG("done on all %d MPDs\n",(int) fMPD.size());

  return 0;

}

//

int MPDhi::DisableBoards() {

  unsigned int i;

  char logstring[512];

  for(i=0; i<fMPD.size(); i++)
    {
      fMPD[i]->DAQ_Disable();
      sprintf(logstring, "MPD %d disabled", i);
      std::cout << logstring << std::endl;;
     }

  return i;

}

//

int MPDhi::ReadSampleData(FILE *fout, int format, int event) {

  int i;
  unsigned int j, k;

  int err;
  int SampleCount, totCount;
  uint32_t SampleBuffer[MAX_SAMPLES];
  
  totCount = 0;
  for(j=0; j<fMPD.size(); j++) {
    for(k=0; k<16; k++) {
      if( fMPD[j]->GetApvEnableMask() & (1 << k) ) {
	fMPD[j]->FIFO_Samples(k, SampleBuffer, &SampleCount, MAX_SAMPLES, &err);
	if( err != 0 ) {
	  std::cout << "ReadoutSamples() returned error = " << err << std::endl;;
	}
	fprintf(fout, "Event %d Board %d Channel %d Samples %d\n", event, j, k, SampleCount);

	totCount+=SampleCount;

	for(i=0; i<SampleCount; i++) {
	  if( (i % 16) == 0 && i > 0 )
	    fprintf(fout, "\n");
	  if( format == 1 ) // to be improved
	    fprintf(fout, "%04x ", (uint32_t)SampleBuffer[i]);
	  else
	    fprintf(fout, "%04d ", (uint32_t)SampleBuffer[i]);
	}
	fprintf(fout, "\n");
      }
    }
  }
  
  return totCount;
  
}

//

int MPDhi::GetNumberEnabledCards() {

  unsigned int i,j;
  int count;

  count = 0;
  for (i=0; i<fMPD.size(); i++) {
    std::cout << __FUNCTION__ << ": MPD/Enabled Mask = " << i << " / 0x" << (std::hex) << fMPD[i]->GetApvEnableMask() << (std::dec) << std::endl; 

    for (j=0;j<16;j++) {
      count += (fMPD[i]->GetApvEnableMask() >> j) & 0x1;
    }
  }

  return count;

}

/**
 * Arm readout
 */
int MPDhi::ArmReadout() {

  unsigned int i;

  for (i=0; i<fMPD.size(); i++) {
    fMPD[i]->ArmReadout();
  }
  fTimeout = 0;

  return 0;

}

/**
 *
 */
unsigned char MPDhi::tBuildMode(int pol, int fre, int rom, int cal, int sam, int ana) {
  
  unsigned char out;
  
  cal = 1-cal; // invert
  
  out = ((pol & 0x1) << 5)
    | ((fre & 0x1) << 4)
    | ((rom & 0x1) << 3)
    | ((cal & 0x1) << 2)
    | ((sam & 0x1) << 1)
    | (ana & 0x1);
  
  return out;
  
}

/**
 * Read one full event (on all configured MPDs)
 * data is stored in internal memory structure
 *
 * return true if done, false on error or timeout
 */

bool MPDhi::EVENT_Read(int max_retry) {

  unsigned int j;
  int itimeout;
  bool mDone;
  int global_fifo_error;
  MPDlo *mm;

  ArmReadout(); // prepare internal variables for readout

  itimeout = 0;
  do { // loop on data available on fifos

    mDone = true;

    MPD_DBG("Number of MPDs = %d\n",(int) fMPD.size());

    for(j=0; j<fMPD.size(); j++)  { // loop on VME boards
      mm = fMPD[j];
      mDone &= mm->FIFO_Read(itimeout, global_fifo_error);
      MPD_DBG("Fifo read done, mDone=%d, Err = 0x%x Timeout=%d (max=%d)\n",mDone,global_fifo_error, itimeout,max_retry);
    }
    itimeout++; // seems may run in a infinite loop without this

  } while ((mDone == false) && (global_fifo_error == 0) && (itimeout < max_retry));  
  
  if( global_fifo_error || (itimeout >= max_retry)) { // timeout or error
      // disable trigger
      // disable data acq
      // soft reset
      // enable data acq
      //enable trigger
      
    FifoClearAndTriggerEnable();
    
    MPD_ERR("Fifo Error %d | Timeout %d of %d (abort current event)\n",
	    global_fifo_error, itimeout, max_retry);

    for (j=0;j<fMPD.size(); j++) {
      MPD_ERR("MPD slot= %d, i2c/sample left= %s\n", fMPD[j]->GetSlot(), (fMPD[j]->GetSampleLeft()).data());
    };

    global_fifo_error = 0;
  }

  if (global_fifo_error == -5) { mDone = true; }
  return mDone;

}

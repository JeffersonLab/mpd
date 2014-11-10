/** \file
 *
 * Multi Purpose Digitizer
 * High level library
 *
 */

#ifndef __MPD_HIGH__
#define __MPD_HIGH__

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
 
#include <iostream>

#include "MPDlo.h"

#define MPD_MAX_WORDS 1024

class MPDhi {

 public:

  MPDhi(int debug=0);

  ~MPDhi();

  int CountAndRegisterBoards(); // search VME boards ??
 
  int DiscoverFEC(); // find front end cards

  int CommonLatAndInit(int lat); // configure latency and initialize boards for daq

  int InitAndReset();  // intialize and reset FEC

  int DisableBoards(); // disable each board

  int LoadPedAndThr(int ped, int thr=0);
  int LoadPedAndThr();

  int EnableAcq();

  int CheckFIFOnotFull();

  int FifoClearAndTriggerEnable();

  int ArmReadout(); // to be done before read data

  bool EVENT_Read(int max_retry);
  // 

  int ReadSampleData(FILE *fout, int format, int event);

  // Utils
  int SearchEndMarker(uint32_t *b, int i0, int i1);


  // Histogramming

  void HISTO_Clear(int histo_board, int histo_ch, int val=0);
  void HISTO_Start(int histo_board, int histo_ch);
  void HISTO_Stop(int histo_board, int histo_ch);
  uint32_t HISTO_Read(int histo_board, int histo_ch, uint32_t *data);

  // Check synchronization
  int FIFO_Synch(int l0_min=0, int l0_max=1500, int l1_min=1800, int l1_max=3500, int l_step=10, 
		 int phase_min=0, int phase_max=60, int phase_step=2);

  // 

  uint16_t getReadMask(int i) { return fReadMask[i]; };

  uint32_t getMissedTriggers(int i);

  int GetBoardCount() { return fMPD.size(); };

  int GetNumberEnabledCards();

  MPDlo *getModule(int i) { 
    if (i < (int) fMPD.size()) {
      return fMPD[i];
    } else { return 0; }
  };

  void addModule(MPDlo *v) { 
    fMPD.push_back(v);
    fMPDcount++; // not required !
    std::cout << __FUNCTION__ << ": Module " << fMPD.size()-1 << " added to list of MPDs" << std::endl;
  };

  void setBusIndex(int val) { fBusIdx = val; };

  int getAcqMode(unsigned int module=0) { 
    if ((module>= 0) && (module<fMPD.size())) {
      return (int) fMPD[module]->GetAcqMode();
    } else {
      return -1;
    };
  };

 private:

  int fBusIdx; // bus index

  //  MPDlo *fMPD[MAX_BOARDS];

  std::vector<MPDlo *> fMPD;

  int fMPDcount;

  uint16_t *fReadMask;
  int fTimeout;

  bool fBrdFull[MAX_BOARDS]; // used during acquisition

  int fDebugLevel;

  // --- internal methods

  int tScanCR(int slot, uint32_t *rev_id, uint32_t *timec);

  unsigned char tBuildMode(int pol, int fre, int rom, int cal, int sam, int ana);
};

#endif

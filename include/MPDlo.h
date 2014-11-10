/**
 * Low Level MPD class
 *
 * Convention: all methods with "Capital Letters"_"something" access the hardware bus; e.g.:
 * INIT_Board()
 * APV_Reset101()
 * ...
 *
 */

#ifndef __GI_ADCVME64X__
#define __GI_ADCVME64X__

#include <unistd.h>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <vector>
#include <stdio.h>

#include "Bus.h"

// --- verbosity macro

#define MPD_VERBOSE 3 // 0 = no print, 1 = errror, 2 = +warning, 3 = +message, 4 = +debug message, 5= +dump

#define MPD_STR "%s[%d] "
#define MPD_ARG __FUNCTION__, __LINE__
#define MPD_PRINT(...) fprintf(stderr, __VA_ARGS__)

#if (MPD_VERBOSE>4)
#define MPD_DUM(_fmt, ...) MPD_PRINT("DUM:" MPD_STR _fmt, MPD_ARG, ##__VA_ARGS__)
#else
#define MPD_DUM(...)
#endif
#if (MPD_VERBOSE>3)
#define MPD_DBG(_fmt, ...) MPD_PRINT("DBG:" MPD_STR _fmt, MPD_ARG, ##__VA_ARGS__)
#else
#define MPD_DBG(...)
#endif
#if (MPD_VERBOSE>2)
#define MPD_MSG(_fmt, ...) MPD_PRINT("MSG:" MPD_STR _fmt, MPD_ARG, ##__VA_ARGS__)
#else
#define MPD_MSG(...)
#endif
#if (MPD_VERBOSE>1)
#define MPD_WRN(_fmt, ...) MPD_PRINT("WRN:" MPD_STR _fmt, MPD_ARG, ##__VA_ARGS__)
#else
#define MPD_WRN(...)
#endif
#if (MPD_VERBOSE>0)
#define MPD_ERR(_fmt, ...) MPD_PRINT("ERR:" MPD_STR _fmt, MPD_ARG, ##__VA_ARGS__)
#else
#define MPD_ERR(...)
#endif

// ----------------

#define MAX_BOARDS 22		// Maximum number of handled boards

#define MPD_APV_CLOCK_PHASE_0	0x00000	// Bit [17..16]
#define MPD_APV_CLOCK_PHASE_90	0x10000
#define MPD_APV_CLOCK_PHASE_180	0x20000
#define MPD_APV_CLOCK_PHASE_270	0x30000

#define MPD_APV_END_BLOCK 0x180000
/*
#define DAQ_MODE_NONE	0
#define DAQ_MODE_APV	1
#define	DAQ_MODE_SAMPLE	2
*/

#define MPD_TRIG_MODE_NONE	0
#define MPD_TRIG_MODE_APV	1
#define	MPD_TRIG_MODE_MULTI	2
#define MPD_TRIG_MODE_CALIB 3

#define MPD_ADS5281_PAT_NONE	0
#define MPD_ADS5281_PAT_SYNC	1
#define MPD_ADS5281_PAT_DESKEW	2
#define MPD_ADS5281_PAT_RAMP	3

#define MPD_DAQ_EVENT		1
#define MPD_DAQ_SAMPLE		2
#define MPD_DAQ_PROCESS         3
#define MPD_DAQ_HISTO		4
#define MPD_DAQ_SYNC		5
#define MPD_DAQ_RAM_TEST	6

#define	EVENT_SIZE	130

#define DAQ_FIFO_SIZE	2048

#define USED_WORDS_ADDR		0x80000
#define FIFO_FLAG_ADDR		0x80020
#define	SYNCED_ERROR_ADDR	0x80024
#define	MISSED_TRIGGER_ADDR	0x80028
#define	READOUT_CONFIG_ADDR	0x80030
#define	TRIG_CONFIG_ADDR	0x80034
#define	ONE_ZERO_THR_ADDR	0x80038
#define	SYNC_ENABLE_ADDR	0x8003C
#define PEDESTAL_BASE_ADDR      0x40000
#define THRESHOLD_BASE_ADDR     0x60000

#define SOFTWARE_CLEAR_MASK	 0x00080000	// @ TRIG_CONFIG_ADDR
#define SOFTWARE_TRIGGER_MASK	 0x00040000	// @ TRIG_CONFIG_ADDR

#define MPD_IN_TRIG  0
#define MPD_IN_TRIG1 0
#define MPD_IN_TRIG2 1
#define MPD_IN_SYNC  2
#define MPD_IN_P0    0
#define MPD_IN_FRONT 1

#define Sleep(msec) usleep( msec * 1000 )

class ApvParameters // actually a structure
{
 public:
  unsigned char i2cAddrScan; // i2c address from card discovery (scan)
  unsigned char i2cAddr;
  
  // config settings
  short i2c;
  short adc;
  unsigned char Ipre;
  unsigned char Ipcasc;
  unsigned char Ipsf;
  unsigned char Isha;
  unsigned char Issf;
  unsigned char Ipsp;
  unsigned char Imuxin;
  unsigned char Ispare;
  unsigned char Ical;
  unsigned char Vfp;
  unsigned char Vfs;
  unsigned char Vpsp;
  unsigned char Cdrv;
  unsigned char Csel;
  unsigned char Latency;
  unsigned char Muxgain;
  unsigned char Mode;

/****************************************************
MODE REGISTER BITS:
n.      Function        Value = 0       Value = 1
--------------------------------------------------
7       Not Used
6       Not Used
5       Preamp Pol.     Non-Inverting   Inverting
4       Read-out Freq.  20MHz           40MHz 
3       Read-out Mode   Deconvolution   Peak  
2       Calibr. Inhibit OFF             ON
1       Trigger Mode    3-sample        1-sample
0       Analogue Bias   OFF             ON
******************************************************/
  // derived settings
  int fNumberSample; // set to the number of samples expected from this card (trig_num * peak_mode)

  // runtime variables
  int fReadCount;
  uint32_t *fBuffer; // data buffer
  int fBufSize;
  int fBi0, fBi1, fBs;

};


class MPDlo
{
 private:
  
  uint32_t fBaseAddr; // MPD base address
  int fSlot; // rotary switch
  int fBus; // bus index where the MPD is connected to
  bool fEnable; // if false, the mpd is disabled (e.g. not connected)
  uint32_t FpgaRevision;
  uint32_t FpgaCompileTime;
  uint32_t PllConfigOffset;
  uint32_t AdcConfigOffset;
  uint32_t I2CcontrollerOffset;
  uint32_t HistogrammerOffset;
  uint32_t ApvFifoOffset;
  uint32_t SdramOffset;
  uint32_t Last0Offset;
  uint32_t Last1Offset;

  uint16_t fApv_enable_mask;

  // config settings
  int   fCalibLatency;
  short fTriggerNumber;
  short fTriggerMode;
  short fAcqMode; // histo, event process ...

  bool fInPath[2][3]; // define the path of the trigger, synch and clock ...

  bool fEventBuilding;

  short fCommonNoiseSubtraction;
  int  fCommonOffset;

  int fLevel_0, fLevel_1;

  int fChannelMark;

  // adc (Ads5281) settings
  int fAdcClockPhase[2];
  int fAdcGain[2][8];
  bool fAdcInvert[2];
  int fAdcPattern[2];

  // i2c
  int fI2CSpeed;
  int fI2CMaxRetry;

  // pedestal and threshold
  int fPed[2048];
  int fThr[2048];
  int fPedCommon, fThrCommon;
  std::string fPedThrPath;

  bool fReadDone;

  // ---

  int fApv_count;

  // ****

  Bus *fCommLink; // bus handler

  // wrap routines for board access from bus

  int BUS_Read(uint32_t Offset, void *Data) {
    return fCommLink->Read(fBaseAddr + Offset, Data);
  };
  int BUS_Write(uint32_t Offset, void *Data) {
    return fCommLink->Write(fBaseAddr + Offset, Data);
  };
  int BUS_BlockRead(uint32_t Offset, int size, void *Buffer, int *Transferred) {
    return fCommLink->BlockRead(fBaseAddr + Offset, size, Buffer, Transferred);
  };
  int BUS_BlockWrite(uint32_t Offset, int size, void *Buffer, int *Transferred) {
    return fCommLink->BlockWrite(fBaseAddr + Offset, size, Buffer, Transferred);
  };

  // I2C interface
  int I2C_SendByte(unsigned char byteval, int start);
  int I2C_ReceiveByte(unsigned char *byteval);
  int I2C_SendStop(void);
  int I2C_SendNack(void);

  /* From AdcConfig.cpp */
  int ADS5281_Set(int adc, uint32_t val);

  /* From Readout.cpp */
  int FIFO_WaitNotEmpty(int channel, int max_retry);
  int FIFO_ReadSingle(int channel, uint32_t *event, int &nread, int max_retry);
  int FIFO_ReadSingle(int channel, int blen, uint32_t *event, int &nread); // new method

  bool IsEndBlock(uint32_t val) { 
    return ((val & MPD_APV_END_BLOCK) == MPD_APV_END_BLOCK) ? true : false; 
  };

  // return adc (fifo) index from vector index of fAPV  
  int v2a(int ia) { return fApv[ia].adc; };
  // return i2c address from vector index of fAPV  
  int v2i(int ia) { return fApv[ia].i2c; };

 public:

  MPDlo(Bus *cl, int s, int b=0);
  ~MPDlo(void);

  std::vector<ApvParameters> fApv;

  int GetSlot(void) { return fSlot; };
  void SetSlot(int s) { fSlot = s; };
  int GetBus(void) { return fBus; };
  uint32_t GetBaseAddress(void) { return fBaseAddr; };

  int ApvGetAdc(int ia) { return fApv[ia].adc; };
  int ApvGetI2C(int ia) { return fApv[ia].i2c; };

  /* All of them return a VME related error code (see CVErrorCodes)
   * or -10 in case of status register readback timeout
   */
  
  /* From AdcConfig.cpp */
  int ADS5281_InvertChannels(int adc);
  int ADS5281_NonInvertChannels(int adc);
  int ADS5281_SetParameters(int adc);	/* adc == 1, 2 */
  int ADS5281_Normal(int adc);	/* adc == 1, 2 */
  int ADS5281_Sync(int adc);	/* adc == 1, 2 */
  int ADS5281_Deskew(int adc);	/* adc == 1, 2 */
  int ADS5281_Ramp(int adc);	/* adc == 1, 2 */
  int ADS5281_SetGain(int adc, 
		      int gain0, int gain1, int gain2, int gain3, 
		      int gain4, int gain5, int gain6, int gain7);

  /* From Histo.cpp */
  int HISTO_Clear(int ch, int val=0);	/* ch == 0, 15 */
  int HISTO_Start(int ch);	/* ch == 0, 15 */
  int HISTO_Stop(int ch);	/* ch == 0, 15 */
  int HISTO_GetIntegral(int ch, uint32_t *integral);	/* ch == 0, 15 */
  int HISTO_Read(int ch, uint32_t *histogram);	/* ch == 0, 15; uint32_t histogram[4096] */

  // I2C Interface, public
  int I2C_Init(void);
  int I2C_ByteWrite(unsigned char dev_addr, unsigned char int_addr, int ndata, unsigned char *data);
  int I2C_ByteRead(unsigned char dev_addr, unsigned char int_addr, int ndata, unsigned char *data);
  int I2C_ByteWriteRead(unsigned char dev_addr, unsigned char int_addr, int ndata, unsigned char *data);
  int I2C_ByteRead1(unsigned char dev_addr, unsigned char *data);
  int I2C_ApvReset(void);

  /* From ApvConfig.cpp */
  int APV_Write(unsigned char apv_addr, unsigned char reg_addr, unsigned char val);
  int APV_Read(unsigned char apv_addr, unsigned char reg_addr, unsigned char *val);
  bool APV_Try(unsigned char apv_addr);
  int APV_Scan(void);
  int APV_Config(int apv_index);

  int ApvGetPeakMode();
  int ApvGetFrequency();
  unsigned char ApvGetMaxLatency();
  int ApvGetLatency(int ia) { return fApv[ia].Latency; };
  int ApvGetCalibrationMode(int ia) { return (1 - ((fApv[ia].Mode >> 2) & 0x1)); };
  int ApvGetMode(int ia) { return (fApv[ia].Mode &0x3F); };

  int ApvGetSample(int ia) { return fApv[ia].fNumberSample; };

  void ApvSetSampleLeft(int ia) { fApv[ia].fReadCount = fApv[ia].fNumberSample; };
  void ApvDecSampleLeft(int ia, int n=1) { fApv[ia].fReadCount -= n; };
  bool ApvReadDone(int ia) { 
    if (fApv[ia].fReadCount <= 0) return true; 
    return false;
  };
  int ApvGetSampleLeft(int ia) { return (fApv[ia].fNumberSample - ApvGetBufferSample(ia)); };
  int ApvGetSampleIdx(int ia) { return (fApv[ia].fNumberSample - fApv[ia].fReadCount); };

  std::string GetSampleLeft() {
    std::stringstream s;
    for (unsigned int i=0; i<fApv.size(); i++) {
      s << " " << fApv[i].i2c << "/" << ApvGetSampleLeft(i);
      MPD_DBG(" i2c=%d left=%d\n", fApv[i].i2c,  ApvGetSampleLeft(i));
    }
    std::string ss;
    ss = s.str();
    return ss;
  };

  int ArmReadout();

  unsigned int ApvGetSize() { return fApv.size(); };

  void ApvBufferAlloc(int ia) {
    fApv[ia].fBufSize = 6*fApv[ia].fNumberSample*(EVENT_SIZE+2); // at least 6 times larger @@@
    fApv[ia].fBuffer = (uint32_t *) malloc(fApv[ia].fBufSize*sizeof(uint32_t));
    fApv[ia].fBi1 = 0;
    MPD_DBG("Fifo %d, buffer allocated with word size %d\n",v2a(ia), fApv[ia].fBufSize);

  };
  void ApvBufferFree(int ia) {
    if (fApv[ia].fBuffer != 0) {
      free(fApv[ia].fBuffer);
      MPD_DBG("Fifo %d, buffer released\n",v2a(ia));
    };
  };
  void ApvIncBufferPointer(int ia, int b) {
    fApv[ia].fBi1 += b;
  };

  uint32_t *ApvGetBufferPointer(int ia, int ib) {
    MPD_DBG("Fifo %d, retrieved pointer from position %d\n",v2a(ia), ib);
    if (ib<fApv[ia].fBufSize) { // probably not required !!
      return &(fApv[ia].fBuffer[ib]);
    }
    MPD_ERR("Fifo %d, index %d is out of range (%d)\n",v2a(ia), ib, fApv[ia].fBufSize);
    exit(1);
  };

  int ApvGetBufferSample(int ia) {
    int ix = fApv[ia].fBi1 / EVENT_SIZE;
    MPD_DBG("Fifo = %d has %d samples (%d bytes) stored\n",v2a(ia), ix, fApv[ia].fBi1);
    return ix;

  };

  uint32_t *ApvGetBufferPWrite(int ia) {
    return ApvGetBufferPointer(ia, fApv[ia].fBi1);
  };

  uint32_t ApvGetBufferElement(int ia, int ib=0) {
    return fApv[ia].fBuffer[ib];
  };

  int ApvGetBufferAvailable(int ia) {
    return fApv[ia].fBufSize - fApv[ia].fBi1 - 1; // @@@@ Apr/2013 tobe verified
  };

  int ApvGetBufferLength(int ia) {
    return fApv[ia].fBi1;
  };

  int ApvGetEventSize(int ia) {
    return EVENT_SIZE; // TO BE CHANGED (variable size !!!)
  };

  /* From TrigConfig.cpp */  
  int TRIG_BitSet();
  int TRIG_BitClear();
  int TRIG_Enable();
  int TRIG_GetMissed(uint32_t *missed);  
  int TRIG_Disable(void);

  int DELAY25_Set(int apv1_delay, int apv2_delay);


  int FIFO_IsFull(int channel, int *full);
  int FIFO_GetNwords(int channel, int *nwords);
  int FIFO_IsEmpty(int channel, int *empty);
  int FIFO_ClearAll(void);
  int FIFO_GetAllFlags(uint16_t *full, uint16_t *empty);
  int FIFO_IsSynced(int channel, int *synced);
  int FIFO_AllSynced(int *synced);
  int FIFO_HasError(int channel, int *error);

  bool FIFO_ReadAll(int &timeout, int &global_fifo_error);
  bool FIFO_ReadAllNew(int &timeout, int &global_fifo_error);

  bool FIFO_Read(int &timeout, int &global_fifo_error) {
    if (IsEnabled()) {
      if (GetAcqMode() == MPD_DAQ_PROCESS) { // To be generalized
	return FIFO_ReadAllNew(timeout, global_fifo_error);
      } else {
	return FIFO_ReadAll(timeout, global_fifo_error);
      }
    }
    return true;
  };

  int FIFO_Samples(int channel, uint32_t *event, int *nread, int max_samples, int *err); // ???



  int DAQ_Enable(void);
  int DAQ_Disable(void);
  //		int DaqConfig(int mode, int test);
  int DAQ_Config(void);
  
  /* General */
  int CR_Scan(int slot, uint32_t *rev_id, uint32_t *timec);
  void BOARD_Init(void);
  int APV_Reset101(void);
  int LM95235_Read(int *t);

  /* Inlines Get/Set */
  void SetApvEnableMask(uint16_t mask) { fApv_enable_mask |= mask; }
  void ResetApvEnableMask() { fApv_enable_mask = 0; }
  uint16_t GetApvEnableMask(void) { return fApv_enable_mask; }

  void SetZeroLevel(uint16_t level) { fLevel_0 = level; }
  int GetZeroLevel(void) { return fLevel_0; }
  void SetOneLevel(uint16_t level) { fLevel_1 = level; }
  int GetOneLevel(void) { return fLevel_1; }

  void SetChannelMark(int v=255) { fChannelMark = v; } // channel mark 0-127 for frame alignment, mark>127 disabled
  int GetChannelMark() { return fChannelMark; }

  void SetCommonNoiseSubtraction(short val) { fCommonNoiseSubtraction = val; };
  short GetCommonNoiseSubtraction(void) { return fCommonNoiseSubtraction; };

  void SetEventBuilding(bool val) { fEventBuilding = val; };
  bool GetEventBuilding(void) { return fEventBuilding; };
 
  void SetCommonOffset(int val) { fCommonOffset = val; };
  int GetCommonOffset(void) { return fCommonOffset; };

  void SetCalibLatency(int val) { fCalibLatency = val; };
  int GetCalibLatency(void) { return fCalibLatency; };

  void SetTriggerNumber(int val) { fTriggerNumber = val; };
  int GetTriggerNumber(void) { return fTriggerNumber; };

  void SetTriggerMode(int lat, int num) {
    if( num == 0 )
      fTriggerMode = MPD_TRIG_MODE_NONE;
    else {
      if (lat>0) { 
	fTriggerMode = MPD_TRIG_MODE_CALIB;
      } else {
	if( num == 1 )
	  fTriggerMode = MPD_TRIG_MODE_APV;
	else
	  if( num > 1 )
	    fTriggerMode = MPD_TRIG_MODE_MULTI;
      }
    }
    SetTriggerNumber(num);
    SetCalibLatency(lat);
    MPD_DBG("Calib Latency = %d, Trigger Mode = 0x%x\n",lat, fTriggerMode);
  };
  int GetTriggerMode(void) { return fTriggerMode; };

  void SetAcqMode(std::string name) {
    fAcqMode = 0; // disabled
    if( name == "ramtest" ) fAcqMode = MPD_DAQ_RAM_TEST;
    if( name == "histo" ) fAcqMode = MPD_DAQ_HISTO;
    if( name == "event" ) fAcqMode = MPD_DAQ_EVENT;
    if( name == "process" ) fAcqMode = MPD_DAQ_PROCESS;
    if( name == "sample" ) fAcqMode = MPD_DAQ_SAMPLE;
    if( name == "sync" ) fAcqMode = MPD_DAQ_SYNC;

    MPD_DBG("Acquisition Mode = 0x%x (%s)\n",fAcqMode, name.data());
  };
  int GetAcqMode(void) { return fAcqMode; };

  
  void SetInPath(bool t1P0, bool t2P0, bool tFront, bool sP0, bool sFront) {
    fInPath[MPD_IN_P0][MPD_IN_TRIG1] = t1P0;
    fInPath[MPD_IN_P0][MPD_IN_TRIG2] = t2P0;
    fInPath[MPD_IN_FRONT][MPD_IN_TRIG] = tFront;
    fInPath[MPD_IN_P0][MPD_IN_SYNC] = sP0;
    fInPath[MPD_IN_FRONT][MPD_IN_SYNC] = sFront;
  };
  void SetInPath(int conn, int signal, bool val) { fInPath[conn%2][signal%3] = val; };
  bool GetInPath(int conn, int signal) { return fInPath[conn%2][signal%3]; };
  int GetInPathI(int conn, int signal) { return ((fInPath[conn%2][signal%3] == true) ? 1 : 0); };

  // adc set/get methods
  void SetAdcClockPhase(int adc, int phase) { fAdcClockPhase[adc] = phase; };
  int GetAdcClockPhase(int adc) { 
    int clock_phase;
    clock_phase = fAdcClockPhase[adc];
    if( GetFpgaRevision() < 2 ) { // no delay line
      switch (clock_phase) {
      case 0: 
	clock_phase = MPD_APV_CLOCK_PHASE_0;
	break;
      case 1:
      case 90:
	clock_phase = MPD_APV_CLOCK_PHASE_90;
	break;
      case 2:
      case 180:
	clock_phase = MPD_APV_CLOCK_PHASE_180;
	break;
      case 3:
      case 270:
	clock_phase = MPD_APV_CLOCK_PHASE_270;
	break;
      default: 
	clock_phase = MPD_APV_CLOCK_PHASE_0;
	MPD_WRN("Warning: apv_clock phase %d out of range for Fpga rev. %d", clock_phase, GetFpgaRevision());
      }
    }
    return clock_phase; 
  };

  void SetAdcGain(int adc, int ch, int g) {fAdcGain[adc][ch] = g; };
  int GetAdcGain(int adc, int ch) {return fAdcGain[adc][ch]; };
  void SetAdcInvert(int adc, bool val) {fAdcInvert[adc] = val; };
  bool GetAdcInvert(int adc) {return fAdcInvert[adc]; };
  void SetAdcPattern(int adc, int p) {fAdcPattern[adc] = p; };
  int GetAdcPattern(int adc) {return fAdcPattern[adc]; };

  // i2c set/get
  void SetI2CSpeed(int val) { fI2CSpeed = val; };
  int GetI2CSpeed(void) { return fI2CSpeed; };

  void SetI2CMaxRetry(int val) { fI2CMaxRetry = val; };
  int GetI2CMaxRetry(void) { return fI2CMaxRetry; };

  //-------------
  int GetApvCount(void) { return fApv.size(); };
  void AddApv(ApvParameters v);

  uint32_t GetFpgaRevision(void) {
    if (FpgaRevision == 99999) {
      MPD_ERR("Fpga revision not set yet, something wrong! exit(%d)",0);
      exit(0);
    }
    return FpgaRevision; 
  };
  void SetFpgaRevision(uint32_t r=99999) { FpgaRevision = r; };

  uint32_t GetHWRevision(void) {
    uint32_t d = GetFpgaRevision();
    return ((d>>24)&0xff);
  };

  uint32_t GetFWRevision(void) {
    uint32_t d = GetFpgaRevision();
    return (d&0xff);
  };

  uint32_t GetFpgaCompileTime(void) {return FpgaCompileTime; };
  void SetFpgaCompileTime(uint32_t t) { FpgaCompileTime = t; };

  void Enable(void) { fEnable = true; };
  void Disable(void) { fEnable = false; };
  bool IsEnabled(void) { return fEnable; };

  // *** Pedestal and Threshold

  int PED_Write(int ch, int *ped_even, int *ped_odd);
  int PED_Write(int ch, int v=0);
  int PED_Read(int ch, int *ped_even, int *ped_odd);
  
  int THR_Write(int ch, int *thr_even, int *thr_odd);
  int THR_Write(int ch, int v=0);
  int THR_Read(int ch, int *thr_even, int *thr_odd);

  int PEDTHR_Write();

  void SetPedThrPath(std::string val) { fPedThrPath = val; };
  std::string GetPedThrPath(void) { return fPedThrPath; };
  int SetPedThr(int ch, int p, int t) {
    if (ch>=0 && ch<2048) {
      fPed[ch] = p;
      fThr[ch] = t;
      return 1;
    }
    return 0;
  };
  int GetPed(int ch) { 
    if (ch>=0 && ch<2048) { return fPed[ch]; } else { return fPedCommon; }
  };
  int GetThr(int ch) { 
    if (ch>=0 && ch<2048) { return fThr[ch]; } else { return fThrCommon; }
  };

  int *GetApvPed(int ach) { if (ach>=0 && ach<16) { return &(fPed[ach*128]); } else { return 0; }};
  int *GetApvThr(int ach) { if (ach>=0 && ach<16) { return &(fThr[ach*128]); } else { return 0; }};

  void SetPedThrCommon(int p, int t) {
    fPedCommon = p;
    fThrCommon = t;
  };
  int GetPedCommon(void) { return fPedCommon; };
  int GetThrCommon(void) { return fThrCommon; };
  int ReadPedThr(std::string pname="");

  int SearchEndMarker(uint32_t *b, int i0, int i1);  
  void ApvShiftDataBuffer(int k, int i0);

};

#endif

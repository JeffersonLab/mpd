#ifndef __MPDLIB__
#define __MPDLIB__
/******************************************************************************
 *
 *  mpdLib.h
 *             - Driver library header file for the MultiPurpose
 *             Digitizer (MPD) using a VxWorks 5.5 (PPC) or Linux
 *             2.6.18 (Intel) or later based single board computer.
 *
 *  Author: Bryan Moffit
 *          Jefferson Lab Data Acquisition Group
 *          November 2014
 *
 */

#include "stdio.h"
#ifndef VXWORKS
#include "stdint.h"
#endif
#define MPD_MAGIC_VALUE       0x43524F4D
#define MPD_MAX_BOARDS             21
#define MPD_SSP_MAX_BOARDS         21
//#define MPD_MAX_APV                16
#define MPD_MAX_APV                31

#define MPD_SUPPORTED_FIRMWARE 0x23
#define MPD_HDMI_SUPPORTED_FIRMWARE_TIME 0x5b84f50f

#define MPD_APV_CLOCK_PHASE_0	0x00000	// Bit [17..16]
#define MPD_APV_CLOCK_PHASE_90	0x10000
#define MPD_APV_CLOCK_PHASE_180	0x20000
#define MPD_APV_CLOCK_PHASE_270	0x30000

#define MPD_TRIG_MODE_NONE	0
#define MPD_TRIG_MODE_APV	1
#define	MPD_TRIG_MODE_MULTI	2
#define MPD_TRIG_MODE_CALIB 3

#define MPD_DAQ_EVENT		1
#define MPD_DAQ_SAMPLE		2
#define MPD_DAQ_PROCESS         3
#define MPD_DAQ_HISTO		4
#define MPD_DAQ_SYNC		5
#define MPD_DAQ_RAM_TEST	6


#define	EVENT_SIZE	130

#define DAQ_FIFO_SIZE	1024

#define MPD_DEBUG_GEN     (1<<0)
#define MPD_DEBUG_DMA     (1<<2)
#define MPD_DEBUG_APVINIT (1<<3)
#define MPD_DEBUG_I2C     (1<<4)

#define MPD_MSG(format, ...) {printf("%s: ",__FUNCTION__); printf(format, ## __VA_ARGS__);}
#define MPD_WARN(format, ...) {printf("%s: WARNING",__FUNCTION__); printf(format, ## __VA_ARGS__);}
#define MPD_DUMP(format, ...) {if(mpdPrintDebug&2) {printf("%s: ",__FUNCTION__); printf(format, ## __VA_ARGS__);} }
#define MPD_DBG(format, ...) {if(mpdPrintDebug&1) {printf("%s: DEBUG: ",__FUNCTION__); printf(format, ## __VA_ARGS__);} }
#define MPD_DBGN(x,format, ...) {if(mpdPrintDebug&x) {printf("%s: DEBUG%d: ",__FUNCTION__, x); printf(format, ## __VA_ARGS__);} }
#define MPD_ERR(format, ...) {fprintf(stdout,"%s: ERROR: ",__FUNCTION__); fprintf(stdout,format, ## __VA_ARGS__);}

struct output_buffer_struct /* .ob_status */
{
  /* 0x0200 */  volatile uint32_t evb_fifo_word_count;
  /* 0x0204 */  volatile uint32_t event_count;
  /* 0x0208 */  volatile uint32_t block_count;
  /* 0x020C */  volatile uint32_t trigger_count;
  /* 0x0210 */  volatile uint32_t missed_trigger;
  /* 0x0214 */  volatile uint32_t incoming_trigger;
  /* 0x0218 */  volatile uint32_t sdram_fifo_wr_addr;
  /* 0x021C */  volatile uint32_t sdram_fifo_rd_addr;
  /* 0x0220 */  volatile uint32_t sdram_flag_wc;
  /* 0x0224 */  volatile uint32_t output_buffer_flag_wc;
  /* 0x0228 */  volatile uint32_t latched_full;
  /* 0x022C */  volatile uint32_t blank;
};

struct mpd_i2c_control_struct /* .i2c */
{
  /* 0x0400 */ volatile uint32_t clock_prescaler_low;
  /* 0x0404 */ volatile uint32_t clock_prescaler_high;
  /* 0x0408 */ volatile uint32_t control;
  /* 0x040C */ volatile uint32_t tx_rx;
  /* 0x0410 */ volatile uint32_t comm_stat;
  /* 0x0414 */          uint32_t blank[2];
  /* 0x041C */ volatile uint32_t apv_reset;
};

struct mpd_channel_flags_struct /* .ch_flags; */
{
  /* 0x30000 */ volatile uint32_t used_word_ch_pair[16];
  /* 0x30040 */ volatile uint32_t fifo_status;
  /* 0x30044 */ volatile uint32_t sync_status;
};

struct mpd_histogrammer_struct /* .histo[2]; */
{
  /* 0x01000 */ volatile uint32_t csr;
  /* 0x01004 */ volatile uint32_t count;
};

struct mpd_struct
{
  /* 0x00000 */ volatile uint32_t magic_value;
  /* 0x00004 */ volatile uint32_t manuf_id;
  /* 0x00008 */ volatile uint32_t board_id;
  /* 0x0000C */ volatile uint32_t revision_id;
  /* 0x00010 */ volatile uint32_t compile_time;
  /* 0x00014 */          uint32_t blank0[(0x100-0x14)>>2];

  /* 0x00100 */ volatile uint32_t reset_reg;
  /* 0x00104 */ volatile uint32_t io_config;
  /* 0x00108 */ volatile uint32_t sample_per_event;
  /* 0x0010C */ volatile uint32_t event_per_block;
  /* 0x00110 */ volatile uint32_t busy_thr;
  /* 0x00114 */ volatile uint32_t busy_thr_local;
  /* 0x00118 */ volatile uint32_t readout_config;
  /* 0x0011C */ volatile uint32_t trigger_config;
  /* 0x00120 */ volatile uint32_t trigger_delay;
  /* 0x00124 */ volatile uint32_t sync_period;
  /* 0x00128 */ volatile uint32_t marker_channel;
  /* 0x0012C */ volatile uint32_t channel_enable;
  /* 0x00130 */ volatile uint32_t zero_threshold;
  /* 0x00134 */ volatile uint32_t one_threshold;
  /* 0x00138 */ volatile uint32_t fir_coefficients[8];
  /* 0x00158 */          uint32_t blank1[(0x180-0x158)>>2];

  /* 0x00180 */ volatile uint32_t a24_bar;
  /* 0x00184 */ volatile uint32_t multiboard_config;
  /* 0x00188 */ volatile uint32_t multiboard_add_low;
  /* 0x0018C */ volatile uint32_t multiboard_add_high;
  /* 0x00190 */ volatile uint32_t fiber_status_ctrl;
  /* 0x00194 */ volatile uint32_t obuf_base_addr;
  /* 0x00198 */ volatile uint32_t sdram_base_addr;
  /* 0x0019C */ volatile uint32_t sdram_bank;
  /* 0x001A0 */          uint32_t blank2[(0x200-0x1A0)>>2];

  /* 0x00200 */ struct output_buffer_struct ob_status;
  /* 0x00230 */          uint32_t blank3[(0x300-0x230)>>2];
  /* 0x00300 */ volatile uint32_t adc_config;
  /* 0x00304 */          uint32_t blank4[(0x380-0x304)>>2];


  /* 0x00380 */ volatile uint32_t serial_memory_if[8];
  /* 0x003A0 */ volatile uint32_t remote_update[4];
  /* 0x003B0 */          uint32_t blank5[(0x400-0x3B0)>>2];

  /* 0x00400 */ struct mpd_i2c_control_struct i2c;
  /* 0x00420 */          uint32_t blank6[(0x1000-0x420)>>2];

  /* 0x01000 */ struct mpd_histogrammer_struct histo[2];
  /* 0x01010 */          uint32_t blank7[(0x4000-0x1010)>>2];
  /* 0x04000 */ volatile uint32_t histo_memory[2][(0x4000)>>2];
  /* 0x0C000 */          uint32_t blank8[(0x10000-0xC000)>>2];
  /* 0x10000 */ volatile uint32_t data_ch[16][(0x2000)>>2];
  /* 0x30000 */ struct mpd_channel_flags_struct ch_flags;
  /* 0x30048 */          uint32_t blank10[(0x34000-0x30048)>>2];
  /* 0x34000 */ volatile uint32_t ped[16][(0x200)>>2];
  /* 0x36000 */ volatile uint32_t thres[16][(0x200)>>2];

};

#define MPD_I2C_CONTROL_ENABLE_CORE  0x80

#define MPD_I2C_COMMSTAT_NACK        0x08
#define MPD_I2C_COMMSTAT_WRITE       0x10
#define MPD_I2C_COMMSTAT_READ        0x20
#define MPD_I2C_COMMSTAT_STOP        0x40
#define MPD_I2C_COMMSTAT_NACK_RECV   0x80
#define MPD_I2C_COMMSTAT_START_WRITE 0x90

#define MPD_I2C_COMMSTAT_STA  (1<<7)
#define MPD_I2C_COMMSTAT_STO  (1<<6)
#define MPD_I2C_COMMSTAT_RD   (1<<5)
#define MPD_I2C_COMMSTAT_WR   (1<<4)
#define MPD_I2C_COMMSTAT_ACK  (1<<3)
#define MPD_I2C_COMMSTAT_TIP  (1<<1)
#define MPD_I2C_COMMSTAT_IACK (1<<0)

#define MPD_ROCONFIG_I2C_ENABLE_0_7  (1<<9)
#define MPD_ROCONFIG_I2C_ENABLE_8_15 (1<<10)
#define MPD_ROCONFIG_I2C_ENABLE_MASK 0x0000600


#define MPD_I2C_APVRESET_ASYNC_SET   0x00
#define MPD_I2C_APVRESET_ASYNC_CLEAR 0x01

#define MPD_APVDAQ_TRIGCONFIG_ENABLE_MACH 0x00001000
#define SOFTWARE_CLEAR_MASK	 0x00080000	// @ TRIG_CONFIG_ADDR
#define SOFTWARE_TRIGGER_MASK	 0x00040000	// @ TRIG_CONFIG_ADDR

#define MPD_IN_TRIG  0
#define MPD_IN_TRIG1 0
#define MPD_IN_TRIG2 1
#define MPD_IN_SYNC  2
#define MPD_IN_P0    0
#define MPD_IN_FRONT 1

#define MPD_FIBER_DISABLE          (1<<0)
#define MPD_SFP_TRANSMIT_DISABLED  (1<<1)
#define MPD_FIBER_RESET            (1<<3)
#define MPD_FIBER_ERROR_COUNT_MASK 0x00000FF0
#define MPD_FIBER_FRAME_ERROR      (1<<12)
#define MPD_FIBER_HARD_ERROR       (1<<13)
#define MPD_SFP_PRESENT            (1<<14)
#define MPD_SFP_LOS                (1<<15)
#define MPD_FIBER_CHANNEL_UP       (1<<31)

#define MPD_ADS5281_PAT_NONE    0
#define MPD_ADS5281_PAT_SYNC    1
#define MPD_ADS5281_PAT_DESKEW  2
#define MPD_ADS5281_PAT_RAMP    3

#define MPD_DELAY25_CHANNEL_ENABLED    (1<<6)
#define MPD_DELAY25_CHANNEL_DELAY_MASK   0x3F
#define MPD_DELAY25_GCR_RESET_DLL      (1<<6)
#define MPD_DELAY25_GCR_CLOCK_MASK       0x03
#define MPD_DELAY25_GCR_40MHZ               0
#define MPD_DELAY25_GCR_80MHZ               1
#define MPD_DELAY25_GCR_32MHZ               2
#define MPD_DELAY25_GCR_64MHZ               3

typedef struct apvparm_struct // actually a structure
{
  uint8_t i2cAddrScan; // i2c address from card discovery (scan)
  uint8_t i2cAddr;

  short enabled; // if 0 card is disabled

  // config settings
  short i2c;
  short adc;
  uint8_t Ipre;
  uint8_t Ipcasc;
  uint8_t Ipsf;
  uint8_t Isha;
  uint8_t Issf;
  uint8_t Ipsp;
  uint8_t Imuxin;
  uint8_t Ispare;
  uint8_t Ical;
  uint8_t Vfp;
  uint8_t Vfs;
  uint8_t Vpsp;
  uint8_t Cdrv;
  uint8_t Csel;
  uint8_t Latency;
  uint8_t Muxgain;
  uint8_t Mode;

  /****************************************************
   * MODE REGISTER BITS:
   * n.      Function        Value = 0       Value = 1
   * --------------------------------------------------
   * 7       Not Used
   * 6       Not Used
   * 5       Preamp Pol.     Non-Inverting   Inverting
   * 4       Read-out Freq.  20MHz           40MHz
   * 3       Read-out Mode   Deconvolution   Peak
   * 2       Calibr. Inhibit OFF             ON
   * 1       Trigger Mode    3-sample        1-sample
   * 0       Analogue Bias   OFF             ON
   ******************************************************/

  // derived settings
  int fNumberSample; // set to the number of samples expected from this card (trig_num * peak_mode)

  // runtime variables
  int fReadCount;
  uint32_t *fBuffer; // data buffer
  int fBufSize;
  int fBi0, fBi1, fBs;

} ApvParameters;


typedef struct mpd_priv_struct
{

  uint32_t fBaseAddr; // MPD base address
  int fSlot; // rotary switch
  int fBus; // bus index where the MPD is connected to
  int fEnable; // if false, the mpd is disabled (e.g. not connected)
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
  uint16_t nAPV; // number of apv in mpd config file (EC)
  uint32_t CtrlHdmiMask[2]; // APVaddr = bit, HDMI output 0 = lower, 1 = upper
  uint32_t CtrlHdmiInitMask; // Bits indicate which APV addresses have had CtrlHdmiMask[2] checked.

  // config settings
  int   fCalibLatency;
  int   fTriggerLatency;
  short fTriggerNumber;
  short fTriggerMode;
  short fAcqMode; // histo, event process ...

  int fInPath[2][3]; // define the path of the trigger, synch and clock ...

  short fInLevelTTL[2]; // input lemo logic level 0=NIM, 1=TTL
  short fOutLevelTTL[2]; // output lemo logic level 0=NIM, 1=TTL

  int fEventBuilding;
  int fEventPerBlock;
  int fUseSdram;
  int fFastReadout;

  short fCommonNoiseSubtraction;
  int  fCommonOffset;

  int fLevel_0, fLevel_1;

  int fChannelMark;

  // adc (Ads5281) settings
  int fAdcClockPhase[2];
  int fAdcGain[2][8];
  int fAdcInvert[2];
  int fAdcPattern[2];

  // FIR
  int fFIR;
  int fFIRcoeff[16];

  // i2c
  int fI2CSpeed;
  int fI2CMaxRetry;

  // pedestal and threshold
  int fPed[2048];
  int fThr[2048];
  int fPedCommon, fThrCommon;
  char *fPedThrPath;

  int fReadDone;

  // ---

  int fApv_count;
} mpdParameters;

/* initialization flags */
#define MPD_INIT_SKIP                 (1<<16)
#define MPD_INIT_USE_ADDRLIST         (1<<17)
#define MPD_INIT_SKIP_FIRMWARE_CHECK  (1<<18)
#define MPD_INIT_SSP_MODE             (1<<19)
#define MPD_INIT_NO_CONFIG_FILE_CHECK (1<<20)

/* Function Prototypes */
STATUS mpdInit (UINT32 addr, UINT32 addr_inc, int nadc, int iFlag);
int  mpdCheckAddresses(int id);
int  mpdSlot(uint32_t i);
int mpdSetPrintDebug(int debug);
void mpdParametersInit();

int mpdSetSSPFiberMap_preInit(int ssp, uint32_t mpdmask);
uint32_t mpdGetSSPFiberMask(int sspSlot);
void mpdSetZeroLevel(int id, uint16_t level);
int  mpdGetZeroLevel(int id);
void mpdSetOneLevel(int id, uint16_t level);
int  mpdGetOneLevel(int id);
void mpdSetChannelMark(int id, int v);
int  mpdGetChannelMark(int id);
void mpdSetCommonNoiseSubtraction(int id, short val);
short mpdGetCommonNoiseSubtraction(int id);
void mpdSetEventBuilding(int id, int val);
int  mpdGetEventBuilding(int id);

void mpdSetEventPerBlock(int id, int val);
int  mpdGetEventPerBlock(int id);
void mpdSetUseSdram(int id, int val);
int  mpdGetUseSdram(int id);
void mpdSetFastReadout(int id, int val);
int  mpdGetFastReadout(int id);
void mpdSetCommonOffset(int id, int val);
int  mpdGetCommonOffset(int id);
void mpdSetCalibLatency(int id, int val);
int  mpdGetCalibLatency(int id);
void mpdSetTriggerNumber(int id, int val);
int  mpdGetTriggerNumber(int id);
void mpdSetTriggerMode(int id, int lat, int tlat, int num);
int  mpdGetTriggerMode(int id);
void mpdSetAcqMode(int id, char *name);
int  mpdGetAcqMode(int id);
void mpdSetInPath0(int id, int t1P0, int t2P0, int tFront, int sP0, int sFront);
void mpdSetInPath(int id, int conn, int signal, int val);
int  mpdGetInPath(int id, int conn, int signal);
int  mpdGetInPathI(int id, int conn, int signal);
void mpdSetInputLevel(int id, int conn, short val);
short mpdGetInputLevel(int id, int conn);
void mpdSetOutputLevel(int id, int conn, short val);
short mpdGetOutputLevel(int id, int conn);

uint32_t mpdGetFpgaRevision(int id);
void mpdSetFpgaRevision(int id, uint32_t r);
uint32_t mpdGetHWRevision(int id);
uint32_t mpdGetFWRevision(int id);
uint32_t mpdGetFpgaCompileTime(int id);
void mpdSetFpgaCompileTime(int id, uint32_t t);

int  mpdLM95235_Read(int id, double *core_t, double *air_t);

/* service methods */
int mpdGetNumberAPV(int id); // get number of apv in config file
void mpdSetNumberAPV(int id, uint16_t v); // set number of apv in config file
int mpdGetNumberConfiguredAPV(int id); // return number of apv properly configured by the hardware

/* I2C methods */
void mpdSetI2CSpeed(int id, int val);
int  mpdGetI2CSpeed(int id);
void mpdSetI2CMaxRetry(int id, int val);
int  mpdGetI2CMaxRetry(int id);

int  mpdI2C_ApvReset(int id);
int  mpdI2C_Init(int id);
int  mpdI2C_ByteWrite(int id, uint8_t dev_addr, uint8_t int_addr,
		 int ndata, uint8_t *data);
int  mpdI2C_ByteRead(int id, uint8_t dev_addr, uint8_t int_addr,
		int ndata, uint8_t *data);
#if 0
int  mpdI2C_ByteWriteRead(int id, uint8_t dev_addr, uint8_t int_addr,
			  int ndata, uint8_t *data);
int  mpdI2C_ByteRead1(int id, uint8_t dev_addr, uint8_t *data);
#endif
int mpdAPV_SetCtrlHdmi(int id, uint8_t apv_addr);

/* ADC set/get methods */
void mpdSetAdcClockPhase(int id, int adc, int phase);
int  mpdGetAdcClockPhase(int id, int adc);
void mpdSetAdcGain(int id, int adc, int ch, int g);
int  mpdGetAdcGain(int id, int adc, int ch);
void mpdSetAdcInvert(int id, int adc, int val);
int  mpdGetAdcInvert(int id, int adc);
void mpdSetAdcPattern(int id, int adc, int p);
int  mpdGetAdcPattern(int id, int adc);


/* APV methods */
int  mpdApvGetLatency(int id, int ia);
int  mpdApvGetCalibrationMode(int id, int ia);
int  mpdApvGetMode(int id, int ia);
int  mpdApvGetSample(int id, int ia);
void mpdApvSetSampleLeft(int id, int ia);
void mpdApvDecSampleLeft(int id, int ia, int n);
int  mpdApvReadDone(int id, int ia);
int  mpdApvGetSampleLeft(int id, int ia);
int  mpdApvGetSampleIdx(int id, int ia);
int  mpdAPV_Reset101(int id);
int  mpdAPV_SoftTrigger(int id);
int  mpdAPV_Try(int id, uint8_t apv_addr);
int  mpdAPV_TryHdmi(int id, uint8_t apv_addr, int ihdmi);

void mpdSetApvEnableMask(int id, uint16_t mask);
void mpdResetApvEnableMask(int id);
uint16_t mpdGetApvEnableMask(int id);
int  mpdAPV_Scan(int id);
int  mpdAPV_Write(int id, uint8_t apv_addr, uint8_t reg_addr, uint8_t val);
int  mpdAPV_Read(int id, uint8_t apv_addr, uint8_t reg_addr, uint8_t *val);
int  mpdAPV_Config(int id, int apv_index);
int  mpdApvGetPeakMode(int id);

void mpdAddApv(int id, ApvParameters v);
int  mpdApvGetFrequency(int id);
uint8_t mpdApvGetMaxLatency(int id);
void mpdApvBufferAlloc(int id, int ia) ;
void mpdApvBufferFree(int id, int ia);
void mpdApvIncBufferPointer(int id, int ia, int b);
uint32_t* mpdApvGetBufferPointer(int id, int ia, int ib);
int  mpdApvGetBufferSample(int id, int ia);
uint32_t* mpdApvGetBufferPWrite(int id, int ia);
uint32_t mpdApvGetBufferElement(int id, int ia, int ib);
int  mpdApvGetBufferAvailable(int id, int ia);
int  mpdApvGetBufferLength(int id, int ia);
int  mpdApvGetEventSize(int id, int ia);
int mpdApvGetAdc(int id, int ia);

short mpdApvEnabled(int id, int ia);

int  mpdArmReadout(int id);

// trigger methods
int  mpdTRIG_BitSet(int id);
int  mpdTRIG_BitClear(int id);
int  mpdTRIG_Enable(int id);
int  mpdTRIG_PauseEnable(int id, int time);
int  mpdTRIG_SetFrontInput(int id, int choice);
int  mpdTRIG_GSetFrontInput(int choice);

int  mpdTRIG_Disable(int id);
int  mpdTRIG_GetMissed(int id, uint32_t *missed);
int  mpdDELAY25_Set(int id, int apv1_delay, int apv2_delay);
int  mpdDELAY25_GStatus();

// adc methods
int  mpdADS5281_Config(int id); // (EC)
int  mpdADS5281_Set(int id, int adc, uint32_t val);
int  mpdADS5281_InvertChannels(int id, int adc);
int  mpdADS5281_NonInvertChannels(int id, int adc);
int  mpdADS5281_SetParameters(int id, int adc);
int  mpdADS5281_Normal(int id, int adc);
int  mpdADS5281_Sync(int id, int adc);
int  mpdADS5281_Deskew(int id, int adc);
int  mpdADS5281_Ramp(int id, int adc);
int  mpdADS5281_SetGain(int id, int adc,
			int gain0, int gain1, int gain2, int gain3,
			int gain4, int gain5, int gain6, int gain7);

int mpdFIR_Config(int id);

// histogramming methods

int  mpdHISTO_MemTest(int id);
int  mpdHISTO_Clear(int id, int ch, int val);
int  mpdHISTO_Start(int id, int ch);
int  mpdHISTO_Stop(int id, int ch);
int  mpdHISTO_GetIntegral(int id, int ch, uint32_t *integral);
int  mpdHISTO_Read(int id, int ch, uint32_t *histogram);

// Daq-Readout methods
int mpdOBUF_GetFlags(int id, int *empty, int *full, int *nwords);
int mpdOBUF_Read(int id, volatile uint32_t *data, int size, int *wrec);
int mpdOBUF_ReadLL(volatile uint32_t * data, int size, int *wrec, int rflag);

int mpdSDRAM_GetParam(int id, int *init, int *overrun, int *rdaddr, int *wraddr, int *nwords);

int  mpdFIFO_ReadSingle(int id, int channel, volatile uint32_t *dbuf, int *wrec, int max_retry);
int  mpdFIFO_ReadSingle0(int id, int channel, int blen, uint32_t *event, int *nread);
int  mpdFIFO_Samples(int id,
		     int channel,
		     uint32_t *event, int *nread, int max_samples, int *err);
int  mpdFIFO_IsSynced(int id, int channel, int *synced);
int  mpdFIFO_AllSynced(int id, int *synced);
int  mpdFIFO_HasError(int id, int channel, int *error);
int  mpdFIFO_GetAllFlags(int id, uint16_t *full, uint16_t *empty);
int  mpdFIFO_IsFull(int id, int channel, int *full);
int  mpdFIFO_GetNwords(int id, int channel, int *nwords);
int  mpdFIFO_IsEmpty(int id, int channel, int *empty);
int  mpdFIFO_ClearAll(int id);
int  mpdFIFO_WaitNotEmpty(int id, int channel, int max_retry);

int  mpdFIFO_ReadAll(int id, int *timeout, int *global_fifo_error);


int  mpdSearchEndMarker(uint32_t *b, int i0, int i1);
void mpdApvShiftDataBuffer(int id, int k, int i0);
int  mpdDAQ_Enable(int id);
int  mpdDAQ_Disable(int id);
int  mpdDAQ_Config(int id);

// ***** Pedestal and Thresholds handling routines
int  mpdPED_Write0(int id, int ch, int *ped_even, int *ped_odd);
int  mpdPED_Write(int id, int ch, int v);
int  mpdPED_Read(int id, int ch, int *ped_even, int *ped_odd);
int  mpdTHR_Write0(int id, int ch, int *thr_even, int *thr_odd);
int  mpdTHR_Write(int id, int ch, int v);
int  mpdTHR_Read(int id, int ch, int *thr_even, int *thr_odd);
int  mpdPEDTHR_Write(int id);

void mpdSetPedThrPath(int id, char *val);
char *mpdGetPedThrPath(int id);
int  mpdSetPedThr(int id, int ch, int p, int t);
int  mpdGetPed(int id, int ch);
int  mpdGetThr(int id, int ch);
int  *mpdGetApvPed(int id, int ach);
int  *mpdGetApvThr(int id, int ach);

void mpdSetPedThrCommon(int id, int p, int t);
int  mpdGetPedCommon(int id);
int  mpdGetThrCommon(int id);

void mpdSetNmpdToInit(int n);
int mpdGetNumberMPD();

void mpdSetFIRenable(int id, int flag);
int mpdGetFIRenable(int id);
void mpdSetFIRcoeff(int id, int idx, int val);
int mpdGetFIRcoeff(int id, int idx);


#ifdef NOTDONE
int  mpdReadPedThr(int id, std::string pname);
#endif /* NOTDONE */

int  mpdGetEBWordCount(int id);
int  mpdGetEventCount(int id);
int  mpdGetBlockCount(int id);
int  mpdOBUF_GetBlockCount(int id);
int  mpdGetTriggerCount(int id);
int  mpdGetTriggerReceivedCount(int id);
int  mpdSetFiberTestMode(int id, int enable, int period);
int  mpdSetSamplesPerEvent(int id, int samples);
int  mpdSetBlocklevel(int id, int blocklevel);
int  mpdSetBusyThreshold(int id, int thres);
int  mpdSetLocalBusyThreshold(int id, int thres);
int  mpdSetTriggerDelay(int id, int delay);

int  mpdFiberStatus(int id);
int  mpdFiberEnable(int id);
int  mpdFiberDisable(int id);
/* ASMI Commands  - write the flash memory */
void mpdASMI_reset(int id);
int mpdASMI_rdid(int id);
int mpdASMI_rdstatus(int id);
void mpdASMI_sec_erase(int id, uint32_t addr);      /* addr must be in the sector to erase */
int mpdASMI_rd(int id, uint32_t addr);
void mpdASMI_wr(int id, uint32_t addr, uint32_t data);

/* Remote Update commands - reboot factory or user FPGA image */
int mpdRUPD_reconfigure(int id, int pgm);
void mpdRUPD_setup(int id, int pgm);
void mpdRUPD_wr_param(int id, int par, int val);
int mpdRUPD_rd_param(int id, int par);

int mpdGStatus(int sflag);
int mpdApvStatus(int id, uint16_t apv_mask);
int mpdReset(int id, int pflag);
#endif /* __MPDLIB__ */


/*
  Local Variables:
  compile-command: "make -k install"
  End:
 */

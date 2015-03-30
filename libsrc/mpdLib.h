#ifndef __MPDLIB__
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

#define __MPDLIB__

#define MPD_BOARD_ID       0x12345678
#define MPD_MAX_BOARDS             21
#define MPD_MAX_APV                16

#define MPD_SUPPORTED_FIRMWARE 0x23

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

#define MPD_MSG(format, ...) {printf("%s: ",__FUNCTION__); printf(format, ## __VA_ARGS__);}
#define MPD_DUM(format, ...) {printf("%s: ",__FUNCTION__); printf(format, ## __VA_ARGS__);}
#define MPD_DBG(format, ...) {printf("%s: DEBUG: ",__FUNCTION__); printf(format, ## __VA_ARGS__);}
#define MPD_ERR(format, ...) {fprintf(stderr,"%s: ERROR: ",__FUNCTION__); fprintf(stderr,format, ## __VA_ARGS__);}

struct mpd_i2c_control_struct
{
  /* 0x0000 */ volatile uint32_t Clock_Prescaler_low;
  /* 0x0004 */ volatile uint32_t Clock_Prescaler_high;
  /* 0x0008 */ volatile uint32_t Control;
  /* 0x000C */ volatile uint32_t TxRx;
  /* 0x0010 */ volatile uint32_t CommStat;
  /* 0x0014 */          uint32_t blank0;
  /* 0x001C */ volatile uint32_t ApvReset;
  /* 0x0020 */
};

#define MPD_I2C_CONTROL_ENABLE_CORE  0x80

#define MPD_I2C_COMMSTAT_NACK        0x08
#define MPD_I2C_COMMSTAT_WRITE       0x10
#define MPD_I2C_COMMSTAT_READ        0x20
#define MPD_I2C_COMMSTAT_STOP        0x40
#define MPD_I2C_COMMSTAT_NACK_RECV   0x80
#define MPD_I2C_COMMSTAT_START_WRITE 0x90

#define MPD_I2C_APVRESET_ASYNC_SET   0x00
#define MPD_I2C_APVRESET_ASYNC_CLEAR 0x01

struct mpd_hist_block_struct
{
  /* 0x0000 */ volatile uint32_t Memory[(0x4000-0x0)>>2];
  /* 0x4000 */ volatile uint32_t CSR;
  /* 0x4004 */ volatile uint32_t Histo_Count;
  /* 0x4008 */          uint32_t blank0[(0x8000-0x4008)>>2];
  /* 0x8000 */
};

struct mpd_histogrammer_struct
{
  /* 0x000000 */ struct mpd_hist_block_struct block[2];
  /* 0x010000 */
};

struct mpd_ped_thres_apv_pair_struct
{
  /* 0x0000 */ volatile uint32_t ram[(0x0200-0x0)>>2];
  /* 0x0200 */          uint32_t blank0[(0x4000-0x0200)>>2];
  /* 0x4000 */
};

struct mpd_apv_daq_struct
{
  /* 0x000000 */ volatile uint32_t Data_Ch[16][(0x4000-0x0)>>2];
  /* 0x040000 */ struct   mpd_ped_thres_apv_pair_struct Ped[8];
  /* 0x060000 */ struct   mpd_ped_thres_apv_pair_struct Thres[8];
  /* 0x07C200 */          uint32_t blank0[(0x080000-0x07C200)>>2];
  /* 0x080000 */ volatile uint32_t Used_Word_Ch_Pair[8];
  /* 0x080020 */ volatile uint32_t FIFO_Status;
  /* 0x080024 */ volatile uint32_t Sync_Status;
  /* 0x080028 */ volatile uint32_t Missed_Trigger;
  /* 0x08002C */          uint32_t blank1;
  /* 0x080030 */ volatile uint32_t Readout_Config;
  /* 0x080034 */ volatile uint32_t Trig_Gen_Config;
  /* 0x080038 */ volatile uint32_t Logic_Thresholds;
  /* 0x08003C */ volatile uint32_t Control;
  /* 0x080040 */ volatile uint32_t SDRAM_Fifo_WAC;
  /* 0x080044 */ volatile uint32_t SDRAM_Fifo_RAC;
  /* 0x080048 */ volatile uint32_t SDRAM_Fifo_Word_Count;
  /* 0x08004C */ volatile uint32_t Output_Fifo_Word_Count;
  /* 0x080050 */          uint32_t blank2[(0x084000-0x080050)>>2];
  /* 0x084000 */ volatile uint32_t Trigger_Time_Fifo[4096];
  /* 0x088000 */
};

#define MPD_APVDAQ_TRIGCONFIG_ENABLE_MACH 0x00001000
#define SOFTWARE_CLEAR_MASK	 0x00080000	// @ TRIG_CONFIG_ADDR
#define SOFTWARE_TRIGGER_MASK	 0x00040000	// @ TRIG_CONFIG_ADDR

#define MPD_IN_TRIG  0
#define MPD_IN_TRIG1 0
#define MPD_IN_TRIG2 1
#define MPD_IN_SYNC  2
#define MPD_IN_P0    0
#define MPD_IN_FRONT 1


struct mpd_struct 
{
  /* 0x00000000 */ volatile uint32_t            SdramFifo[(0x01000000-0x0)>>2];
  /* 0x01000000 */ volatile uint32_t            AdcConfig;
  /* 0x01000004 */          uint32_t            blank1[(0x02000000-0x01000004)>>2];
  /* 0x02000000 */ struct   mpd_i2c_control_struct  I2C;
  /* 0x02000004 */          uint32_t            blank2[(0x03000000-0x02000004)>>2];
  /* 0x03000000 */ struct   mpd_histogrammer_struct Histo;
  /* 0x03010000 */          uint32_t            blank3[(0x04000000-0x03010000)>>2];
  /* 0x04000000 */ struct   mpd_apv_daq_struct      ApvDaq;
  /* 0x04880000 */          uint32_t            blank4[(0x05000000-0x04880000)>>2];
  /* 0x05000000 */ volatile uint32_t            SdramChip0[(0x06000000-0x05000000)>>2];
  /* 0x06000000 */ volatile uint32_t            SdramChip1[(0x07000000-0x06000000)>>2];
};


typedef struct apvparm_struct // actually a structure
{
  uint8_t i2cAddrScan; // i2c address from card discovery (scan)
  uint8_t i2cAddr;
  
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

  // config settings
  int   fCalibLatency;
  short fTriggerNumber;
  short fTriggerMode;
  short fAcqMode; // histo, event process ...

  int fInPath[2][3]; // define the path of the trigger, synch and clock ...

  int fEventBuilding;

  short fCommonNoiseSubtraction;
  int  fCommonOffset;

  int fLevel_0, fLevel_1;

  int fChannelMark;

  // adc (Ads5281) settings
  int fAdcClockPhase[2];
  int fAdcGain[2][8];
  int fAdcInvert[2];
  int fAdcPattern[2];

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
#define MPD_INIT_SKIP                (1<<16)
#define MPD_INIT_USE_ADDRLIST        (1<<17)
#define MPD_INIT_SKIP_FIRMWARE_CHECK (1<<18)


/* Function Prototypes */
STATUS mpdInit (UINT32 addr, UINT32 addr_inc, int nadc, int iFlag);
int  mpdCheckAddresses(int id);
int  mpdSlot(uint32_t i);
int  mpdLM95235_Read(int id, int *t);

/* I2C methods */
int  mpdI2C_ApvReset(int id);
int  mpdI2C_Init(int id);
int  mpdI2C_ByteWrite(int id, uint8_t dev_addr, uint8_t int_addr, 
		 int ndata, uint8_t *data);
int  mpdI2C_ByteRead(int id, uint8_t dev_addr, uint8_t int_addr, 
		int ndata, uint8_t *data);
int  mpdI2C_ByteWriteRead(int id, uint8_t dev_addr, uint8_t int_addr, 
			  int ndata, uint8_t *data);
int  mpdI2C_ByteRead1(int id, uint8_t dev_addr, uint8_t *data);

/* APV methods */
int  mpdAPV_Reset101(int id);
int  mpdAPV_Try(int id, uint8_t apv_addr);

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
int  mpdArmReadout(int id);


#endif /* __MPDLIB__ */

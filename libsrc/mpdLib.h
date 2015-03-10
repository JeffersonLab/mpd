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


struct mpd_i2c_control_struct
{
  /* 0x0000 */ volatile unsigned int Clock_Prescaler_low;
  /* 0x0004 */ volatile unsigned int Clock_Prescaler_high;
  /* 0x0008 */ volatile unsigned int Control;
  /* 0x000C */ volatile unsigned int TxRx;
  /* 0x0010 */ volatile unsigned int CommStat;
  /* 0x0014 */          unsigned int blank0;
  /* 0x001C */ volatile unsigned int ApvReset;
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
  /* 0x0000 */ volatile unsigned int Memory[(0x4000-0x0)>>2];
  /* 0x4000 */ volatile unsigned int CSR;
  /* 0x4004 */ volatile unsigned int Histo_Count;
  /* 0x4008 */          unsigned int blank0[(0x8000-0x4008)>>2];
  /* 0x8000 */
};

struct mpd_histogrammer_struct
{
  /* 0x000000 */ struct mpd_hist_block_struct block[2];
  /* 0x010000 */
};

struct mpd_ped_thres_apv_pair_struct
{
  /* 0x0000 */ volatile unsigned int ram[(0x0200-0x0)>>2];
  /* 0x0200 */          unsigned int blank0[(0x4000-0x0200)>>2];
  /* 0x4000 */
};

struct mpd_apv_daq_struct
{
  /* 0x000000 */ volatile unsigned int Data_Ch[16][(0x4000-0x0)>>2];
  /* 0x040000 */ struct   mpd_ped_thres_apv_pair_struct Ped[8];
  /* 0x060000 */ struct   mpd_ped_thres_apv_pair_struct Thres[8];
  /* 0x07C200 */          unsigned int blank0[(0x080000-0x07C200)>>2];
  /* 0x080000 */ volatile unsigned int Used_Word_Ch_Pair[8];
  /* 0x080020 */ volatile unsigned int FIFO_Status;
  /* 0x080024 */ volatile unsigned int Sync_Status;
  /* 0x080028 */ volatile unsigned int Missed_Trigger;
  /* 0x08002C */          unsigned int blank1;
  /* 0x080030 */ volatile unsigned int Readout_Config;
  /* 0x080034 */ volatile unsigned int Trig_Gen_Config;
  /* 0x080038 */ volatile unsigned int Logic_Thresholds;
  /* 0x08003C */ volatile unsigned int Control;
  /* 0x080040 */ volatile unsigned int SDRAM_Fifo_WAC;
  /* 0x080044 */ volatile unsigned int SDRAM_Fifo_RAC;
  /* 0x080048 */ volatile unsigned int SDRAM_Fifo_Word_Count;
  /* 0x08004C */ volatile unsigned int Output_Fifo_Word_Count;
  /* 0x080050 */          unsigned int blank2[(0x084000-0x080050)>>2];
  /* 0x084000 */ volatile unsigned int Trigger_Time_Fifo[4096];
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
  /* 0x00000000 */ volatile unsigned int            SdramFifo[(0x01000000-0x0)>>2];
  /* 0x01000000 */ volatile unsigned int            AdcConfig;
  /* 0x01000004 */          unsigned int            blank1[(0x02000000-0x01000004)>>2];
  /* 0x02000000 */ struct   mpd_i2c_control_struct  I2C;
  /* 0x02000004 */          unsigned int            blank2[(0x03000000-0x02000004)>>2];
  /* 0x03000000 */ struct   mpd_histogrammer_struct Histo;
  /* 0x03010000 */          unsigned int            blank3[(0x04000000-0x03010000)>>2];
  /* 0x04000000 */ struct   mpd_apv_daq_struct      ApvDaq;
  /* 0x04880000 */          unsigned int            blank4[(0x05000000-0x04880000)>>2];
  /* 0x05000000 */ volatile unsigned int            SdramChip0[(0x06000000-0x05000000)>>2];
  /* 0x06000000 */ volatile unsigned int            SdramChip1[(0x07000000-0x06000000)>>2];
};


typedef struct apvparm_struct // actually a structure
{
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
  unsigned int *fBuffer; // data buffer
  int fBufSize;
  int fBi0, fBi1, fBs;

} ApvParameters;

/* initialization flags */
#define MPD_INIT_SKIP                (1<<16)
#define MPD_INIT_USE_ADDRLIST        (1<<17)
#define MPD_INIT_SKIP_FIRMWARE_CHECK (1<<18)


/* Function Prototypes */
STATUS mpdInit (UINT32 addr, UINT32 addr_inc, int nadc, int iFlag);
int  mpdCheckAddresses(int id);
int  mpdSlot(unsigned int i);
int  mpdLM95235_Read(int *t);

/* I2C methods */
int  mpdI2C_ApvReset(int id);
int  mpdI2C_Init(int id);
int  mpdI2C_ByteWrite(int id, unsigned char dev_addr, unsigned char int_addr, 
		 int ndata, unsigned char *data);
int  mpdI2C_ByteRead(int id, unsigned char dev_addr, unsigned char int_addr, 
		int ndata, unsigned char *data);
int  mpdI2C_ByteWriteRead(int id, unsigned char dev_addr, unsigned char int_addr, 
			  int ndata, unsigned char *data);
int  mpdI2C_ByteRead1(int id, unsigned char dev_addr, unsigned char *data);
int  mpdI2C_SendByte(int id, unsigned char byteval, int start);
int  mpdI2C_ReceiveByte(int id, unsigned char *byteval);
int  mpdI2C_SendStop(int id);
int  mpdI2C_SendNack(int id);

/* APV methods */
int  mpdAPV_Reset101(int id);
int  mpdAPV_Try(int id, unsigned char apv_addr);

void mpdSetApvEnableMask(int id, unsigned short mask);
void mpdResetApvEnableMask(int id);
unsigned short mpdGetApvEnableMask(int id);
int  mpdAPV_Scan(int id);
int  mpdAPV_Write(int id, unsigned char apv_addr, unsigned char reg_addr, unsigned char val);
int  mpdAPV_Read(int id, unsigned char apv_addr, unsigned char reg_addr, unsigned char *val);
int  mpdAPV_Config(int id, int apv_index);
int  mpdApvGetPeakMode(int id);

void mpdAddApv(int id, ApvParameters v);
int  mpdApvGetFrequency(int id);
unsigned char mpdApvGetMaxLatency(int id);
int  mpdArmReadout(int id);


#endif /* __MPDLIB__ */

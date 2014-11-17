/**
 * MPD Low Level class and methods
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <string.h>
#include "MPDlo.h"

using namespace std;

MPDlo::MPDlo(Bus *cl, int s, int b)
{

  /*
   * MPD base address id defined by the switch on the board
   */
  const uint32_t SlotBaseAddr[MAX_BOARDS] = 
    {	0x00000000,	// NOT USED
	0x08000000,
	0x10000000,
	0x18000000,
	0x20000000,
	0x28000000,
	0x30000000,
	0x38000000,
	0x40000000,
	0x48000000,
	0x50000000,
	0x58000000,
	0x60000000,
	0x68000000,
	0x70000000,
	0x78000000,
	0x80000000,
	0x88000000,
	0x90000000,
	0x98000000,
	0xA0000000,
	0x00000000	// DESY trial on VME 32
    };

  fCommLink = cl;
  fBaseAddr = SlotBaseAddr[s];
  fSlot = s; // rotary switch

  fBus = b;

  fEnable = false;

  PllConfigOffset = 0;
  AdcConfigOffset = 0x1000000;
  I2CcontrollerOffset = 0x2000000;
  HistogrammerOffset = 0x3000000;
  ApvFifoOffset = 0x4000000;
  SdramOffset = 0x5000000;
  Last0Offset = 0x6000000;
  Last1Offset = 0x7000000;
  
  ResetApvEnableMask();
  SetAdcClockPhase(0,0);
  SetAdcClockPhase(0,1);

  SetFpgaRevision(); // default

  fApv.clear();
}

MPDlo::~MPDlo()
{
  for (unsigned int i=0;i<fApv.size(); i++) {
    ApvBufferFree(i);
  };
  fApv.clear();
  cout << __FUNCTION__ << " deleted" << endl;
}

/*
 * Scan the CR/CSR to discover the board
 *
 * If slot is negative, use internal fSlot value (from configuration file)
 * 
 * Return BUS_OK in case of board present, rev_id and timec should be different from 0  
 */
int MPDlo::CR_Scan(int slot, uint32_t *rev_id, uint32_t *timec)
{
  uint32_t cr_data, addr;
  uint32_t j;
  int success;
  uint32_t cr_len = 9;
  uint32_t cr_data_OK[13] = {'C', 'R',
			     0x08, 0x00, 0x30,		// Manufacturer ID (CERN)
			     0x00, 0x03, 0x09, 0x04,		// Board ID
			     0x00, 0x00, 0x00, 0x01 };	// Revision ID
  
  uint32_t CrCsrBaseAddr[MAX_BOARDS] = 
    {	0x00000000,	// NOT USED
	0x00080000,
	0x00100000,
	0x00180000,
	0x00200000,
	0x00280000,
	0x00300000,
	0x00380000,
	0x00400000,
	0x00480000,
	0x00500000,
	0x00580000,
	0x00600000,
	0x00680000,
	0x00700000,
	0x00780000,
	0x00800000,
	0x00880000,
	0x00900000,
	0x00980000,
	0x00A00000,
	0x00000000	// DESY trial on VME 32
    };
  
  *rev_id = 0;
  *timec = 0;

  if (slot < 0) { slot = fSlot; } // use object slot

  MPD_DBG("Start scan on MPD slot = %d\n",slot);

  if( slot < 1 || slot > (MAX_BOARDS-1) )
    return -1;

  addr = CrCsrBaseAddr[slot] + 0x1C;
  for(j=0; j<cr_len; j++) // loop on control register bytes
    {
      success = fCommLink->ReadCR(addr, &cr_data);
      if( success != BUS_OK )
	return success;
      if( cr_data != cr_data_OK[j] )
	return -5;
      addr += 4;
    }
  for(j=0; j<4; j++) // revision ID
    {
      success = fCommLink->ReadCR(addr, &cr_data);
      *rev_id <<= 8;
      *rev_id |= (cr_data & 0xFF);
      addr += 4;
      if( success != BUS_OK )
	return success;
    }

  if( *rev_id > 1 )
    {
      addr = CrCsrBaseAddr[slot] + 0x5C;
      for(j=0; j<4; j++) // time revision
	{
	  success = fCommLink->ReadCR(addr, &cr_data);
	  *timec <<= 8;
	  *timec |= (cr_data & 0xFF);
	  addr += 4;
	  if( success != BUS_OK )
	    return success;
	}
    }
  return BUS_OK;

}

/**
 *
 *
 */

void MPDlo::BOARD_Init(void)
{
  int i;

  MPD_MSG("Initializing MPD board %d:%d\n",fBus, fSlot);
  
  // ** I2C **
  if( I2C_Init() != BUS_OK )
    MPD_ERR("i2c init failed\n");

  // ** Delay lines (clock phase) **
  if( GetFpgaRevision() >= 2 ) {
    DELAY25_Set(GetAdcClockPhase(0), GetAdcClockPhase(1));
    MPD_MSG("Clock Phase = %d %d\n",GetAdcClockPhase(0), GetAdcClockPhase(1));
  }

  // ** APV **
  if( I2C_ApvReset() != BUS_OK )
    MPD_ERR("i2c apv reset failed\n");

  for(i=0; i<GetApvCount(); i++) {
    if( APV_Config(i) != BUS_OK )
      MPD_ERR("apv config card i2c=%d failed\n",fApv[i].i2c);
  }

  // ** ADCs **
  for (i=0;i<2;i++) {
    if( ADS5281_SetParameters(i) != BUS_OK )
      MPD_ERR("parameter set of ADC %d failed\n",i);

    switch( GetAdcPattern(i) ) {
    case MPD_ADS5281_PAT_NONE: ADS5281_Normal(i); break;
    case MPD_ADS5281_PAT_SYNC: ADS5281_Sync(i); break;
    case MPD_ADS5281_PAT_DESKEW: ADS5281_Deskew(i); break;
    case MPD_ADS5281_PAT_RAMP: ADS5281_Ramp(i); break;
    }
    if( GetAdcInvert(i) ) {
      ADS5281_InvertChannels(i);
    } else {
      ADS5281_NonInvertChannels(i);
    }

    //    for (k=0;k<8;k++) {
    if( ADS5281_SetGain(i, 
			GetAdcGain(i,0), GetAdcGain(i,1), GetAdcGain(i,2), GetAdcGain(i,3),
			GetAdcGain(i,4), GetAdcGain(i,5), GetAdcGain(i,6), GetAdcGain(i,7)) != BUS_OK )
      MPD_WRN("Set ADC Gain group %d failed\n",i);
    //    }

  }  

  MPD_MSG("Initialization of MPD board %d:%d done\n",fBus,fSlot);

}

/**
 *
 *
 */

int MPDlo::LM95235_Read(int *t)
{
  const unsigned char LM95235_i2c_addr  = 0x4C;
  //  const unsigned char Local_TempS_MSB_addr  = 0x00;	// Read only
  //  const unsigned char Local_TempS_LSB_addr  = 0x30;	// Read only
  //  const unsigned char Remote_TempS_MSB_addr  = 0x01;	// Read only
  //  const unsigned char Remote_TempS_LSB_addr  = 0x10;	// Read only
  const unsigned char Remote_TempU_MSB_addr  = 0x31;	// Read only
  const unsigned char Remote_TempU_LSB_addr  = 0x32;	// Read only
  //  const unsigned char ConfigReg2_addr  = 0xBF;
  //  const unsigned char RemoteOffset_H_addr  = 0x11;
  //  const unsigned char RemoteOffset_L_addr  = 0x12;
  //  const unsigned char ConfigReg1_addr  = 0x03;	// also 0x09
  //  const unsigned char ConvRate_addr  = 0x04;	// also 0x0A
  const unsigned char OneShot_addr  = 0x0F;	// Write only
  const unsigned char Status1_addr  = 0x02;	// Read only
  //  const unsigned char Status2_addr  = 0x33;	// Read only
  //  const unsigned char ManufID_addr  = 0xFE;	// Read only, returns 0x01
  //  const unsigned char RevID_addr  = 0xFF;	// Read only

  unsigned char val;
  int success, retry_count;

  success = I2C_ByteWrite((unsigned char)(LM95235_i2c_addr<<1), OneShot_addr, 0, &val);
  success = I2C_ByteWrite((unsigned char)(LM95235_i2c_addr<<1), Status1_addr, 0, &val);
  val = 0x80;
  retry_count = 0;
  while( (val & 0x80) && (retry_count++ < 100) )
    success = I2C_ByteRead1((unsigned char)(LM95235_i2c_addr<<1), &val);
  success = I2C_ByteWrite((unsigned char)(LM95235_i2c_addr<<1), Remote_TempU_MSB_addr, 0, &val);
  success = I2C_ByteRead1((unsigned char)(LM95235_i2c_addr<<1), &val);
  *t = val;
  success = I2C_ByteWrite((unsigned char)(LM95235_i2c_addr<<1), Remote_TempU_LSB_addr, 0, &val);
  success = I2C_ByteRead1((unsigned char)(LM95235_i2c_addr<<1), &val);
  // aggiornare t
  return success;

}


//======================================================
// i2c methods

int MPDlo::I2C_ApvReset(void)
{
  uint32_t addr;
  uint32_t data;
  int success;
  
  addr = I2CcontrollerOffset + 28;	/* APV Reset Register */
  data = 0x01;	/* Async Clear APV_RESET line (negated) */
  if( (success = BUS_Write(addr, &data)) != BUS_OK )
    return success;
  data = 0x00;	/* Async Set APV_RESET line (negated) */
  return BUS_Write(addr, &data);
  
}

/*
 * set I2c speed and other stuff
 */
int MPDlo::I2C_Init(void)
{
  uint32_t addr;
  uint32_t data, rdata;
  int success;
  
  /* 
     clock_prescaler = prescaler_high * 256 + prescaler_low
     period = clock_prescale/10 us
  */

  addr = I2CcontrollerOffset;		/* Prescaler low (RW) */
  addr = I2CcontrollerOffset + 4;	/* Prescaler high (RW) */
  addr = I2CcontrollerOffset + 8;	/* Control Register (RW) */
  addr = I2CcontrollerOffset + 12;	/* TX (W) and RX (R) Register */
  addr = I2CcontrollerOffset + 16;	/* Command (W) and Status (R) Register */
	
  addr = I2CcontrollerOffset + 8;	/* Control Register (RW) */
  data = 0x00;	/* Disable I2C Core and interrupts */
  if( (success = BUS_Write(addr, &data)) != BUS_OK )
    {
      MPD_ERR("disabling i2c core and interface (offset 0x%x) failed (ret. %d)\n",addr,success);
      return success;
    }

  int ispeed = GetI2CSpeed();
  
  addr = I2CcontrollerOffset;		/* Prescaler low (RW) */
  data = (ispeed & 0xff); /* works for 20 m long cable */
  
  if( (success = BUS_Write(addr, &data)) != BUS_OK )
    {
      MPD_ERR("write i2c low prescaler (offset 0x%x) failed (ret. %d)\n",addr,success);
      return success;
    }
  BUS_Read(addr, &rdata);
  MPD_DBG("i2c low prescaler register (0x%x) set/read : %d / %d\n",addr,data,rdata);

  addr = I2CcontrollerOffset + 4;		/* Prescaler high (RW) */
  data = (ispeed>>8) & 0xff;  
  if( (success = BUS_Write(addr, &data)) != BUS_OK )
    {
      MPD_ERR("write i2c high prescaler (offset 0x%x) failed (ret. %d)\n",addr,success);
      return success;
    }

  BUS_Read(addr, &rdata);
  MPD_DBG("i2c high prescaler register (0x%x) set/read : %d / %d\n",addr,data,rdata);

  MPD_MSG("i2c speed prescale = %d, (period = %f us, frequency = %f kHz)\n", ispeed, ispeed/10., 10000./ispeed);
  
  addr = I2CcontrollerOffset + 8;	/* Control Register (RW) */
  data = 0x80;	/* Enable I2C Core and disable interrupts */
  if( (success = BUS_Write(addr, &data)) != BUS_OK )
    MPD_ERR("enabling i2c core and disable interrups (offset 0x%x) failed (ret. %d)\n", addr, success);
  return success;

}

int MPDlo::I2C_ByteWrite(unsigned char dev_addr, unsigned char int_addr, int ndata, unsigned char *data)
{
  int success, i;
  
  if( (success = I2C_SendByte((unsigned char)(dev_addr & 0xFE), 1)) != BUS_OK )
    {
      if( success <= -10 )
	{
	  I2C_SendStop();
	  return success;
	}
      else
	return I2C_SendStop();
    }
  //  usleep(300);
  if( (success = I2C_SendByte(int_addr, 0)) != BUS_OK )
    {
      if( success <= -10 )
	{
	  I2C_SendStop();
	  return success;
	}
      else
	return I2C_SendStop();
    }
  //  usleep(300);
  for(i=0; i<ndata; i++)
    if( (success = I2C_SendByte(data[i], 0)) != BUS_OK )
      {
	if( success <= -10 )
	  {
	    I2C_SendStop();
	    return success;
	  }
	else
	  return I2C_SendStop();
      }
  //  usleep(300);
  success = I2C_SendStop();
  // usleep(300);
  return success;

}

int MPDlo::I2C_ByteRead(unsigned char dev_addr, unsigned char int_addr, int ndata, unsigned char *data)
{
  int success, i;
  
  if( (success = I2C_SendByte((unsigned char)(dev_addr | 0x01), 1)) != BUS_OK )
    return I2C_SendStop();
  
  if( (success = I2C_SendByte(int_addr, 0)) != BUS_OK )
    return I2C_SendStop();
  
  for(i=0; i<ndata; i++)
    if( (success = I2C_ReceiveByte(data+i)) != BUS_OK )
      return I2C_SendStop();
  
  return I2C_SendStop();
}

int MPDlo::I2C_ByteWriteRead(unsigned char dev_addr, unsigned char int_addr, int ndata, unsigned char *data)
{
  int success, i;
  
  if( (success = I2C_SendByte((unsigned char)(dev_addr & 0xFE), 1)) != BUS_OK )
    return I2C_SendStop();
  
  if( (success = I2C_SendByte(int_addr, 0)) != BUS_OK )
		return I2C_SendStop();
  
  for(i=0; i<ndata; i++)
    if( (success = I2C_ReceiveByte(data+i)) != BUS_OK )
      return I2C_SendStop();
  
  return I2C_SendStop();
}

int MPDlo::I2C_ByteRead1(unsigned char dev_addr, unsigned char *data)
{
  int success;
  
  if( (success = I2C_SendByte((unsigned char)(dev_addr & 0xFE), 1)) != BUS_OK )
    return I2C_SendStop();
  
  if( (success = I2C_ReceiveByte(data)) != BUS_OK )
    return I2C_SendStop();
  
  return I2C_SendStop();
}

int MPDlo::I2C_SendByte(unsigned char byteval, int start)
{
  uint32_t addr;
  uint32_t data;
  int success, retry_count;
  
  addr = I2CcontrollerOffset + 12;	/* TX (W) and RX (R) Register */
  data = byteval;
  if( (success = BUS_Write(addr, &data)) != BUS_OK )
    return success;

  addr = I2CcontrollerOffset + 16;	/* Command (W) and Status (R) Register */
  if( start )
    data = 0x90;	/* START + WRITE */
  else
    data = 0x10;	/* WRITE */
  if( (success = BUS_Write(addr, &data)) != BUS_OK )
    return success;
  
  retry_count = 0;
  data = 0x00000002;
  while( (data & 0x00000002) != 0 && success == BUS_OK && retry_count < GetI2CMaxRetry() )
    {
      success = BUS_Read(addr, &data);
      retry_count++;
    }
  if( success != BUS_OK )
    return success;
  if( retry_count >= GetI2CMaxRetry() )
    return -10;
  if( data & 0x00000080 )	/* NACK received */
    return -20;
  return success;

}

int MPDlo::I2C_ReceiveByte(unsigned char *byteval)
{
  uint32_t addr;
  uint32_t data;
  int success, retry_count;
	
  *byteval = 0;
  addr = I2CcontrollerOffset + 16;	/* Command (W) and Status (R) Register */
  data = 0x20;	/* READ */
  if( (success = BUS_Write(addr, &data)) != BUS_OK )
    return success;

  retry_count = 0;
  data = 0x00000002;
  while( (data & 0x00000002) != 0 && success == BUS_OK && retry_count < GetI2CMaxRetry() )
    {
      success = BUS_Read(addr, &data);
      retry_count++;
    }
  if( success != BUS_OK )
    return success;
  if( retry_count >= GetI2CMaxRetry() )
    return -10;
  if( data & 0x00000080 )	/* NACK received */
    return -20;
  addr = I2CcontrollerOffset + 12;	/* TX (W) and RX (R) Register */
  if( (success = BUS_Read(addr, &data)) != BUS_OK )
    return success;
  *byteval = data;
  return success;
}

int MPDlo::I2C_SendStop(void)
{
  uint32_t addr;
  uint32_t data;
  
  addr = I2CcontrollerOffset + 16;	/* Command (W) and Status (R) Register */
  data = 0x40;	/* STOP */
  return BUS_Write(addr, &data);
}

int MPDlo::I2C_SendNack(void)
{
  uint32_t addr;
  uint32_t data;

  addr = I2CcontrollerOffset + 16;	/* Command (W) and Status (R) Register */
  data = 0x08;	/* NACK */
  return BUS_Write(addr, &data);
}

//======================================================
// apv methods

//using namespace std;

/* APV mode register bit settings */
#define ANALOG_BIAS	0x01
#define TRIG_MODE	0x02
#define CAL_INHIBIT	0x04
#define RO_MODE		0x08
#define RO_FREQ		0x10
#define PRE_POL		0x20

#define ANALOG_BIAS_ON	ANALOG_BIAS
#define ANALOG_BIAS_OFF	0x00
#define TRIG_MODE_1	TRIG_MODE
#define TRIG_MODE_3	0x00
#define CAL_INHIBIT_ON	CAL_INHIBIT
#define CAL_INHIBIT_OFF	0x00
#define RO_MODE_PEAK	RO_MODE
#define RO_MODE_DEC	0x00
#define RO_FREQ_40	RO_FREQ
#define RO_FREQ_20	0x00
#define PRE_POL_INV	PRE_POL
#define PRE_POL_NONINV	0x00

/* Internal APV register addresses */
static const unsigned char ipre_addr = 0x20;
static const unsigned char ipcasc_addr = 0x22;
static const unsigned char ipsf_addr = 0x24;
static const unsigned char isha_addr = 0x26;
static const unsigned char issf_addr = 0x28;
static const unsigned char ipsp_addr = 0x2A;
static const unsigned char imuxin_addr = 0x2C;
static const unsigned char ispare_addr = 0x2E;
static const unsigned char ical_addr = 0x30;
static const unsigned char vfp_addr = 0x32;
static const unsigned char vfs_addr = 0x34;
static const unsigned char vpsp_addr = 0x36;
static const unsigned char cdrv_addr = 0x38;
static const unsigned char csel_addr = 0x3A;
static const unsigned char mode_addr = 0x02;
static const unsigned char latency_addr = 0x04;
static const unsigned char muxgain_addr = 0x06;
static const unsigned char error_addr = 0x00;

static const unsigned char def_Mode =	PRE_POL_NONINV | RO_FREQ_40 |
  RO_MODE_PEAK   | CAL_INHIBIT_OFF |
  TRIG_MODE_1    | ANALOG_BIAS_ON;

int MPDlo::APV_Reset101(void)
{
  uint32_t addr, data;
  int success;

  addr = ApvFifoOffset + TRIG_CONFIG_ADDR;
  if( (success = BUS_Read(addr, &data)) != BUS_OK )
    return success;
  data |= 0x00001000;	// Enable trig machine
  data |= SOFTWARE_CLEAR_MASK;
  if( (success = BUS_Write(addr, &data)) != BUS_OK )
    return success;
  data &= ~SOFTWARE_CLEAR_MASK;
  data &= ~0x00001000;	// Disable trig machine
  return BUS_Write(addr, &data);
}

/*
 * return true if apv is present
 */
bool MPDlo::APV_Try(unsigned char apv_addr) // i2c addr
{
  unsigned char x = 0xEC;
  return ((APV_Read(apv_addr, mode_addr, &x) == BUS_OK) ? true : false);
  //	return APV_Write(apv_addr, mode_addr, def_Mode);
}


int MPDlo::APV_Scan() {

  // print information (debug only, should be done in verbose mode only)
  cout << __FUNCTION__ << ": Blind scan: " << endl;
  for (int k=0;k<31;k++) {
    if ( APV_Try(k) ) {
      cout << __FUNCTION__ << ": APV i2c = " << k << " found in MPD " << GetBus() << ":" << GetSlot() << endl;
    }
  }
  cout << __FUNCTION__ << ": Blind scan done" << endl;

  ResetApvEnableMask();
  vector<ApvParameters>::iterator it = fApv.begin();
  while (it<fApv.end()) { // look at configured cards only
    cout << __FUNCTION__ << ": Try " << (*it).i2c << " " << (*it).adc << endl;
    if ( APV_Try((*it).i2c) && (*it).adc>-1 ) {
      cout << __FUNCTION__ << ": APV i2c = " << (*it).i2c << " matched in MPD " << GetBus() << ":" << GetSlot() << endl;
      SetApvEnableMask((1 << (*it).adc));
      cout << __FUNCTION__ << ": APV enable mask 0x" << (hex) << GetApvEnableMask() << (dec) << endl;
      it++;
    } else {
      cout << __FUNCTION__ << ": APV i2c = " << (*it).i2c << " does not respond, it is removed from db" << endl;
      it = fApv.erase(it);
    }

  }

  cout << __FUNCTION__ << ": " << fApv.size() << " APV found matching settings" << endl;

  return (int) fApv.size();

}


int MPDlo::APV_Write(unsigned char apv_addr, unsigned char reg_addr, unsigned char val)
{
  return I2C_ByteWrite((unsigned char)((0x20 | apv_addr)<<1), reg_addr, 1, &val);
}

int MPDlo::APV_Read(unsigned char apv_addr, unsigned char reg_addr, unsigned char *val)
{
  int success;
  unsigned char rval;

  usleep(500);

  success = I2C_ByteWrite((unsigned char)((0x20 | apv_addr)<<1), (reg_addr|0x01), 1, val);
  if( success != BUS_OK )
    return success;
  
  usleep(500);
  success = I2C_ByteRead1((unsigned char)((0x20 | apv_addr)<<1), &rval);

  MPD_MSG("Set / Get = 0x%x 0x%x\n",*val,rval);

  return success;
}

int MPDlo::APV_Config(int apv_index)
{
  int success, i;

  unsigned char apv_addr, reg_addr, val;

  apv_addr = fApv[apv_index].i2c;

  MPD_MSG("APV card i2c=%d to ADC (fifo)=%d (from config file)\n",(int) apv_addr, v2a(apv_index));
  for (i=0;i<18;i++) {
    switch (i) {
    case 0:
      reg_addr = mode_addr;
      val = 0;
      break;
    case 1:
      reg_addr = ipre_addr;
      val = fApv[apv_index].Ipre;
      break;
    case 2:
      reg_addr = ipcasc_addr;
      val = fApv[apv_index].Ipcasc;
      break;
    case 3:
      reg_addr = ipsf_addr;
      val = fApv[apv_index].Ipsf;
      break;
    case 4:
      reg_addr = isha_addr;
      val = fApv[apv_index].Isha;
      break;
    case 5:
      reg_addr = issf_addr;
      val = fApv[apv_index].Issf;
      break;
    case 6:
      reg_addr = ipsp_addr;
      val = fApv[apv_index].Ipsp;
      break;
    case 7:
      reg_addr = imuxin_addr;
      val = fApv[apv_index].Imuxin;
      break;
    case 8:
      reg_addr = ispare_addr;
      val = fApv[apv_index].Ispare;
      break;
    case 9:
      reg_addr = ical_addr;
      val = fApv[apv_index].Ical;
      break;
    case 10:
      reg_addr = vfp_addr;
      val = fApv[apv_index].Vfp;
      break;
    case 11:
      reg_addr = vfs_addr;
      val = fApv[apv_index].Vfs;
      break;
    case 12:
      reg_addr = vpsp_addr;
      val = fApv[apv_index].Vpsp;
      break;
    case 13:
      reg_addr = cdrv_addr;
      val = fApv[apv_index].Cdrv;
      break;
    case 14:
      reg_addr = csel_addr;
      val = fApv[apv_index].Csel;
      break;
    case 15:
      reg_addr = latency_addr;
      val = fApv[apv_index].Latency;
      break;
    case 16:
      reg_addr = muxgain_addr;
      val = fApv[apv_index].Muxgain;
      break;
    case 17:
      reg_addr = mode_addr;
      val = fApv[apv_index].Mode;
      break;
      
    default:
      cout << __FUNCTION__ << "/ ERROR: This message should not appear, please check code consistency" << endl;
    }

    usleep(300);
    success = APV_Write(apv_addr, reg_addr, val);
	
    if (success != BUS_OK) {
      cout << __FUNCTION__ << "/ I2C Bus Error: i/addr/reg/val/err " << dec << i << " " 
	   << (int) apv_addr << hex 
	   << " 0x" << (int) reg_addr << " 0x" << (int) val << " 0x" << success << dec << endl;
      return success;
    }
  } // end loop

  fApv[apv_index].fNumberSample = GetTriggerNumber()*ApvGetPeakMode(); // must be set !!
  ApvBufferFree(apv_index);
  ApvBufferAlloc(apv_index); // alloc readout buffer

  return success;

}

/**
 * Return the setting value of the number of samples per trigger (1 or 3)
 * this is the same for all Apvs 
 * return -1 if there are no APV connected
 */
int MPDlo::ApvGetPeakMode() {

  int c=-1;

  if (fApv.size()>0) {
    c = 3 - (fApv[0].Mode & 2);
  }

  return c;
}

void MPDlo::AddApv(ApvParameters v) { 
  fApv.push_back(v);
  fApv[fApv.size()-1].fBuffer = 0;

  cout << __FUNCTION__ << ": APV " << fApv.size()-1 << " added to list of FECs" << endl;
}

/*
 * Return the clock frequency of the APV (0 = 20 MHz, 1 = 40 MHz)
 * the frequency must be the same for all APV on the same MPD
 * return -1 in case of error (no APV on MPDs)
 */

int MPDlo::ApvGetFrequency() {
  int c=0;

  if (fApv.size()>0) {
    c = (fApv[0].Mode & 0x10) >> 4;
  }

  return c;
}

/**
 * Return max latency of all APVs in a single MPD
 */
unsigned char MPDlo::ApvGetMaxLatency() {
  unsigned char c=0;

  for (unsigned int i=0;i<fApv.size();i++) {
    c = (fApv[i].Latency > c) ? fApv[i].Latency : c;
  }
  return c;
}

/**
 * Set the number of samples / event expected from the single Apv
 */
int MPDlo::ArmReadout() {

  for (unsigned int i=0;i<fApv.size();i++) {
    ApvSetSampleLeft(i); // improve peak mode
    fApv[i].fBi0 = 0; // begin of buffer (should be always 0)
    fApv[i].fBs = 0;  // end of event (last sample end mark)
    fApv[i].fBi1 = 0; // end of buffer
  }
  fReadDone = false;

  return 0;

}

//======================================================
// trigger methods

int MPDlo::TRIG_BitSet() { 
/*
	uint32_t addr;
	uint32_t data;
	int success;

	addr = fBaseAddr + ApvFifoOffset;
	
	addr += 0x20000;
	addr += 0x28;	
	if( (success = BUS_Read(addr, &data)) != BUS_OK )
	  return success;
	data |= 0x40000000;
	return BUS_Write(addr, &data);
*/
	return 0;
}

int MPDlo::TRIG_BitClear() {
/*
	uint32_t addr;
	uint32_t data;
	int success;

	addr = fBaseAddr + ApvFifoOffset;
	
	addr += 0x20000;
	addr += 0x28;	
	if( (success = BUS_Read(addr, &data)) != BUS_OK )
	  return success;
	data &= 0xBFFFFFFF;
	return BUS_Write(addr, &data);
*/
	return 0;
}

int MPDlo::TRIG_Enable() {
  uint32_t addr;
  uint32_t data;
  int success;

  unsigned char sync_period;
  unsigned char reset_latency;
  unsigned char mark_ch;

  mark_ch = (unsigned char) GetChannelMark(); // set one of the 128 channels of all apv to 0xfff (mark a single channel of the frame)

  sync_period = (ApvGetFrequency() == 1) ? 34 : 69; // synch period in number of clock - 1 (34 @ 40 MHz, 69 @ 20 MHz) @@@ To be checked, ask Paolo

  reset_latency = 15 + ApvGetMaxLatency(); // @@@ To be ckecked, ask paolo for meaning

  addr =ApvFifoOffset + SYNC_ENABLE_ADDR;
  data = mark_ch << 24 | sync_period << 16 | GetApvEnableMask();

  if( (success = BUS_Write(addr, &data)) != BUS_OK )
    return success;

  addr = ApvFifoOffset + TRIG_CONFIG_ADDR;
  if( GetFpgaRevision() < 2 )
    data =  GetAdcClockPhase(0) | 	// This works only for ADC board rev 0
      ((GetTriggerMode() & 0x07) << 12) |
      ((GetTriggerNumber() & 0x0F) << 8) | reset_latency;
  else
    data = 
      ((GetCalibLatency() & 0xFF) << 24) |
      ((GetInPathI(MPD_IN_FRONT, MPD_IN_TRIG) & 0x01) << 23) |
      ((GetInPathI(MPD_IN_P0, MPD_IN_TRIG2) & 0x01) << 22) |
      ((GetInPathI(MPD_IN_P0, MPD_IN_TRIG1) & 0x01) << 21) |
      ((GetInPathI(MPD_IN_FRONT, MPD_IN_SYNC) & 0x01) << 17) |
      ((GetInPathI(MPD_IN_P0, MPD_IN_SYNC) & 0x01) << 16) |
      //	  ((test_mode & 0x01) << 15) |
      ((GetTriggerMode() & 0x07) << 12) |
      ((GetTriggerNumber() & 0x0F) << 8) | reset_latency;

  if( (success = BUS_Write(addr, &data)) != BUS_OK )
    return success;

  addr = ApvFifoOffset + ONE_ZERO_THR_ADDR;
  data = (GetOneLevel() << 16) | GetZeroLevel();

  /*
    std::cout << __FUNCTION__ << " Trigger Mode : " << (trig_mode & 0x7) << std::endl;
    std::cout << __FUNCTION__ << " Trigger Front: " << (EnTrig_Front & 0x1) << std::endl;
    std::cout << __FUNCTION__ << " Number of Triggers: " << (max_trig_out & 0xF) << std::endl;
  */
  return BUS_Write(addr, &data);
}

int MPDlo::TRIG_Disable(void)
{
  uint32_t addr;
  uint32_t data;
  int success;
  
  addr = ApvFifoOffset + SYNC_ENABLE_ADDR;
  data = 0;
  if( (success = BUS_Write(addr, &data)) != BUS_OK )
    return success;
  addr = ApvFifoOffset + TRIG_CONFIG_ADDR;	// TRIG_GEN_CONFIG
  return BUS_Write(addr, &data);

}

int MPDlo::TRIG_GetMissed(uint32_t *missed)
{
  uint32_t base_addr, missed_addr;
  
  base_addr = ApvFifoOffset;	
  missed_addr = base_addr + MISSED_TRIGGER_ADDR;

  return BUS_Read(missed_addr, missed);
}

/**
 * Set the delay chip that define the time delay between adc_clock and apv_clock
 *
 */

int MPDlo::DELAY25_Set(int apv1_delay, int apv2_delay)
{
  unsigned char val;

  //      I2C_ByteWrite(0xF0 , (apv2_delay & 0x3F) | 0x40, 0, &val);      // CR0: APV2 out
  I2C_ByteWrite(0xF0 , 0x40, 0, &val);    // CR0: APV2 out not delayed
  Sleep(10);
  //      I2C_ByteWrite(0xF2 , 0x40, 0, &val);    // CR1: ADC1 out not delayed
  I2C_ByteWrite(0xF2 , (apv1_delay & 0x3F) | 0x40, 0, &val);      // CR1: ADC1 clock delayed
  Sleep(10);
  //      I2C_ByteWrite(0xF4 , 0x40, 0, &val);    // CR2: ADC2 out not delayed
  I2C_ByteWrite(0xF4 , (apv2_delay & 0x3F) | 0x40, 0, &val);      // CR2: ADC2 clock delayed
  Sleep(10);
  I2C_ByteWrite(0xF6 , 0x00, 0, &val);    // CR3: Not used output
  Sleep(10);
  //      I2C_ByteWrite(0xF8 , (apv1_delay & 0x3F) | 0x40, 0, &val);      // CR4: APV1 out
  I2C_ByteWrite(0xF8 , 0x40, 0, &val);    // CR4: APV1 out not delayed
  Sleep(10);
  I2C_ByteWrite(0xFA , 0x00, 0, &val);    // GCR (40 MHz)
  Sleep(10);

  return 0;
}

//======================================================
// adc methods

#define MPD_ADC_TOUT 1000
#define MPD_ADC_USLEEP 50

/**
 * adc = 0 or 1 (first or second adc in MPD)
 */
int MPDlo::ADS5281_Set(int adc, uint32_t val) {

  int success, retry_count;
  uint32_t data;

  data = (adc == 0) ? 0x40000000 : 0x80000000 ;
  data |= val;
  if( (success = BUS_Write(AdcConfigOffset, &data)) != BUS_OK ) return success;
  usleep(MPD_ADC_USLEEP);

  MPD_DUM("Write Adc= %d: value= 0x%x, success= 0x%x\n",adc, data, success);

  retry_count = 0;
  data = 0xC0000000;
  while( ((data & 0xFFFFFFF) != val) && (success == BUS_OK) && (retry_count < MPD_ADC_TOUT) ) {
    success = BUS_Read(AdcConfigOffset, &data);
    MPD_DUM("Read Adc= %d: value= 0x%x, success= 0x%x\n",adc, data, success);
    usleep(MPD_ADC_USLEEP);
    retry_count++;
  }

  MPD_DBG("Adc= %d: set/get value= 0x%x/0x%x, success= 0x%x, retry= %d\n",adc, val, data, success, retry_count);

  if( success != BUS_OK ) return success;
  if( retry_count >= MPD_ADC_TOUT ) return BUS_TIMEOUT;

  return success;

}

int MPDlo::ADS5281_InvertChannels(int adc)	/* adc == 0, 1 */
{
  MPD_MSG("Board= %d ADC= %d Inverted Polarity\n",fSlot, adc);
  return ADS5281_Set(adc, 0x2400FF);
}

int MPDlo::ADS5281_NonInvertChannels(int adc)	/* adc == 0, 1 */
{
  MPD_MSG("Board= %d ADC= %d Direct Polarity\n",fSlot, adc);
  return ADS5281_Set(adc, 0x240000);
  
}

int MPDlo::ADS5281_SetParameters(int adc)	/* adc == 0, 1 */
{
  int success;
  
  if (GetHWRevision()>=4) {
    if ((success = ADS5281_Set(adc, 0x428021)) != BUS_OK) { return success; } // Differential clock
  } else {
    if ((success = ADS5281_Set(adc, 0x428020)) != BUS_OK) { return success; } // Single ended clock
  }
  usleep(MPD_ADC_USLEEP);
  return ADS5281_Set(adc, 0x110000);
}

int MPDlo::ADS5281_Normal(int adc)	/* adc == 0, 1 */
{
  int success;

  MPD_MSG("Board= %d ADC= %d No Pattern (Normal Acq)\n",fSlot, adc);
  if ((success = ADS5281_Set(adc, 0x450000)) != BUS_OK) { return success; }
  usleep(MPD_ADC_USLEEP);
  return ADS5281_Set(adc, 0x250000);
}

int MPDlo::ADS5281_Sync(int adc)	/* adc == 0, 1 */
{
  int success;

  MPD_MSG("Board= %d ADC= %d Sync Pattern\n",fSlot, adc);
  if ((success = ADS5281_Set(adc, 0x250000)) != BUS_OK) { return success; }
  usleep(MPD_ADC_USLEEP);
  return ADS5281_Set(adc, 0x450002);
}

int MPDlo::ADS5281_Deskew(int adc)	/* adc == 0, 1 */
{
  int success;

  MPD_MSG("Board= %d ADC= %d Deskew Pattern\n",fSlot, adc);
  if ((success = ADS5281_Set(adc,0x250000)) != BUS_OK) { return success; }
  usleep(MPD_ADC_USLEEP);
  return ADS5281_Set(adc, 0x450001);
}

int MPDlo::ADS5281_Ramp(int adc)	/* adc == 0, 1 */
{
  int success;

  MPD_MSG("Board= %d ADC= %d Ramp Pattern\n",fSlot, adc);
  if ((success = ADS5281_Set(adc, 0x450000)) != BUS_OK) { return success; }
  usleep(MPD_ADC_USLEEP);
  return ADS5281_Set(adc, 0x250040);
}


int MPDlo::ADS5281_SetGain(int adc, 
			   int gain0, int gain1, int gain2, int gain3, 
			   int gain4, int gain5, int gain6, int gain7)
{
  uint32_t data;
  int success;

  MPD_MSG("Board= %d ADC= %d Set Gain %d %d %d %d %d %d %d %d\n",fSlot, adc,
	  gain0, gain1, gain2, gain3, gain4, gain5, gain6, gain7);

  data = 0x2A0000;
  data |= gain0 | (gain1<<4) | (gain2<<8) | (gain3<<12);
  if ((success = ADS5281_Set(adc, data)) != BUS_OK) { return success; }
  usleep(MPD_ADC_USLEEP);
  data = 0x2B0000;
  data |= gain7 | (gain6<<4) | (gain5<<8) | (gain4<<12);
  return ADS5281_Set(adc, data);

}

//======================================================
// histogramming methods

int MPDlo::HISTO_Clear(int ch, int val)	/* ch == 0, 15 */
{
  uint32_t addr;
  uint32_t data[4096];
  int success, i, j;

  addr = HistogrammerOffset;
  if( ch >= 8 ) addr += 0x8000;

  if (val<0) {
    for(i = 0; i<4096; i++) data[i] = i;
  } else {
    for(i = 0; i<4096; i++) data[i] = val;
  }

#ifdef BLOCK_TRANSFER
  success = BUS_BlockWrite(addr, 4096, data, &j);
  if( j != (4096) ) success = BUS_GENERIC_ERROR;
#else

  int ntimes = 4096 / 64;
  for(i = 0; i<ntimes; i++) {
    for(j = 0; j<64; j++) {	/* single word transfer */
      success = BUS_Write(addr, &(data[i*64+j]));
      addr += 4;
      if( success != BUS_OK )
	break;
    }
  }
#endif
  return success;
}

int MPDlo::HISTO_Start(int ch)	/* ch == 0, 15 */
{
  uint32_t addr;
  uint32_t data;

  addr = (ch < 8) ? 0x4000 : 0xC000;
  addr += HistogrammerOffset;

  data = 0x80 | (ch & 0x07);

  return BUS_Write(addr, &data);
}

int MPDlo::HISTO_Stop(int ch)	/* ch == 0, 15 */
{
  uint32_t addr;
  uint32_t data;

  addr = (ch < 8) ? 0x4000 : 0xC000;
  addr += HistogrammerOffset;

  data = (ch & 0x07);

  return BUS_Write(addr, &data);
}

int MPDlo::HISTO_GetIntegral(int ch, uint32_t *integral)	/* ch == 0, 15 */
{
  uint32_t addr;
  
  addr = (ch < 8) ? 0x4004 : 0xC004;
  addr += HistogrammerOffset;

  return BUS_Read(addr, integral);
}

int MPDlo::HISTO_Read(int ch, uint32_t *histogram)	/* ch == 0, 15; uint32_t histogram[4096] */
{
  uint32_t addr;
  int success, j;

  addr = HistogrammerOffset;
  if( ch >= 8 ) addr += 0x8000;
#ifdef BLOCK_TRANSFER
  success = BUS_BlockRead(addr, 4096, histogram, &j);
  if( j != 4096 ) {
    MPD_ERR("Block Transfer returned %d 32bit words, 4096 expected\n",j);
    success = BUS_GENERIC_ERROR;
  }
#else
  int ntimes = 4096 / 64;
  for(int i = 0; i<ntimes; i++) {
    for(j = 0; j<64; j++) {	/* single word transfer */
      success = BUS_Read(addr, histogram+i*64+j);
      addr += 4;
      if( success != BUS_OK ) break;
    }
  }
#endif
  return success;
}


//======================================================
// Daq-Readout methods


/**
 * Readout Fifo
 * Standard Event Mode (no zero suppression or pedestal subtraction)
 * Return 0 when something has been read, error otherwise
 */
int MPDlo::FIFO_ReadSingle(int channel,     // apv channel (FIFO)
			   uint32_t *dbuf, // data buffer
			   int &wrec,      // max number of words to get / return words received 
			   int max_retry)   // max number of retry for timeout
//			   int &err)        // return error code
//			   int &n_events)   // number of events
{
  
  uint32_t base_addr, fifo_addr;
  int success, i, size;
  int nwords; // words available in fifo
  int wmax; // maximum word acceptable

  base_addr = ApvFifoOffset;
  fifo_addr = base_addr + (channel << 14);

  wmax = wrec;
  wrec=0; // returned words

  nwords = 0;

  i = 0;
  while( (nwords <= 0) && (i <= max_retry) ) {
    if( max_retry > 0 ) i++;
    success = FIFO_GetNwords(channel, &nwords);
    if( success != BUS_OK ) return success;
  }

  size = (nwords < wmax) ? nwords : wmax;

  MPD_DBG("fifo ch = %d, words in fifo= %d, retries= %d (max %d)\n",channel, nwords,i, max_retry);

  if( i > max_retry ) {
    MPD_ERR(" max retry = %d, count=%d nword=%d\n", max_retry, i, nwords);
    return BUS_TBC;
  }

#ifdef BLOCK_TRANSFER
  success = BUS_BlockRead(fifo_addr, size, dbuf, &wrec); // trasfer 4 byte words!!
  MPD_DBG("Block Read fifo ch = %d, words requested = %d, returned = %d\n", channel, size, wrec);
#else
  for(i=0; i<size; i++) {
    success = BUS_Read(fifo_addr+(i*4), (dbuf+i));
    if( success != BUS_OK ) return success;
    wrec+=1;
  }
  MPD_DBG("Read apv = %d, wrec = %d, success = %d\n", channel, wrec, success);
#endif

  if( wrec != size ) {
    MPD_DBG("Count Mismatch: %d expected %d\n", wrec, size);
    return BUS_MISMATCH;
  }
  return success;
}

/*
 * For Zero Suppression Processor / Pedestal Subtraction
 *
 * channel : ADC Channel (correspond to a single APV)
 * blen    : buffer lenght available
 * event   : event buffer
 * nread   : the number of words read back
 *
 * return error or 0 on success
 *
 */

int MPDlo::FIFO_ReadSingle(int channel, int blen, uint32_t *event, int &nread)
{

  uint32_t base_addr, fifo_addr;
  int rval, nwords, size;
  int n_part;

  base_addr = ApvFifoOffset;
  fifo_addr = base_addr + (channel << 14);
  
  nread = 0;
  nwords = 0;
  n_part = 0;
  
  rval = FIFO_GetNwords(channel, &nwords);
  if( rval != BUS_OK ) return 2;
  
  size = (nwords > blen) ? blen : nwords; // cannot exceed memory buffer size

  if (size>0) {

#ifdef BLOCK_TRANSFER
    rval = BUS_BlockRead(fifo_addr, size, event, &n_part);
#else
    for(int i=0; i<size; i++) {
      rval = BUS_Read(fifo_addr+(i*4), (event+i));
      if( rval != BUS_OK ) { 
	return rval; 
      } 
      n_part++;
    }
#endif

#ifdef DEBUG_DUMP
    printf("\nReadData: %x %d\n",fifo_addr,channel);
    for (i=0;i<n_part;i++) {
      if ((i%16) == 0) printf("\n%4d",i);
      printf(" %6x", event[i]);
    }

    printf(" -> %d\n", n_part);
#endif
    nread += n_part;

    if( n_part != size ) return 1;

  }

  return nread;

}

int MPDlo::FIFO_Samples(int channel, uint32_t *event, int *nread, int max_samples, int *err)
{
  uint32_t base_addr, fifo_addr;
  int success, nwords;

  base_addr = ApvFifoOffset;
	
  *err = 1;
  *nread = 0;
  fifo_addr = base_addr + (channel << 14);
  nwords = DAQ_FIFO_SIZE;
  if( nwords > max_samples )
    {
      *err = 2;
      return BUS_GENERIC_ERROR;
    }
#ifdef BLOCK_TRANSFER
  success = BUS_BlockRead(fifo_addr, nwords, event, nread);
#else
  for(int i=0; i<nwords; i++)
    BUS_Read(fifo_addr+(i*4), (event+i));
  *nread = nwords*4;
#endif
  *nread /= 4;
  if( *nread == nwords )
    *err = 0;
  return success;
}

int MPDlo::FIFO_IsSynced(int channel, int *synced)
{
  uint32_t base_addr, error_addr;
  uint32_t data;
  uint32_t channel_mask, synced_mask;
  int success;

  base_addr = ApvFifoOffset;	
  error_addr = base_addr + SYNCED_ERROR_ADDR;
  channel_mask = 1 << (channel & 0x0F);
  synced_mask = channel_mask << 16;	/* @ error_addr */

  success = BUS_Read(error_addr, &data);
  if( data & synced_mask )
    *synced = 1;
  else
    *synced = 0;
  return success;
}

/*
 * Return the synch status of each FE, (one bit = one FE, 1 means synched) 
 */
int MPDlo::FIFO_AllSynced(int *synced)
{
  uint32_t base_addr, error_addr;
  uint32_t data;
  int success;

  base_addr = ApvFifoOffset;	
  error_addr = base_addr + SYNCED_ERROR_ADDR;

  success = BUS_Read(error_addr, &data);
  *synced = (data>>16);
  return success;
}

int MPDlo::FIFO_HasError(int channel, int *error)
{
  uint32_t base_addr, error_addr;
  uint32_t data;
  uint32_t channel_mask, error_mask;
  int success;

  base_addr = ApvFifoOffset;	
  error_addr = base_addr + SYNCED_ERROR_ADDR;
  channel_mask = 1 << (channel & 0x0F);
  error_mask = channel_mask;			/* @ error_addr */

  success = BUS_Read(error_addr, &data);
  if( data & error_mask )
    *error = 1;
  else
    *error = 0;
  return success;
}

int MPDlo::FIFO_GetAllFlags(uint16_t *full, uint16_t *empty)
{
  uint32_t base_addr, flag_addr;
  uint32_t data;
  int success;

  base_addr = ApvFifoOffset;	
  flag_addr = base_addr + FIFO_FLAG_ADDR;

  success = BUS_Read(flag_addr, &data);
  *full =  data >> 16;
  *empty = data & 0xFFFF;
  return success;
}

int MPDlo::FIFO_IsFull(int channel, int *full)
{
  uint32_t base_addr, flag_addr;
  uint32_t data;
  uint32_t channel_mask, full_mask;
  int success;
  
  base_addr = ApvFifoOffset;	
  flag_addr = base_addr + FIFO_FLAG_ADDR;
  channel_mask = 1 << (channel & 0x0F);
  full_mask = channel_mask << 16;		/* @ flag_addr */

  success = BUS_Read(flag_addr, &data);
  if( data & full_mask )
    *full = 1;
  else
    *full = 0;
  return success;
}

int MPDlo::FIFO_GetNwords(int channel, int *nwords) // can be optimized reading both consecutive channels
{
  uint32_t base_addr, nwords_addr;
  uint32_t data;
  uint32_t nwords_addr_offset[16] = {0,0,4,4,8,8,12,12,16,16,20,20,24,24,28,28};
  int success;
  
  base_addr = ApvFifoOffset;	
  nwords_addr = base_addr + USED_WORDS_ADDR + nwords_addr_offset[channel];

  success =BUS_Read(nwords_addr, &data);
  if( channel % 2 )	// if odd
    *nwords =  ((data & 0xFFFF0000) >> 16) & 0xFFFF;
  else
    *nwords =  data & 0xFFFF;
  return success;
}

int MPDlo::FIFO_IsEmpty(int channel, int *empty)
{
  uint32_t base_addr, flag_addr;
  uint32_t data;
  uint32_t channel_mask, empty_mask;
  int success;

  base_addr = ApvFifoOffset;	
  flag_addr = base_addr + FIFO_FLAG_ADDR;
  channel_mask = 1 << (channel & 0x0F);
  empty_mask = channel_mask;			/* @ flag_addr */

  success = BUS_Read(flag_addr, &data);
  if( data & empty_mask )
    *empty = 1;
  else
    *empty = 0;
  return success;
}

int MPDlo::FIFO_ClearAll(void)
{
  uint32_t addr;
  uint32_t data, oldval;
  int success;

// Issue a pulse on READOUT_CONFIG[31]
  addr = ApvFifoOffset;
  addr += READOUT_CONFIG_ADDR;
  if( (success = BUS_Read(addr, &oldval)) != BUS_OK )
    return success;
  data = oldval | 0x80000000;
  if( (success = BUS_Write(addr, &data)) != BUS_OK )
    return success;
  data = oldval & 0x7FFFFFFF;
  return BUS_Write(addr, &data);
}


int MPDlo::FIFO_WaitNotEmpty(int channel, int max_retry)
{
  uint32_t base_addr, flag_addr;
  uint32_t data;
  uint32_t channel_mask, empty_mask;
  int success, retry_count, fifo_empty;

  base_addr = ApvFifoOffset;	
  flag_addr = base_addr + FIFO_FLAG_ADDR;
  channel_mask = 1 << (channel & 0x0F);
  empty_mask = channel_mask;			/* @ flag_addr */

  retry_count = 0;
  fifo_empty = 1;
  success = BUS_OK;

  while( fifo_empty && success == BUS_OK && retry_count <= max_retry )
    {
      success = BUS_Read(flag_addr, &data);
      if( max_retry > 0 )
	retry_count++;
      if( data & empty_mask )
	fifo_empty = 1;
      else
	fifo_empty = 0;
      if( retry_count > max_retry )
	return -10;
    }
  return success;
}

/**
 * Standard event mode (no process)
 *
 * return true if all cards have been read
 */

bool MPDlo::FIFO_ReadAll(int &timeout, int &global_fifo_error) {

  unsigned int k;
  int sample_left;

  int nread, err;

  sample_left = 0;
  global_fifo_error = 0;

  if (fReadDone == false) { // at least one MPD FIFO needs to be read
    for(k=0; k<fApv.size(); k++) { // loop on ADC channels on single board

      if (ApvReadDone(k) == false) { // APV FIFO has data to be read

	nread = ApvGetBufferAvailable(k);
	if (nread>0) { // space in memory buffer
	  err = FIFO_ReadSingle(v2a(k), ApvGetBufferPWrite(k), nread, 100000); 

	  ApvIncBufferPointer(k,nread);

	  global_fifo_error |= err; // ???	  
	} else { // no space in memory buffer
	  MPD_ERR("No space in memory buffer for MPD slot=%d, APV i2c=%d adc=%d\n", GetSlot(), v2a(k), v2i(k));
	}

	if ((err == BUS_TIMEOUT) || (nread == 0)) timeout++; // timeout

	int n = ApvGetBufferSample(k);

	MPD_DBG("Fifo= %d, word rec= %d, event rec= %d, error=%d\n",v2a(k),nread ,n, global_fifo_error);

	sample_left += ApvGetSampleLeft(k);

      }

      MPD_DBG("Fifo= %d, total sample left= %d\n",k, sample_left);

    } // loop on ADC
    fReadDone = (sample_left>0) ? false : true;  
  } // if fReadDone
  
  return fReadDone;

}

// ***** Utility functions for sparse readout *****

/**
 *
 * Search sample End Marker in Data Buffer
 * b = buffer of the ADC
 * i0, i1: first and last elements of the buffer to be checked
 *
 * return the location of the end marker or -1 if not found
 */
int MPDlo::SearchEndMarker(uint32_t *b, int i0, int i1) {

  int i;

  if (i0>=i1) { return -1; }
  for (i=i0;i<i1;i++) {
    if ((b[i] & 0x180000) == 0x180000) {
      return i;
      break;
    } 
  }

  return -1;
}

/*
 * k = apv vector index
 * i0 = fBs
 */

void MPDlo::ApvShiftDataBuffer(int k, int i0) {

  uint32_t *b;

  b = ApvGetBufferPointer(k, 0);
  int delta = fApv[k].fBi1 - i0;

  MPD_DBG("Move block of %d words from %d to 0\n",delta,i0);

  if (delta>0) {
    memmove(&b[0],&b[i0],sizeof(uint32_t)*delta); // areas may overlap
  }

  fApv[k].fBi0 = 0; // to be removed
  fApv[k].fBi1 = delta;
  
  MPD_DBG("Fifo= %d cleaned (data shifted) write pointer at=%d\n",v2a(k),fApv[k].fBi1);

}

bool MPDlo::FIFO_ReadAllNew(int &timeout, int &global_fifo_error) {

  int n;
  unsigned int k;
  int sample_left;

  int nread, err;
  uint32_t multiple_events;

  int ii1=0;

  n=1; // single event mode

  sample_left = 0;

  if (fReadDone == false) { // MPD fifos need to be read
    for(k=0; k<fApv.size(); k++) { // loop on ADC channels on single board

      if (ApvReadDone(k) == false) { // APV FIFO has data to be read
  
	//      sample_left += ApvGetSampleLeft(k);

        //      ii0=fApv[k].fBi0;
        //      ii1=fApv[k].fBi1;
        //      idx = SearchEndMarker(ApvGetBufferPointer(k),ii0,ii1);
        //      fApv[k].fBi0 = ii1;
        //      cout << __FUNCTION__ << dec << " " << j << " " << k;

	uint32_t *bptr = ApvGetBufferPointer(k,ii1);
      
	int bsiz = fApv[k].fBufSize - ii1; // space left in buffer

	err = FIFO_ReadSingle(v2a(k), bsiz, bptr, nread);

	// ***
	if( nread > 0 ) {
	  for (int i=0; i<nread; i++) {
	    if (IsEndBlock(bptr[i])) { 
	      ApvDecSampleLeft(k,1);
	      if (ApvGetSampleLeft(k) == 0) {
		fApv[k].fBs = ii1 + i; // pointer to the last sample word
	      }
	    }
	  }

	  fApv[k].fBi1=ii1+nread;

	} else {
	  timeout++;
	}
	
	if( err != 2 )
	  global_fifo_error |= err;
      
	if (err == 2) timeout++;
      
	if( n > 1 ) multiple_events++; // not used 
	
      } else { // ...
#ifdef DEBUG_DUMP
	cout << __FUNCTION__ << ": xxxx " << endl;
#endif
      }
      
      sample_left += ApvGetSampleLeft(k);

    } // loop on ADC FIFOs
    fReadDone = (sample_left == 0) ? true : false;      
  } 
  
  return fReadDone;

}

/**
 *
 */

int MPDlo::DAQ_Enable(void)
{
  DAQ_Config();
  return TRIG_Enable();
}

int MPDlo::DAQ_Disable(void)
{
  uint32_t addr;
  uint32_t data;
  int success;

  addr = ApvFifoOffset;
	
  addr += READOUT_CONFIG_ADDR;
  data = 0;
  if( (success = BUS_Write(addr, &data)) != BUS_OK )
    return success;
  return TRIG_Disable();

}

int MPDlo::DAQ_Config() {

  uint32_t addr, data;
  int success;
  short evtbld;

  evtbld = GetEventBuilding() ? 1 : 0;

  addr = ApvFifoOffset;
	
  addr += READOUT_CONFIG_ADDR;
  data = (GetAcqMode() & 0x07) | // ((test & 0x01) << 15) |
    ((GetCommonOffset() & 0xfff) << 16) | 
    ((GetCommonNoiseSubtraction() & 0x1) << 28) | 
    ((evtbld & 0x1) << 30);
  if( (success = BUS_Write(addr, &data)) != BUS_OK )
    return success;

  return FIFO_ClearAll();

  for (unsigned int i=0;i<fApv.size();i++) {
    fApv[i].fBi0 = 0;
    fApv[i].fBi1 = 0;
  }

}


// ***** Pedestal and Thresholds handling routines

/**
 *
 */

int MPDlo::PED_Write(int ch, int *ped_even, int *ped_odd)	// even, odd is the apv channles, ch = 0..7
{
  uint32_t addr, data;
  int success, i;

  addr = ApvFifoOffset;
	
  addr += PEDESTAL_BASE_ADDR + (ch << 14);
  for(i=0; i<128; i++)
    {
      data = ped_even[i] | (ped_odd[i] << 16);
      if( (success = BUS_Write(addr, &data)) != BUS_OK )
	return success;
      addr += 4;
    }
  return success;
}

int MPDlo::PED_Write(int ch, int v)	// ch = 0..7
{
  int i, data[128];
  
  for(i=0; i<128; i++)
    data[i] = v;
  return PED_Write(ch, data, data);
}

int MPDlo::PED_Read(int ch, int *ped_even, int *ped_odd)	// TBD
{
  for(int i=0; i<128;  i++)
    ped_even[i] = ped_odd[i] = 0;	// TBD
  return BUS_OK;
}


int MPDlo::THR_Write(int ch, int *thr_even, int *thr_odd)	// ch = 0..7
{
  uint32_t addr, data;
  int success, i;

  addr = ApvFifoOffset;
	
  addr += THRESHOLD_BASE_ADDR + (ch << 14);
  for(i=0; i<128; i++)
    {
      data = thr_even[i] | (thr_odd[i] << 16);
      if( (success = BUS_Write(addr, &data)) != BUS_OK )
	return success;
      addr += 4;
    }
  return success;
}

int MPDlo::THR_Write(int ch, int v)	// ch = 0..7
{
  int i, data[128];
  
  for(i=0; i<128; i++)
    data[i] = v;
  return THR_Write(ch, data, data);
}

int MPDlo::THR_Read(int ch, int *thr_even, int *thr_odd)	// TBD
{
  for(int i=0; i<128;  i++)
    thr_even[i] = thr_odd[i] = 0;	// TBD
  return BUS_OK;
}

/**
 * Load pedestal and threshold data into the MPD
 */
int MPDlo::PEDTHR_Write() {

  for (int ia=0;ia<8;ia++) {// loop on apv,  odd and even apv are download simultaneously
    PED_Write(ia, GetApvPed(2*ia), GetApvPed(2*ia+1));
    THR_Write(ia, GetApvThr(2*ia), GetApvThr(2*ia+1));
  }

  return 0;

}



/**
 * Read Pedestals and Thresholds of a single MPD from the file pname
 * if pname is empty, use the stored PedThrPath value
 */
int MPDlo::ReadPedThr(std::string pname) {

  string line;

  int ch, ped, thr;
  int count,i;

  if (pname.empty()) {
    pname = GetPedThrPath();
  };

  for (i=0;i<2048;i++) {
    SetPedThr(i, GetPedCommon(), GetThrCommon());
  };

  std::ifstream infile(pname.data(), std::ifstream::in);
  if (infile.is_open()) {
    count=0;
    while (infile.good()) {
      getline(infile, line);
      if (line.find("#") != 0) { // no comment line
	std::istringstream iss(line);
	iss >> ch >> ped >> thr;
	count += SetPedThr(ch, ped, thr);
      }
    }    
    cout << __FUNCTION__ << ": " << count << " channels read from file " << pname << " and set" << endl;
    infile.close();
  } else {
    cout << __FUNCTION__ << ": Warning, unable to open file " << pname << endl;
    cout << __FUNCTION__ << ": Warning, set pedestal and threshold to common values" << endl; 
  }

  return count;

}

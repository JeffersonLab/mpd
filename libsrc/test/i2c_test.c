/*
 * File:
 *    i2c_test.c
 *
 * Description:
 *    Low level testing of i2c core.
 *
 *
 */


#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "jvme.h"
#include "mpdLib.h"

extern mpdParameters fMpd[(MPD_MAX_BOARDS) + 1];
extern int mpdPrintDebug;
extern volatile struct mpd_struct *MPDp[(MPD_MAX_BOARDS + 1)];
#define CHECKMPD(x) (((int32_t)MPDp[x]==-1) || (x<0) || (x>21))

void MPDtemp(uint32_t slot);
void cpuDelay(int ticks) { usleep(ticks/45);}
int  myI2C_SendByte(int id, uint8_t byteval, uint8_t command);
int  myI2C_ReceiveByte(int id, uint8_t *byteval, uint8_t command);
int  myAPV_Write(int id, uint8_t apv_addr, uint8_t reg_addr, uint8_t val);
int  myAPV_Read(int id, uint8_t apv_addr, uint8_t reg_addr, uint8_t *val);
int  myI2C_ByteWrite(int id, uint8_t dev_addr, uint8_t reg_addr, int ndata, uint8_t *data);
int  myI2C_ByteRead(int id, uint8_t dev_addr, uint8_t reg_addr, int ndata, uint8_t *data);
int  myAPV_Try(int id, uint8_t apv_addr);
int  myAPV_Scan(int id);
int  myI2C_SendAck(int id);
int  myI2C_HdmiEnable(int id, int upper);
int  myI2C_HdmiDisable(int id);

int
main(int argc, char *argv[])
{
  int stat, slot;

  if (argc > 1)
    {
      slot = atoi(argv[1]);

      if ((slot < 0) || (slot > 22))
	{
	  printf("invalid slot... using 21");
	  slot = 21;
	}
    }
  else
    slot = 21;

  printf("\n %s: slot = %d\n", argv[0], slot);
  printf("----------------------------\n");

  stat = vmeOpenDefaultWindows();
  if(stat != OK)
    goto CLOSE;

  vmeBusLock();

  if(mpdInit((slot << 19), 0, 1, MPD_INIT_NO_CONFIG_FILE_CHECK) <= 0)
    {
      printf("%s: Init error \n",
	     __func__);
      goto CLOSE;
    }

  slot = mpdSlot(0);

  myI2C_HdmiDisable(slot);
  mpdSetI2CSpeed(slot, 950);

  mpdI2C_Init(slot);

  mpdSetPrintDebug(0);
  MPDtemp(slot);
  /* mpdSetPrintDebug(0xff); */

  myAPV_Scan(slot);
  MPDtemp(slot);

 CLOSE:
  vmeBusUnlock();

  vmeCloseDefaultWindows();

  exit(0);
}

int
myAPV_TryHdmi(int id, uint8_t apv_addr, int ihdmi)
{

  int timeout = 0;
  uint8_t x = apv_addr + 32, y = 0;
  int ret = ERROR;
  static const uint8_t latency_addr = 0x04;
  uint8_t reg = latency_addr;

  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }


  myI2C_HdmiEnable(id, ihdmi);

  while ((ret != OK) && (timeout < 10))
    {
      ret = myAPV_Write(id, apv_addr, reg, x);
      timeout++;
    }

  MPD_DBGN(MPD_DEBUG_APVINIT,
	   "WRITE  i2c_addr 0x%02x  dev_addr = 0x%02x  val = 0x%02x  to = %d  ret %d\n",
	   apv_addr, reg, x, timeout, ret);

   if(ret != OK)
    return ret;

  /* Unmodified MPDs will stop having reliable i2c if READ commands
     are used to the APVs */
  if(mpdGetFpgaCompileTime(id) < 0x59b7d9e6)
    return ret;

  timeout = -1; ret = ERROR;

  while ((ret != OK) && (timeout < 10))
    {
      ret = myAPV_Read(id, apv_addr, (reg + 1), &y);
      timeout++;
    }


  MPD_DBGN(MPD_DEBUG_APVINIT,
	   "READ  i2c_addr 0x%02x  dev_addr = 0x%02x  val = 0x%02x  to = %d  ret %d\n",
	   apv_addr, reg, y, timeout, ret);

  if(ret != OK)
    return ret;

  if(x != y)
    {
      MPD_DBGN(MPD_DEBUG_APVINIT,
	   "Slot %2d   i2c 0x%02x  W != R (0x%x != 0x%x)\n", id, apv_addr, x, y);
      return ERROR;
    }

  return ret;
}

int
myAPV_Scan(int id)
{
  int ihdmi = 0, iapv = 0, ispeed = 0;;

  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  mpdI2C_Init(id);

  for (iapv = 0; iapv < 0x3F; iapv++)
    {
      for (ihdmi = 0; ihdmi < 2; ihdmi++)
	{
	  if (myAPV_TryHdmi(id, iapv, ihdmi) == OK)
	    {
	      if(fMpd[id].CtrlHdmiInitMask & (1ULL<<iapv))
		{
		  MPD_ERR("MPD %d: Multiple APV cards with i2c addr 0x%02x (separate HDMI control)\n", id, iapv);
		  break;
		}
	      MPD_MSG("MPD %d HDMI %d found candidate card at i2c addr 0x%02x 0x%04x\n",
		     id, ihdmi, iapv, vmeRead32(&MPDp[id]->readout_config));
	      fMpd[id].CtrlHdmiMask[ihdmi] |= (1ULL<<iapv);
	      fMpd[id].CtrlHdmiInitMask  |= (1ULL<<iapv);
	      break;
	    }
	}
    }

  MPD_DBGN(MPD_DEBUG_APVINIT,
	   "HdmiMask[0]      = 0x%016llx\n", fMpd[id].CtrlHdmiMask[0]);
  MPD_DBGN(MPD_DEBUG_APVINIT,
	   "HdmiMask[1]      = 0x%016llx\n", fMpd[id].CtrlHdmiMask[1]);
  MPD_DBGN(MPD_DEBUG_APVINIT,
	   "CtrlHdmiInitMask = 0x%016llx\n", fMpd[id].CtrlHdmiInitMask);

  return OK;
}

// MPD temp readout test
void
MPDtemp(uint32_t slot)
{
  uint8_t devaddr, devreg, rdata;
  int timeout, ret;

  devaddr = 0x4C;
  devreg  = 0x0;
  rdata = 0;

  timeout = 0; ret = ERROR;
  while ((ret != OK) && (timeout < 10))
    {
      ret = myAPV_Read(slot, devaddr, devreg, &rdata);
      timeout++;
    }

  printf("\n Temperature (signed local): 0x%x (%d)  to = %d  ret = %d\n",
	 rdata,
	 rdata, timeout, ret);

  devaddr = 0x4C;
  devreg  = 0x01;
  rdata = 0;

  timeout = 0; ret = ERROR;
  while ((ret != OK) && (timeout < 10))
    {
      ret =   myAPV_Read(slot, devaddr, devreg, &rdata);
      timeout++;
    }

  printf("\n Temperature (signed remotes): 0x%x (%d) \n",
	 rdata,
	 rdata);

  devaddr = 0x4C;
  devreg  = 0x31;
  rdata = 0;

  timeout = 0; ret = ERROR;
  while ((ret != OK) && (timeout < 10))
    {
      ret =   myAPV_Read(slot, devaddr, devreg, &rdata);
      timeout++;
    }

  printf("\n Temperature (unsigned remote): 0x%x (%d) \n",
	 rdata,
	 rdata);

  devaddr = 0x4C;
  devreg  = 0xFF;
  rdata = 0;

  timeout = 0; ret = ERROR;
  while ((ret != OK) && (timeout < 10))
    {
      ret =   myAPV_Read(slot, devaddr, devreg, &rdata);
      timeout++;
    }

  printf("\n Device Revision Code: 0x%x (%d) \n\n",
	 rdata,
	 rdata);

}


int
myAPV_Write(int id, uint8_t apv_addr, uint8_t reg_addr, uint8_t val)
{
  int success;

  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  success = myI2C_ByteWrite(id, apv_addr, reg_addr, 1, &val);

  MPD_DBG("Slot %d  apv_addr = 0x%x  reg_addr = 0x%x  val = 0x%x success = %d\n",
	  id, apv_addr, reg_addr, val, success);

  return success;
}

int
myAPV_Read(int id, uint8_t apv_addr, uint8_t reg_addr, uint8_t * val)
{
  int success;

  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  usleep(500);

  success =
    myI2C_ByteRead(id, (uint8_t) (apv_addr), reg_addr, 1, val);

  MPD_DBG("Slot %d  apv_addr = 0x%x  reg_addr = 0x%x  val = 0x%x  success = %d\n",
	  id, apv_addr, reg_addr, *val, success);

  return success;
}


int
myI2C_ByteWrite(int id, uint8_t dev_addr, uint8_t reg_addr,
		int ndata, uint8_t *data)
{
  uint8_t command = 0;
  int idata, rval = OK;

  command = MPD_I2C_COMMSTAT_STA | MPD_I2C_COMMSTAT_WR;
  rval = myI2C_SendByte(id, (dev_addr << 1), command);
  if (rval != OK)
    {
      myI2C_SendAck(id);
      return rval;
    }

  command = MPD_I2C_COMMSTAT_WR;
  rval = myI2C_SendByte(id, reg_addr, command);
  if (rval != OK)
    {
      myI2C_SendAck(id);
      return rval;
    }

  command = MPD_I2C_COMMSTAT_WR;

  for(idata = 0; idata < ndata; idata++)
    {
      if((idata + 1) == ndata) /* Stop for last data in array */
	command |= MPD_I2C_COMMSTAT_STO;

      rval = myI2C_SendByte(id, data[idata], command);
      if (rval != OK)
	{
	  myI2C_SendAck(id);
	  return rval;
	}
    }

  return OK;
}

int
myI2C_ByteRead(int id, uint8_t dev_addr, uint8_t reg_addr,
	       int ndata, uint8_t *data)
{
  uint8_t command = 0, rdata = 0;
  int idata, rval = OK;

  command = MPD_I2C_COMMSTAT_STA | MPD_I2C_COMMSTAT_WR;
  rval = myI2C_SendByte(id, (dev_addr << 1), command);
  if (rval != OK)
    {
      myI2C_SendAck(id);
      return ERROR;
    }

  command = MPD_I2C_COMMSTAT_STO | MPD_I2C_COMMSTAT_WR;

  rval = myI2C_SendByte(id, reg_addr, command);
  if (rval != OK)
    {
      myI2C_SendAck(id);
      return ERROR;
    }

  command = MPD_I2C_COMMSTAT_STA | MPD_I2C_COMMSTAT_WR;
  rval = myI2C_SendByte(id, (dev_addr << 1) | 1, command);
  if (rval != OK)
    {
      myI2C_SendAck(id);
      return ERROR;
    }

  command = MPD_I2C_COMMSTAT_RD;
  /* command |=  MPD_I2C_COMMSTAT_ACK | MPD_I2C_COMMSTAT_STO; */

  for (idata = 0; idata < ndata; idata++)
    {
      if((idata + 1) == ndata) /* Stop for last data in array */
	command |=  MPD_I2C_COMMSTAT_ACK | MPD_I2C_COMMSTAT_STO; // FIXME: Not sure about the ACK.  (Need for each read, or just the end?) // FIXME: still and issue

      rval = myI2C_ReceiveByte(id, &rdata, command);
      if (rval != OK)
	{
	  myI2C_SendAck(id);
	  return ERROR;
	}

      data[idata] = rdata;
    }

  return OK;
}

int
myI2C_SendByte(int id, uint8_t byteval, uint8_t command)
{
  uint32_t rdata = 0;
  int retry_count = 0, rval = OK;

  vmeWrite32(&MPDp[id]->i2c.tx_rx, byteval);

  vmeWrite32(&MPDp[id]->i2c.comm_stat, command);
  rdata = vmeRead32(&MPDp[id]->i2c.comm_stat);
  while( (rdata & 0x00000002) != 0 && retry_count < 10 )
    {
      usleep(1);
      rdata = vmeRead32(&MPDp[id]->i2c.comm_stat);
      if(retry_count > 0) printf("%2d: 0x%08x\n", retry_count, rdata);
      retry_count++;
    }

  if( retry_count >= 10 )
    rval = ERROR;

  if( rdata & MPD_I2C_COMMSTAT_NACK_RECV )	/* NACK received */
    {
      //printf("%s: NACK received 0x%x\n",
      //      	     __func__, rdata);
      rval = ERROR;
    }

  return rval;
}


int
myI2C_ReceiveByte(int id, uint8_t *byteval, uint8_t command)
{
  uint32_t rdata = 0;
  int retry_count = 0, rval = OK;

  vmeWrite32(&MPDp[id]->i2c.comm_stat, command);

  rdata = vmeRead32(&MPDp[id]->i2c.comm_stat);
  while( (rdata & 0x00000002) != 0 && retry_count < 10 )
    {
      usleep(1);
      rdata = vmeRead32(&MPDp[id]->i2c.comm_stat);
      if(retry_count > 0) printf("%2d: 0x%08x\n", retry_count, rdata);
      retry_count++;
    }

  if (retry_count >= 10)
    rval = ERROR;

  *byteval = vmeRead32(&MPDp[id]->i2c.tx_rx);

  return rval;
}

int
myI2C_SendAck(int id)
{
  /* return 0; */
  return myI2C_SendByte(id, 0, MPD_I2C_COMMSTAT_ACK);
}

int
myI2C_HdmiEnable(int id, int upper)
{
  uint32_t data;
  int rval = OK;

  data = vmeRead32(&MPDp[id]->readout_config);

  if( upper == 0 )
    {
      MPD_DBG("Enabling I2C on lower HDMI cable only\n");
      data &= ~0x00000400;	// clear bit 10
      data |= 0x00000200;		// set bit 9
    }
  else
    {
      MPD_DBG("Enabling I2C on upper HDMI cable only\n");
      data &= ~0x00000200;	// clear bit 9
      data |= 0x00000400;		// set bit 10
    }

  vmeWrite32(&MPDp[id]->readout_config, data);

  usleep(10);

  return rval;
}

int
myI2C_HdmiDisable(int id)
{
  uint32_t data;
  int rval = OK;

  data = vmeRead32(&MPDp[id]->readout_config);

  MPD_DBG("Disabling I2C on BOTH HDMI cables\n");
  data &= ~0x00000400;	// clear bit 10
  data &= ~0x00000200;	// clear bit 9

  vmeWrite32(&MPDp[id]->readout_config, data);

  usleep(10);

  return rval;
}

/*
  Local Variables:
  compile-command: "make -k -B i2c_test"
  End:
 */

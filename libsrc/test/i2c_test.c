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

extern volatile struct mpd_struct *MPDp[(MPD_MAX_BOARDS + 1)];

void MPDtemp(uint32_t slot);
void cpuDelay(int ticks) { usleep(ticks/45);}
void myI2C_SendByte(int id, uint8_t byteval, uint8_t command);
void myI2C_ReceiveByte(int id, uint8_t *byteval, uint8_t command);

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

  MPDtemp(slot);
 CLOSE:
  vmeBusUnlock();

  vmeCloseDefaultWindows();

  exit(0);
}

// MPD temp readout test
void
MPDtemp(uint32_t slot)
{
  uint8_t devaddr, devreg, command, rdata;

  if(mpdInit((slot << 19), 0, 1, MPD_INIT_NO_CONFIG_FILE_CHECK) <= 0)
    {
      printf("%s: Init error \n",
	     __func__);
      return;
    }

  slot = mpdSlot(0);

  mpdSetI2CSpeed(slot, 0x33f);

  mpdI2C_Init(slot);

  devaddr = 0x4C;
  devreg  = 0x0;
  command = 0;
  rdata = 0;

  command = MPD_I2C_COMMSTAT_STA | MPD_I2C_COMMSTAT_WR;
  myI2C_SendByte(slot, (devaddr << 1), command);

  command = MPD_I2C_COMMSTAT_WR;
  myI2C_SendByte(slot, devreg, command);

  command = MPD_I2C_COMMSTAT_STA | MPD_I2C_COMMSTAT_WR;
  myI2C_SendByte(slot, (devaddr << 1) | 1, command);

  command = MPD_I2C_COMMSTAT_STO | MPD_I2C_COMMSTAT_RD | MPD_I2C_COMMSTAT_ACK;
  myI2C_ReceiveByte(slot, &rdata, command);

  printf("\n Temperature (signed local): 0x%x (%d) \n",
	 rdata,
	 rdata);

  devaddr = 0x4C;
  devreg  = 0x01;
  command = 0;
  rdata = 0;

  command = MPD_I2C_COMMSTAT_STA | MPD_I2C_COMMSTAT_WR;
  myI2C_SendByte(slot, (devaddr << 1), command);

  command = MPD_I2C_COMMSTAT_WR;
  myI2C_SendByte(slot, devreg, command);

  command = MPD_I2C_COMMSTAT_STA | MPD_I2C_COMMSTAT_WR;
  myI2C_SendByte(slot, (devaddr << 1) | 1, command);

  command = MPD_I2C_COMMSTAT_STO | MPD_I2C_COMMSTAT_RD | MPD_I2C_COMMSTAT_ACK;
  myI2C_ReceiveByte(slot, &rdata, command);

  printf("\n Temperature (signed remotes): 0x%x (%d) \n",
	 rdata,
	 rdata);

  devaddr = 0x4C;
  devreg  = 0x31;
  command = 0;
  rdata = 0;

  command = MPD_I2C_COMMSTAT_STA | MPD_I2C_COMMSTAT_WR;
  myI2C_SendByte(slot, (devaddr << 1), command);

  command = MPD_I2C_COMMSTAT_WR;
  myI2C_SendByte(slot, devreg, command);

  command = MPD_I2C_COMMSTAT_STA | MPD_I2C_COMMSTAT_WR;
  myI2C_SendByte(slot, (devaddr << 1) | 1, command);

  command = MPD_I2C_COMMSTAT_STO | MPD_I2C_COMMSTAT_RD | MPD_I2C_COMMSTAT_ACK;
  myI2C_ReceiveByte(slot, &rdata, command);

  printf("\n Temperature (unsigned remote): 0x%x (%d) \n",
	 rdata,
	 rdata);

  devaddr = 0x4C;
  devreg  = 0xFF;
  command = 0;
  rdata = 0;

  command = MPD_I2C_COMMSTAT_STA | MPD_I2C_COMMSTAT_WR;
  myI2C_SendByte(slot, (devaddr << 1), command);

  command = MPD_I2C_COMMSTAT_WR;
  myI2C_SendByte(slot, devreg, command);

  command = MPD_I2C_COMMSTAT_STA | MPD_I2C_COMMSTAT_WR;
  myI2C_SendByte(slot, (devaddr << 1) | 1, command);

  command = MPD_I2C_COMMSTAT_STO | MPD_I2C_COMMSTAT_RD | MPD_I2C_COMMSTAT_ACK;
  myI2C_ReceiveByte(slot, &rdata, command);

  printf("\n Device Revision Code: 0x%x (%d) \n\n",
	 rdata,
	 rdata);

}

void
myI2C_SendByte(int id, uint8_t byteval, uint8_t command)
{
  uint32_t rdata = 0;
  int retry_count = 0;

  vmeWrite32(&MPDp[id]->i2c.tx_rx, byteval);

  vmeWrite32(&MPDp[id]->i2c.comm_stat, command);
  rdata = vmeRead32(&MPDp[id]->i2c.comm_stat);
  while( (rdata & 0x00000002) != 0 && retry_count < 10 )
    {
      usleep(10);
      rdata = vmeRead32(&MPDp[id]->i2c.comm_stat);
      if(retry_count > 0) printf("%2d: 0x%08x\n", retry_count, rdata);
      retry_count++;
    }

}


void
myI2C_ReceiveByte(int id, uint8_t *byteval, uint8_t command)
{
  uint32_t rdata = 0;
  int retry_count = 0;

  vmeWrite32(&MPDp[id]->i2c.comm_stat, command);
  rdata = vmeRead32(&MPDp[id]->i2c.comm_stat);
  while( (rdata & 0x00000002) != 0 && retry_count < 10 )
    {
      usleep(10);
      rdata = vmeRead32(&MPDp[id]->i2c.comm_stat);
      if(retry_count > 0) printf("%2d: 0x%08x\n", retry_count, rdata);
      retry_count++;
    }

  *byteval = vmeRead32(&MPDp[id]->i2c.tx_rx);
}


/*
  Local Variables:
  compile-command: "make -k -B i2c_test"
  End:
 */

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
#include "mpdConfig.h"

extern int mpdPrintDebug;
/* extern volatile struct mpd_struct *MPDp[(MPD_MAX_BOARDS + 1)]; */
/* #define CHECKMPD(x) (((int32_t)MPDp[x]==-1) || (x<0) || (x>21)) */

void MPDtemp(uint32_t slot);
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

  mpdConfigInit("config_apv.txt");
  mpdConfigLoad();

  if(mpdInit((slot << 19), (1<<19), 2, 0) <= 0)
    {
      printf("%s: Init error \n",
	     __func__);
      goto CLOSE;
    }

  slot = mpdSlot(0);

  mpdI2C_Init(slot);

  mpdAPV_Scan(slot);
  mpdSetPrintDebug(0);

  mpdDELAY25_GStatus();
  mpdDELAY25_Set(slot, 0x12, 0x31);
  mpdDELAY25_Set(slot+1, 0x22, 0x33);
  mpdDELAY25_GStatus();

  /* apv reset */
  printf("Do APV reset on MPD slot %d\n", slot);
  if (mpdI2C_ApvReset(slot) != OK)
    {
      printf("ERR: apv reset failed on mpd %d\n",slot);
    }



  /* // apv configuration */
  printf("Configure single APV on MPD slot %d\n",slot);
  int iapv;

  for (iapv=0; iapv < mpdGetNumberAPV(slot); iapv++)
    {
      if (mpdAPV_Config(slot ,iapv) != OK)
  	{
  	  printf("ERR: config apv card %d failed in mpd %d\n", iapv, slot);
  	}
    }

  mpdApvStatus(slot, 0x0a000000);

  double core_t = 0., air_t = 0.;
  mpdLM95235_Read(slot, &core_t, &air_t);

  MPD_MSG("Slot %2d: Board temperatures: core: %.2f degC air: %.2f degC\n",
	  slot, core_t, air_t);


  /* mpdGStatus(0); */

 CLOSE:
  vmeBusUnlock();

  vmeCloseDefaultWindows();

  exit(0);
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
      ret = mpdI2C_ByteRead(slot, devaddr, devreg, 1, &rdata);
      timeout++;
    }

  MPD_MSG("\n Temperature (signed local): 0x%x (%d)  to = %d  ret = %d\n",
	 rdata,
	 rdata, timeout, ret);

  devaddr = 0x4C;
  devreg  = 0x01;
  rdata = 0;

  timeout = 0; ret = ERROR;
  while ((ret != OK) && (timeout < 10))
    {
      ret = mpdI2C_ByteRead(slot, devaddr, devreg, 1, &rdata);
      timeout++;
    }

  MPD_MSG("\n Temperature (signed remotes): 0x%x (%d) \n",
	 rdata,
	 rdata);

  devaddr = 0x4C;
  devreg  = 0x31;
  rdata = 0;

  timeout = 0; ret = ERROR;
  while ((ret != OK) && (timeout < 10))
    {
      ret = mpdI2C_ByteRead(slot, devaddr, devreg, 1, &rdata);
      timeout++;
    }

  MPD_MSG("\n Temperature (unsigned remote): 0x%x (%d) \n",
	 rdata,
	 rdata);

  devaddr = 0x4C;
  devreg  = 0xFF;
  rdata = 0;

  timeout = 0; ret = ERROR;
  while ((ret != OK) && (timeout < 10))
    {
      ret = mpdI2C_ByteRead(slot, devaddr, devreg, 1, &rdata);
      timeout++;
    }

  MPD_MSG("\n Device Revision Code: 0x%x (%d) \n\n",
	 rdata,
	 rdata);

}


/*
  Local Variables:
  compile-command: "make -k -B i2c_test"
  End:
 */

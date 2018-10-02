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

void MPDtemp(uint32_t slot);

int
Download(int slot)
{
  mpdConfigInit("config_apv.txt");
  mpdConfigLoad();

    // discover MPDs and initialize memory mapping
  mpdInit(slot<<19, 0, 1, MPD_INIT_NO_CONFIG_FILE_CHECK);
  int fnMPD = mpdGetNumberMPD();

  if (fnMPD <= 0)
    {				// test all possible vme slot ?
      printf("ERR: no MPD discovered, cannot continue\n");
      return ERROR;
    }

  printf(" MPD discovered = %d\n", fnMPD);

  // APV configuration on all active MPDs
  int impd, iapv, id, rval;
  for (impd = 0; impd < fnMPD; impd++)
    {				// only active mpd set
      id = mpdSlot(impd);

      rval = mpdHISTO_MemTest(id);

      printf(" Try initialize I2C mpd in slot %d\n", id);
      if (mpdI2C_Init(id) != OK)
	{
	  printf("WRN: I2C fails on MPD %d\n", id);
	}
      if (mpdI2C_ApvReset(id) != OK)
	{
	  printf("WRN: I2C ApvReset fails on MPD %d\n", id);
	}
      printf("Try APV discovery and init on MPD slot %d\n", id);
      if (mpdAPV_Scan(id) <= 0)
	{			// no apd found, skip next
	  continue;
	}

      // board configuration (APV-ADC clocks phase)
      printf("Do DELAY setting on MPD slot %d\n", id);
      mpdDELAY25_Set(id, mpdGetAdcClockPhase(id, 0), mpdGetAdcClockPhase(id, 1));

      // apv reset----this check will never fail...see "mpdI2C_ApvReset()"
      printf("Do APV reset on MPD slot %d\n", id);
      if (mpdI2C_ApvReset(id) != OK)
	{
	  printf("ERR: apv resert faild on mpd %d\n", id);
	}

      // apv configuration
      printf("Configure single APV on MPD slot %d\n", id);
      for (iapv = 0; iapv < mpdGetNumberAPV(id); iapv++)
	{
	  if (mpdAPV_Config(id, iapv) != OK)
	    {
	      printf("ERR: config apv card %d failed in mpd %d\n", iapv, id);
	    }
	}

      // configure adc on MPD
      printf("Configure ADC on MPD slot %d\n", id);
      mpdADS5281_Config(id);

      // configure fir
      // not implemented yet

      // 101 reset on the APV
      printf("Do 101 Reset on MPD slot %d\n", id);
      mpdAPV_Reset101(id);

      // <- MPD+APV initialization ends here

    }				// end loop on mpds
  //END of MPD configure

  // summary report
  printf("Configured APVs (ADC 15 ... 0)\n");
  int ibit;
  for (impd = 0; impd < fnMPD; impd++)
    {
      id = mpdSlot(impd);

      if (mpdGetApvEnableMask(id) != 0)
	{
	  printf("MPD %2d : ", id);
	  iapv = 0;
	  for (ibit = 15; ibit >= 0; ibit--)
	    {
	      if (mpdGetApvEnableMask(id) & (1 << ibit))
		{
		  printf("1");
		  iapv++;
		}
	      else
		{
		  printf(".");
		}
	    }
	  printf(" (#APV %d)\n", iapv);
	}
    }

  return OK;
}


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

  Download(slot);



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
  compile-command: "make -k -B rol_test"
  End:
 */

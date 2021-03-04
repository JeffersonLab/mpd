/*
 * File:
 *    mpdI2CScan.c
 *
 * Description:
 *    Low level scan of i2c address for apvs
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

void configure_default_apv(int id, int napv);
int i2c_scan(int id);

int
main(int argc, char *argv[])
{
  int stat, slot;
  extern int nmpd;

  if (argc > 1)
    {
      slot = atoi(argv[1]);

      if ((slot < 0) || (slot > 32))
	{
	  printf("invalid slot... using 2");
	  slot = 2;
	}
    }
  else
    slot = 2;

  printf("\n %s: slot = %d\n", argv[0], slot);
  printf("----------------------------\n");

  stat = vmeOpenDefaultWindows();
  if(stat != OK)
    goto CLOSE;

  vmeCheckMutexHealth(1);
  vmeBusLock();

  int32_t iapv, nApv = 32;

  configure_default_apv(slot, nApv);

  if(mpdInit((slot << 19), (1<<19), 1, MPD_INIT_NO_CONFIG_FILE_CHECK) < 0)
    {
      if(nmpd < 1)
	{
	  printf("%s: Init error \n",
		 __func__);
	  goto CLOSE;
	}
    }

  slot = mpdSlot(0);
  printf("MPD slot %2d config:\n", slot);


  printf(" - Initialize I2C\n");

#ifdef DEBUGI2C
  fprintf(fDebugMode, "I2C INIT\n");
#endif
  if (mpdI2C_Init(slot) != OK)
    {
      printf(" * * FAILED\n");
    }

  printf(" - APV discovery and init\n");
#ifdef DEBUGI2C
  fprintf(fDebugMode, "I2C SCAN\n");
#endif

  if (i2c_scan(slot) <= 0)
    {			// no apv found, skip next
      printf(" * * FAILED\n");
    }
  mpdSetPrintDebug(0);

  /* apv reset */
  printf("Do APV reset on MPD slot %d\n", slot);
  if (mpdI2C_ApvReset(slot) != OK)
    {
      printf("ERR: apv reset failed on mpd %d\n",slot);
    }

  usleep(10);

  /* // apv configuration */
  printf(" - Configure Individual APVs\n");
  printf(" - - ");

  unsigned int apvConfigMask = 0;
  int itry, badTry = 0;
  for (itry = 0; itry < 3; itry++)
    {

      if(badTry)
	{
	  printf(" ******** RETRY ********\n");
	  printf(" - - ");
	  fflush(stdout);
	}
      badTry = 0;
      for (iapv = 0; iapv < mpdGetNumberConfiguredAPV(slot); iapv++)
	{
	  apvConfigMask |= (1 << mpdApvGetAdc(slot, iapv));

	  printf("%2d ", iapv);
	  fflush(stdout);

#ifdef DEBUGI2C
	  fprintf(fDebugMode, "APV CONFIG %d\n", iapv);
#endif
	  if (mpdAPV_Config(slot, iapv) != OK)
	    {
	      printf(" * * FAILED for APV %2d\n", iapv);
	      if(iapv < (mpdGetNumberAPV(slot) - 1))
		printf(" - - ");
	      fflush(stdout);
	      badTry = 1;
	    }
	  mpdSetPrintDebug(0);
	}
      printf("\n");
      fflush(stdout);

      if(badTry)
	{
	  printf(" ***** APV RESET *****\n");
	  fflush(stdout);
	  mpdI2C_ApvReset(slot);
	}
      else
	{
	  if(itry > 0)
	    {
	      printf(" ****** SUCCESS!!!! ******\n");
	      fflush(stdout);
	    }
	  break;
	}
    }

  // summary report
  printf("\n");


  int ibit = 0;
  printf("Found and Configured APVs: \n");

  printf("  MPD %2d : ", slot);
  iapv = 0;
  for (ibit = 15; ibit >= 0; ibit--)
    {
      if (((ibit + 1) % 4) == 0)
	printf(" ");
      if (mpdGetApvEnableMask(slot) & (1 << ibit))
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
  printf("\n");

  mpdApvStatus(slot, mpdGetApvEnableMask(slot));

  /* mpdInit(..) disables fiber mode, re-enable it for incoming fiber connections */
  mpdFiberEnable(slot);
  printf(" --- Fiber Mode enabled ---\n");

 CLOSE:
  vmeBusUnlock();

  vmeCloseDefaultWindows();

  exit(0);
}

void
configure_default_apv(int id, int napv)
{
  int iapv;

  mpdSetNumberAPV(id, napv);
  int speed = 500; // (x10^-5 s) period of the i2c clock --- 1124
  int timeout = 200; // number of retries before timeout

  mpdSetI2CSpeed(id, speed);
  mpdSetI2CMaxRetry(id, timeout);

  ApvParameters ap;

  ap.Ipre = 85;
  ap.Ipcasc = 45;
  ap.Ipsf = 30;
  ap.Isha = 30;
  ap.Issf = 30;
  ap.Ipsp = 48;
  ap.Imuxin = 30;
  ap.Ispare = 0;
  ap.Vfp = 30;
  ap.Vfs = 60;
  ap.Vpsp = 30;
  ap.Ical = 120;
  ap.Cdrv = 0xEF;
  ap.Csel = 0;

  ap.Latency = 22;
  ap.Muxgain = 4;

  uint8_t CalPulse = 0;
  int apv_freq  = 1; // Readout Frequency  : 0=20 MHz, 1=40 MHz
  uint8_t apv_smode = 0; // Sampling Mode      : 0=3 samples, 1=1 sample / trigger
  uint8_t Polarization = 0;
  uint8_t ReadOutMode  = 1;
  uint8_t AnalogBias   = 1;
  uint8_t mode = (Polarization << 5) |
    (apv_freq << 4) |
    (ReadOutMode << 3) |
    ((1 - CalPulse) << 2) |
    (apv_smode << 1) |
    AnalogBias;

  ap.Mode = mode;

  for(iapv=0; iapv<napv; iapv++)
    {
      ap.i2c = iapv;
      ap.adc = iapv;
      mpdAddApv(id, ap);
    }

}

int
i2c_scan(int id)
{
  int iapv = 0, iapv_addr = 0, ihdmi = 0;
  int nFound = 0;
  unsigned int configApvMask = 0;

  extern ApvParameters fApv[(MPD_MAX_BOARDS) + 1][MPD_MAX_APV];
  extern mpdParameters fMpd[(MPD_MAX_BOARDS) + 1];
  extern uint32_t mpdRead32(volatile uint32_t * reg);
  extern void mpdWrite32(volatile uint32_t * reg, uint32_t val);
  extern int mpdPrintDebug;
  extern volatile struct mpd_struct *MPDp[(MPD_MAX_BOARDS + 1)];
  extern int nApv[(MPD_MAX_BOARDS) + 1];
  extern unsigned short fApvEnableMask[(MPD_MAX_BOARDS) + 1];
  extern int I2C_CtrlHdmiEnable(int id, int upper);
  extern int mpdAPV_TryNoHdmi(int id, uint8_t apv_addr);

#define CHECKMPD(x) ((MPDp[x]==NULL) || ((long)MPDp[x]==-1) || (x<0) || (x>21))

  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  // !!! Blindscan used to determine which HDMI output to use
  MPD_DBGN(MPD_DEBUG_APVINIT,
	   "MPD %2d Blind scan on %d apvs: \n", id, MPD_MAX_APV);

  /* Construct the i2c mask of APVs in config file */
  for (iapv = 0; iapv < fMpd[id].nAPV; iapv++)
    {
      if(fApv[id][iapv].adc > -1)
	configApvMask |= (1 << fApv[id][iapv].i2c);
    }

  MPD_DBGN(MPD_DEBUG_APVINIT,
	   "MPD %2d: %2d APV found config files (i2c adr mask = 0x%08x)\n",
	   id, fMpd[id].nAPV, configApvMask);


  /* Turn on both i2c repeaters */
  I2C_CtrlHdmiEnable(id, 2);

  for (iapv_addr = 0; iapv_addr < MPD_MAX_APV; iapv_addr++)
    {
      if (mpdAPV_TryNoHdmi(id, iapv_addr) == OK)
	{
	  MPD_DBGN(MPD_DEBUG_APVINIT,
		   "MPD %2d HDMI %d found candidate card at i2c addr 0x%02x 0x%04x\n",
		   id, ihdmi, iapv_addr, mpdRead32(&MPDp[id]->readout_config));

	  fMpd[id].CtrlHdmiInitMask  |= (1 << iapv_addr);
	  nFound++;
	}
    }

  MPD_DBGN(MPD_DEBUG_APVINIT,
	   "MPD %2d: %2d APV found in blind scan (i2c adr mask = 0x%08x)\n",
	  id, nFound, (uint32_t)(fMpd[id].CtrlHdmiInitMask));

  /* Retry for those missing APVs that are defined in config file */
  if(configApvMask != fMpd[id].CtrlHdmiInitMask)
    {
      int ibit = 0, retry = 0;
      for(ibit = 0; ibit < MPD_MAX_APV; ibit++)
	{
	  if(configApvMask & (1<<ibit))
	    {
	      if((fMpd[id].CtrlHdmiInitMask & (1 << ibit)) == 0)
		{
		  MPD_DBGN(MPD_DEBUG_APVINIT,
			  "Did not find APV with i2c address 0x%x\n",
			  ibit);

		  retry = 19;
		  while ((mpdAPV_TryNoHdmi(id, ibit) != OK) && (retry < 20))
		    {
		      MPD_DBGN(MPD_DEBUG_APVINIT,
			      "%2d: Retry NOT successfull\n", retry);
		      // FIXME: Need something to do here.
		      usleep(20);
		      retry++;
		    }

		  if(retry < 20)
		    {
		      MPD_MSG("Retry successfull!\n");
		      fMpd[id].CtrlHdmiInitMask  |= (1 << ibit);
		      nFound++;
		    }
		}
	    }
	}
      MPD_DBGN(MPD_DEBUG_APVINIT,
	       "MPD %2d: %2d APV found after retry (i2c adr mask = 0x%08x)\n",
	       id, nFound, (uint32_t)(fMpd[id].CtrlHdmiInitMask));
    }

  mpdResetApvEnableMask(id);

  nApv[id] = 0;

  for (iapv = 0; iapv < fMpd[id].nAPV; iapv++)
    {
      MPD_DBGN(MPD_DEBUG_APVINIT,
	       "Try i2c=0x%02x adc=%2d : \n", fApv[id][iapv].i2c,
	       fApv[id][iapv].adc);

      if ((fMpd[id].CtrlHdmiInitMask & (1 << fApv[id][iapv].i2c)) == 0)
	{
	  MPD_DBGN(MPD_DEBUG_APVINIT,
	  "Slot %d: I2C 0x%02x  ADC %d  Not found in blindscan.  It is disabled\n",
		   id, fApv[id][iapv].i2c, fApv[id][iapv].adc);
	  fApvEnableMask[id] &= ~(1 << fApv[id][iapv].adc);
	  fApv[id][iapv].enabled = 0;
	  continue;
	}

      MPD_DBGN(MPD_DEBUG_APVINIT,
	       "0x%02x matched in MPD in slot %d\n", fApv[id][iapv].i2c, id);

      mpdSetApvEnableMask(id, (1 << fApv[id][iapv].adc));

      fApv[id][iapv].enabled = 1;
      nApv[id]++;
    }

  MPD_DBG("MPD %2d: %2d APV found matching settings (adc mask = 0x%08x)\n",
	  id, nApv[id], fApvEnableMask[id]);

  return nApv[id];
}

/*
  Local Variables:
  compile-command: "make -k -B mpdI2CScan"
  End:
 */

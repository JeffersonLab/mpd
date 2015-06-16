/*----------------------------------------------------------------------------*/
/**
 * @mainpage
 * <pre>
 *  mpdConfig.c
 *           - Configuration library for the MultiPurpose Digitizer (MPD)
 *             using a VxWorks 5.5 (PPC) or Linux 2.6.18 (Intel) or
 *             later based single board computer.
 *
 *  Author: Bryan Moffit
 *          Jefferson Lab Data Acquisition Group
 *          November 2014
 *
 * </pre>
 *----------------------------------------------------------------------------*/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include "libconfig/libconfig.h"
#include "jvme.h"
#include "mpdConfig.h"
#include "mpdLib.h"

static config_t mpd_cfg;
static config_setting_t *bus_setting, *mpd_setting, *run_setting;
static uint32_t nMPD;

int
mpdConfigInit(char *confFileName)
{
  if(confFileName==NULL)
    {
      printf("%s: ERROR: confFileName may not be NULL\n",
	     __FUNCTION__);
      return ERROR;
    }

  config_init(&mpd_cfg);
  
  /* Read the file. If there is an error, report it and exit. */
  if(! config_read_file(&mpd_cfg, confFileName))
    {
      printf("%s: ERROR: %s:%d - %s\n", 
	     __FUNCTION__,config_error_file(&mpd_cfg),
	     config_error_line(&mpd_cfg), config_error_text(&mpd_cfg));
      config_destroy(&mpd_cfg);
      return(ERROR);
    }

  printf("%s: %s: Version = %s\n",
	 __FUNCTION__,
	 confFileName,
	 mpdReadCfgString(&mpd_cfg, "version", 0));

  bus_setting = config_lookup(&mpd_cfg,"bus");
  if(bus_setting==NULL)
    {
      printf("%s: ERROR: Unable to find bus setting in %s\n",
	     __FUNCTION__,
	     confFileName);
      return ERROR;
    }

  // Assume it's the first and only. Otherwise, we'll need to loop to find the right one.
  bus_setting = config_setting_get_elem(bus_setting,0);

  mpd_setting = config_setting_get_member(bus_setting,"mpd");
  if(mpd_setting==NULL)
    {
      printf("%s: ERROR: Unable to find mpd setting in %s\n",
	     __FUNCTION__,
	     confFileName);
      return ERROR;
    }
  
  nMPD = config_setting_length(mpd_setting);

  run_setting = config_lookup(&mpd_cfg,"run");

  return OK;
}

int
mpdConfigLoad()
{
  config_setting_t *mpdset=NULL, 
    *adc_setting=NULL, *adcset=NULL, 
    *apv_setting=NULL, *apvset=NULL,
    *i2c_setting=NULL; /* settings to iterate */
  int impd=0, iadc=0, iapv=0, igain=0;
  int slot=0;
  int nApv;
  char *pattern;

  for(impd=0; impd<nMPD; impd++)
    {
      mpdset = config_setting_get_elem(mpd_setting,impd);

      slot = mpdReadSettingInt(mpdset, "rotary", 0);

      printf("%s: Loading MPD settings = %2d rotary = %2d\n",
	     __FUNCTION__,impd, slot);

      if(slot<0)
	{
	  printf("%s: Module %d is disabled (negative rotary)\n",
		 __FUNCTION__, impd);
	  continue;
	}

      // Initialize MPD here...

      mpdSetZeroLevel(slot, mpdReadSettingInt(mpdset, "zero_level", 0));
      mpdSetOneLevel(slot, mpdReadSettingInt(mpdset, "one_level", 0));

      mpdSetTriggerMode(slot, 
			mpdReadSettingInt(mpdset, "calib_latency", 0),
			mpdReadSettingInt(mpdset, "trigger_number", 0));;

      mpdSetAcqMode(slot,
		    (char *)mpdReadSettingString(run_setting,"mode",0));

      mpdSetCommonNoiseSubtraction(slot,
				   (short)mpdReadSettingInt(mpdset, "common_noise_subtraction",0));

      mpdSetEventBuilding(slot,
			  mpdReadSettingInt(mpdset, "event_building",0));
      mpdSetCommonOffset(slot,
			 mpdReadSettingInt(mpdset, "common_offset",0));

      mpdSetPedThrCommon(slot,
			 mpdReadSettingInt(mpdset, "ped_common",0),
			 mpdReadSettingInt(mpdset, "thr_common",0));

      mpdSetPedThrPath(slot,
		       (char *)mpdReadSettingString(mpdset, "pedthr_file",0));

      mpdSetChannelMark(slot,
			mpdReadSettingInt(mpdset, "channel_mark",0));

#ifdef NOTDONE
      mpdReadPedThr(slot);
#endif
      adc_setting = config_setting_get_member(mpd_setting,"adc");
      for(iadc=0; iadc<2; iadc++)
	{
	  adcset = config_setting_get_elem(adc_setting,iadc);

	  mpdSetAdcClockPhase(slot, iadc,
			      mpdReadSettingInt(adcset,"clock_phase",0));
	  printf("%s: Adc %d Clock Phase %d\n",
		 __FUNCTION__,iadc,mpdReadSettingInt(adcset,"clock_phase",0));

	  for(igain=0; igain<8; igain++)
	    {
	      mpdSetAdcGain(slot, iadc, igain,
			    mpdReadSettingInt(adcset,"gain",igain));
	    }

	  mpdSetAdcInvert(slot, iadc, mpdReadSettingInt(adcset, "invert", 0));

	  pattern = (char *)mpdReadSettingString(adcset, "pattern", 0);
	  if( strcmp(pattern,"none")==0)
	    mpdSetAdcPattern(slot, iadc, MPD_ADS5281_PAT_NONE);
	  if( strcmp(pattern,"sync")==0)
	    mpdSetAdcPattern(slot, iadc, MPD_ADS5281_PAT_SYNC);
	  if( strcmp(pattern,"deskew")==0)
	    mpdSetAdcPattern(slot, iadc, MPD_ADS5281_PAT_DESKEW);
	  if( strcmp(pattern,"ramp")==0)
	    mpdSetAdcPattern(slot, iadc, MPD_ADS5281_PAT_RAMP);

	}

      i2c_setting = config_setting_get_member(mpd_setting,"i2c");
      mpdSetI2CSpeed(slot, mpdReadSettingInt(i2c_setting, "speed",0));
      mpdSetI2CMaxRetry(slot, mpdReadSettingInt(i2c_setting, "timeout",0));

      apv_setting = config_setting_get_member(mpd_setting,"apv");

      nApv = config_setting_length(apv_setting);
      printf("%s: %d APV elements in given MPD %d\n",
	     __FUNCTION__,nApv,slot);
      if(nApv>16)
	{
	  printf("%s: ERROR: Too many APV settings (%d)\n",
		 __FUNCTION__,nApv);
	  return ERROR;
	}

      /* APV */
      int apv_freq=0, apv_smode=0, mode=0;
      int apv_count=0;
      
      apv_freq  = mpdReadSettingInt(mpdset, "apv_Frequency",0);
      apv_smode = mpdReadSettingInt(mpdset, "apv_SampleMode",0);

      for(iapv=0; iapv<nApv; iapv++)
	{
	  apvset = config_setting_get_elem(apv_setting,iapv);
	  
	  if(mpdReadSettingInt(apvset, "i2c",0) < 0)
	    continue;

	  ApvParameters gApv;
	  memset((char *)&gApv, 0, sizeof(ApvParameters));

	  gApv.i2c = mpdReadSettingInt(apvset, "i2c",0);
	  gApv.adc = mpdReadSettingInt(apvset, "adc",0);

	  gApv.Ipre    = mpdReadSettingInt(apvset, "Ipre",0);
	  gApv.Ipcasc  = mpdReadSettingInt(apvset, "Ipcasc",0);
	  gApv.Ipsf    = mpdReadSettingInt(apvset, "Ipsf",0);
	  gApv.Isha    = mpdReadSettingInt(apvset, "Isha",0);
	  gApv.Issf    = mpdReadSettingInt(apvset, "Issf",0);
	  gApv.Ipsp    = mpdReadSettingInt(apvset, "Ipsp",0);
	  gApv.Imuxin  = mpdReadSettingInt(apvset, "Imuxin",0);
	  gApv.Ispare  = mpdReadSettingInt(apvset, "Ispare",0);
	  gApv.Ical    = mpdReadSettingInt(apvset, "Ical",0);
	  gApv.Vfp     = mpdReadSettingInt(apvset, "Vfp",0);
	  gApv.Vfs     = mpdReadSettingInt(apvset, "Vfs",0);
	  gApv.Vpsp    = mpdReadSettingInt(apvset, "Vpsp",0);
	  gApv.Cdrv    = mpdReadSettingInt(apvset, "Cdrv",0);
	  gApv.Csel    = mpdReadSettingInt(apvset, "Csel",0);
	  gApv.Latency = mpdReadSettingInt(apvset, "Latency",0);
	  gApv.Muxgain = mpdReadSettingInt(apvset, "Muxgain",0);

	  mode = (mpdReadSettingInt(apvset, "Polarization",0) << 5) |
	    (apv_freq << 4) |
	    (mpdReadSettingInt(apvset, "ReadOutMode",0) << 3) |
	    ((1 - mpdReadSettingInt(apvset, "ReadOutMode",0)) << 2) |
	    (apv_smode << 1) |
	    mpdReadSettingInt(apvset, "AnalogBias",0);

	  gApv.Mode = mode;

	  mpdAddApv(slot, gApv);
	  apv_count++;
	}
      
      printf("%s: %2d APV loaded in Slot=%d MPD settings\n",
	     __FUNCTION__,apv_count,slot);
      
    }
  return OK;
}

int
mpdReadCfgInt(config_t *cfg, char *name, int index)
{
  int rval=0;

  if(config_lookup_int(cfg, name, &rval))
    return rval;
  else
    return ERROR;
}

float
mpdReadCfgFloat(config_t *cfg, char *name, int index)
{
  double rval=0;

  if(config_lookup_float(cfg, name, &rval))
    return rval;
  else
    return -1;
}

const char *
mpdReadCfgString(config_t *cfg, char *name, int index)
{
  const char *rval=NULL;

  if(config_lookup_string(cfg, name, &rval))
    return rval;
  else
    return NULL;

}

int
mpdReadSettingSize(config_setting_t *setting, char *name)
{
  int rval=0;

  rval = config_setting_length(setting);

  return rval;
}

int
mpdReadSettingInt(config_setting_t *setting, char *name, int index)
{
  int rval=0;

  if(config_setting_lookup_int(setting, name, &rval))
    return rval;
  else
    return -1;

}

double
mpdReadSettingFloat(config_setting_t *setting, char *name, int index)
{
  double rval=0;

  if(config_setting_lookup_float(setting, name, &rval))
    return rval;
  else
    return -1;
}

const char *
mpdReadSettingString(config_setting_t *setting, char *name, int index)
{
  const char *rval=0;

  if(config_setting_lookup_string(setting, name, &rval))
    return rval;
  else
    return NULL;

}


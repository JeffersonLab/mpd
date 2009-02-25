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
 *          Danning Di / UVa / July 2015
 *          Evaristo Cisbani / INFN / July 2015
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
//static config_setting_t *bus_setting, *mpd_setting, *run_setting, *bus_setting0;
//static config_setting_t *d_setting, *d_bus_setting, *d_mpd_setting, *d_busset;
static config_setting_t *d_setting, *d_busset;
static config_setting_t *bus_set[2], *mpd_set[2], *run_set[2], *bus_set0[2]; // second element is pointer to default val
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

  bus_set[0] = config_lookup(&mpd_cfg,"bus");
  if(bus_set[0]==NULL)
    {
      printf("%s: ERROR: Unable to find bus setting in %s\n",
	     __FUNCTION__,
	     confFileName);
      return ERROR;
    }

  // Assume it's the first and only. Otherwise, we'll need to loop to find the right one.
  bus_set0[0] = config_setting_get_elem(bus_set[0],0);

  mpd_set[0] = config_setting_get_member(bus_set0[0],"mpd");
  if(mpd_set[0]==NULL)
    {
      printf("%s: ERROR: Unable to find mpd setting in %s\n",
	     __FUNCTION__,
	     confFileName);
      return ERROR;
    }
  
  nMPD = config_setting_length(mpd_set[0]);

  run_set[0] = config_lookup(&mpd_cfg,"run");
  
  d_setting = config_lookup(&mpd_cfg,"default");
  bus_set[1]   = config_setting_get_member(d_setting,"bus");
  d_busset        = config_setting_get_elem(bus_set[1],0);

  mpd_set[1]   = config_setting_get_member(d_busset,"mpd");
  run_set[1]   = config_setting_get_member(d_setting,"run");

  printf("Number of MPDs in config file = %d\n",nMPD);
  return OK;
}

int
mpdConfigLoad()
{
  config_setting_t *mpdset[2]={NULL,NULL},
    *adc_set[2]={NULL, NULL}, 
      *adcset[2]={NULL, NULL},
	*apv_set[2]={NULL, NULL}, 
	  *apvset[2]={NULL, NULL},
	    *i2c_set[2]={NULL, NULL}; /* settings to iterate */
  int impd=0, iadc=0, iapv=0, igain=0;
  int slot=0;
  int nApv;
  char *pattern;

  printf("%s: Start\n",__FUNCTION__);

  for(impd=0; impd<nMPD; impd++)
    {
      mpdset[0] = config_setting_get_elem(mpd_set[0],impd);
      mpdset[1] = config_setting_get_elem(mpd_set[1],0);

      slot = mpdReadSettingInt(mpdset, "rotary", 0);
      
      printf("%s: Loading MPD idx= %2d rotary (slot) = %2d\n",
	     __FUNCTION__,impd, slot);

      if(slot<0)
	{
	  printf("%s: Module %d is disabled (negative rotary)\n",
		 __FUNCTION__, impd);
	  continue;
	}

      // Initialize MPD here...
      mpdSetCommonNoiseSubtraction(slot, (short) mpdReadSettingInt(mpdset, "common_noise_subtraction",0));

      mpdSetZeroLevel(slot, mpdReadSettingInt(mpdset, "zero_level", 0));
      mpdSetOneLevel(slot, mpdReadSettingInt(mpdset, "one_level", 0));

      mpdSetTriggerMode(slot, 
      			mpdReadSettingInt(mpdset, "calib_latency", 0),
      			mpdReadSettingInt(mpdset, "trigger_latency", 0),
      			mpdReadSettingInt(mpdset, "trigger_number", 0));

      
      mpdSetAcqMode(slot,
		    (char *)mpdReadSettingString(run_set,"mode",0));

      mpdSetEventBuilding(slot,
			  mpdReadSettingInt(mpdset, "event_building",0));


      mpdSetEventPerBlock(slot,
			  mpdReadSettingInt(mpdset, "event_per_block",0));


      mpdSetUseSdram(slot,
			  mpdReadSettingInt(mpdset, "use_sdram",0));

      mpdSetFastReadout(slot,
			  mpdReadSettingInt(mpdset, "fast_readout",0));



      mpdSetCommonOffset(slot,
			 mpdReadSettingInt(mpdset, "common_offset",0));

      mpdSetPedThrCommon(slot,
			 mpdReadSettingInt(mpdset, "ped_common",0),
			 mpdReadSettingInt(mpdset, "thr_common",0));

      mpdSetPedThrPath(slot,
		       (char *)mpdReadSettingString(mpdset, "pedthr_file",0));

      mpdSetChannelMark(slot,mpdReadSettingInt(mpdset, "channel_mark",0));


      mpdSetInPath0(slot, 
		    mpdReadSettingBool(mpdset, "en_trig1_P0",0),
		    mpdReadSettingBool(mpdset, "en_trig2_P0",0),
		    mpdReadSettingBool(mpdset, "en_trig_Front",0),
		    mpdReadSettingBool(mpdset, "en_sync_P0",0),
		    mpdReadSettingBool(mpdset, "en_sync_Front",0));

      mpdSetInputLevel(slot, 0, mpdReadSettingInt(mpdset, "input_0_level", 0));
      mpdSetInputLevel(slot, 1, mpdReadSettingInt(mpdset, "input_1_level", 0));

      mpdSetOutputLevel(slot, 0, mpdReadSettingInt(mpdset, "output_0_level",0));
      mpdSetOutputLevel(slot, 1, mpdReadSettingInt(mpdset, "output_1_level",0));

      int ifir;
      mpdSetFIRenable(slot, mpdReadSettingInt(mpdset, "fir_enable", 0));
      for (ifir=0;ifir<16;ifir++) {
	mpdSetFIRcoeff(slot, ifir, 
		       mpdReadSettingInt(mpdset, "fir_coeff", ifir));
      }

#ifdef NOTDONE
      mpdReadPedThr(slot);
#endif


      adc_set[0] = config_setting_get_member(mpdset[0],"adc");
      adc_set[1] = config_setting_get_member(mpdset[1],"adc");

      for(iadc=0; iadc<2; iadc++)
	{    
	  adcset[0] = config_setting_get_elem(adc_set[0],iadc); 
	  adcset[1] = config_setting_get_elem(adc_set[1],0); 
	  mpdSetAdcClockPhase(slot, iadc, mpdReadSettingInt(adcset,"clock_phase",0));
	  
	  for(igain=0; igain<8; igain++)
	    {
	      mpdSetAdcGain(slot, iadc, igain,
			    mpdReadSettingInt(adcset,"gain",igain));
	    }

	  mpdSetAdcInvert(slot, iadc, mpdReadSettingBool(adcset, "invert", 0));

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

      i2c_set[0] = config_setting_get_member(mpdset[0],"i2c");
      i2c_set[1] = config_setting_get_member(mpdset[1],"i2c");
      mpdSetI2CSpeed(slot, mpdReadSettingInt(i2c_set, "speed",0));
      mpdSetI2CMaxRetry(slot, mpdReadSettingInt(i2c_set, "timeout",0));

      apv_set[0] = config_setting_get_member(mpdset[0],"apv");
      apv_set[1] = config_setting_get_member(mpdset[1],"apv");

      nApv = config_setting_length(apv_set[0]);
      printf("%s: %d APV elements in given MPD %d\n",
	     __FUNCTION__,nApv,slot);
      if(nApv>16)
	{
	  printf("%s: ERROR: Too many APV settings (%d)\n",
		 __FUNCTION__,nApv);
	  return ERROR;
	}

      mpdSetNumberAPV(slot, nApv);

      /* APV */
      int apv_freq=0, apv_smode=0, mode=0;
      int apv_count=0;
      
      apv_freq  = mpdReadSettingInt(mpdset, "apv_Frequency",0);
      apv_smode = mpdReadSettingInt(mpdset, "apv_SampleMode",0);

      for(iapv=0; iapv<nApv; iapv++)
	{
	  apvset[0] = config_setting_get_elem(apv_set[0],iapv);
	  apvset[1] = config_setting_get_elem(apv_set[1],0);
	  
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

	  //	  gApv.CalPulse = mpdReadSettingInt(apvset, "CalPulse",0);
	  //	  if(mpdReadSettingInt(apvset, "CalPulse",0)==-1){
	  //	  gApv.CalPulse = mpdReadSettingInt(d_apvset, "CalPulse",0);}
	  //      CalPulse is not in this ConfigFile originally

	  mode = (mpdReadSettingInt(apvset, "Polarization",0) << 5) |
	    (apv_freq << 4) |
	    (mpdReadSettingInt(apvset, "ReadOutMode",0) << 3) |
	    ((1 - mpdReadSettingInt(apvset, "CalPulse",0)) << 2) |
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
mpdReadSettingInt(config_setting_t **setting, char *name, int index)
{
  int rval=0;
  config_setting_t *member;

  member = config_setting_get_member(setting[0],name);
  //  if(config_setting_lookup_int(setting[0], name, &rval))
  if (member != NULL) {
    if (config_setting_length(member)) { // array
      rval = config_setting_get_int_elem(member,index);
    } else {
      config_setting_lookup_int(setting[0], name, &rval);
    }
    //    printf(" %s = %d\n", name, rval);
    return rval;
  }
  else 
    if(config_setting_lookup_int(setting[1], name, &rval)) {
      //      printf(" %s = %d (default)\n",name, rval);
      return rval;
    }

  printf("%s: cannot get %s setting (neither specific nor default)\n",__FUNCTION__,name);
  return -1;

}

int
mpdReadSettingBool(config_setting_t **setting, char *name, int index)
{
  int rval=0;
  config_setting_t *member;

  member = config_setting_get_member(setting[0],name);
  //  if(config_setting_lookup_int(setting[0], name, &rval))
  if (member != NULL) {
    if (config_setting_length(member)) { // array
      rval = config_setting_get_bool_elem(member,index);
    } else {
      config_setting_lookup_bool(setting[0], name, &rval);
    }
    //    printf(" %s = %d\n", name, rval);
    return rval;
  }
  else 
    if(config_setting_lookup_bool(setting[1], name, &rval)) {
      //   printf(" %s = %d (default)\n",name, rval);
      return rval;
    }

  printf("%s: cannot get %s setting (neither specific nor default)\n",__FUNCTION__,name);
  return -1;

}

double
mpdReadSettingFloat(config_setting_t **setting, char *name, int index)
{
  double rval=0;

  if(config_setting_lookup_float(setting[0], name, &rval))
    return rval;
  else
    if(config_setting_lookup_float(setting[1], name, &rval))
      return rval;

  printf("%s: cannot get %s setting (neither specific nor default)\n",__FUNCTION__,name);
  return -1;

}

const char *
mpdReadSettingString(config_setting_t **setting, char *name, int index)
{
  const char *rval=0;

  if(config_setting_lookup_string(setting[0], name, &rval))
    return rval;
  else
    if(config_setting_lookup_string(setting[1], name, &rval))
      return rval;

  printf("%s: cannot get %s setting (neither specific nor default)\n",__FUNCTION__,name);
  return NULL;

}


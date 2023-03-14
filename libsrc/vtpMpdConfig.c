/*----------------------------------------------------------------------------*/
/**
 * @mainpage
 * <pre>
 *  vtpMpdConfig.c
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
#include "libconfig.h"
#include "vtp.h"
#include "mpdConfig.h"
#include "mpdLib.h"
#include "vtpMpdConfig.h"

static config_t mpd_cfg;
static char *config_include_directory;
//static config_setting_t *bus_setting, *mpd_setting, *run_setting, *bus_setting0;
//static config_setting_t *d_setting, *d_bus_setting, *d_mpd_setting, *d_busset;
static config_setting_t *d_setting, *d_busset;
static config_setting_t *bus_set[2], *vtp_set[2], *mpd_set[2], *run_set[2], *vtp_set0[2]; // second element is pointer to default val
static uint32_t nMPD;
static uint64_t vtpFiberMask;
static int32_t initOK = ERROR;

extern int mpdPrintDebug;

typedef struct
{
  char name[256];
  int32_t offset;
} layer_latency_t;

layer_latency_t *layer; // pointer to array of layers
uint8_t nlayers = 0;

int
vtpMpdConfigInit(char *confFileName)
{
  if(confFileName==NULL)
    {
      MPD_ERR("confFileName may not be NULL\n");
      initOK = ERROR;
      return ERROR;
    }

  config_init(&mpd_cfg);

  /* Read the file. If there is an error, report it and exit. */
  if(! config_read_file(&mpd_cfg, confFileName))
    {
      MPD_ERR("%s:%d - %s\n",
	      config_error_file(&mpd_cfg),
	      config_error_line(&mpd_cfg), config_error_text(&mpd_cfg));
      config_destroy(&mpd_cfg);
      initOK = ERROR;

      return(ERROR);
    }

  MPD_MSG("%s: Version = %s\n",
	 confFileName,
	 mpdReadCfgString(&mpd_cfg, "version", 0));


  vtp_set[0] = config_lookup(&mpd_cfg,"vtp");
  if(vtp_set[0]==NULL)
    {
      MPD_ERR("Unable to find vtp setting in %s\n",
	      confFileName);
      initOK = ERROR;
      return ERROR;
    }

  // Assume it's the first and only.
  vtp_set0[0] = config_setting_get_elem(vtp_set[0],0);
  if(config_setting_length(vtp_set[0]) > 1)
    {
      MPD_WARN("Too many VTPs defined.  Using the first.\n");
    }

  mpd_set[0] = config_setting_get_member(vtp_set0[0],"ports");
  if(mpd_set[0]==NULL)
    {
      printf("%s: ERROR: Unable to find fiber port setting in %s\n",
	     __FUNCTION__,
	     confFileName);
      initOK = ERROR;
      return ERROR;
    }

  nMPD = config_setting_length(mpd_set[0]);
  mpdSetNmpdToInit(nMPD);

  d_setting = config_lookup(&mpd_cfg,"default");
  bus_set[1]   = config_setting_get_member(d_setting,"bus");
  d_busset        = config_setting_get_elem(bus_set[1],0);

  mpd_set[1]   = config_setting_get_member(d_busset,"mpd");
  run_set[1]   = config_setting_get_member(d_setting,"run");

  /* Get the layer latency settings */
  config_setting_t *layer_latency_setting =
    config_lookup(&mpd_cfg,"layer_latency");

  if(layer_latency_setting)
    {
      nlayers = config_setting_length(layer_latency_setting);
    }
  else
    {
      /* Get it from defaults */
      layer_latency_setting = config_setting_get_member(d_setting, "layer_latency");
      if(layer_latency_setting)
	{
	  nlayers = config_setting_length(layer_latency_setting);
	}
      else
	{
	  MPD_ERR(" layer_latency setting is undefined in config file\n");
	  initOK = ERROR;
	  return ERROR;
	}
    }

  MPD_DBG(" nlayers (layer_latency) = %d\n", nlayers);
  /* Alloc an array of layer latency settings */
  layer = (layer_latency_t *)malloc(nlayers*sizeof(layer_latency_t));

  /* Fill er up */
  int ilayer;

  for(ilayer = 0; ilayer < nlayers; ilayer++)
    {
      config_setting_t *ll_elem =
	config_setting_get_elem(layer_latency_setting, ilayer);

      const char *thisname;
      config_setting_lookup_string(ll_elem, "name", &thisname);
      strncpy(layer[ilayer].name, thisname, 256);

      config_setting_lookup_int(ll_elem, "offset", &layer[ilayer].offset);

      MPD_DBG(" layer.name = %s  layer.offset = %d\n",
	      layer[ilayer].name, layer[ilayer].offset);
    }



  printf("%s: Number of MPDs in config file = %d\n",__FUNCTION__,nMPD);
  initOK = OK;
  return OK;
}

int
vtpMpdConfigLoad()
{
  config_setting_t *mpdset[2]={NULL,NULL},
    *adc_set[2]={NULL, NULL},
      *adcset[2]={NULL, NULL},
	*apv_set[2]={NULL, NULL},
	  *apvset[2]={NULL, NULL},
	    *i2c_set[2]={NULL, NULL}; /* settings to iterate */
  int iadc=0, iapv=0, igain=0;
  uint64_t impd=0;
  int fiberPort=0;
  int nApv;
  char *pattern;

  MPD_DBG("Start\n");

  if(initOK==ERROR)
    {
      MPD_ERR("Initialization failed. Bailing\n");
      return ERROR;
    }

  MPD_MSG("Configuring VTP\n", __func__);

  vtpFiberMask = 0;
  mpdParametersInit();

  for(impd=0; impd<nMPD; impd++)
    {
      mpdset[0] = config_setting_get_elem(mpd_set[0],impd);
      mpdset[1] = config_setting_get_elem(mpd_set[1],0);

      fiberPort = mpdReadSettingInt(mpdset, "fiberPort", 0);
      vtpFiberMask |= (0x1ull << fiberPort);

      MPD_DBG("Loading MPD idx= %2d fiberPort = %2d\n",
	      impd, fiberPort);

      if(fiberPort<0)
	{
	  MPD_DBG("Module %d is disabled (negative rotary)\n",
		  impd);
	  continue;
	}
      mpdset[0] = config_setting_get_member(mpdset[0], "mpd");
      // Initialize MPD here...
      mpdSetCommonNoiseSubtraction(fiberPort, (short) mpdReadSettingInt(mpdset, "common_noise_subtraction",0));

      mpdSetZeroLevel(fiberPort, mpdReadSettingInt(mpdset, "zero_level", 0));
      mpdSetOneLevel(fiberPort, mpdReadSettingInt(mpdset, "one_level", 0));

      mpdSetTriggerMode(fiberPort,
      			mpdReadSettingInt(mpdset, "calib_latency", 0),
      			mpdReadSettingInt(mpdset, "trigger_latency", 0),
      			mpdReadSettingInt(mpdset, "trigger_number", 0));

      //      if(run_set[1])
      //	{
      //	  mpdSetAcqMode(fiberPort,
      //			(char *)mpdReadSettingString(run_set,"mode",0));
      //	}

      mpdSetEventBuilding(fiberPort,
			  mpdReadSettingInt(mpdset, "event_building",0));


      mpdSetEventPerBlock(fiberPort,
			  mpdReadSettingInt(mpdset, "event_per_block",0));


      mpdSetUseSdram(fiberPort,
			  mpdReadSettingInt(mpdset, "use_sdram",0));

      mpdSetFastReadout(fiberPort,
			  mpdReadSettingInt(mpdset, "fast_readout",0));



      mpdSetCommonOffset(fiberPort,
			 mpdReadSettingInt(mpdset, "common_offset",0));

      mpdSetPedThrCommon(fiberPort,
			 mpdReadSettingInt(mpdset, "ped_common",0),
			 mpdReadSettingInt(mpdset, "thr_common",0));

      mpdSetPedThrPath(fiberPort,
		       (char *)mpdReadSettingString(mpdset, "pedthr_file",0));

      mpdSetChannelMark(fiberPort,mpdReadSettingInt(mpdset, "channel_mark",0));


      mpdSetInPath0(fiberPort,
		    mpdReadSettingBool(mpdset, "en_trig1_P0",0),
		    mpdReadSettingBool(mpdset, "en_trig2_P0",0),
		    mpdReadSettingBool(mpdset, "en_trig_Front",0),
		    mpdReadSettingBool(mpdset, "en_sync_P0",0),
		    mpdReadSettingBool(mpdset, "en_sync_Front",0));

      mpdSetInputLevel(fiberPort, 0, mpdReadSettingInt(mpdset, "input_0_level", 0));
      mpdSetInputLevel(fiberPort, 1, mpdReadSettingInt(mpdset, "input_1_level", 0));

      mpdSetOutputLevel(fiberPort, 0, mpdReadSettingInt(mpdset, "output_0_level",0));
      mpdSetOutputLevel(fiberPort, 1, mpdReadSettingInt(mpdset, "output_1_level",0));

      int ifir;

      mpdSetFIRenable(fiberPort, mpdReadSettingInt(mpdset, "fir_enable", 0));
      for (ifir=0;ifir<16;ifir++) {
	mpdSetFIRcoeff(fiberPort, ifir,
		       mpdReadSettingInt(mpdset, "fir_coeff", ifir));
      }

#ifdef NOTDONE
      mpdReadPedThr(fiberPort);
#endif


      adc_set[0] = config_setting_get_member(mpdset[0],"adc");
      adc_set[1] = config_setting_get_member(mpdset[1],"adc");

      for(iadc=0; iadc<2; iadc++)
	{
	  adcset[0] = config_setting_get_elem(adc_set[0],iadc);
	  adcset[1] = config_setting_get_elem(adc_set[1],0);
	  mpdSetAdcClockPhase(fiberPort, iadc, mpdReadSettingInt(adcset,"clock_phase",0));

	  for(igain=0; igain<8; igain++)
	    {
	      mpdSetAdcGain(fiberPort, iadc, igain,
			    mpdReadSettingInt(adcset,"gain",igain));
	    }

	  mpdSetAdcInvert(fiberPort, iadc, mpdReadSettingBool(adcset, "invert", 0));
	  pattern = (char *)mpdReadSettingString(adcset, "pattern", 0);
	  if( strcmp(pattern,"none")==0)
	    mpdSetAdcPattern(fiberPort, iadc, MPD_ADS5281_PAT_NONE);
	  if( strcmp(pattern,"sync")==0)
	    mpdSetAdcPattern(fiberPort, iadc, MPD_ADS5281_PAT_SYNC);
	  if( strcmp(pattern,"deskew")==0)
	    mpdSetAdcPattern(fiberPort, iadc, MPD_ADS5281_PAT_DESKEW);
	  if( strcmp(pattern,"ramp")==0)
	    mpdSetAdcPattern(fiberPort, iadc, MPD_ADS5281_PAT_RAMP);

	}

      i2c_set[0] = config_setting_get_member(mpdset[0],"i2c");
      i2c_set[1] = config_setting_get_member(mpdset[1],"i2c");

      mpdSetI2CSpeed(fiberPort, mpdReadSettingInt(i2c_set, "speed",0));
      mpdSetI2CMaxRetry(fiberPort, mpdReadSettingInt(i2c_set, "timeout",0));
      apv_set[0] = config_setting_get_member(mpdset[0],"apv");
      apv_set[1] = config_setting_get_member(mpdset[1],"apv");

      nApv = config_setting_length(apv_set[0]);
      MPD_DBG("%d APV elements in given MPD %d\n",
	      nApv,fiberPort);
      if(nApv>16)
	{
	  MPD_ERR("Too many APV settings (%d)\n",
		  nApv);
	  return ERROR;
	}

      mpdSetNumberAPV(fiberPort, nApv);

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

	  const char *layername = mpdReadSettingString(apvset, "layer", 0);
	  if(layer != NULL)
	    {
	      strncpy(gApv.layer, layername, 256);
	    }
	  MPD_DBG("layer = %s\n", gApv.layer);

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
	  // apply correction based on layer name and offset
	  int ilayer;
	  for(ilayer = 0; ilayer < nlayers; ilayer++)
	    {
	      if(strncmp(layername, layer[ilayer].name, 256) == 0)
		{
		  MPD_DBG(" Updating latency for layer %s  offset = %d  latency = %d\n",
			  layer[ilayer].name, layer[ilayer].offset, gApv.Latency);
		  gApv.Latency += layer[ilayer].offset;
		  MPD_DBG("   --> new latency = %d\n",
			  gApv.Latency);
		  break;
		}
	    }

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

	  mpdAddApv(fiberPort, gApv);
	  apv_count++;
	}

      MPD_DBG("%2d APV loaded in fiberPort = %d MPD settings\n",
	      apv_count,fiberPort);

    }

  mpdSetVTPFiberMap_preInit(vtpFiberMask);

  return OK;
}

int
vtpMpdConfigWrite(char *filename)
{
  if(filename==NULL)
    {
      printf("%s: ERROR: filanem may not be NULL\n", __func__);
      return -1;
    }

  /* write the modified config back */
  config_write_file(&mpd_cfg, filename);

  return 0;
}

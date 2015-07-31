#ifndef __MPDCONFIG_H_
#define __MPDCONFIG_H_
#include "libconfig.h"
/*----------------------------------------------------------------------------*/
/**
 * @mainpage
 * <pre>
 *  mpdConfig.c
 *         - Header for Configuration library for the MultiPurpose
 *           Digitizer (MPD) using a VxWorks 5.5 (PPC) or Linux 2.6.18
 *           (Intel) or later based single board computer.
 *
 *  Author: Bryan Moffit
 *          Jefferson Lab Data Acquisition Group
 *          November 2014
 *
 * </pre>
 *----------------------------------------------------------------------------*/

int   mpdConfigInit(char *confFileName);
int mpdConfigLoad();
int   mpdReadCfgInt(config_t *cfg, char *name, int index);
float mpdReadCfgFloat(config_t *cfg, char *name, int index);
const char *mpdReadCfgString(config_t *cfg, char *name, int index);
int   mpdReadSettingSize(config_setting_t *setting, char *name);
int   mpdReadSettingInt(config_setting_t **setting, char *name, int index);
int   mpdReadSettingBool(config_setting_t **setting, char *name, int index);
double mpdReadSettingFloat(config_setting_t **setting, char *name, int index);
const char *mpdReadSettingString(config_setting_t **setting, char *name, int index);



#endif /* __MPDCONFIG_H_ */

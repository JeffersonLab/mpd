#ifndef __SSPMPDCONFIG_H_
#define __SSPMPDCONFIG_H_
#include "libconfig.h"
/*----------------------------------------------------------------------------*/
/**
 * @mainpage
 * <pre>
 *  sspMpdConfig.h
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

int  sspMpdConfigInit(char *confFileName);
int  sspMpdConfigLoad();

#endif /* __SSPMPDCONFIG_H_ */

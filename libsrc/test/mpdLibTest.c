/*
 * File:
 *    mpdLibTest.c
 *
 * Description:
 *    Simply evolving program to test the mpd library
 *
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "jvme.h"
#include "sspLib.h"
#include "mpdLib.h"
#include "../mpdConfig.h"

#define DO(x) {					\
    if(x!=OK) goto CLOSE;			\
  }

extern unsigned int sspMpdReadReg(int id, int impd, unsigned int reg);

int 
main(int argc, char *argv[]) 
{
  int iFlag=0;
  printf("\nMPD Library Tests\n");
  printf("----------------------------\n");

  DO(vmeOpenDefaultWindows());

  DO(mpdCheckAddresses(0));

#ifdef WITHSSP
  iFlag = 0xFFFF0000 | SSP_INIT_SKIP_FIRMWARE_CHECK;

  sspInit(4<<19,1<<19,1,iFlag);
  sspCheckAddresses(0);

  sspMpdFiberReset(0);
  sspMpdFiberLinkReset(0,0xffffffff);

  sspMpdDisable(sspSlot(0), 0xffffffff);
  sspMpdEnable(sspSlot(0), 0x1);

  sspSoftReset(0);
  
  iFlag = MPD_INIT_SSP_MODE | MPD_INIT_NO_CONFIG_FILE_CHECK;
#else
  iFlag = MPD_INIT_NO_CONFIG_FILE_CHECK;
#endif
  mpdInit(6<<19,0,1,iFlag);
  mpdSetBlocklevel(0, 255);

 CLOSE:
  vmeCloseDefaultWindows();

  exit(0);
}


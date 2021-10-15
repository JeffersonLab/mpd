/*
 * File:
 *    mpdApvConfig
 *
 * Description:
 *    Initialize the MPD at <slot number> and APVs defined in <configfile>
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

char filename[255] = "/home/sbs-onl/cfg/config_apv_INFN.txt";
char progname[255];

void
Usage()
{
  printf("\n");
  printf("%s: Initialize the MPD at <slot number> and APVs defined in <configfile>\n",
	 __FILE__);
  printf("\n");
  printf("   %s <slot number> <configfile>\n", progname);
  printf("\n");
  printf("     if <configfile> is not provided, this one will be used:\n");
  printf("      %s\n", filename);
  printf("\n");

}

int
main(int argc, char *argv[])
{
  int stat, slot;

  strncpy(progname, argv[0], 255);

  if (argc > 1)
    {
      slot = atoi(argv[1]);

      if ((slot < 0) || (slot > 32))
	{
	  printf("invalid slot... using 21");
	  slot = 2;
	}
      if(argc > 2)
	{
	  strncpy(filename, argv[2], 255);
	}
    }
  else
    {
      Usage();
      return 0;
    }

  printf("\n %s: slot = %d\n", argv[0], slot);
  printf("----------------------------\n");

  stat = vmeOpenDefaultWindows();
  if(stat != OK)
    goto CLOSE;

  vmeCheckMutexHealth(1);
  vmeBusLock();


  if(mpdConfigInit(filename) < 0)
    {
      printf(" Config initialization ERROR!\n");
      goto CLOSE;
    }
  mpdConfigLoad();

/* #define DEBUGI2C */
#ifdef DEBUGI2C
  vmeSetVMEDebugMode(1);
  vmeSetVMEDebugModeOutputFilename("out.dbg");
  extern FILE *fDebugMode;
#endif

  if(mpdInit((slot << 19), (1<<19), 1, 0) < 0)
    {
      printf("%s: Init error \n",
	     __func__);
      goto CLOSE;
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
  /* mpdSetPrintDebug(MPD_DEBUG_APVINIT); */
  if (mpdAPV_Scan(slot) <= 0)
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
  int iapv;
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
      for (iapv = 0; iapv < mpdGetNumberAPV(slot); iapv++)
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
  printf("APVs in Config File (ADC 15 ... 0):\n");
  printf("  MPD %2d : ", slot);
  iapv = 0;
  for (ibit = 15; ibit >= 0; ibit--)
    {
      if (((ibit + 1) % 4) == 0)
	printf(" ");
      if (apvConfigMask & (1 << ibit))
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
  printf("Found and Configured APVs: \n");

  /* if (mpdGetApvEnableMask(slot) != 0) */
    {
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
    }
  printf("\n");

/*   mpdGStatus(1); */
  mpdApvStatus(slot, 0);

  /* mpdInit(..) disables fiber mode, re-enable it for incoming fiber connections */
  mpdFiberEnable(slot);
  printf(" --- Fiber Mode enabled ---\n");

 CLOSE:
  vmeBusUnlock();

  vmeCloseDefaultWindows();

  exit(0);
}

/*
  Local Variables:
  compile-command: "make -k mpdApvConfig"
  End:
 */

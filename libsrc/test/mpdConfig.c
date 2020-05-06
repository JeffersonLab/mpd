/*
 * File:
 *    mpdConfig.c
 *
 * Description:
 *    Program to configure the MPD and APVs.
 *      This program just executes the configuration prescription used in rocDownload
 *
 *
 */


#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "jvme.h"
#include "mpdConfig.h"
#include "mpdLib.h"

extern int I2C_SendStop(int id);

int
main(int argc, char *argv[])
{
  int fnMPD = 0;
  int error_status = OK;

  int stat = OK;

  stat = vmeOpenDefaultWindows();
  if (stat != OK)
    {
      printf("Failed to open default VME windows\n");
      goto CLOSE;
    }

  /*****************
   *   MPD SETUP
   *****************/
  int rval = OK;

  /* Read config file and fill internal variables
     Change this to point to your main configuration file */

  rval = mpdConfigInit("config_apv.txt");
  if(rval != OK)
    {
      printf("ERROR: reading configuration file");
      error_status = ERROR;
    }

  mpdConfigLoad();

  /* Init and config MPD+APV */

  // discover MPDs and initialize memory mapping
  mpdInit((3 << 19), 0x80000, 18, 0x0);
  fnMPD = mpdGetNumberMPD();

  if(fnMPD <= 0)
    {				// test all possible vme slot ?
      printf("ERR: no MPD discovered, cannot continue\n");
      exit(-1);
    }

  printf("MPD discovered = %d\n", fnMPD);

  printf("\n");

  // APV configuration on all active MPDs
  int impd, iapv, id;

  for(impd = 0; impd < fnMPD; impd++)
    {				// only active mpd set
      id = mpdSlot(impd);
      printf("MPD slot %2d config:\n", id);


      rval = mpdHISTO_MemTest(id);

      printf(" - Initialize I2C\n");
      fflush(stdout);
      if(mpdI2C_Init(id) != OK)
	{
	  printf(" * * FAILED\n");
	  error_status = ERROR;
	}

      printf(" - APV discovery and init\n");

      fflush(stdout);
      mpdSetPrintDebug(0x0);
      if(mpdAPV_Scan(id) <= 0)
	{			// no apd found, skip next
	  printf(" * * None Found\n");
	  error_status = ERROR;
	  continue;
	}
      mpdSetPrintDebug(0);

      // apv reset
      printf(" - APV Reset\n");
      fflush(stdout);
      if(mpdI2C_ApvReset(id) != OK)
	{
	  printf(" * * FAILED\n");
	  error_status = ERROR;
	}

      usleep(10);
      I2C_SendStop(id);
      // board configuration (APV-ADC clocks phase)
      // (do this while APVs are resetting)
      printf(" - DELAY setting\n");
      fflush(stdout);
      if(mpdDELAY25_Set
	 (id, mpdGetAdcClockPhase(id, 0), mpdGetAdcClockPhase(id, 1)) != OK)
	{
	  printf(" * * FAILED\n");
	  error_status = ERROR;
	}

      // apv configuration
      mpdSetPrintDebug(0);
      printf(" - Configure Individual APVs\n");
      printf(" - - ");
      fflush(stdout);
      int itry, badTry = 0, saveError = error_status;
      error_status = OK;
      for(itry = 0; itry < 3; itry++)
	{
	  if(badTry)
	    {
	      printf(" ******** RETRY ********\n");
	      printf(" - - ");
	      fflush(stdout);
	      error_status = OK;
	    }
	  badTry = 0;
	  for(iapv = 0; iapv < mpdGetNumberAPV(id); iapv++)
	    {
	      printf("%2d ", iapv);
	      fflush(stdout);

	      if(mpdAPV_Config(id, iapv) != OK)
		{
		  printf(" * * FAILED for APV %2d\n", iapv);
		  if(iapv < (mpdGetNumberAPV(id) - 1))
		    printf(" - - ");
		  fflush(stdout);
		  error_status = ERROR;
		  badTry = 1;
		}
	    }
	  printf("\n");
	  fflush(stdout);
	  if(badTry)
	    {
	      printf(" ***** APV RESET *****\n");
	      fflush(stdout);
	      mpdI2C_ApvReset(id);
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

      error_status |= saveError;
      mpdSetPrintDebug(0);

      // configure adc on MPD
      printf(" - Configure ADC\n");
      fflush(stdout);
      if(mpdADS5281_Config(id) != OK)
	{
	  printf(" * * FAILED\n");
	  error_status = ERROR;
	}

      // configure fir
      // not implemented yet

      // RESET101 on the APV
      printf(" - Do APV RESET101\n");
      fflush(stdout);
      if(mpdAPV_Reset101(id) != OK)
	{
	  printf(" * * FAILED\n");
	  error_status = ERROR;
	}

      // <- MPD+APV initialization ends here
      printf("\n");
      fflush(stdout);
    }				// end loop on mpds
  //END of MPD configure

  mpdGStatus(1);

  // summary report
  printf("\n");
  printf("Configured APVs (ADC 15 ... 0)\n");

  int ibit;
  for(impd = 0; impd < fnMPD; impd++)
    {
      id = mpdSlot(impd);

      if(mpdGetApvEnableMask(id) != 0)
	{
	  printf("  MPD %2d : ", id);
	  iapv = 0;
	  for(ibit = 15; ibit >= 0; ibit--)
	    {
	      if(((ibit + 1) % 4) == 0)
		printf(" ");
	      if(mpdGetApvEnableMask(id) & (1 << ibit))
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
  printf("\n");

 CLOSE:
  vmeCloseDefaultWindows();

  exit(0);
}

/*
  Local Variables:
  compile-command: "make -k -B mpdConfig"
  End:
 */

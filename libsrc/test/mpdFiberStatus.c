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

int
main(int argc, char *argv[])
{
  int stat, slot;
  char filename[255] = "/home/sbs-onl/cfg/config_apv_INFN.txt";

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
    slot = 2;

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

  mpdFiberStatus(slot);

  mpdFiberEnable(slot);
  printf(" --- Fiber Mode enabled ---\n");

 CLOSE:
  vmeBusUnlock();

  vmeCloseDefaultWindows();

  exit(0);

}

/*
  Local Variables:
  compile-command: "make -k  mpdFiberStatus"
  End:
 */

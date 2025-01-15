/*
 * File:
 *    mpdTransceiverStatus.c
 *
 * Description:
 *    Attempt to use i2c core to communicate with transceiver
 *    addresses with i2c
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
  char filename[255] = "/daqfs/daq_setups/mpd_transceiver_status/cfg/davme7.cfg";


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

  if(mpdInit((slot << 19), (1<<19), 1, 0) < 0)
    {
      printf("%s: Init error \n",
	     __func__);
      goto CLOSE;
    }

  slot = mpdSlot(0);
  printf("MPD slot %2d config:\n", slot);

  mpdFiberStatus(slot);

  printf(" - Initialize I2C\n");

  if(mpdI2C_Init(slot) != OK)
    {
      printf(" * * FAILED\n");
    }

  mpdTransceiverGStatus();

  mpdFiberEnable(slot);
  printf(" --- Fiber Mode enabled ---\n");

 CLOSE:
  vmeBusUnlock();

  vmeCloseDefaultWindows();

  exit(0);

}

/*
  Local Variables:
  compile-command: "make -k mpdTransceiverStatus "
  End:
*/

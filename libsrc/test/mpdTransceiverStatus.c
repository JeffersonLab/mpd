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
  int stat = 0, slot = 0;
  extern int32_t nmpd;
  nmpd = 1;
  if (argc > 1)
    {
      slot = atoi(argv[1]);
      if ((slot < 0) || (slot > 21))
	{
	  printf("invalid slot... will scan");
	  slot = 2;
	  nmpd = 20;

	}
    }
  else
    {
      slot = 2;
      nmpd = 20;

    }

  stat = vmeOpenDefaultWindows();
  if(stat != OK)
    goto CLOSE;

  vmeCheckMutexHealth(1);
  vmeBusLock();

  if(mpdInit((slot << 19), (1<<19), nmpd, MPD_INIT_NO_CONFIG_FILE_CHECK) < 0)
    {
      if(nmpd <= 0)
	{
	  printf("%s: Init error \n",
		 __func__);
	  goto CLOSE;
	}
    }

  slot = mpdSlot(0);
  mpdSetI2CSpeed(slot, 2000);
  mpdSetI2CMaxRetry(slot, 200);
  mpdI2C_Init(slot);

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

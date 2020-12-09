/*
 * File:
 *    mpdStatus.c
 *
 * Description:
 *    Show status of MPDs
 *
 *
 */


#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "jvme.h"
#include "mpdLib.h"

int
main(int argc, char *argv[])
{
  int stat, slot;
  if (argc > 1)
    {
      slot = atoi(argv[1]);

      if ((slot < 0) || (slot > 32))
	{
	  printf("invalid slot... using 21");
	  slot = 2;
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

  if(mpdInit((slot << 19), (1<<19), 1, MPD_INIT_NO_CONFIG_FILE_CHECK) < 0)
    {
      printf("%s: Init error \n",
	     __func__);
      goto CLOSE;
    }

  slot = mpdSlot(0);
  printf("MPD slot %2d config:\n", slot);
  mpdGStatus(1);
  mpdFiberEnable(slot);
  mpdFiberStatus(slot);

 CLOSE:
  vmeBusUnlock();

  vmeCloseDefaultWindows();

  exit(0);
}

/*
  Local Variables:
  compile-command: "make -k -B mpdStatus"
  End:
 */

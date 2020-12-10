/*
 * File:
 *    mpdStatus.c
 *
 * Description:
 *    Show status of MPDs
 *
 *
 * Usage:
 *      mpdStatus <slot number>
 *
 *   if <slot number> is not provided, or 0... scan for all MPDs.
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
  extern int nmpd;

  nmpd = 1;

  if (argc > 1)
    {
      slot = atoi(argv[1]);

      if ((slot < 2) || (slot > 32))
	{
	  printf("invalid slot... scanning all slots");
	  slot = 2;
	  nmpd=19;
	}
    }
  else
    {
      slot = 2;
      nmpd=19;
    }

  printf("\n %s: slot = %d\n", argv[0], slot);
  printf("----------------------------\n");

  stat = vmeOpenDefaultWindows();
  if(stat != OK)
    goto CLOSE;

  vmeCheckMutexHealth(1);
  vmeBusLock();

  if(mpdInit((slot << 19), (1<<19), nmpd,
	     MPD_INIT_SKIP | MPD_INIT_NO_CONFIG_FILE_CHECK) < 0)
    {
      if(nmpd < 1)
	{
	  printf("%s: Init error \n",
		 __func__);
	  goto CLOSE;
	}
    }

  slot = mpdSlot(0);
  printf("MPD slot %2d config:\n", slot);
  mpdGStatus(1);

  /* mpdInit(..) disables fiber mode, re-enable it for incoming fiber connections */
  int impd;
  for(impd = 0; impd < nmpd; impd++)
    {
      mpdFiberEnable(mpdSlot(impd));

    }

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

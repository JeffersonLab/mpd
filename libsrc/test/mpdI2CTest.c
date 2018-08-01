/*
 * File:
 *    mpdI2CTest.c
 *
 * Description:
 *    Test i2c read/write MPD <---> APV
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
  int stat, slot, ipre;

  if (argc > 1)
    {
      slot = atoi(argv[1]);

      if ((slot < 0) || (slot > 22))
	{
	  printf("invalid slot... using 21");
	  slot = 21;
	}
    }
  else
    slot = 21;

  printf("\nJLAB TI Status... slot = %d\n", slot);
  printf("----------------------------\n");

  stat = vmeOpenDefaultWindows();
  if(stat != OK)
    goto CLOSE;

  vmeBusLock();

  mpdInit((slot << 19), 0, 1, 0x0);

  /* Loop over i2c speed */
  for(ipre = 0; ipre < 0x4B0; ipre++)
    {
      mpdSetI2CSpeed(slot, ipre);
      mpdI2C_Init(slot);


    }

CLOSE:
  vmeBusUnlock();

  vmeCloseDefaultWindows();

  exit(0);
}

/*
  Local Variables:
  compile-command: "make -k -B mpdI2CTest"
  End:
 */

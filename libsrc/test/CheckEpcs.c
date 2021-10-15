/*
  -----------------------------------------------------------------------------

  --- I.N.F.N. Genova - Electronic Department ---

  -----------------------------------------------------------------------------

  Name		:	RdAsmi.cpp

  Project		:

  Description 	:	Test for ASMI Interface using a SIS3104

  Date		:	July 2017
  Release		:	1.0
  Author		:	Paolo Musico



  -----------------------------------------------------------------------------

  EP1AGX60 bit stream length = 16,951,824 bits = 2118978 bytes (2.119 Mbytes) .RBF file size
  EPCS128 = 128 Mbit = 134,217,728 bit = 64 sectors (256 KB = 262144 bytes)
  .RBF data must be swapped (0..7 -> 7..0) before writing ro EPCS

  -----------------------------------------------------------------------------
*/

#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#include "jvme.h"

/* Include MPD definitions */
#include "mpdLib.h"

int
main(int argc, char *argv[])
{

  int fnMPD;

  if(vmeOpenDefaultWindows()!=OK)
    {
      printf("ERROR opening default VME windows\n");
      goto CLOSE;
    }


  // discover MPDs and initialize memory mapping
  mpdInit(0x80000,0x80000,21,MPD_INIT_NO_CONFIG_FILE_CHECK);
  fnMPD = mpdGetNumberMPD();

  if (fnMPD<=0) // test all possible vme slot ?
    {
      printf("ERR: no MPD discovered, cannot continue\n");
      goto CLOSE;
    }

  printf("%s: MPD discovered = %d\n",__FUNCTION__, fnMPD);
  mpdGStatus(0);

  int impd;
  printf("Check EPCS\n");
  for (impd = 0; impd < fnMPD; impd++) // only active mpd set
    {
      mpdASMI_reset(mpdSlot(impd));

      printf(" %2d: ", mpdSlot(impd));

      int id = mpdASMI_rdid(mpdSlot(impd));
      if( id != 0x18)
	{
	  printf("Bad EPCS ID (0x%x)\n", id);
	}
      else
	{
	  printf("OK! (0x%x)\n", id);
	}

      /* Re-enable fiber mode */
      mpdFiberEnable(mpdSlot(impd));
    }

 CLOSE:
  vmeCloseDefaultWindows();
  exit(0);
}

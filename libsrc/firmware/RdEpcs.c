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

#ifdef VXWORKS
#include <vxWorks.h>
#include "vxCompat.h"
#else
#include <stddef.h>
#include <pthread.h>
#endif
#include <stdio.h>
#include <string.h>
#ifdef VXWORKS
#include <logLib.h>
#include <taskLib.h>
#include <intLib.h>
#include <iv.h>
#include <semLib.h>
#include <vxLib.h>
#else
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#endif
#include "jvme.h"

/* Include MPD definitions */
#include "mpdLib.h"

#ifdef VXWORKS
#define MPDLOCK
#define MPDUNLOCK
#else
/* Mutex to guard flexio read/writes */
pthread_mutex_t mpdMutex = PTHREAD_MUTEX_INITIALIZER;
#define MPDLOCK      if(pthread_mutex_lock(&mpdMutex)<0) perror("pthread_mutex_lock");
#define MPDUNLOCK    if(pthread_mutex_unlock(&mpdMutex)<0) perror("pthread_mutex_unlock");
#endif


#include <stdarg.h>
#include <time.h>
#include <signal.h>
#include <math.h>

#include <sys/time.h>		/* for gettimeofday() */

int swap(int);

int
main(int argc, char *argv[])
{

  int i, j, k, d, last, ii;
  uint32_t b;
  char *endptr;
  int fnMPD;
  int slot;


  slot = 0;
  last = 0;
  b = 0;
  while ((i = getopt(argc, argv, "hls:")) != -1)
    switch (i)
      {
      case 'l':
	last = 1;
	break;
      case 's':
	slot = atoi(optarg);
	break;
      case 'h':
	printf
	  ("Usage: RdAsmi [-l] [-s slot] base_addr\nbase_addr can be hex (with 0x), l show last bytes of image\n\n");
	break;
      default:
	break;
      }
  if (optind >= argc)
    {
      printf("Expected base_addr after options\n");
      exit(1);
    }
  b = strtoul(argv[optind], &endptr, 0);
  j = b;
  if (last == 1)
    j += 2118900;

  if (vmeOpenDefaultWindows() != OK)
    {
      printf("ERROR opening default VME windows\n");
      goto CLOSE;
    }


// discover MPDs and initialize memory mapping
  mpdInit(0x80000, 0x80000, 21, MPD_INIT_NO_CONFIG_FILE_CHECK);
  fnMPD = mpdGetNumberMPD();

  if (fnMPD <= 0)		// test all possible vme slot ?
    {
      printf("ERR: no MPD discovered, cannot continue\n");
      goto CLOSE;
    }

  printf("%s: MPD discovered = %d\n", __FUNCTION__, fnMPD);
  for (k = 0; k < fnMPD; k++)	// only active mpd set
    {
      i = mpdSlot(k);
      if (i == slot)
	{
	  mpdASMI_reset(i);
	  printf("EPCS ID = 0x%x\n", mpdASMI_rdid(i));
	  printf("EPCS Status = 0x%02x\n", mpdASMI_rdstatus(i));

	  for (ii = j; ii < (j + 100); ii++)
	    {
	      d = mpdASMI_rd(i, ii);	// bits are swapped respect to .rbf!
	      printf("EPCS data[0x%08x] = 0x%02x (swap = 0x%02x)", ii, d,
		     swap(d));
	      if (ii == (b + 2118977))
		printf(" *** Last word ***");
	      printf("\n");
	    }
//                      printf("EPCS Status = 0x%02x\n", mpdASMI_rdstatus(i));
//                      mpdASMI_sec_erase(i, j);
//                      mpdASMI_wr(i, j, 0xAA);
//                      printf("EPCS data[0x%08x] = 0x%02x\n", j, mpdASMI_rd(i, j));
	}
    }

CLOSE:
  vmeCloseDefaultWindows();
  exit(0);
}

/* Swaps bit 0..7 to 7..0 in a byte */
int
swap(int x)
{
  int i, z = 0;
  for (i = 0; i < 8; i++)
    {
      z <<= 1;
      z |= (x >> i) & 1;
    }
  return z;
}

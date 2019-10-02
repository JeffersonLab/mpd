/*
    -----------------------------------------------------------------------------

                   --- I.N.F.N. Genova - Electronic Department ---

    -----------------------------------------------------------------------------

    Name		:	ProgramEpcs.cpp

    Project		:       MPD

    Description 	:	Standalone EPCS128 programmer using a SIS3104

    Date		:	July 2017
    Release		:	1.0
    Author		:	Paolo Musico



    -----------------------------------------------------------------------------

EP1AGX60 bit stream length = 16,951,824 bits = 2118978 bytes .RBF file size
.RBF differs from effective image by 5 bytes, so it will not work! (QII 13.0sp1)
Use .JIC instead, starting af file offset of 176: this works
EPCS128 = 128 Mbit = 134,217,728 bit = 64 sectors (256 KB = 262144 bytes)
.RBF/.JIC data bit must be swapped (0..7 -> 7..0) before writing ro EPCS

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


#define FACTORY_IMAGE_BASE_ADDR	0	/* Base address for Factory Fpga */
#define USER_IMAGE_BASE_ADDR	0x300000	/* Base address for User Fpga  */
#define BITSTREAM_LEN		2118978	/* bytes */
#define BITSTREAM_OFFSET	176	/* bytes */


int swap(int);
void progress(int);

int
main(int argc, char *argv[])
{

  int errcnt, cnt, i, epcs_id, slot, verify_only, erase_only, factory;
  uint32_t epcs_addr;
  char x, y, ans[10];
  FILE *fin = NULL;
  int fnMPD;
  int k;

  slot = 0;
  verify_only = 0;
  erase_only = 0;
  factory = 0;
  while ((i = getopt(argc, argv, "ehvfs:")) != -1)
    switch (i)
      {
      case 'e':
	erase_only = 1;
	break;
      case 'v':
	verify_only = 1;
	break;
      case 'f':
	factory = 1;
	break;
      case 's':
	slot = atoi(optarg);
	break;
      case 'h':
	printf("Usage: ProgramEpcs [-s slot] [-e] [-v] filename.jic\n");
	printf("use -v to verify only\n\n");
	printf("use -e to sector erase only\n\n");
	exit(1);
	break;
      default:
	break;
      }
  if (optind >= argc && erase_only == 0)
    {
      printf("Expected filename.jic after options\n");
      exit(1);
    }

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

	  printf("\n\nProgramming EPCS128 on MPD in slot %d\n", slot);

	  mpdASMI_reset(i);
	  epcs_id = mpdASMI_rdid(i);
	  if (epcs_id != 0x18)
	    {
	      printf("EPCS ID = 0x%02x, expected = 0x18\n", epcs_id);
	      printf("Exiting...\n");
	      goto CLOSE;
	    }

	  if (erase_only == 0)
	    {
	      if ((fin = fopen(argv[optind], "rb")) == NULL)
		{
		  printf("Can't open input .jic file '%s'\n", argv[optind]);
		  printf("Exiting...\n");
		  goto CLOSE;
		}
	      fseek(fin, BITSTREAM_OFFSET, SEEK_SET);
	    }

	  if (verify_only == 0)
	    {
	      if (factory)
		{
		  printf
		    ("You are programming factory sectors. Failing could not start the board!!!\n");
		  printf("Are you sure to do that? ");
		  scanf("%s", ans);
		  if (ans[0] != 'y' && ans[0] != 'Y')
		    goto CLOSE;
		  epcs_addr = FACTORY_IMAGE_BASE_ADDR;
		}
	      else
		epcs_addr = USER_IMAGE_BASE_ADDR;
	      printf("Erasing 10 sectors starting at address 0x%08x ",
		     epcs_addr);
	      fflush(stdout);
	      for (cnt = 0; cnt < 10; cnt++)
		{
		  mpdASMI_sec_erase(i, epcs_addr + (cnt << 18));
		  printf(".");
		  fflush(stdout);
		}
	      printf(" Done.\n");
	      if (erase_only == 1)
		goto CLOSE;

	      printf("Programming EPCS128 starting at address 0x%08x\n",
		     epcs_addr);
	      for (cnt = 0; cnt < BITSTREAM_LEN; cnt++)
		{

		  fread(&x, 1, 1, fin);
		  mpdASMI_wr(i, epcs_addr++, swap(x));
		  progress(cnt);
		}
	      printf("\n");
	      printf("Programming done!\n");

	    }
	  printf("\nVeryfing...\n");
	  fseek(fin, BITSTREAM_OFFSET, SEEK_SET);
	  errcnt = 0;
	  if (factory)
	    epcs_addr = FACTORY_IMAGE_BASE_ADDR;
	  else
	    epcs_addr = USER_IMAGE_BASE_ADDR;
	  for (cnt = 0; cnt < BITSTREAM_LEN; cnt++)
	    {

	      y = swap(mpdASMI_rd(i, epcs_addr++));
	      fread(&x, 1, 1, fin);
	      if (x != y)
		{
		  errcnt++;
		  printf
		    ("Error %d: at addr 0x%06x got 0x%02x, expected 0x%02x\n",
		     errcnt, epcs_addr - 1, y & 0xff, x & 0xff);
		}
	      progress(cnt);
	      if (errcnt > 100)
		{
		  printf("\nFound more than %d error(s)\n", errcnt);
		  break;
		}
	    }
	  printf("\n");
	  if (errcnt)
	    printf("Found %d error(s)\n", errcnt);
	  else
	    printf("Verify OK!\n");

	  fclose(fin);
	}
    }
CLOSE:
  vmeCloseDefaultWindows();
  exit(0);
}

void
progress(int c)
{
  if (((c % 6000) == 0) && c)
    {
      printf(".");
      fflush(stdout);
    }
  if (((c % 300000) == 0) && c)
    {
      printf("\n");
      fflush(stdout);
    }
}

/* Swaps bit 0..7 to 7..0 in a byte */
int
swap(int x)
{
  int i;
  int z = 0;
  for (i = 0; i < 8; i++)
    {
      z <<= 1;
      z |= (x >> i) & 1;
    }
  return z;
}

/*
// ASMI Commands
void asmi_reset(void)
{
	uint32_t x;

	x = 7;
	mpdWrite32(BoardBaseAddr+ASMI_CMD_ADDR, x);
	x = 0x80000000;
	while( x & 0x80000000 )
		x = mpdRead32(BoardBaseAddr+ASMI_CMD_ADDR);
}

int asmi_rdid(void)
{
	uint32_t x, id;

	x = 1;
	mpdWrite32(BoardBaseAddr+ASMI_CMD_ADDR, x);
	x = 0x80000000;
	while( x & 0x80000000 )
		x = mpdRead32(BoardBaseAddr+ASMI_CMD_ADDR);
	id = ((x&0x0000FF00) >> 8);
	return id;
}

int asmi_rdstatus(void)
{
	uint32_t x, status;

	x = 5;
	mpdWrite32(BoardBaseAddr+ASMI_CMD_ADDR, x);
	x = 0x80000000;
	while( x & 0x80000000 )
		x = mpdRead32(BoardBaseAddr+ASMI_CMD_ADDR);
	status = ((x&0x00FF0000) >> 16);
	return status;
}

void asmi_sec_erase(uint32_t addr)	// addr must be in the sector to erase
{
	uint32_t x;

	x = (addr&0x00FFFFFF)<<8;
//printf("asmi_sec_erase(): x = 0x%08x\n", x);
	mpdWrite32(BoardBaseAddr+ASMI_DATA_ADDR, x);
	x = 4;
	mpdWrite32(BoardBaseAddr+ASMI_CMD_ADDR, x);
	x = 0x80000000;
	while( x & 0x80000000 )
		x = mpdRead32(BoardBaseAddr+ASMI_CMD_ADDR);
}

int asmi_rd(uint32_t addr)
{
	uint32_t x;

	x = (addr&0x00FFFFFF)<<8;
//printf("asmi_rd(): x = 0x%08x\n", x);
	mpdWrite32(BoardBaseAddr+ASMI_DATA_ADDR, x);
	x = 2;
	mpdWrite32(BoardBaseAddr+ASMI_CMD_ADDR, x);
	x = 0x80000000;
	while( x & 0x80000000 )
		x = mpdRead32(BoardBaseAddr+ASMI_CMD_ADDR);
	return (x&0xFF);
}

void asmi_wr(uint32_t addr, uint32_t data)
{
	uint32_t x;

	x = ((addr&0x00FFFFFF)<<8) | (data&0xFF);
//printf("asmi_wr(): x = 0x%08x\n", x);
	mpdWrite32(BoardBaseAddr+ASMI_DATA_ADDR, x);
	x = 3;
	mpdWrite32(BoardBaseAddr+ASMI_CMD_ADDR, x);
	x = 0x80000000;
	while( x & 0x80000000 )
		x = mpdRead32(BoardBaseAddr+ASMI_CMD_ADDR);
}
*/

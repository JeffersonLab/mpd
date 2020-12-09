/*
    -----------------------------------------------------------------------------

                   --- I.N.F.N. Genova - Electronic Department ---

    -----------------------------------------------------------------------------

    Name		:	RunUserFpga.cpp

    Project		:	MPD

    Description 	:	Run User FPGA configuration using a SIS3104

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
pthread_mutex_t   mpdMutex = PTHREAD_MUTEX_INITIALIZER;
#define MPDLOCK      if(pthread_mutex_lock(&mpdMutex)<0) perror("pthread_mutex_lock");
#define MPDUNLOCK    if(pthread_mutex_unlock(&mpdMutex)<0) perror("pthread_mutex_unlock");
#endif

#include <stdarg.h>
#include <time.h>
#include <signal.h>
#include <math.h>

#include <sys/time.h>	/* for gettimeofday() */


#define FACTORY_IMAGE_BASE_ADDR	0x000000
#define USER_IMAGE_BASE_ADDR	0x300000	/* Base address for User Fpga (factory image starts at 0) */




int main(int argc, char *argv[])
{

	int i, k, slot, factory, verify_only;
	int fnMPD;
	int zz;
	time_t compile_time;

	slot = 0;
	verify_only = 0;
	factory = 0;
	while( (i = getopt(argc,argv,"hvfs:")) != -1 )
		switch( i )
		{
			case 'v': verify_only = 1; break;
			case 'f': factory = 1; break;
			case 's': slot = atoi(optarg); break;
			case 'h': printf("Usage: RunUserFpga [-s slot] [-v] [-f]\n");
				  printf("use -v to verify only\nuse -f to revert to factory image\n\n");
				  exit(0); break;
			default: break;
		}

	if(vmeOpenDefaultWindows()!=OK)
	{
		printf("ERROR opening default VME windows\n");
		goto CLOSE;
	}


// Discover MPDs and do memory mapping
	mpdInit(0x80000,0x80000,21,MPD_INIT_NO_CONFIG_FILE_CHECK);
//	mpdInit(0x40000<<slot,0x80000,1,MPD_INIT_NO_CONFIG_FILE_CHECK);
	fnMPD = mpdGetNumberMPD();

	if (fnMPD<=0) // test all possible vme slot ?
	{
		printf("ERR: no MPD discovered in slot %d, cannot continue\n", slot);
		goto CLOSE;
	} 

	for(k=0; k<fnMPD; k++)
	{
		i = mpdSlot(k);
		if( i == slot )
		{
			if( verify_only == 0 )
			{
				if( factory )
				{
					printf("\n\nRunning Factory FPGA image on MPD in slot %d\n", slot);
					zz = mpdRUPD_reconfigure(i,0);
				}
				else
				{
					printf("\n\nRunning Application/User FPGA image on MPD in slot %d\n", slot);
					zz = mpdRUPD_reconfigure(i,(USER_IMAGE_BASE_ADDR>>16)&0x7F);
				}
				usleep(1000000);
				mpdInit(0x80000,0x80000,21,MPD_INIT_NO_CONFIG_FILE_CHECK);
//				mpdInit(0x40000<<slot,0x80000,1,MPD_INIT_NO_CONFIG_FILE_CHECK);
			}
// Verify
			zz = mpdRUPD_rd_param(i, 5);
			if( zz )
				printf("\n\nMPD FPGA in slot %d is running Application/User code\n", slot);
			else
				printf("\n\nMPD FPGA in slot %d is running Factory code\n", slot);


		printf(" MPD Slot %d - Firmware Revision ID = 0x%x\n", slot, mpdGetFpgaRevision(slot));
		compile_time = mpdGetFpgaCompileTime(slot);
		printf(" MPD Slot %d - Firmware Revision Time: %s\n", slot, ctime((const time_t *)&compile_time));
		}
	}
CLOSE:
	vmeCloseDefaultWindows();
	exit(0);
}

/*
int rupd_reconfigure(int pgm)
{
	uint32_t x;
	int par0, par5;

	rupd_setup(pgm);

	x = 0x80;
	mpdWrite32(BoardBaseAddr+RUPD_CMD_ADDR, x);
	x = 0x80000000;
	while( x & 0x80000000 )
	{
		usleep(1000);
		x = mpdRead32(BoardBaseAddr+RUPD_CMD_ADDR);
	}

	par0 = rupd_rd_param(0);
	par5 = rupd_rd_param(5);
	printf("par0 = %d, par5 = %d\n", par0, par5);
	return par5;
}

void rupd_setup(int pgm)
{
	rupd_wr_param(0, 4);	// Reconfigure from logic array signal
	usleep(20000);
	rupd_wr_param(3, 0);	// Watchdog disable
	usleep(20000);
	rupd_wr_param(5, pgm ? 1 : 0);	// Application(1) - Factory(0)
	usleep(20000);
	rupd_wr_param(4, pgm);	// Image page address
	usleep(20000);
}

void rupd_wr_param(int par, int val)
{
	uint32_t x;

	x = (val & 0x3FF) | (par & 0x7) << 16;
	mpdWrite32(BoardBaseAddr+RUPD_DATA_ADDR, x);
	x = 2;
	usleep(5000);
	mpdWrite32(BoardBaseAddr+RUPD_CMD_ADDR, x);
	x = 0x80000000;
	while( x & 0x80000000 )
	{
		usleep(1000);
		x = mpdRead32(BoardBaseAddr+RUPD_CMD_ADDR);
	}
}

int rupd_rd_param(int par)
{
	uint32_t x;

	x = (par & 0x7) << 16;
	mpdWrite32(BoardBaseAddr+RUPD_DATA_ADDR, x);
	x = 1;
	usleep(5000);
	mpdWrite32(BoardBaseAddr+RUPD_CMD_ADDR, x);
	x = 0x80000000;
	while( x & 0x80000000 )
	{
		usleep(1000);
		x = mpdRead32(BoardBaseAddr+RUPD_CMD_ADDR);
	}
	return x & 0x3FF;
}
*/

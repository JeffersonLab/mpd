/*----------------------------------------------------------------------------*/
/**
 * @mainpage
 * <pre>
 *  mpdLib.c
 *             - Driver library for the MultiPurpose Digitizer (MPD)
 *             using a VxWorks 5.5 (PPC) or Linux 2.6.18 (Intel) or
 *             later based single board computer.
 *
 *  Author: Bryan Moffit
 *          Jefferson Lab Data Acquisition Group
 *          November 2014
 *
 * </pre>
 *----------------------------------------------------------------------------*/

#ifdef VXWORKS
#include <vxWorks.h>
#include "vxCompat.h"
#else
#include <stddef.h>
#include <pthread.h>
#include "jvme.h"
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
#endif

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

/* Define external Functions */
#ifdef VXWORKS
IMPORT  STATUS sysBusToLocalAdrs(int, char *, char **);
IMPORT  STATUS intDisconnect(int);
IMPORT  STATUS sysIntEnable(int);
IMPORT  STATUS sysIntDisable(int);
IMPORT  STATUS sysVmeDmaDone(int, int);
IMPORT  STATUS sysVmeDmaSend(UINT32, UINT32, int, BOOL);

#define EIEIO    __asm__ volatile ("eieio")
#define SYNC     __asm__ volatile ("sync")
#endif

/* Define Interrupts variables */
BOOL              mpdIntRunning  = FALSE;                    /* running flag */
int               mpdIntID       = -1;                       /* id number of ADC generating interrupts */
LOCAL VOIDFUNCPTR mpdIntRoutine  = NULL;                     /* user interrupt service routine */
LOCAL int         mpdIntArg      = 0;                        /* arg to user routine */
LOCAL UINT32      mpdIntLevel    = 0;         /* default VME interrupt level */
LOCAL UINT32      mpdIntVec      = 0;           /* default interrupt Vector */

/* Define global variables */
int nmpd = 0;                                       /* Number of MPDs in Crate */
int mpdA32Base   = 0x08000000;                      /* Minimum VME A32 Address for use by MPDs */
int mpdA32Offset = 0x08000000;                      /* Difference in CPU A32 Base - VME A32 Base */
int mpdA24Offset = 0x0;                             /* Difference in CPU A24 Base - VME A24 Base */
volatile struct mpd_struct *MPDp[(MPD_MAX_BOARDS+1)]; /* pointers to MPD memory map */
volatile uint32_t *MPDpd[(MPD_MAX_BOARDS+1)];      /* pointers to MPD FIFO memory */
volatile uint32_t *MPDpmb;                        /* pointer to Multblock window */
int mpdID[MPD_MAX_BOARDS];                           /* array of slot numbers for MPDs */
uint32_t mpdAddrList[MPD_MAX_BOARDS];            /* array of a24 addresses for MPDs */
int mpdRev[(MPD_MAX_BOARDS+1)];                      /* Board Revision Info for each module */
unsigned short mpdChanDisable[(MPD_MAX_BOARDS+1)];   /* Disabled Channel Mask for each Module*/
int mpdInited=0;                                    /* >0 if Library has been Initialized before */
int mpdMaxSlot=0;                                   /* Highest Slot hold an MPD */
int mpdMinSlot=0;                                   /* Lowest Slot holding an MPD */
int mpdSource=0;                                    /* Signal source for MPD system control*/
int mpdBlockLevel=0;                                /* Block Level for ADCs */
int mpdIntCount = 0;                                /* Count of interrupts from MPD */
int mpdBlockError=0; /* Whether (>0) or not (0) Block Transfer had an error */
ApvParameters fApv[(MPD_MAX_BOARDS)+1][MPD_MAX_APV];
mpdParameters fMpd[(MPD_MAX_BOARDS)+1];
unsigned short fApvEnableMask[(MPD_MAX_BOARDS)+1];
int nApv[(MPD_MAX_BOARDS)+1];
int fReadDone[(MPD_MAX_BOARDS)+1];


/* Internal APV register addresses */
static const uint8_t ipre_addr = 0x20;
static const uint8_t ipcasc_addr = 0x22;
static const uint8_t ipsf_addr = 0x24;
static const uint8_t isha_addr = 0x26;
static const uint8_t issf_addr = 0x28;
static const uint8_t ipsp_addr = 0x2A;
static const uint8_t imuxin_addr = 0x2C;
static const uint8_t ispare_addr = 0x2E;
static const uint8_t ical_addr = 0x30;
static const uint8_t vfp_addr = 0x32;
static const uint8_t vfs_addr = 0x34;
static const uint8_t vpsp_addr = 0x36;
static const uint8_t cdrv_addr = 0x38;
static const uint8_t csel_addr = 0x3A;
static const uint8_t mode_addr = 0x02;
static const uint8_t latency_addr = 0x04;
static const uint8_t muxgain_addr = 0x06;
static const uint8_t error_addr = 0x00;

/* Static methods */
/* I2C */
static int  I2C_SendByte(int id, uint8_t byteval, int start);
static int  I2C_ReceiveByte(int id, uint8_t *byteval);
static int  I2C_SendStop(int id);
static int  I2C_SendNack(int id);

void mpdSetZeroLevel(int id, uint16_t level) { fMpd[id].fLevel_0 = level; }
int  mpdGetZeroLevel(int id) { return fMpd[id].fLevel_0; }
void mpdSetOneLevel(int id, uint16_t level) { fMpd[id].fLevel_1 = level; }
int  mpdGetOneLevel(int id) { return fMpd[id].fLevel_1; }

void mpdSetChannelMark(int id, int v) { fMpd[id].fChannelMark = v; } // channel mark 0-127 for frame alignment, mark>127 disabled
int  mpdGetChannelMark(int id) { return fMpd[id].fChannelMark; }

void mpdSetCommonNoiseSubtraction(int id, short val) { fMpd[id].fCommonNoiseSubtraction = val; };
short mpdGetCommonNoiseSubtraction(int id) { return fMpd[id].fCommonNoiseSubtraction; };

void mpdSetEventBuilding(int id, int val) { fMpd[id].fEventBuilding = val; };
int mpdGetEventBuilding(int id) { return fMpd[id].fEventBuilding; };
 
void mpdSetCommonOffset(int id, int val) { fMpd[id].fCommonOffset = val; };
int mpdGetCommonOffset(int id) { return fMpd[id].fCommonOffset; };

void mpdSetCalibLatency(int id, int val) { fMpd[id].fCalibLatency = val; };
int mpdGetCalibLatency(int id) { return fMpd[id].fCalibLatency; };

void mpdSetTriggerNumber(int id, int val) { fMpd[id].fTriggerNumber = val; };
int mpdGetTriggerNumber(int id) { return fMpd[id].fTriggerNumber; };

void mpdSetTriggerMode(int id, int lat, int num) {
  if( num == 0 )
    fMpd[id].fTriggerMode = MPD_TRIG_MODE_NONE;
  else {
    if (lat>0) { 
      fMpd[id].fTriggerMode = MPD_TRIG_MODE_CALIB;
    } else {
      if( num == 1 )
	fMpd[id].fTriggerMode = MPD_TRIG_MODE_APV;
      else
	if( num > 1 )
	  fMpd[id].fTriggerMode = MPD_TRIG_MODE_MULTI;
    }
  }
  mpdSetTriggerNumber(id, num);
  mpdSetCalibLatency(id, lat);
  printf("%s: Calib Latency = %d, Trigger Mode = 0x%x\n",
	  __FUNCTION__, lat, fMpd[id].fTriggerMode);
};
int mpdGetTriggerMode(int id) { return fMpd[id].fTriggerMode; };

void mpdSetAcqMode(int id, char *name) {
  fMpd[id].fAcqMode = 0; // disabled
  // FIXME: String comp here
  if( name == "ramtest" ) fMpd[id].fAcqMode = MPD_DAQ_RAM_TEST;
  if( name == "histo" ) fMpd[id].fAcqMode = MPD_DAQ_HISTO;
  if( name == "event" ) fMpd[id].fAcqMode = MPD_DAQ_EVENT;
  if( name == "process" ) fMpd[id].fAcqMode = MPD_DAQ_PROCESS;
  if( name == "sample" ) fMpd[id].fAcqMode = MPD_DAQ_SAMPLE;
  if( name == "sync" ) fMpd[id].fAcqMode = MPD_DAQ_SYNC;

  printf("%s: Acquisition Mode = 0x%x (%s)\n",
	 __FUNCTION__,fMpd[id].fAcqMode, name);
};
int mpdGetAcqMode(int id) { return fMpd[id].fAcqMode; };

  
void mpdSetInPath0(int id, int t1P0, int t2P0, int tFront, int sP0, int sFront) {
  fMpd[id].fInPath[MPD_IN_P0][MPD_IN_TRIG1] = t1P0;
  fMpd[id].fInPath[MPD_IN_P0][MPD_IN_TRIG2] = t2P0;
  fMpd[id].fInPath[MPD_IN_FRONT][MPD_IN_TRIG] = tFront;
  fMpd[id].fInPath[MPD_IN_P0][MPD_IN_SYNC] = sP0;
  fMpd[id].fInPath[MPD_IN_FRONT][MPD_IN_SYNC] = sFront;
};
void mpdSetInPath(int id, int conn, int signal, int val) { fMpd[id].fInPath[conn%2][signal%3] = val; };
int  mpdGetInPath(int id, int conn, int signal) { return fMpd[id].fInPath[conn%2][signal%3]; };
int  mpdGetInPathI(int id, int conn, int signal) { return ((fMpd[id].fInPath[conn%2][signal%3] == 1) ? 1 : 0); };

uint32_t mpdGetFpgaRevision(int id) {
  if (fMpd[id].FpgaRevision == 99999) {
    printf("%s: Fpga revision not set yet, something wrong! exit(%d)",
	   __FUNCTION__,0);
/*     exit(0); */
  }
  return fMpd[id].FpgaRevision; 
}
void mpdSetFpgaRevision(int id, uint32_t r) { fMpd[id].FpgaRevision = r; }

uint32_t mpdGetHWRevision(int id) {
  uint32_t d = mpdGetFpgaRevision(id);
  return ((d>>24)&0xff);
}

uint32_t mpdGetFWRevision(int id) {
  uint32_t d = mpdGetFpgaRevision(id);
  return (d&0xff);
}

uint32_t mpdGetFpgaCompileTime(int id) {return fMpd[id].FpgaCompileTime; }
void mpdSetFpgaCompileTime(int id, uint32_t t) { fMpd[id].FpgaCompileTime = t; }


/**
 * @defgroup Config Initialization/Configuration
 * @defgroup Status Status
 * @defgroup Readout Data Readout
 * @defgroup IntPoll Interrupt/Polling
 * @defgroup Deprec Deprecated - To be removed
 */

/**
 *  @ingroup Config
 *  @brief Initialize JLAB MPD Library. 
 *
 * @param addr
 *  - A24 VME Address of the MPD
 * @param addr_inc
 *  - Amount to increment addr to find the next MPD
 * @param nmpd
 *  - Number of times to increment
 *
 *  @param iFlag 18 bit integer
 * <pre>
 *      bit 16:  Exit before board initialization
 *             0 Initialize MPD (default behavior)
 *             1 Skip initialization (just setup register map pointers)
 *
 *      bit 17:  Use mpdAddrList instead of addr and addr_inc
 *               for VME addresses.
 *             0 Initialize with addr and addr_inc
 *             1 Use mpdAddrList 
 *
 *      bit 18:  Skip firmware check.  Useful for firmware updating.
 *             0 Perform firmware check
 *             1 Skip firmware check
 * </pre>
 *      
 *
 * @return OK, or ERROR if the address is invalid or a board is not present.
 */
#ifdef SKIPYOU
STATUS 
mpdInit(UINT32 addr, UINT32 addr_inc, int nmpd, int iFlag)
{
  int ii, res, errFlag = 0;
  int boardID = 0;
  int maxSlot = 1;
  int minSlot = 21;
  int trigSrc=0, clkSrc=0, srSrc=0;
  uint32_t rdata, laddr, laddr_inc, a32addr, a16addr=0;
  volatile struct mpd_struct *mpd;
  uint16_t sdata;
  int noBoardInit=0;
  int useList=0;
  int noFirmwareCheck=0;

  /* Check if we have already Initialized boards before */
  if((mpdInited>0) && (mpdID[0] != 0)) 
    {
#ifdef NOTSURE
      /* Hard Reset of all MPD boards in the Crate */
      for(ii=0;ii<nmpd;ii++) 
	{
	  vmeWrite32(&(MPDp[mpdID[ii]]->csr),MPD_CSR_HARD_RESET);
	}
      taskDelay(5);
#endif /* NOTSURE */
    }
  
  /* Check if we are to exit when pointers are setup */
  noBoardInit=(iFlag&MPD_INIT_SKIP)>>16;

  /* Check if we're initializing using a list */
  useList=(iFlag&MPD_INIT_USE_ADDRLIST)>>17;

  /* Are we skipping the firmware check? */
  noFirmwareCheck=(iFlag&MPD_INIT_SKIP_FIRMWARE_CHECK)>>18;

  /* Check for valid address */
  if(addr==0) 
    {
      printf("mpdInit: ERROR: Must specify a Bus (VME-based A24) address for MPD 0\n");
      return(ERROR);
    }
  else if(addr > 0x00ffffff) 
    { /* A24 Addressing */
      printf("mpdInit: ERROR: A32 Addressing not allowed for MPD configuration space\n");
      return(ERROR);
    }
  else
    { /* A24 Addressing */
      if( ((addr_inc==0)||(nmpd==0)) && (useList==0) )
	nmpd = 1; /* assume only one MPD to initialize */

      /* get the MPD address */
#ifdef VXWORKS
      res = sysBusToLocalAdrs(0x39,(char *)addr,(char **)&laddr);
#else
      res = vmeBusToLocalAdrs(0x39,(char *)addr,(char **)&laddr);
#endif
      if (res != 0) 
	{
#ifdef VXWORKS
	  printf("mpdInit: ERROR in sysBusToLocalAdrs(0x39,0x%x,&laddr) \n",addr);
#else
	  printf("mpdInit: ERROR in vmeBusToLocalAdrs(0x39,0x%x,&laddr) \n",addr);
#endif
	  return(ERROR);
	}
      mpdA24Offset = laddr - addr;
    }

#ifdef NOTSURE
  /* Init Some Global variables */
  mpdSource = iFlag&MPD_SOURCE_MASK;
  mpdInited = nmpd = 0;
  bzero((char *)mpdChanDisable,sizeof(mpdChanDisable));
  bzero((char *)mpdID,sizeof(mpdID));
#endif /* NOTSURE */

  for (ii=0;ii<nmpd;ii++) 
    {
      if(useList==1)
	{
	  laddr_inc = mpdAddrList[ii] + mpdA24Offset;
	}
      else
	{
	  laddr_inc = laddr +ii*addr_inc;
	}
      mpd = (struct mpd_struct *)laddr_inc;

#ifdef NOTSURE
      /* Check if Board exists at that address */
#ifdef VXWORKS
      res = vxMemProbe((char *) &(mpd->version),VX_READ,4,(char *)&rdata);
#else
      res = vmeMemProbe((char *) &(mpd->version),4,(char *)&rdata);
#endif
      if(res < 0) 
	{
#ifdef VXWORKS
	  printf("mpdInit: WARN: No addressable board at addr=0x%x\n",(UINT32) mpd);
#else
	  printf("mpdInit: WARN: No addressable board at VME (Local) addr=0x%x (0x%x)\n",
		 (UINT32) laddr_inc-mpdA24Offset, (UINT32) mpd);
#endif
	  errFlag = 1;
	  continue;
	}
      else 
	{
	  /* Check that it is an MPD board */
	  if((rdata&MPD_BOARD_MASK) != MPD_BOARD_ID) 
	    {
	      printf("%s: WARN: For board at 0x%x, Invalid Board ID: 0x%x\n",
		     __FUNCTION__,
		     (UINT32) mpd-mpdA24Offset, rdata);
	      continue;
	    }
	  else 
	    {

	      /* Check if this is board has a valid slot number */
	      boardID =  ((vmeRead32(&(mpd->intr)))&MPD_SLOT_ID_MASK)>>16;

	      if((boardID <= 0)||(boardID >21)) 
		{
		  printf("%s: ERROR: For Board at 0x%x,  Slot number is not in range: %d\n",
			 __FUNCTION__,(UINT32) mpd-mpdA24Offset, boardID);
		  continue;
		}

	      if(!noFirmwareCheck)
		{
		  /* Check Control FPGA firmware version */
		  if( (rdata&MPD_VERSION_MASK) < MPD_SUPPORTED_CTRL_FIRMWARE )
		    {
		      printf("%s: ERROR: Slot %2d: Control FPGA Firmware (0x%02x) not supported by this driver.\n",
			     __FUNCTION__,boardID, rdata & MPD_VERSION_MASK);
		      printf("\tUpdate to 0x%02x to use this driver.\n",MPD_SUPPORTED_CTRL_FIRMWARE);
		      continue;
		    }

		  /* Check Processing FPGA firmware version */
		  proc_version = 
		    (uint16_t)(vmeRead32(&mpd->adc_status[0]) & MPD_ADC_VERSION_MASK);

		  for(icheck=0; icheck<MPD_SUPPORTED_PROC_FIRMWARE_NUMBER; icheck++)
		    {
		      if(proc_version == supported_proc[icheck])
			proc_supported=1;
		    }

		  if(proc_supported==0)
		    {
		      printf("%s: ERROR: Slot %2d: Proc FPGA Firmware (0x%02x) not supported by this driver.\n",
			     __FUNCTION__,boardID, proc_version);
		      printf("\tSupported Proc Firmware:  ");
		      for(icheck=0; icheck<MPD_SUPPORTED_PROC_FIRMWARE_NUMBER; icheck++)
			{
			  printf("0x%02x ",supported_proc[icheck]);
			}
		      printf("\n");
		      continue;
		    }
		}
	      else
		{
		  /* Check Control FPGA firmware version */
		  if( (rdata&MPD_VERSION_MASK) < MPD_SUPPORTED_CTRL_FIRMWARE )
		    {
		      printf("%s: WARN: Slot %2d: Control FPGA Firmware (0x%02x) not supported by this driver (ignored).\n",
			     __FUNCTION__,boardID, rdata & MPD_VERSION_MASK);
		    }

		  /* Check Processing FPGA firmware version */
		  proc_version = 
		    (uint16_t)(vmeRead32(&mpd->adc_status[0]) & MPD_ADC_VERSION_MASK);

		  for(icheck=0; icheck<MPD_SUPPORTED_PROC_FIRMWARE_NUMBER; icheck++)
		    {
		      if(proc_version == supported_proc[icheck])
			proc_supported=1;
		    }

		  if(proc_supported==0)
		    {
		      printf("%s: WARN: Slot %2d: Proc FPGA Firmware (0x%02x) not supported by this driver (ignored).\n",
			     __FUNCTION__,boardID, proc_version & MPD_VERSION_MASK);
		    }
		}

	      MPDp[boardID] = (struct mpd_struct *)(laddr_inc);
	      mpdRev[boardID] = rdata&MPD_VERSION_MASK;
	      mpdProcRev[boardID] = proc_version;
	      mpdID[nmpd] = boardID;
	      if(boardID >= maxSlot) maxSlot = boardID;
	      if(boardID <= minSlot) minSlot = boardID;
	      
	      printf("Initialized MPD %2d  Slot #%2d at VME (Local) address 0x%06x (0x%08x) \n",
		     nmpd,mpdID[nmpd],
		     (UINT32) MPDp[(mpdID[nmpd])]-mpdA24Offset,
		     (UINT32) MPDp[(mpdID[nmpd])]);
	    }
	  nmpd++;
	}
    }

  /* Hard Reset of all MPD boards in the Crate */
  if(!noBoardInit)
    {
      for(ii=0;ii<nmpd;ii++) 
	{
	  vmeWrite32(&(MPDp[mpdID[ii]]->reset),MPD_RESET_ALL);
	}
      taskDelay(60); 
    }

  /* Initialize Interrupt variables */
  mpdIntID = -1;
  mpdIntRunning = MPDLSE;
  mpdIntLevel = MPD_VME_INT_LEVEL;
  mpdIntVec = MPD_VME_INT_VEC;
  mpdIntRoutine = NULL;
  mpdIntArg = 0;

  /* Calculate the A32 Offset for use in Block Transfers */
#ifdef VXWORKS
  res = sysBusToLocalAdrs(0x09,(char *)mpdA32Base,(char **)&laddr);
  if (res != 0) 
    {
      printf("mpdInit: ERROR in sysBusToLocalAdrs(0x09,0x%x,&laddr) \n",mpdA32Base);
      return(ERROR);
    } 
  else 
    {
      mpdA32Offset = laddr - mpdA32Base;
    }
#else
  res = vmeBusToLocalAdrs(0x09,(char *)mpdA32Base,(char **)&laddr);
  if (res != 0) 
    {
      printf("mpdInit: ERROR in vmeBusToLocalAdrs(0x09,0x%x,&laddr) \n",mpdA32Base);
      return(ERROR);
    } 
  else 
    {
      mpdA32Offset = laddr - mpdA32Base;
    }
#endif


  for(ii=0;ii<nmpd;ii++) 
    {
    
      /* Program an A32 access address for this MPD's FIFO */
      a32addr = mpdA32Base + ii*MPD_MAX_A32_MEM;
#ifdef VXWORKS
      res = sysBusToLocalAdrs(0x09,(char *)a32addr,(char **)&laddr);
      if (res != 0) 
	{
	  printf("mpdInit: ERROR in sysBusToLocalAdrs(0x09,0x%x,&laddr) \n",a32addr);
	  return(ERROR);
	}
#else
      res = vmeBusToLocalAdrs(0x09,(char *)a32addr,(char **)&laddr);
      if (res != 0) 
	{
	  printf("mpdInit: ERROR in vmeBusToLocalAdrs(0x09,0x%x,&laddr) \n",a32addr);
	  return(ERROR);
	}
#endif
      MPDpd[mpdID[ii]] = (uint32_t *)(laddr);  /* Set a pointer to the FIFO */
      if(!noBoardInit)
	{
	  vmeWrite32(&(MPDp[mpdID[ii]]->adr32),(a32addr>>16) + 1);  /* Write the register and enable */
	
	  /* Set Default Block Level to 1 */
	  vmeWrite32(&(MPDp[mpdID[ii]]->blk_level),1);
	}
      mpdBlockLevel=1;

    }

  /* If there are more than 1 MPD in the crate then setup the Muliblock Address
     window. This must be the same on each board in the crate */
  if(nmpd > 1) 
    {
      a32addr = mpdA32Base + (nmpd+1)*MPD_MAX_A32_MEM; /* set MB base above individual board base */
#ifdef VXWORKS
      res = sysBusToLocalAdrs(0x09,(char *)a32addr,(char **)&laddr);
      if (res != 0) 
	{
	  printf("mpdInit: ERROR in sysBusToLocalAdrs(0x09,0x%x,&laddr) \n",a32addr);
	  return(ERROR);
	}
#else
      res = vmeBusToLocalAdrs(0x09,(char *)a32addr,(char **)&laddr);
      if (res != 0) 
	{
	  printf("mpdInit: ERROR in vmeBusToLocalAdrs(0x09,0x%x,&laddr) \n",a32addr);
	  return(ERROR);
	}
#endif
      MPDpmb = (uint32_t *)(laddr);  /* Set a pointer to the FIFO */
      if(!noBoardInit)
	{
	  for (ii=0;ii<nmpd;ii++) 
	    {
	      /* Write the register and enable */
	      vmeWrite32(&(MPDp[mpdID[ii]]->adr_mb),
			 (a32addr+MPD_MAX_A32MB_SIZE) + (a32addr>>16) + MPD_A32_ENABLE);
	    }
	}    
      /* Set First Board and Last Board */
      mpdMaxSlot = maxSlot;
      mpdMinSlot = minSlot;
      if(!noBoardInit)
	{
	  vmeWrite32(&(MPDp[minSlot]->ctrl1),
		     vmeRead32(&(MPDp[minSlot]->ctrl1)) | MPD_FIRST_BOARD);
	  vmeWrite32(&(MPDp[maxSlot]->ctrl1),
		     vmeRead32(&(MPDp[maxSlot]->ctrl1)) | MPD_LAST_BOARD);
	}    
    }

  if(!noBoardInit)
    mpdInited = nmpd;

#endif /* NOTSURE */

  if(errFlag > 0) 
    {
      printf("mpdInit: WARN: Unable to initialize all requested MPD Modules (%d)\n",
	     nmpd);
      if(nmpd > 0)
	printf("mpdInit: %d MPD(s) successfully initialized\n",nmpd );
      return(ERROR);
    } 
  else 
    {
      return(OK);
    }
}
#endif /*SKIPYOU*/

int
mpdCheckAddresses(int id)
{
  uint32_t offset=0, expected=0, base=0;
  
  if(id==0) id=mpdID[0];

  if((id<=0) || (id>21) || (MPDp[id] == NULL)) 
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return ERROR;
    }

  printf("%s:\n\t ---------- Checking mpd250 address space ---------- \n",__FUNCTION__);

  base = (uint32_t) &MPDp[id]->SdramFifo[0];

  offset = ((uint32_t) &MPDp[id]->AdcConfig) - base;
  expected = 0x01000000;
  if(offset != expected)
    printf("%s: ERROR MPDp[id]->AdcConfig not at offset = 0x%x (@ 0x%x)\n",
	   __FUNCTION__,expected,offset);

  offset = ((uint32_t) &MPDp[id]->I2C.Clock_Prescaler_low) - base;
  expected = 0x02000000;
  if(offset != expected)
    printf("%s: ERROR MPDp[id]->I2C.Clock_Prescaler_low not at offset = 0x%x (@ 0x%x)\n",
	   __FUNCTION__,expected,offset);

  offset = ((uint32_t) &MPDp[id]->Histo.block[0]) - base;
  expected = 0x03000000;
  if(offset != expected)
    printf("%s: ERROR MPDp[id]->Histo.block[0] not at offset = 0x%x (@ 0x%x)\n",
	   __FUNCTION__,expected,offset);

  offset = ((uint32_t) &MPDp[id]->ApvDaq.Data_Ch[0][0]) - base;
  expected = 0x04000000;
  if(offset != expected)
    printf("%s: ERROR MPDp[id]->ApvDaq.Data_Ch[0][0] not at offset = 0x%x (@ 0x%x)\n",
	   __FUNCTION__,expected,offset);

  offset = ((uint32_t) &MPDp[id]->SdramChip0[0]) - base;
  expected = 0x05000000;
  if(offset != expected)
    printf("%s: ERROR MPDp[id]->SdramChip0[0] not at offset = 0x%x (@ 0x%x)\n",
	   __FUNCTION__,expected,offset);

  offset = ((uint32_t) &MPDp[id]->SdramChip1[0]) - base;
  expected = 0x06000000;
  if(offset != expected)
    printf("%s: ERROR MPDp[id]->SdramChip1[0] not at offset = 0x%x (@ 0x%x)\n",
	   __FUNCTION__,expected,offset);

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Convert an index into a slot number, where the index is
 *          the element of an array of MPDs in the order in which they were
 *          initialized.
 *
 * @param i Initialization number
 * @return Slot number if Successfull, otherwise ERROR.
 *
 */
int
mpdSlot(uint32_t i)
{
  if(i>=nmpd)
    {
      printf("%s: ERROR: Index (%d) >= MPDs initialized (%d).\n",
	     __FUNCTION__,i,nmpd);
      return ERROR;
    }

  return mpdID[i];
}

int 
mpdLM95235_Read(int id, int *t)
{
  const uint8_t LM95235_i2c_addr  = 0x4C;
  //  const uint8_t Local_TempS_MSB_addr  = 0x00;	// Read only
  //  const uint8_t Local_TempS_LSB_addr  = 0x30;	// Read only
  //  const uint8_t Remote_TempS_MSB_addr  = 0x01;	// Read only
  //  const uint8_t Remote_TempS_LSB_addr  = 0x10;	// Read only
  const uint8_t Remote_TempU_MSB_addr  = 0x31;	// Read only
  const uint8_t Remote_TempU_LSB_addr  = 0x32;	// Read only
  //  const uint8_t ConfigReg2_addr  = 0xBF;
  //  const uint8_t RemoteOffset_H_addr  = 0x11;
  //  const uint8_t RemoteOffset_L_addr  = 0x12;
  //  const uint8_t ConfigReg1_addr  = 0x03;	// also 0x09
  //  const uint8_t ConvRate_addr  = 0x04;	// also 0x0A
  const uint8_t OneShot_addr  = 0x0F;	// Write only
  const uint8_t Status1_addr  = 0x02;	// Read only
  //  const uint8_t Status2_addr  = 0x33;	// Read only
  //  const uint8_t ManufID_addr  = 0xFE;	// Read only, returns 0x01
  //  const uint8_t RevID_addr  = 0xFF;	// Read only

  uint8_t val;
  int success, retry_count;
  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<=0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  success = mpdI2C_ByteWrite(id, (uint8_t)(LM95235_i2c_addr<<1), OneShot_addr, 0, &val);
  success = mpdI2C_ByteWrite(id, (uint8_t)(LM95235_i2c_addr<<1), Status1_addr, 0, &val);
  val = 0x80;
  retry_count = 0;
  while( (val & 0x80) && (retry_count++ < 100) )
    success = mpdI2C_ByteRead1(id, (uint8_t)(LM95235_i2c_addr<<1), &val);
  success = mpdI2C_ByteWrite(id, (uint8_t)(LM95235_i2c_addr<<1), Remote_TempU_MSB_addr, 0, &val);
  success = mpdI2C_ByteRead1(id, (uint8_t)(LM95235_i2c_addr<<1), &val);
  *t = val;
  success = mpdI2C_ByteWrite(id, (uint8_t)(LM95235_i2c_addr<<1), Remote_TempU_LSB_addr, 0, &val);
  success = mpdI2C_ByteRead1(id, (uint8_t)(LM95235_i2c_addr<<1), &val);
  // aggiornare t
  return success;

}


/****************************
 * LOW LEVEL routines 
 ****************************/


/* I2C methods */
static int fI2CSpeed;
static int fI2CMaxRetry;

void mpdSetI2CSpeed(int val) { fI2CSpeed = val; };
int  mpdGetI2CSpeed(void) { return fI2CSpeed; };
void mpdSetI2CMaxRetry(int val) { fI2CMaxRetry = val; };
int  mpdGetI2CMaxRetry(void) { return fI2CMaxRetry; };

int 
mpdI2C_ApvReset(int id)
{
  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<=0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  MPDUNLOCK;
  vmeWrite32(&MPDp[id]->I2C.ApvReset, MPD_I2C_APVRESET_ASYNC_CLEAR);
  vmeWrite32(&MPDp[id]->I2C.ApvReset, MPD_I2C_APVRESET_ASYNC_SET);
  MPDUNLOCK;

  return OK;
}

int 
mpdI2C_Init(int id)
{
  uint32_t data, rdata;
  int success = OK;
  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<=0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }
  
  /* 
     clock_prescaler = prescaler_high * 256 + prescaler_low
     period = clock_prescale/10 us
  */

  
  MPDLOCK;
  vmeWrite32(&MPDp[id]->I2C.Control, 0); /* Disable I2C Core and interrupts */

  int ispeed = mpdGetI2CSpeed();

  data = (ispeed & 0xFF);
  vmeWrite32(&MPDp[id]->I2C.Clock_Prescaler_low, data);
  
  rdata = vmeRead32(&MPDp[id]->I2C.Clock_Prescaler_low);

  printf("%s: i2c low prescaler register set/read : %d / %d\n",
	 __FUNCTION__,data,rdata);

  data = (ispeed>>8) & 0xff;
  vmeWrite32(&MPDp[id]->I2C.Clock_Prescaler_high, data);
  rdata = vmeRead32(&MPDp[id]->I2C.Clock_Prescaler_high);

  printf("%s: i2c high prescaler register set/read : %d / %d\n",
	 __FUNCTION__,data,rdata);

  printf("%s: i2c speed prescale = %d, (period = %f us, frequency = %f kHz)\n",
	 __FUNCTION__,ispeed, ispeed/10., 10000./ispeed);
  
  vmeWrite32(&MPDp[id]->I2C.Control, MPD_I2C_CONTROL_ENABLE_CORE);

  MPDUNLOCK;

  return success;

}

int 
mpdI2C_ByteWrite(int id, uint8_t dev_addr, uint8_t int_addr, 
		 int ndata, uint8_t *data)
{
  int success, i;
  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<=0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }
  
  if( (success = I2C_SendByte(id, (uint8_t)(dev_addr & 0xFE), 1)) != OK )
    {
      if( success <= -10 )
	{
	  I2C_SendStop(id);
	  return success;
	}
      else
	return I2C_SendStop(id);
    }
  //  usleep(300);
  if( (success = I2C_SendByte(id, int_addr, 0)) != OK )
    {
      if( success <= -10 )
	{
	  I2C_SendStop(id);
	  return success;
	}
      else
	return I2C_SendStop(id);
    }
  //  usleep(300);
  for(i=0; i<ndata; i++)
    if( (success = I2C_SendByte(id, data[i], 0)) != OK )
      {
	if( success <= -10 )
	  {
	    I2C_SendStop(id);
	    return success;
	  }
	else
	  return I2C_SendStop(id);
      }
  //  usleep(300);
  success = I2C_SendStop(id);
  // usleep(300);
  return success;

}

int
mpdI2C_ByteRead(int id, uint8_t dev_addr, uint8_t int_addr, 
		int ndata, uint8_t *data)
{
  int success, i;
  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<=0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }
  
  if( (success = I2C_SendByte(id, (uint8_t)(dev_addr | 0x01), 1)) != OK )
    return I2C_SendStop(id);
  
  if( (success = I2C_SendByte(id, int_addr, 0)) != OK )
    return I2C_SendStop(id);
  
  for(i=0; i<ndata; i++)
    if( (success = I2C_ReceiveByte(id, data+i)) != OK )
      return I2C_SendStop(id);
  
  return I2C_SendStop(id);
}

int 
mpdI2C_ByteWriteRead(int id, uint8_t dev_addr, uint8_t int_addr, 
		     int ndata, uint8_t *data)
{
  int success, i;
  if((MPDp[id]==NULL) || (id<=0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }
  
  if( (success = I2C_SendByte(id, (uint8_t)(dev_addr & 0xFE), 1)) != OK )
    return I2C_SendStop(id);
  
  if( (success = I2C_SendByte(id, int_addr, 0)) != OK )
    return I2C_SendStop(id);
  
  for(i=0; i<ndata; i++)
    if( (success = I2C_ReceiveByte(id, data+i)) != OK )
      return I2C_SendStop(id);
  
  return I2C_SendStop(id);
}


int 
mpdI2C_ByteRead1(int id, uint8_t dev_addr, uint8_t *data)
{
  int success;
  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<=0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  
  if( (success = I2C_SendByte(id, (uint8_t)(dev_addr & 0xFE), 1)) != OK )
    return I2C_SendStop(id);
  
  if( (success = I2C_ReceiveByte(id, data)) != OK )
    return I2C_SendStop(id);
  
  return I2C_SendStop(id);
}

static int 
I2C_SendByte(int id, uint8_t byteval, int start)
{
  int rval=OK, retry_count;
  uint32_t data=0;
  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<=0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }


  MPDLOCK;

  vmeWrite32(&MPDp[id]->I2C.TxRx, byteval);
  
  if( start )
    vmeWrite32(&MPDp[id]->I2C.CommStat, MPD_I2C_COMMSTAT_START_WRITE);
  else
    vmeWrite32(&MPDp[id]->I2C.CommStat, MPD_I2C_COMMSTAT_WRITE);

  retry_count = 0;
  data = 0x00000002;
  while( (data & 0x00000002) != 0 && retry_count < mpdGetI2CMaxRetry() )
    {
      data = vmeRead32(&MPDp[id]->I2C.CommStat);
      retry_count++;
    }

  if( retry_count >= mpdGetI2CMaxRetry() )
    rval = -10;

  if( data & MPD_I2C_COMMSTAT_NACK_RECV )	/* NACK received */
    rval = -20;

  MPDLOCK;

  return rval;
}

static int 
I2C_ReceiveByte(int id, uint8_t *byteval)
{
  int retry_count;
  int rval=0;
  uint32_t data=0;
  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<=0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  MPDLOCK;
  vmeWrite32(&MPDp[id]->I2C.CommStat, MPD_I2C_COMMSTAT_READ);

  retry_count = 0;
  data = 0x00000002;
  while( (data & 0x00000002) != 0 && retry_count < mpdGetI2CMaxRetry() )
    {
      data = vmeRead32(&MPDp[id]->I2C.CommStat);
      retry_count++;
    }

  if( retry_count >= mpdGetI2CMaxRetry() )
    rval = -10;

  if( data & MPD_I2C_COMMSTAT_NACK_RECV )	/* NACK received */
    rval = -20;

  data = vmeRead32(&MPDp[id]->I2C.TxRx);
  
  *byteval = data;
  MPDUNLOCK;

  return rval;

}

static int 
I2C_SendStop(int id)
{
  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<=0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  MPDLOCK;
  vmeWrite32(&MPDp[id]->I2C.CommStat, MPD_I2C_COMMSTAT_STOP);
  MPDUNLOCK;

  return OK;
}

static int 
I2C_SendNack(int id)
{
  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<=0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }
  
  MPDLOCK;
  vmeWrite32(&MPDp[id]->I2C.CommStat, MPD_I2C_COMMSTAT_NACK);
  MPDUNLOCK;
  
  return OK;
}

/* ADC set/get methods */
void mpdSetAdcClockPhase(int id, int adc, int phase) { fMpd[id].fAdcClockPhase[adc] = phase; };
int  mpdGetAdcClockPhase(int id, int adc) { 
  int clock_phase;
  clock_phase = fMpd[id].fAdcClockPhase[adc];
  if( mpdGetFpgaRevision(id) < 2 ) { // no delay line
    switch (clock_phase) {
    case 0: 
      clock_phase = MPD_APV_CLOCK_PHASE_0;
      break;
    case 1:
    case 90:
      clock_phase = MPD_APV_CLOCK_PHASE_90;
      break;
    case 2:
    case 180:
      clock_phase = MPD_APV_CLOCK_PHASE_180;
      break;
    case 3:
    case 270:
      clock_phase = MPD_APV_CLOCK_PHASE_270;
      break;
    default: 
      clock_phase = MPD_APV_CLOCK_PHASE_0;
      printf("%s: Warning: apv_clock phase %d out of range for Fpga rev. %d", 
	     __FUNCTION__, clock_phase, mpdGetFpgaRevision(id));
    }
  }
  return clock_phase; 
};

void mpdSetAdcGain(int id, int adc, int ch, int g) {fMpd[id].fAdcGain[adc][ch] = g; };
int  mpdGetAdcGain(int id, int adc, int ch) {return fMpd[id].fAdcGain[adc][ch]; };
void mpdSetAdcInvert(int id, int adc, int val) {fMpd[id].fAdcInvert[adc] = val; };
int  mpdGetAdcInvert(int id, int adc) {return fMpd[id].fAdcInvert[adc]; };
void mpdSetAdcPattern(int id, int adc, int p) {fMpd[id].fAdcPattern[adc] = p; };
int  mpdGetAdcPattern(int id, int adc) {return fMpd[id].fAdcPattern[adc]; };


/* APV methods */
int  mpdApvGetLatency(int id, int ia) { return fApv[id][ia].Latency; };
int  mpdApvGetCalibrationMode(int id, int ia) { return (1 - ((fApv[id][ia].Mode >> 2) & 0x1)); };
int  mpdApvGetMode(int id, int ia) { return (fApv[id][ia].Mode &0x3F); };

int  mpdApvGetBufferSample(int id, int ia) {
  int ix = fApv[id][ia].fBi1 / EVENT_SIZE;
  printf("%s: Fifo = %d has %d samples (%d bytes) stored\n",
	 __FUNCTION__,fApv[id][ia].adc, ix, fApv[id][ia].fBi1);
  return ix;
  
};

int  mpdApvGetSample(int id, int ia) { return fApv[id][ia].fNumberSample; };

void mpdApvSetSampleLeft(int id, int ia) { fApv[id][ia].fReadCount = fApv[id][ia].fNumberSample; };
void mpdApvDecSampleLeft(int id, int ia, int n) { fApv[id][ia].fReadCount -= n; };
int  mpdApvReadDone(int id, int ia) { 
  if (fApv[id][ia].fReadCount <= 0) return TRUE; 
  return FALSE;
};
int  mpdApvGetSampleLeft(int id, int ia) { return (fApv[id][ia].fNumberSample - mpdApvGetBufferSample(id,ia)); };
int  mpdApvGetSampleIdx(int id, int ia) { return (fApv[id][ia].fNumberSample - fApv[id][ia].fReadCount); };

int
mpdAPV_Reset101(int id)
{
  uint32_t data;
  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<=0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  MPDLOCK;

  data = vmeRead32(&MPDp[id]->ApvDaq.Trig_Gen_Config);

  data |= MPD_APVDAQ_TRIGCONFIG_ENABLE_MACH;	// Enable trig machine
  data |= SOFTWARE_CLEAR_MASK;

  vmeWrite32(&MPDp[id]->ApvDaq.Trig_Gen_Config, data);

  data &= ~SOFTWARE_CLEAR_MASK;
  data &= ~MPD_APVDAQ_TRIGCONFIG_ENABLE_MACH;	// Disable trig machine

  vmeWrite32(&MPDp[id]->ApvDaq.Trig_Gen_Config, data);

  MPDUNLOCK;

  return OK;
}

/*
 * return true if apv is present
 */
int
mpdAPV_Try(int id, uint8_t apv_addr) // i2c addr
{
  uint8_t x = 0xEC;
  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<=0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }
  return ((mpdAPV_Read(id, apv_addr, mode_addr, &x) == OK) ? 1 : 0);
  //	return APV_Write(apv_addr, mode_addr, def_Mode);
}

void 
mpdSetApvEnableMask(int id, uint16_t mask) 
{ 
  fApvEnableMask[id] |= mask;
}

void 
mpdResetApvEnableMask(int id) 
{ 
  fApvEnableMask[id] = 0;
}

uint16_t
mpdGetApvEnableMask(int id) 
{ 
  return fApvEnableMask[id];
}


int
mpdAPV_Scan(int id) 
{
  int iapv=0;
  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<=0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }
  
  // print information (debug only, should be done in verbose mode only)
  printf("%s: Blind scan: \n",__FUNCTION__);

  for(iapv=0;iapv<MPD_MAX_APV;iapv++) 
    {
      if ( mpdAPV_Try(id, iapv) ) 
	{
	  printf("%s : APV i2c = %d found in MPD in slot %d\n",
		 __FUNCTION__, iapv, id);
	}
    }
  printf("%s: Blind scan done\n",__FUNCTION__);

  mpdResetApvEnableMask(id);

  for(iapv=0; iapv<MPD_MAX_APV; iapv++)
    {
      printf("%s: Try %2d %2d", __FUNCTION__, fApv[id][iapv].i2c, fApv[id][iapv].adc);
      
      if ( mpdAPV_Try(id, fApv[id][iapv].i2c && fApv[id][iapv].adc>-1 ) ) 
	{
	  printf("%s: %d matched in MPD in slot %d\n",
		 __FUNCTION__, fApv[id][iapv].i2c, id);

	  mpdSetApvEnableMask(id, (1 << fApv[id][iapv].adc));
	  printf("%s: APV enable mask 0x%04x\n", 
		 __FUNCTION__,mpdGetApvEnableMask(id));
	}
      else 
	{
	  printf("%s: APV i2c = %d does not respond.  It is removed from db\n",
		 __FUNCTION__,fApv[id][iapv].i2c);
	  fApvEnableMask[id] &= ~(1 << fApv[id][iapv].adc);
	}
    }   
      
  printf("%s: %d APV found matching settings\n",
	 __FUNCTION__,nApv[id]);

  return nApv[id];
}


int 
mpdAPV_Write(int id, uint8_t apv_addr, uint8_t reg_addr, uint8_t val)
{
  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<=0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  return mpdI2C_ByteWrite(id, (uint8_t)((0x20 | apv_addr)<<1), reg_addr, 1, &val);
}

int
mpdAPV_Read(int id, uint8_t apv_addr, uint8_t reg_addr, uint8_t *val)
{
  int success;
  uint8_t rval;

  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<=0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  usleep(500);

  success = mpdI2C_ByteWrite(id, (uint8_t)((0x20 | apv_addr)<<1), (reg_addr|0x01), 1, val);
  if( success != OK )
    return success;
  
  usleep(500);
  success = mpdI2C_ByteRead1(id, (uint8_t)((0x20 | apv_addr)<<1), &rval);

  printf("%s: Set / Get = 0x%x 0x%x\n",
	 __FUNCTION__,*val,rval);

  return success;
}

int
mpdAPV_Config(int id, int apv_index)
{
  int success, i;
  uint8_t apv_addr, reg_addr=0, val=0;

  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<=0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  apv_addr = fApv[id][apv_index].i2c;

  printf("APV card i2c=%d to ADC (fifo)=%d (from config file)\n",
	 (int) apv_addr, fApv[id][apv_index].adc);

  for (i=0;i<18;i++) 
    {
      switch (i) 
	{
	case 0:
	  reg_addr = mode_addr;
	  val = 0;
	  break;
	case 1:
	  reg_addr = ipre_addr;
	  val = fApv[id][apv_index].Ipre;
	  break;
	case 2:
	  reg_addr = ipcasc_addr;
	  val = fApv[id][apv_index].Ipcasc;
	  break;
	case 3:
	  reg_addr = ipsf_addr;
	  val = fApv[id][apv_index].Ipsf;
	  break;
	case 4:
	  reg_addr = isha_addr;
	  val = fApv[id][apv_index].Isha;
	  break;
	case 5:
	  reg_addr = issf_addr;
	  val = fApv[id][apv_index].Issf;
	  break;
	case 6:
	  reg_addr = ipsp_addr;
	  val = fApv[id][apv_index].Ipsp;
	  break;
	case 7:
	  reg_addr = imuxin_addr;
	  val = fApv[id][apv_index].Imuxin;
	  break;
	case 8:
	  reg_addr = ispare_addr;
	  val = fApv[id][apv_index].Ispare;
	  break;
	case 9:
	  reg_addr = ical_addr;
	  val = fApv[id][apv_index].Ical;
	  break;
	case 10:
	  reg_addr = vfp_addr;
	  val = fApv[id][apv_index].Vfp;
	  break;
	case 11:
	  reg_addr = vfs_addr;
	  val = fApv[id][apv_index].Vfs;
	  break;
	case 12:
	  reg_addr = vpsp_addr;
	  val = fApv[id][apv_index].Vpsp;
	  break;
	case 13:
	  reg_addr = cdrv_addr;
	  val = fApv[id][apv_index].Cdrv;
	  break;
	case 14:
	  reg_addr = csel_addr;
	  val = fApv[id][apv_index].Csel;
	  break;
	case 15:
	  reg_addr = latency_addr;
	  val = fApv[id][apv_index].Latency;
	  break;
	case 16:
	  reg_addr = muxgain_addr;
	  val = fApv[id][apv_index].Muxgain;
	  break;
	case 17:
	  reg_addr = mode_addr;
	  val = fApv[id][apv_index].Mode;
	  break;
      
	default:
	  printf("%s: ERROR: This message should not appear, please check code consistency",
		 __FUNCTION__);
	}

      usleep(300);
      success = mpdAPV_Write(id, apv_addr, reg_addr, val);
	
      if (success != OK) {
	printf("%s: ERROR: I2C Bus Error: i/addr/reg/val/err %d/ %d 0x%x 0x%x 0x%x",
	       __FUNCTION__, i, apv_addr, reg_addr, val, success);
	return success;
      }
    } // end loop

#ifdef DOTHISDIFFERENTLY
  fApv[id][apv_index].fNumberSample = GetTriggerNumber()*ApvGetPeakMode(); // must be set !!
  ApvBufferFree(apv_index);
  ApvBufferAlloc(apv_index); // alloc readout buffer
#endif /* DOTHISDIFFERENTLY */

  return success;

}

/**
 * Return the setting value of the number of samples per trigger (1 or 3)
 * this is the same for all Apvs 
 * return -1 if there are no APV connected
 */
int
mpdApvGetPeakMode(int id)
{

  int c=-1;
  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<=0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  if (nApv[id]>0) 
    {
      c = 3 - (fApv[id][0].Mode & 2);
    }
  
  return c;
}

void
mpdAddApv(int id, ApvParameters v) 
{ 
  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<=0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return;
    }


  fApv[id][nApv[id]] = v;
  fApv[id][nApv[id]].fBuffer = 0;
  
  printf("%s: APV %d added to list of FECs\n",
	 __FUNCTION__,
	 nApv[id]);
  nApv[id]++;
}

/*
 * Return the clock frequency of the APV (0 = 20 MHz, 1 = 40 MHz)
 * the frequency must be the same for all APV on the same MPD
 * return -1 in case of error (no APV on MPDs)
 */

int 
mpdApvGetFrequency(int id) 
{
  int c=0;
  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<=0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }


  if (nApv[id]>0) 
    {
      c = (fApv[id][0].Mode & 0x10) >> 4;
    }
  
  return c;
}

/**
 * Return max latency of all APVs in a single MPD
 */
uint8_t
mpdApvGetMaxLatency(int id) 
{
  uint8_t c=0;
  uint32_t iapv=0;
  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<=0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }
  
  for (iapv=0; iapv<nApv[id]; iapv++) 
    {
      c = (fApv[id][iapv].Latency > c) ? fApv[id][iapv].Latency : c;
    }
  return c;
}

/**
 * Set the number of samples / event expected from the single Apv
 */
int 
mpdArmReadout(int id) 
{
  uint32_t iapv=0;
  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<=0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  for (iapv=0; iapv<nApv[id]; iapv++) 
    {
      mpdApvSetSampleLeft(id, iapv); // improve peak mode
      fApv[id][iapv].fBi0 = 0; // begin of buffer (should be always 0)
      fApv[id][iapv].fBs = 0;  // end of event (last sample end mark)
      fApv[id][iapv].fBi1 = 0; // end of buffer
    }
  fReadDone[id] = FALSE;

  return 0;

}

//======================================================
// trigger methods

int 
mpdTRIG_BitSet(int id) 
{ 
  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<=0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }
/*
	uint32_t addr;
	uint32_t data;
	int success;

	addr = fBaseAddr + ApvFifoOffset;
	
	addr += 0x20000;
	addr += 0x28;	
	if( (success = BUS_Read(addr, &data)) != BUS_OK )
	  return success;
	data |= 0x40000000;
	return BUS_Write(addr, &data);
*/
	return 0;
}

int 
mpdTRIG_BitClear(int id) 
{
  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<=0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }
/*
	uint32_t addr;
	uint32_t data;
	int success;

	addr = fBaseAddr + ApvFifoOffset;
	
	addr += 0x20000;
	addr += 0x28;	
	if( (success = BUS_Read(addr, &data)) != BUS_OK )
	  return success;
	data &= 0xBFFFFFFF;
	return BUS_Write(addr, &data);
*/
	return 0;
}

int 
mpdTRIG_Enable(int id) 
{
  uint32_t data;

  uint8_t sync_period;
  uint8_t reset_latency;
  uint8_t mark_ch;
  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<=0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  mark_ch = (uint8_t) mpdGetChannelMark(id); // set one of the 128 channels of all apv to 0xfff (mark a single channel of the frame)

  sync_period = (mpdApvGetFrequency(id) == 1) ? 34 : 69; // synch period in number of clock - 1 (34 @ 40 MHz, 69 @ 20 MHz) @@@ To be checked, ask Paolo

  reset_latency = 15 + mpdApvGetMaxLatency(id); // @@@ To be ckecked, ask paolo for meaning

  data = mark_ch << 24 | sync_period << 16 | mpdGetApvEnableMask(id);

  MPDLOCK;
  vmeWrite32(&MPDp[id]->ApvDaq.Control, data);

  // FIXME: make sure there are no locks in here.
  if( mpdGetFpgaRevision(id) < 2 )
    data =  mpdGetAdcClockPhase(id, 0) | 	// This works only for ADC board rev 0
      ((mpdGetTriggerMode(id) & 0x07) << 12) |
      ((mpdGetTriggerNumber(id) & 0x0F) << 8) | reset_latency;
  else
    data = 
      ((mpdGetCalibLatency(id) & 0xFF) << 24) |
      ((mpdGetInPathI(id, MPD_IN_FRONT, MPD_IN_TRIG) & 0x01) << 23) |
      ((mpdGetInPathI(id, MPD_IN_P0, MPD_IN_TRIG2) & 0x01) << 22) |
      ((mpdGetInPathI(id, MPD_IN_P0, MPD_IN_TRIG1) & 0x01) << 21) |
      ((mpdGetInPathI(id, MPD_IN_FRONT, MPD_IN_SYNC) & 0x01) << 17) |
      ((mpdGetInPathI(id, MPD_IN_P0, MPD_IN_SYNC) & 0x01) << 16) |
      //	  ((test_mode & 0x01) << 15) |
      ((mpdGetTriggerMode(id) & 0x07) << 12) |
      ((mpdGetTriggerNumber(id) & 0x0F) << 8) | reset_latency;

  vmeWrite32(&MPDp[id]->ApvDaq.Trig_Gen_Config, data);

  data = (mpdGetOneLevel(id) << 16) | mpdGetZeroLevel(id);

  /*
    std::cout << __FUNCTION__ << " Trigger Mode : " << (trig_mode & 0x7) << std::endl;
    std::cout << __FUNCTION__ << " Trigger Front: " << (EnTrig_Front & 0x1) << std::endl;
    std::cout << __FUNCTION__ << " Number of Triggers: " << (max_trig_out & 0xF) << std::endl;
  */

  vmeWrite32(&MPDp[id]->ApvDaq.Logic_Thresholds, data);
  MPDUNLOCK;

  return OK;
}

int 
mpdTRIG_Disable(int id)
{
  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<=0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }
  
  MPDLOCK;
  vmeWrite32(&MPDp[id]->ApvDaq.Control, 0);

  vmeWrite32(&MPDp[id]->ApvDaq.Trig_Gen_Config, 0);
  MPDUNLOCK;

  return OK;

}

int 
mpdTRIG_GetMissed(int id, uint32_t *missed)
{
  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<=0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }
  
  MPDLOCK;
  *missed = vmeRead32(&MPDp[id]->ApvDaq.Missed_Trigger);
  MPDUNLOCK;

  return OK;
}

/**
 * Set the delay chip that define the time delay between adc_clock and apv_clock
 *
 */

int 
mpdDELAY25_Set(int id, int apv1_delay, int apv2_delay)
{
  uint8_t val;
  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<=0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  //      mpdI2C_ByteWrite(0xF0 , (apv2_delay & 0x3F) | 0x40, 0, &val);      // CR0: APV2 out
  mpdI2C_ByteWrite(id, 0xF0 , 0x40, 0, &val);    // CR0: APV2 out not delayed
  sleep(10);
  //      mpdI2C_ByteWrite(0xF2 , 0x40, 0, &val);    // CR1: ADC1 out not delayed
  mpdI2C_ByteWrite(id, 0xF2 , (apv1_delay & 0x3F) | 0x40, 0, &val);      // CR1: ADC1 clock delayed
  sleep(10);
  //      mpdI2C_ByteWrite(0xF4 , 0x40, 0, &val);    // CR2: ADC2 out not delayed
  mpdI2C_ByteWrite(id, 0xF4 , (apv2_delay & 0x3F) | 0x40, 0, &val);      // CR2: ADC2 clock delayed
  sleep(10);
  mpdI2C_ByteWrite(id, 0xF6 , 0x00, 0, &val);    // CR3: Not used output
  sleep(10);
  //      mpdI2C_ByteWrite(0xF8 , (apv1_delay & 0x3F) | 0x40, 0, &val);      // CR4: APV1 out
  mpdI2C_ByteWrite(id, 0xF8 , 0x40, 0, &val);    // CR4: APV1 out not delayed
  sleep(10);
  mpdI2C_ByteWrite(id, 0xFA , 0x00, 0, &val);    // GCR (40 MHz)
  sleep(10);

  return 0;
}

//======================================================
// adc methods

#define MPD_ADC_TOUT 1000
#define MPD_ADC_USLEEP 50

/**
 * adc = 0 or 1 (first or second adc in MPD)
 */
int 
mpdADS5281_Set(int id, int adc, uint32_t val) 
{
/*   int success, retry_count; */
  uint32_t data;

  data = (adc == 0) ? 0x40000000 : 0x80000000 ;
  data |= val;

  MPDLOCK;
  vmeWrite32(&MPDp[id]->AdcConfig, data);
  usleep(MPD_ADC_USLEEP);
  MPDUNLOCK;

  return OK;

  // BM: Retain this stuff if there's a problem writing to the AdcConfig 
/* #ifdef DUM */
/*   MPD_DUM("Write Adc= %d: value= 0x%x, success= 0x%x\n",adc, data, success); */
/* #endif */

/*   retry_count = 0; */
/*   data = 0xC0000000; */
/*   while( ((data & 0xFFFFFFF) != val) && (success == BUS_OK) && (retry_count < MPD_ADC_TOUT) ) { */
/*     success = BUS_Read(AdcConfigOffset, &data); */
/* #ifdef DUM */
/*     MPD_DUM("Read Adc= %d: value= 0x%x, success= 0x%x\n",adc, data, success); */
/* #endif */
/*     usleep(MPD_ADC_USLEEP); */
/*     retry_count++; */
/*   } */

/* #ifdef DEBUG */
/*   printf("%s: Adc= %d: set/get value= 0x%x/0x%x, success= 0x%x, retry= %d\n", */
/* 	 __FUNCTION__,adc, val, data, success, retry_count); */
/* #endif */

/*   if( success != BUS_OK ) return success; */
/*   if( retry_count >= MPD_ADC_TOUT ) return BUS_TIMEOUT; */

/*   return success; */

}

int 
mpdADS5281_InvertChannels(int id, int adc)	/* adc == 0, 1 */
{
  printf("%s: Board= %d ADC= %d Inverted Polarity\n",
	 __FUNCTION__,fMpd[id].fSlot, adc);
  return mpdADS5281_Set(id, adc, 0x2400FF);
}

int 
mpdADS5281_NonInvertChannels(int id, int adc)	/* adc == 0, 1 */
{
  printf("%s: Board= %d ADC= %d Direct Polarity\n",
	 __FUNCTION__, fMpd[id].fSlot, adc);
  return mpdADS5281_Set(id, adc, 0x240000);
  
}

int 
mpdADS5281_SetParameters(int id, int adc)	/* adc == 0, 1 */
{
  int success;
  
  if (mpdGetHWRevision(id)>=4) 
    {
      if ((success = mpdADS5281_Set(id, adc, 0x428021)) != OK) { return success; } // Differential clock
    } 
  else 
    {
      if ((success = mpdADS5281_Set(id, adc, 0x428020)) != OK) { return success; } // Single ended clock
    }
  usleep(MPD_ADC_USLEEP);
  return mpdADS5281_Set(id, adc, 0x110000);
}

int 
mpdADS5281_Normal(int id, int adc)	/* adc == 0, 1 */
{
  int success;

  MPD_MSG("Board= %d ADC= %d No Pattern (Normal Acq)\n",fMpd[id].fSlot, adc);
  if ((success = mpdADS5281_Set(id, adc, 0x450000)) != OK) { return success; }
  usleep(MPD_ADC_USLEEP);
  return mpdADS5281_Set(id, adc, 0x250000);
}

int 
mpdADS5281_Sync(int id, int adc)	/* adc == 0, 1 */
{
  int success;

  MPD_MSG("Board= %d ADC= %d Sync Pattern\n",fMpd[id].fSlot, adc);
  if ((success = mpdADS5281_Set(id, adc, 0x250000)) != OK) { return success; }
  usleep(MPD_ADC_USLEEP);
  return mpdADS5281_Set(id, adc, 0x450002);
}

int 
mpdADS5281_Deskew(int id, int adc)	/* adc == 0, 1 */
{
  int success;

  MPD_MSG("Board= %d ADC= %d Deskew Pattern\n",fMpd[id].fSlot, adc);
  if ((success = mpdADS5281_Set(id, adc,0x250000)) != OK) { return success; }
  usleep(MPD_ADC_USLEEP);
  return mpdADS5281_Set(id, adc, 0x450001);
}

int 
mpdADS5281_Ramp(int id, int adc)	/* adc == 0, 1 */
{
  int success;

  MPD_MSG("Board= %d ADC= %d Ramp Pattern\n",fMpd[id].fSlot, adc);
  if ((success = mpdADS5281_Set(id, adc, 0x450000)) != OK) { return success; }
  usleep(MPD_ADC_USLEEP);
  return mpdADS5281_Set(id, adc, 0x250040);
}


int 
mpdADS5281_SetGain(int id, int adc, 
			   int gain0, int gain1, int gain2, int gain3, 
			   int gain4, int gain5, int gain6, int gain7)
{
  uint32_t data;
  int success;

  MPD_MSG("Board= %d ADC= %d Set Gain %d %d %d %d %d %d %d %d\n",fMpd[id].fSlot, adc,
	  gain0, gain1, gain2, gain3, gain4, gain5, gain6, gain7);

  data = 0x2A0000;
  data |= gain0 | (gain1<<4) | (gain2<<8) | (gain3<<12);
  if ((success = mpdADS5281_Set(id, adc, data)) != OK) { return success; }
  usleep(MPD_ADC_USLEEP);
  data = 0x2B0000;
  data |= gain7 | (gain6<<4) | (gain5<<8) | (gain4<<12);
  return mpdADS5281_Set(id, adc, data);

}

//======================================================
// histogramming methods

int 
mpdHISTO_Clear(int id, int ch, int val)	/* ch == 0, 15 */
{
  uint32_t data[4096];
  int success=OK, i, j;
  int block=0;

  if( ch >= 8 ) block = 1;

  if (val<0) {
    for(i = 0; i<4096; i++) data[i] = i;
  } else {
    for(i = 0; i<4096; i++) data[i] = val;
  }

#ifdef BLOCK_TRANSFER
  success = BUS_BlockWrite(addr, 4096, data, &j);
  if( j != (4096) ) success = BUS_GENERIC_ERROR;
#else

  int ntimes = 4096 / 64;
  for(i = 0; i<ntimes; i++) {
    for(j = 0; j<64; j++) {	/* single word transfer */
      vmeWrite32(&MPDp[id]->Histo.block[block].Memory[i*64+j], data[i*64+j]);
    }
  }
#endif
  return success;
}

int 
mpdHISTO_Start(int id, int ch)	/* ch == 0, 15 */
{
  uint32_t data;
  int block=0;

  if(ch > 8) block = 1;

  data = 0x80 | (ch & 0x07);

  MPDLOCK;
  vmeWrite32(&MPDp[id]->Histo.block[block].CSR, data);
  MPDUNLOCK;

  return OK;
}

int 
mpdHISTO_Stop(int id, int ch)	/* ch == 0, 15 */
{
  uint32_t data;
  int block=0;

  if(ch > 8) block = 1;

  data = (ch & 0x07);

  MPDLOCK;
  vmeWrite32(&MPDp[id]->Histo.block[block].CSR, data);
  MPDUNLOCK;

  return OK;
}

int 
mpdHISTO_GetIntegral(int id, int ch, uint32_t *integral)	/* ch == 0, 15 */
{
  int block=0;

  if(ch > 8) block = 1;
  
  MPDLOCK;
  *integral = vmeRead32(&MPDp[id]->Histo.block[block].Histo_Count);
  MPDUNLOCK;

  return OK;
}

int 
mpdHISTO_Read(int id, int ch, uint32_t *histogram)	/* ch == 0, 15; uint32_t histogram[4096] */
{
  int success=OK, i, j;
  int block=0;

  if( ch >= 8 ) block=1;
#ifdef BLOCK_TRANSFER
  success = BUS_BlockRead(addr, 4096, histogram, &j);
  if( j != 4096 ) {
    MPD_ERR("Block Transfer returned %d 32bit words, 4096 expected\n",j);
    success = BUS_GENERIC_ERROR;
  }
#else
  int ntimes = 4096 / 64;
  for(i = 0; i<ntimes; i++) {
    for(j = 0; j<64; j++) {	/* single word transfer */
      histogram[i*64+j] = vmeRead32(&MPDp[id]->Histo.block[block].Memory[i*64+j]);
    }
  }
#endif
  return success;
}


//======================================================
// Daq-Readout methods
#ifdef NOTDONE


/**
 * Readout Fifo
 * Standard Event Mode (no zero suppression or pedestal subtraction)
 * Return 0 when something has been read, error otherwise
 */
int MPDlo::FIFO_ReadSingle(int id, 
			   int channel,     // apv channel (FIFO)
			   uint32_t *dbuf, // data buffer
			   int &wrec,      // max number of words to get / return words received 
			   int max_retry)   // max number of retry for timeout
//			   int &err)        // return error code
//			   int &n_events)   // number of events
{
  
  uint32_t base_addr, fifo_addr;
  int success, i, size;
  int nwords; // words available in fifo
  int wmax; // maximum word acceptable

  base_addr = ApvFifoOffset;
  fifo_addr = base_addr + (channel << 14);

  wmax = wrec;
  wrec=0; // returned words

  nwords = 0;

  i = 0;
  while( (nwords <= 0) && (i <= max_retry) ) {
    if( max_retry > 0 ) i++;
    success = FIFO_GetNwords(channel, &nwords);
    if( success != BUS_OK ) return success;
  }

  size = (nwords < wmax) ? nwords : wmax;

  MPD_DBG("fifo ch = %d, words in fifo= %d, retries= %d (max %d)\n",channel, nwords,i, max_retry);

  if( i > max_retry ) {
    MPD_ERR(" max retry = %d, count=%d nword=%d\n", max_retry, i, nwords);
    return BUS_TBC;
  }

#ifdef BLOCK_TRANSFER
  success = BUS_BlockRead(fifo_addr, size, dbuf, &wrec); // trasfer 4 byte words!!
  MPD_DBG("Block Read fifo ch = %d, words requested = %d, returned = %d\n", channel, size, wrec);
#else
  for(i=0; i<size; i++) {
    success = BUS_Read(fifo_addr+(i*4), (dbuf+i));
    if( success != BUS_OK ) return success;
    wrec+=1;
  }
  MPD_DBG("Read apv = %d, wrec = %d, success = %d\n", channel, wrec, success);
#endif

  if( wrec != size ) {
    MPD_DBG("Count Mismatch: %d expected %d\n", wrec, size);
    return BUS_MISMATCH;
  }
  return success;
}

/*
 * For Zero Suppression Processor / Pedestal Subtraction
 *
 * channel : ADC Channel (correspond to a single APV)
 * blen    : buffer lenght available
 * event   : event buffer
 * nread   : the number of words read back
 *
 * return error or 0 on success
 *
 */

int MPDlo::FIFO_ReadSingle(int id, int channel, int blen, uint32_t *event, int &nread)
{

  uint32_t base_addr, fifo_addr;
  int rval, nwords, size;
  int n_part;

  base_addr = ApvFifoOffset;
  fifo_addr = base_addr + (channel << 14);
  
  nread = 0;
  nwords = 0;
  n_part = 0;
  
  rval = FIFO_GetNwords(channel, &nwords);
  if( rval != BUS_OK ) return 2;
  
  size = (nwords > blen) ? blen : nwords; // cannot exceed memory buffer size

  if (size>0) {

#ifdef BLOCK_TRANSFER
    rval = BUS_BlockRead(fifo_addr, size, event, &n_part);
#else
    for(int i=0; i<size; i++) {
      rval = BUS_Read(fifo_addr+(i*4), (event+i));
      if( rval != BUS_OK ) { 
	return rval; 
      } 
      n_part++;
    }
#endif

#ifdef DEBUG_DUMP
    printf("\nReadData: %x %d\n",fifo_addr,channel);
    for (i=0;i<n_part;i++) {
      if ((i%16) == 0) printf("\n%4d",i);
      printf(" %6x", event[i]);
    }

    printf(" -> %d\n", n_part);
#endif
    nread += n_part;

    if( n_part != size ) return 1;

  }

  return nread;

}

int MPDlo::FIFO_Samples(int id, 
			int channel, 
			uint32_t *event, int *nread, int max_samples, int *err)
{
  uint32_t base_addr, fifo_addr;
  int success, nwords;

  base_addr = ApvFifoOffset;
	
  *err = 1;
  *nread = 0;
  fifo_addr = base_addr + (channel << 14);
  nwords = DAQ_FIFO_SIZE;
  if( nwords > max_samples )
    {
      *err = 2;
      return BUS_GENERIC_ERROR;
    }
#ifdef BLOCK_TRANSFER
  success = BUS_BlockRead(fifo_addr, nwords, event, nread);
#else
  for(int i=0; i<nwords; i++)
    BUS_Read(fifo_addr+(i*4), (event+i));
  *nread = nwords*4;
#endif
  *nread /= 4;
  if( *nread == nwords )
    *err = 0;
  return success;
}

int MPDlo::FIFO_IsSynced(int id, int channel, int *synced)
{
  uint32_t base_addr, error_addr;
  uint32_t data;
  uint32_t channel_mask, synced_mask;
  int success;

  base_addr = ApvFifoOffset;	
  error_addr = base_addr + SYNCED_ERROR_ADDR;
  channel_mask = 1 << (channel & 0x0F);
  synced_mask = channel_mask << 16;	/* @ error_addr */

  success = BUS_Read(error_addr, &data);
  if( data & synced_mask )
    *synced = 1;
  else
    *synced = 0;
  return success;
}

/*
 * Return the synch status of each FE, (one bit = one FE, 1 means synched) 
 */
int MPDlo::FIFO_AllSynced(int id, int *synced)
{
  uint32_t base_addr, error_addr;
  uint32_t data;
  int success;

  base_addr = ApvFifoOffset;	
  error_addr = base_addr + SYNCED_ERROR_ADDR;

  success = BUS_Read(error_addr, &data);
  *synced = (data>>16);
  return success;
}

int MPDlo::FIFO_HasError(int id, int channel, int *error)
{
  uint32_t base_addr, error_addr;
  uint32_t data;
  uint32_t channel_mask, error_mask;
  int success;

  base_addr = ApvFifoOffset;	
  error_addr = base_addr + SYNCED_ERROR_ADDR;
  channel_mask = 1 << (channel & 0x0F);
  error_mask = channel_mask;			/* @ error_addr */

  success = BUS_Read(error_addr, &data);
  if( data & error_mask )
    *error = 1;
  else
    *error = 0;
  return success;
}

int MPDlo::FIFO_GetAllFlags(int id, uint16_t *full, uint16_t *empty)
{
  uint32_t base_addr, flag_addr;
  uint32_t data;
  int success;

  base_addr = ApvFifoOffset;	
  flag_addr = base_addr + FIFO_FLAG_ADDR;

  success = BUS_Read(flag_addr, &data);
  *full =  data >> 16;
  *empty = data & 0xFFFF;
  return success;
}

int MPDlo::FIFO_IsFull(int channel, int *full)
{
  uint32_t base_addr, flag_addr;
  uint32_t data;
  uint32_t channel_mask, full_mask;
  int success;
  
  base_addr = ApvFifoOffset;	
  flag_addr = base_addr + FIFO_FLAG_ADDR;
  channel_mask = 1 << (channel & 0x0F);
  full_mask = channel_mask << 16;		/* @ flag_addr */

  success = BUS_Read(flag_addr, &data);
  if( data & full_mask )
    *full = 1;
  else
    *full = 0;
  return success;
}

int MPDlo::FIFO_GetNwords(int id, int channel, int *nwords) // can be optimized reading both consecutive channels
{
  uint32_t base_addr, nwords_addr;
  uint32_t data;
  uint32_t nwords_addr_offset[16] = {0,0,4,4,8,8,12,12,16,16,20,20,24,24,28,28};
  int success;
  
  base_addr = ApvFifoOffset;	
  nwords_addr = base_addr + USED_WORDS_ADDR + nwords_addr_offset[channel];

  success =BUS_Read(nwords_addr, &data);
  if( channel % 2 )	// if odd
    *nwords =  ((data & 0xFFFF0000) >> 16) & 0xFFFF;
  else
    *nwords =  data & 0xFFFF;
  return success;
}

int MPDlo::FIFO_IsEmpty(int id, int channel, int *empty)
{
  uint32_t base_addr, flag_addr;
  uint32_t data;
  uint32_t channel_mask, empty_mask;
  int success;

  base_addr = ApvFifoOffset;	
  flag_addr = base_addr + FIFO_FLAG_ADDR;
  channel_mask = 1 << (channel & 0x0F);
  empty_mask = channel_mask;			/* @ flag_addr */

  success = BUS_Read(flag_addr, &data);
  if( data & empty_mask )
    *empty = 1;
  else
    *empty = 0;
  return success;
}

int MPDlo::FIFO_ClearAll(int id)
{
  uint32_t addr;
  uint32_t data, oldval;
  int success;

  // Issue a pulse on READOUT_CONFIG[31]
  addr = ApvFifoOffset;
  addr += READOUT_CONFIG_ADDR;
  if( (success = BUS_Read(addr, &oldval)) != BUS_OK )
    return success;
  data = oldval | 0x80000000;
  if( (success = BUS_Write(addr, &data)) != BUS_OK )
    return success;
  data = oldval & 0x7FFFFFFF;
  return BUS_Write(addr, &data);
}


int MPDlo::FIFO_WaitNotEmpty(int id, int channel, int max_retry)
{
  uint32_t base_addr, flag_addr;
  uint32_t data;
  uint32_t channel_mask, empty_mask;
  int success, retry_count, fifo_empty;

  base_addr = ApvFifoOffset;	
  flag_addr = base_addr + FIFO_FLAG_ADDR;
  channel_mask = 1 << (channel & 0x0F);
  empty_mask = channel_mask;			/* @ flag_addr */

  retry_count = 0;
  fifo_empty = 1;
  success = BUS_OK;

  while( fifo_empty && success == BUS_OK && retry_count <= max_retry )
    {
      success = BUS_Read(flag_addr, &data);
      if( max_retry > 0 )
	retry_count++;
      if( data & empty_mask )
	fifo_empty = 1;
      else
	fifo_empty = 0;
      if( retry_count > max_retry )
	return -10;
    }
  return success;
}

/**
 * Standard event mode (no process)
 *
 * return true if all cards have been read
 */

bool MPDlo::FIFO_ReadAll(int id, int &timeout, int &global_fifo_error) {

  unsigned int k;
  int sample_left;

  int nread, err;

  sample_left = 0;
  global_fifo_error = 0;

  if (fReadDone == false) { // at least one MPD FIFO needs to be read
    for(k=0; k<fApv.size(); k++) { // loop on ADC channels on single board

      if (ApvReadDone(k) == false) { // APV FIFO has data to be read

	nread = ApvGetBufferAvailable(k);
	if (nread>0) { // space in memory buffer
	  err = FIFO_ReadSingle(v2a(k), ApvGetBufferPWrite(k), nread, 100000); 

	  ApvIncBufferPointer(k,nread);

	  global_fifo_error |= err; // ???	  
	} else { // no space in memory buffer
	  MPD_ERR("No space in memory buffer for MPD slot=%d, APV i2c=%d adc=%d\n", GetSlot(), v2a(k), v2i(k));
	}

	if ((err == BUS_TIMEOUT) || (nread == 0)) timeout++; // timeout

	int n = ApvGetBufferSample(k);

	MPD_DBG("Fifo= %d, word rec= %d, event rec= %d, error=%d\n",v2a(k),nread ,n, global_fifo_error);

	sample_left += ApvGetSampleLeft(k);

      }

      MPD_DBG("Fifo= %d, total sample left= %d\n",k, sample_left);

    } // loop on ADC
    fReadDone = (sample_left>0) ? false : true;  
  } // if fReadDone
  
  return fReadDone;

}

// ***** Utility functions for sparse readout *****

/**
 *
 * Search sample End Marker in Data Buffer
 * b = buffer of the ADC
 * i0, i1: first and last elements of the buffer to be checked
 *
 * return the location of the end marker or -1 if not found
 */
int MPDlo::SearchEndMarker(int id, uint32_t *b, int i0, int i1) {

  int i;

  if (i0>=i1) { return -1; }
  for (i=i0;i<i1;i++) {
    if ((b[i] & 0x180000) == 0x180000) {
      return i;
      break;
    } 
  }

  return -1;
}

/*
 * k = apv vector index
 * i0 = fBs
 */

void MPDlo::ApvShiftDataBuffer(int id, int k, int i0) {

  uint32_t *b;

  b = ApvGetBufferPointer(k, 0);
  int delta = fApv[k].fBi1 - i0;

  MPD_DBG("Move block of %d words from %d to 0\n",delta,i0);

  if (delta>0) {
    memmove(&b[0],&b[i0],sizeof(uint32_t)*delta); // areas may overlap
  }

  fApv[k].fBi0 = 0; // to be removed
  fApv[k].fBi1 = delta;
  
  MPD_DBG("Fifo= %d cleaned (data shifted) write pointer at=%d\n",v2a(k),fApv[k].fBi1);

}

bool MPDlo::FIFO_ReadAllNew(int id, int &timeout, int &global_fifo_error) {

  int n;
  unsigned int k;
  int sample_left;

  int nread, err;
  uint32_t multiple_events;

  int ii1=0;

  n=1; // single event mode

  sample_left = 0;

  if (fReadDone == false) { // MPD fifos need to be read
    for(k=0; k<fApv.size(); k++) { // loop on ADC channels on single board

      if (ApvReadDone(k) == false) { // APV FIFO has data to be read
  
	//      sample_left += ApvGetSampleLeft(k);

        //      ii0=fApv[k].fBi0;
        //      ii1=fApv[k].fBi1;
        //      idx = SearchEndMarker(ApvGetBufferPointer(k),ii0,ii1);
        //      fApv[k].fBi0 = ii1;
        //      cout << __FUNCTION__ << dec << " " << j << " " << k;

	uint32_t *bptr = ApvGetBufferPointer(k,ii1);
      
	int bsiz = fApv[k].fBufSize - ii1; // space left in buffer

	err = FIFO_ReadSingle(v2a(k), bsiz, bptr, nread);

	// ***
	if( nread > 0 ) {
	  for (int i=0; i<nread; i++) {
	    if (IsEndBlock(bptr[i])) { 
	      ApvDecSampleLeft(k,1);
	      if (ApvGetSampleLeft(k) == 0) {
		fApv[k].fBs = ii1 + i; // pointer to the last sample word
	      }
	    }
	  }

	  fApv[k].fBi1=ii1+nread;

	} else {
	  timeout++;
	}
	
	if( err != 2 )
	  global_fifo_error |= err;
      
	if (err == 2) timeout++;
      
	if( n > 1 ) multiple_events++; // not used 
	
      } else { // ...
#ifdef DEBUG_DUMP
	cout << __FUNCTION__ << ": xxxx " << endl;
#endif
      }
      
      sample_left += ApvGetSampleLeft(k);

    } // loop on ADC FIFOs
    fReadDone = (sample_left == 0) ? true : false;      
  } 
  
  return fReadDone;

}

/**
 *
 */

int MPDlo::DAQ_Enable(int id)
{
  DAQ_Config();
  return TRIG_Enable();
}

int MPDlo::DAQ_Disable(int id)
{
  uint32_t addr;
  uint32_t data;
  int success;

  addr = ApvFifoOffset;
	
  addr += READOUT_CONFIG_ADDR;
  data = 0;
  if( (success = BUS_Write(addr, &data)) != BUS_OK )
    return success;
  return TRIG_Disable();

}

int MPDlo::DAQ_Config(int id) {

  uint32_t addr, data;
  int success;
  short evtbld;

  evtbld = GetEventBuilding() ? 1 : 0;

  addr = ApvFifoOffset;
	
  addr += READOUT_CONFIG_ADDR;
  data = (GetAcqMode() & 0x07) | // ((test & 0x01) << 15) |
    ((GetCommonOffset() & 0xfff) << 16) | 
    ((GetCommonNoiseSubtraction() & 0x1) << 28) | 
    ((evtbld & 0x1) << 30);
  if( (success = BUS_Write(addr, &data)) != BUS_OK )
    return success;

  return FIFO_ClearAll();

  for (unsigned int i=0;i<fApv.size();i++) {
    fApv[i].fBi0 = 0;
    fApv[i].fBi1 = 0;
  }

}


// ***** Pedestal and Thresholds handling routines

/**
 *
 */

int MPDlo::PED_Write(int id, int ch, int *ped_even, int *ped_odd)	// even, odd is the apv channles, ch = 0..7
{
  uint32_t addr, data;
  int success, i;

  addr = ApvFifoOffset;
	
  addr += PEDESTAL_BASE_ADDR + (ch << 14);
  for(i=0; i<128; i++)
    {
      data = ped_even[i] | (ped_odd[i] << 16);
      if( (success = BUS_Write(addr, &data)) != BUS_OK )
	return success;
      addr += 4;
    }
  return success;
}

int MPDlo::PED_Write(int id, int ch, int v)	// ch = 0..7
{
  int i, data[128];
  
  for(i=0; i<128; i++)
    data[i] = v;
  return PED_Write(ch, data, data);
}

int MPDlo::PED_Read(int id, int ch, int *ped_even, int *ped_odd)	// TBD
{
  for(int i=0; i<128;  i++)
    ped_even[i] = ped_odd[i] = 0;	// TBD
  return BUS_OK;
}


int MPDlo::THR_Write(int id, int ch, int *thr_even, int *thr_odd)	// ch = 0..7
{
  uint32_t addr, data;
  int success, i;

  addr = ApvFifoOffset;
	
  addr += THRESHOLD_BASE_ADDR + (ch << 14);
  for(i=0; i<128; i++)
    {
      data = thr_even[i] | (thr_odd[i] << 16);
      if( (success = BUS_Write(addr, &data)) != BUS_OK )
	return success;
      addr += 4;
    }
  return success;
}

int MPDlo::THR_Write(int id, int ch, int v)	// ch = 0..7
{
  int i, data[128];
  
  for(i=0; i<128; i++)
    data[i] = v;
  return THR_Write(ch, data, data);
}

int MPDlo::THR_Read(int id, int ch, int *thr_even, int *thr_odd)	// TBD
{
  for(int i=0; i<128;  i++)
    thr_even[i] = thr_odd[i] = 0;	// TBD
  return BUS_OK;
}

/**
 * Load pedestal and threshold data into the MPD
 */
int MPDlo::PEDTHR_Write(int id) {

  for (int ia=0;ia<8;ia++) {// loop on apv,  odd and even apv are download simultaneously
    PED_Write(ia, GetApvPed(2*ia), GetApvPed(2*ia+1));
    THR_Write(ia, GetApvThr(2*ia), GetApvThr(2*ia+1));
  }

  return 0;

}



/**
 * Read Pedestals and Thresholds of a single MPD from the file pname
 * if pname is empty, use the stored PedThrPath value
 */
int MPDlo::ReadPedThr(int id, std::string pname) {

  string line;

  int ch, ped, thr;
  int count,i;

  if (pname.empty()) {
    pname = GetPedThrPath();
  };

  for (i=0;i<2048;i++) {
    SetPedThr(i, GetPedCommon(), GetThrCommon());
  };

  std::ifstream infile(pname.data(), std::ifstream::in);
  if (infile.is_open()) {
    count=0;
    while (infile.good()) {
      getline(infile, line);
      if (line.find("#") != 0) { // no comment line
	std::istringstream iss(line);
	iss >> ch >> ped >> thr;
	count += SetPedThr(ch, ped, thr);
      }
    }    
    cout << __FUNCTION__ << ": " << count << " channels read from file " << pname << " and set" << endl;
    infile.close();
  } else {
    cout << __FUNCTION__ << ": Warning, unable to open file " << pname << endl;
    cout << __FUNCTION__ << ": Warning, set pedestal and threshold to common values" << endl; 
  }

  return count;

}

#endif /* NOTDONE */

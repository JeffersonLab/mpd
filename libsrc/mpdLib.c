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
LOCAL UINT32      mpdIntLevel    = FA_VME_INT_LEVEL;         /* default VME interrupt level */
LOCAL UINT32      mpdIntVec      = FA_VME_INT_VEC;           /* default interrupt Vector */

/* Define global variables */
int nmpd = 0;                                       /* Number of MPDs in Crate */
int mpdA32Base   = 0x08000000;                      /* Minimum VME A32 Address for use by MPDs */
int mpdA32Offset = 0x08000000;                      /* Difference in CPU A32 Base - VME A32 Base */
int mpdA24Offset = 0x0;                             /* Difference in CPU A24 Base - VME A24 Base */
volatile struct mpd_struct *FAp[(FA_MAX_BOARDS+1)]; /* pointers to MPD memory map */
volatile unsigned int *FApd[(FA_MAX_BOARDS+1)];      /* pointers to MPD FIFO memory */
volatile unsigned int *FApmb;                        /* pointer to Multblock window */
int mpdID[FA_MAX_BOARDS];                           /* array of slot numbers for MPDs */
unsigned int mpdAddrList[FA_MAX_BOARDS];            /* array of a24 addresses for MPDs */
int mpdRev[(FA_MAX_BOARDS+1)];                      /* Board Revision Info for each module */
unsigned short mpdChanDisable[(FA_MAX_BOARDS+1)];   /* Disabled Channel Mask for each Module*/
int mpdInited=0;                                    /* >0 if Library has been Initialized before */
int mpdMaxSlot=0;                                   /* Highest Slot hold an MPD */
int mpdMinSlot=0;                                   /* Lowest Slot holding an MPD */
int mpdSource=0;                                    /* Signal source for MPD system control*/
int mpdBlockLevel=0;                                /* Block Level for ADCs */
int mpdIntCount = 0;                                /* Count of interrupts from MPD */
int mpdBlockError=FA_BLOCKERROR_NO_ERROR; /* Whether (>0) or not (0) Block Transfer had an error */

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
 *       Low 6 bits - Specifies the default Signal distribution (clock,trigger) 
 *                    sources for the board (Internal, FrontPanel, VXS, VME(Soft))
 *       bit    0:  defines Sync Reset source
 *                     0  VME (Software Sync-Reset)
 *                     1  Front Panel/VXS/P2 (Depends on Clk/Trig source selection)
 *       bits 3-1:  defines Trigger source
 *               0 0 0  VME (Software Triggers)
 *               0 0 1  Front Panel Input
 *               0 1 0  VXS (P0) 
 *               (all others Undefined - default to VME/Software)
 *       bits 5-4:  defines Clock Source
 *           0 0  Internal 250MHz Clock
 *           0 1  Front Panel 
 *           1 0  VXS (P0)
 *           1 1  P2 Connector (Backplane)
 * </pre>
 *
 * <pre>
 *       Common Modes of Operation:
 *           Value = 0  CLK (Int)  TRIG (Soft)   SYNC (Soft)    (Debug/Test Mode)
 *                   2  CLK (Int)  TRIG (FP)     SYNC (Soft)    (Single Board
 *                   3  CLK (Int)  TRIG (FP)     SYNC (FP)         Modes)
 *                0x10  CLK (FP)   TRIG (Soft)   SYNC (Soft)
 *                0x13  CLK (FP)   TRIG (FP)     SYNC (FP)      (VME SDC Mode)
 *                0x20  CLK (VXS)  TRIG (Soft)   SYNC (Soft)
 *                0x25  CLK (VXS)  TRIG (VXS)    SYNC (VXS)     (VXS SD Mode)
 *
 *
 *      High 10bits - A16 Base address of MPD Signal Distribution Module
 *                    This board can control up to 7 MPD Boards.
 *                    Clock Source must be set to Front Panel (bit4 = 1)
 *
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
STATUS 
mpdInit(UINT32 addr, UINT32 addr_inc, int nadc, int iFlag)
{
  int ii, res, errFlag = 0;
  int boardID = 0;
  int maxSlot = 1;
  int minSlot = 21;
  int trigSrc=0, clkSrc=0, srSrc=0;
  unsigned int rdata, laddr, laddr_inc, a32addr, a16addr=0;
  volatile struct mpd_struct *mpd;
  unsigned short sdata;
  int noBoardInit=0;
  int useList=0;
  int noFirmwareCheck=0;
  unsigned short supported_proc[MPD_SUPPORTED_PROC_FIRMWARE_NUMBER]
    = {MPD_SUPPORTED_PROC_FIRMWARE};
  unsigned short proc_version=0;
  int icheck=0, proc_supported=0;

  /* Check if we have already Initialized boards before */
  if((mpdInited>0) && (mpdID[0] != 0)) 
    {
      /* Hard Reset of all MPD boards in the Crate */
      for(ii=0;ii<nmpd;ii++) 
	{
	  vmeWrite32(&(MPDp[mpdID[ii]]->csr),MPD_CSR_HARD_RESET);
	}
      taskDelay(5);
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
      if( ((addr_inc==0)||(nadc==0)) && (useList==0) )
	nadc = 1; /* assume only one MPD to initialize */

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

  /* Init Some Global variables */
  mpdSource = iFlag&MPD_SOURCE_MASK;
  mpdInited = nmpd = 0;
  bzero((char *)mpdChanDisable,sizeof(mpdChanDisable));
  bzero((char *)mpdID,sizeof(mpdID));

  for (ii=0;ii<nadc;ii++) 
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
		    (unsigned short)(vmeRead32(&mpd->adc_status[0]) & MPD_ADC_VERSION_MASK);

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
		    (unsigned short)(vmeRead32(&mpd->adc_status[0]) & MPD_ADC_VERSION_MASK);

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

  if(!noBoardInit)
    {
      /* what are the Trigger Sync Reset and Clock sources */
      if (mpdSource == MPD_SOURCE_VXS)
	{
	  printf("mpdInit: Enabling MPD for VXS Clock ");
	  clkSrc  = MPD_REF_CLK_P0;
	  switch (iFlag&0xf) 
	    {
	    case 0: case 1:
	    case 8: case 10: case 12: case 14:
	    case 9: case 11: case 13: case 15:
	      printf("and Software Triggers (Soft Sync Reset)\n");
	      trigSrc = MPD_TRIG_VME | MPD_ENABLE_SOFT_TRIG;
	      srSrc   = MPD_SRESET_VME | MPD_ENABLE_SOFT_SRESET;
	      break;
	    case 2:
	      printf("and Front Panel Triggers (Soft Sync Reset)\n");
	      trigSrc = MPD_TRIG_FP_ISYNC;
	      srSrc   = MPD_SRESET_VME | MPD_ENABLE_SOFT_SRESET;
	      break;
	    case 3:
	      printf("and Front Panel Triggers (FP Sync Reset)\n");
	      trigSrc = MPD_TRIG_FP_ISYNC;
	      srSrc   = MPD_SRESET_FP_ISYNC;
	      break;
	    case 4: case 6:
	      printf("and VXS Triggers (Soft Sync Reset)\n");
	      trigSrc = MPD_TRIG_P0_ISYNC;
	      srSrc   = MPD_SRESET_VME | MPD_ENABLE_SOFT_SRESET;
	      break;
	    case 5: case 7:
	      printf("and VXS Triggers (VXS Sync Reset)\n");
	      trigSrc = MPD_TRIG_P0_ISYNC;
	      srSrc   = MPD_SRESET_P0_ISYNC;
	      break;
	    }
	}
      else if (mpdSource == MPD_SOURCE_SDC) 
	{
	  printf("mpdInit: Enabling MPD for SDC Clock (Front Panel) ");
	  clkSrc  = MPD_REF_CLK_FP;
	  switch (iFlag&0xf) 
	    {
	    case 0: case 1:
	    case 8: case 10: case 12: case 14:
	    case 9: case 11: case 13: case 15:
	      printf("and Software Triggers (Soft Sync Reset)\n");
	      trigSrc = MPD_TRIG_VME | MPD_ENABLE_SOFT_TRIG;
	      srSrc   = MPD_SRESET_VME | MPD_ENABLE_SOFT_SRESET;
	      break;
	    case 2: case 4: case 6:
	      printf("and Front Panel Triggers (Soft Sync Reset)\n");
	      trigSrc = MPD_TRIG_FP_ISYNC;
	      srSrc   = MPD_SRESET_VME | MPD_ENABLE_SOFT_SRESET;
	      break;
	    case 3: case 5: case 7:
	      printf("and Front Panel Triggers (FP Sync Reset)\n");
	      trigSrc = MPD_TRIG_FP_ISYNC;
	      srSrc   = MPD_SRESET_FP_ISYNC;
	      break;
	    }
	}
      else 
	{  /* Use internal Clk */
	  printf("mpdInit: Enabling MPD Internal Clock, ");
	  clkSrc = MPD_REF_CLK_INTERNAL;
	  switch (iFlag&0xf) 
	    {
	    case 0: case 1:
	    case 8: case 10: case 12: case 14:
	    case 9: case 11: case 13: case 15:
	      printf("and Software Triggers (Soft Sync Reset)\n");
	      trigSrc = MPD_TRIG_VME | MPD_ENABLE_SOFT_TRIG;
	      srSrc   = MPD_SRESET_VME | MPD_ENABLE_SOFT_SRESET ;
	      break;
	    case 2:
	      printf("and Front Panel Triggers (Soft Sync Reset)\n");
	      trigSrc = MPD_TRIG_FP_ISYNC;
	      srSrc   = MPD_SRESET_VME | MPD_ENABLE_SOFT_SRESET;
	      break;
	    case 3:
	      printf("and Front Panel Triggers (FP Sync Reset)\n");
	      trigSrc = MPD_TRIG_FP_ISYNC;
	      srSrc   = MPD_SRESET_FP_ISYNC;
	      break;
	    case 4: case 6:
	      printf("and VXS Triggers (Soft Sync Reset)\n");
	      trigSrc = MPD_TRIG_P0_ISYNC;
	      srSrc   = MPD_SRESET_VME | MPD_ENABLE_SOFT_SRESET;
	      break;
	    case 5: case 7:
	      printf("and VXS Triggers (VXS Sync Reset)\n");
	      trigSrc = MPD_TRIG_P0_ISYNC;
	      srSrc   = MPD_SRESET_P0_ISYNC;
	      break;
	    }
	}
    }

  /* Enable Clock source - Internal Clk enabled by dempdult */ 
  if(!noBoardInit)
    {
      for(ii=0;ii<nmpd;ii++) 
	{
	  vmeWrite32(&(MPDp[mpdID[ii]]->ctrl1),(clkSrc | MPD_ENABLE_INTERNAL_CLK)) ;
	}
      taskDelay(20);


      /* Hard Reset FPGAs and FIFOs */
      for(ii=0;ii<nmpd;ii++) 
	{
	  vmeWrite32(&(MPDp[mpdID[ii]]->reset),
		     (MPD_RESET_ADC_FPGA1 | MPD_RESET_ADC_FIFO1 |
		      MPD_RESET_DAC | MPD_RESET_EXT_RAM_PT));

	  /* Release reset on MGTs */
	  vmeWrite32(&(MPDp[mpdID[ii]]->mgt_ctrl),MPD_RELEASE_MGT_RESET);
	  vmeWrite32(&(MPDp[mpdID[ii]]->mgt_ctrl),MPD_MGT_RESET);
	  vmeWrite32(&(MPDp[mpdID[ii]]->mgt_ctrl),MPD_RELEASE_MGT_RESET);
	}
      taskDelay(5);
    }

  /* Write configuration registers with dempdult/defined Sources */
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
      MPDpd[mpdID[ii]] = (unsigned int *)(laddr);  /* Set a pointer to the FIFO */
      if(!noBoardInit)
	{
	  vmeWrite32(&(MPDp[mpdID[ii]]->adr32),(a32addr>>16) + 1);  /* Write the register and enable */
	
	  /* Set Dempdult Block Level to 1 */
	  vmeWrite32(&(MPDp[mpdID[ii]]->blk_level),1);
	}
      mpdBlockLevel=1;

      /* Setup Trigger and Sync Reset sources */
      if(!noBoardInit)
	{
	  vmeWrite32(&(MPDp[mpdID[ii]]->ctrl1),
		     vmeRead32(&(MPDp[mpdID[ii]]->ctrl1)) | 
		     (srSrc | trigSrc) );
	}
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
      MPDpmb = (unsigned int *)(laddr);  /* Set a pointer to the FIFO */
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

  if(errFlag > 0) 
    {
      printf("mpdInit: WARN: Unable to initialize all requested MPD Modules (%d)\n",
	     nadc);
      if(nmpd > 0)
	printf("mpdInit: %d MPD(s) successfully initialized\n",nmpd );
      return(ERROR);
    } 
  else 
    {
      return(OK);
    }
}

int
mpdCheckAddresses(int id)
{
  unsigned int offset=0, expected=0, base=0;
  
  if(id==0) id=mpdID[0];

  if((id<=0) || (id>21) || (MPDp[id] == NULL)) 
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return ERROR;
    }

  printf("%s:\n\t ---------- Checking mpd250 address space ---------- \n",__FUNCTION__);

  base = (unsigned int) &MPDp[id]->SdramFifo[0];

  offset = ((unsigned int) &MPDp[id]->AdcConfig) - base;
  expected = 0x01000000;
  if(offset != expected)
    printf("%s: ERROR MPDp[id]->AdcConfig not at offset = 0x%x (@ 0x%x)\n",
	   __FUNCTION__,expected,offset);

  offset = ((unsigned int) &MPDp[id]->I2C.Clock_Prescaler_low) - base;
  expected = 0x02000000;
  if(offset != expected)
    printf("%s: ERROR MPDp[id]->I2C.Clock_Prescaler_low not at offset = 0x%x (@ 0x%x)\n",
	   __FUNCTION__,expected,offset);

  offset = ((unsigned int) &MPDp[id]->Histo.block[0]) - base;
  expected = 0x03000000;
  if(offset != expected)
    printf("%s: ERROR MPDp[id]->Histo.block[0] not at offset = 0x%x (@ 0x%x)\n",
	   __FUNCTION__,expected,offset);

  offset = ((unsigned int) &MPDp[id]->ApvDaq.Data_Ch[0][0]) - base;
  expected = 0x04000000;
  if(offset != expected)
    printf("%s: ERROR MPDp[id]->ApvDaq.Data_Ch[0][0] not at offset = 0x%x (@ 0x%x)\n",
	   __FUNCTION__,expected,offset);

  offset = ((unsigned int) &MPDp[id]->SdramChip0[0]) - base;
  expected = 0x05000000;
  if(offset != expected)
    printf("%s: ERROR MPDp[id]->SdramChip0[0] not at offset = 0x%x (@ 0x%x)\n",
	   __FUNCTION__,expected,offset);

  offset = ((unsigned int) &MPDp[id]->SdramChip1[0]) - base;
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
mpdSlot(unsigned int i)
{
  if(i>=nmpd)
    {
      printf("%s: ERROR: Index (%d) >= MPDs initialized (%d).\n",
	     __FUNCTION__,i,nmpd);
      return ERROR;
    }

  return mpdID[i];
}


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
LOCAL UINT32      mpdIntLevel    = 0;         /* default VME interrupt level */
LOCAL UINT32      mpdIntVec      = 0;           /* default interrupt Vector */

/* Define global variables */
int nmpd = 0;                                       /* Number of MPDs in Crate */
int mpdA32Base   = 0x08000000;                      /* Minimum VME A32 Address for use by MPDs */
int mpdA32Offset = 0x08000000;                      /* Difference in CPU A32 Base - VME A32 Base */
int mpdA24Offset = 0x0;                             /* Difference in CPU A24 Base - VME A24 Base */
volatile struct mpd_struct *MPDp[(MPD_MAX_BOARDS+1)]; /* pointers to MPD memory map */
volatile unsigned int *MPDpd[(MPD_MAX_BOARDS+1)];      /* pointers to MPD FIFO memory */
volatile unsigned int *MPDpmb;                        /* pointer to Multblock window */
int mpdID[MPD_MAX_BOARDS];                           /* array of slot numbers for MPDs */
unsigned int mpdAddrList[MPD_MAX_BOARDS];            /* array of a24 addresses for MPDs */
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
unsigned short fApvEnableMask[(MPD_MAX_BOARDS)+1];
int nApv[(MPD_MAX_BOARDS)+1];
int fReadDone[(MPD_MAX_BOARDS)+1];

/* Internal APV register addresses */
static const unsigned char ipre_addr = 0x20;
static const unsigned char ipcasc_addr = 0x22;
static const unsigned char ipsf_addr = 0x24;
static const unsigned char isha_addr = 0x26;
static const unsigned char issf_addr = 0x28;
static const unsigned char ipsp_addr = 0x2A;
static const unsigned char imuxin_addr = 0x2C;
static const unsigned char ispare_addr = 0x2E;
static const unsigned char ical_addr = 0x30;
static const unsigned char vfp_addr = 0x32;
static const unsigned char vfs_addr = 0x34;
static const unsigned char vpsp_addr = 0x36;
static const unsigned char cdrv_addr = 0x38;
static const unsigned char csel_addr = 0x3A;
static const unsigned char mode_addr = 0x02;
static const unsigned char latency_addr = 0x04;
static const unsigned char muxgain_addr = 0x06;
static const unsigned char error_addr = 0x00;


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
  unsigned int rdata, laddr, laddr_inc, a32addr, a16addr=0;
  volatile struct mpd_struct *mpd;
  unsigned short sdata;
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

int 
mpdLM95235_Read(int *t)
{
  const unsigned char LM95235_i2c_addr  = 0x4C;
  //  const unsigned char Local_TempS_MSB_addr  = 0x00;	// Read only
  //  const unsigned char Local_TempS_LSB_addr  = 0x30;	// Read only
  //  const unsigned char Remote_TempS_MSB_addr  = 0x01;	// Read only
  //  const unsigned char Remote_TempS_LSB_addr  = 0x10;	// Read only
  const unsigned char Remote_TempU_MSB_addr  = 0x31;	// Read only
  const unsigned char Remote_TempU_LSB_addr  = 0x32;	// Read only
  //  const unsigned char ConfigReg2_addr  = 0xBF;
  //  const unsigned char RemoteOffset_H_addr  = 0x11;
  //  const unsigned char RemoteOffset_L_addr  = 0x12;
  //  const unsigned char ConfigReg1_addr  = 0x03;	// also 0x09
  //  const unsigned char ConvRate_addr  = 0x04;	// also 0x0A
  const unsigned char OneShot_addr  = 0x0F;	// Write only
  const unsigned char Status1_addr  = 0x02;	// Read only
  //  const unsigned char Status2_addr  = 0x33;	// Read only
  //  const unsigned char ManufID_addr  = 0xFE;	// Read only, returns 0x01
  //  const unsigned char RevID_addr  = 0xFF;	// Read only

  unsigned char val;
  int success, retry_count;

  success = I2C_ByteWrite((unsigned char)(LM95235_i2c_addr<<1), OneShot_addr, 0, &val);
  success = I2C_ByteWrite((unsigned char)(LM95235_i2c_addr<<1), Status1_addr, 0, &val);
  val = 0x80;
  retry_count = 0;
  while( (val & 0x80) && (retry_count++ < 100) )
    success = I2C_ByteRead1((unsigned char)(LM95235_i2c_addr<<1), &val);
  success = I2C_ByteWrite((unsigned char)(LM95235_i2c_addr<<1), Remote_TempU_MSB_addr, 0, &val);
  success = I2C_ByteRead1((unsigned char)(LM95235_i2c_addr<<1), &val);
  *t = val;
  success = I2C_ByteWrite((unsigned char)(LM95235_i2c_addr<<1), Remote_TempU_LSB_addr, 0, &val);
  success = I2C_ByteRead1((unsigned char)(LM95235_i2c_addr<<1), &val);
  // aggiornare t
  return success;

}


/****************************
 * LOW LEVEL routines 
 ****************************/


/* I2C methods */

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
  unsigned int data, rdata;
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

  int ispeed = GetI2CSpeed();

  vmeWrite32(&MPDp[id]->I2C.Clock_Prescaler_low, (ispeed & 0xFF));
  
  rdata = vmeRead32(&MPDp[id]->I2C.Clock_Prescaler_low);

  printf("%s: i2c low prescaler register set/read : %d / %d\n",
	 __FUNCTION__,data,rdata);

  vmeWrite32(&MPDp[id]->I2C.Clock_Prescaler_high, (ispeed>>8) & 0xff);
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
mpdI2C_ByteWrite(int id, unsigned char dev_addr, unsigned char int_addr, 
		 int ndata, unsigned char *data)
{
  int success, i;
  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<=0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }
  
  if( (success = mpdI2C_SendByte(id, (unsigned char)(dev_addr & 0xFE), 1)) != OK )
    {
      if( success <= -10 )
	{
	  mpdI2C_SendStop(id);
	  return success;
	}
      else
	return mpdI2C_SendStop(id);
    }
  //  usleep(300);
  if( (success = mpdI2C_SendByte(id, int_addr, 0)) != OK )
    {
      if( success <= -10 )
	{
	  mpdI2C_SendStop(id);
	  return success;
	}
      else
	return mpdI2C_SendStop(id);
    }
  //  usleep(300);
  for(i=0; i<ndata; i++)
    if( (success = I2C_SendByte(id, data[i], 0)) != OK )
      {
	if( success <= -10 )
	  {
	    mpdI2C_SendStop(id);
	    return success;
	  }
	else
	  return mpdI2C_SendStop(id);
      }
  //  usleep(300);
  success = mpdI2C_SendStop(id);
  // usleep(300);
  return success;

}

int
mpdI2C_ByteRead(int id, unsigned char dev_addr, unsigned char int_addr, 
		int ndata, unsigned char *data)
{
  int success, i;
  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<=0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }
  
  if( (success = mpdI2C_SendByte(id, (unsigned char)(dev_addr | 0x01), 1)) != OK )
    return mpdI2C_SendStop(id);
  
  if( (success = I2C_SendByte(id, int_addr, 0)) != OK )
    return mpdI2C_SendStop(id);
  
  for(i=0; i<ndata; i++)
    if( (success = I2C_ReceiveByte(id, data+i)) != OK )
      return mpdI2C_SendStop(id);
  
  return mpdI2C_SendStop(id);
}

int 
mpdI2C_ByteWriteRead(int id, unsigned char dev_addr, unsigned char int_addr, 
		     int ndata, unsigned char *data)
{
  int success, i;
  if((MPDp[id]==NULL) || (id<=0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }
  
  if( (success = mpdI2C_SendByte(id, (unsigned char)(dev_addr & 0xFE), 1)) != OK )
    return mpdI2C_SendStop(id);
  
  if( (success = mpdI2C_SendByte(id, int_addr, 0)) != OK )
    return mpdI2C_SendStop(id);
  
  for(i=0; i<ndata; i++)
    if( (success = mpdI2C_ReceiveByte(id, data+i)) != OK )
      return mpdI2C_SendStop(id);
  
  return mpdI2C_SendStop(id);
}


int 
mpdI2C_ByteRead1(int id, unsigned char dev_addr, unsigned char *data)
{
  int success;
  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<=0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  
  if( (success = I2C_SendByte(id, (unsigned char)(dev_addr & 0xFE), 1)) != OK )
    return mpdI2C_SendStop(id);
  
  if( (success = I2C_ReceiveByte(data)) != OK )
    return mpdI2C_SendStop(id);
  
  return mpdI2C_SendStop(id);
}

int 
mpdI2C_SendByte(int id, unsigned char byteval, int start)
{
  int rval=OK, retry_count;
  unsigned int data=0;
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
  while( (data & 0x00000002) != 0 && retry_count < GetI2CMaxRetry() )
    {
      data = vmeRead32(&MPDp[id]->I2C.CommStat);
      retry_count++;
    }

  if( retry_count >= GetI2CMaxRetry() )
    rval = -10;

  if( data & MPD_I2C_COMMSTAT_NACK_RECV )	/* NACK received */
    rval = -20;

  MPDLOCK;

  return rval;
}

int 
mpdI2C_ReceiveByte(int id, unsigned char *byteval)
{
  int retry_count;
  int rval=0;
  unsigned int data=0;
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
  while( (data & 0x00000002) != 0 && retry_count < GetI2CMaxRetry() )
    {
      data = vmeRead32(&MPDp[id]->I2C.CommStat);
      retry_count++;
    }

  if( retry_count >= GetI2CMaxRetry() )
    rval = -10;

  if( data & MPD_I2C_COMMSTAT_NACK_RECV )	/* NACK received */
    rval = -20;

  data = vmeRead32(&MPDp[id]->I2C.TxRx);
  
  *byteval = data;
  MPDUNLOCK;

  return rval;

}

int 
mpdI2C_SendStop(int id)
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

int 
mpdI2C_SendNack(int id)
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

/* APV methods */

int
mpdAPV_Reset101(int id)
{
  unsigned int addr, data;
  int success;
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
mpdAPV_Try(int id, unsigned char apv_addr) // i2c addr
{
  unsigned char x = 0xEC;
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
mpdSetApvEnableMask(int id, unsigned short mask) 
{ 
  fApvEnableMask[id] |= mask;
}

void 
mpdResetApvEnableMask(int id) 
{ 
  fApvEnableMask[id] = 0;
}

unsigned short
mpdGetApvEnableMask(int id) 
{ 
  return fApvEnableMask[id];
}


int
mpdAPV_Scan(int id) 
{
  int iapv=0;
  unsigned int apvmask=0;
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
	  printf("%s : APV i2c = %d found in MPD %d : %d\n",
		 __FUNCTION__,iapv,GetBus(), GetSlot());
	}
    }
  printf("%s: Blind scan done\n",__FUNCTION__);

  mpdResetApvEnableMask(id);

  for(iapv=0; iapv<MPD_MAX_APV; iapv++)
    {
      printf("%s: Try %2d %2d", __FUNCTION__, fApv[id][iapv].i2c, fApv[id][iapv].adc);
      
      if ( mpdAPV_Try(id, fApv[id][iapv].i2c && fApv[id][iapv].adc>-1 ) ) 
	{
	  printf("%s: %d matched in MPD %d %d\n",
		 __FUNCTION__, fApv[id][iapv].i2c, GetBus(), GetSlot());

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
mpdAPV_Write(int id, unsigned char apv_addr, unsigned char reg_addr, unsigned char val)
{
  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<=0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  return mpdI2C_ByteWrite(id, (unsigned char)((0x20 | apv_addr)<<1), reg_addr, 1, &val);
}

int
mpdAPV_Read(int id, unsigned char apv_addr, unsigned char reg_addr, unsigned char *val)
{
  int success;
  unsigned char rval;

  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<=0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  usleep(500);

  success = mpdI2C_ByteWrite(id, (unsigned char)((0x20 | apv_addr)<<1), (reg_addr|0x01), 1, val);
  if( success != OK )
    return success;
  
  usleep(500);
  success = mpdI2C_ByteRead1(id, (unsigned char)((0x20 | apv_addr)<<1), &rval);

  MPD_MSG("Set / Get = 0x%x 0x%x\n",*val,rval);

  return success;
}

int
mpdAPV_Config(int id, int apv_index)
{
  int success, i;
  unsigned char apv_addr, reg_addr, val;

  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<=0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  apv_addr = fApv[id][apv_index].i2c;

  printf("APV card i2c=%d to ADC (fifo)=%d (from config file)\n",
	 (int) apv_addr, v2a(apv_index));

  for (i=0;i<18;i++) {
    switch (i) {
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

  // FIXME: Need another way to do this
  fApv[id][nApv[id]] = v;
  fApv[id][nApv[id]].fBuffer = 0;
  
  printf("%s: APV %d added to list of FECs\n",
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

  // FIXME: Need another way to check this
  if (nApv[id]>0) 
    {
      c = (fApv[id][0].Mode & 0x10) >> 4;
    }
  
  return c;
}

/**
 * Return max latency of all APVs in a single MPD
 */
unsigned char
mpdApvGetMaxLatency(int id) 
{
  unsigned char c=0;
  unsigned int iapv=0;
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
  unsigned int iapv=0;
  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<=0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  for (iapv=0; iapv<nApv[id]; iapv++) 
    {
      ApvSetSampleLeft(id, iapv); // improve peak mode
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
	unsigned int addr;
	unsigned int data;
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
	unsigned int addr;
	unsigned int data;
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
  unsigned int addr;
  unsigned int data;
  int success;

  unsigned char sync_period;
  unsigned char reset_latency;
  unsigned char mark_ch;
  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<=0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  mark_ch = (unsigned char) GetChannelMark(); // set one of the 128 channels of all apv to 0xfff (mark a single channel of the frame)

  sync_period = (mpdApvGetFrequency(id) == 1) ? 34 : 69; // synch period in number of clock - 1 (34 @ 40 MHz, 69 @ 20 MHz) @@@ To be checked, ask Paolo

  reset_latency = 15 + mpdApvGetMaxLatency(id); // @@@ To be ckecked, ask paolo for meaning

  data = mark_ch << 24 | sync_period << 16 | mpdGetApvEnableMask(id);

  MPDLOCK;
  vmeWrite32(&MPDp[id]->ApvDaq.Control, data);

  // FIXME: make sure there are no locks in here.
  if( GetFpgaRevision() < 2 )
    data =  GetAdcClockPhase(0) | 	// This works only for ADC board rev 0
      ((GetTriggerMode() & 0x07) << 12) |
      ((GetTriggerNumber() & 0x0F) << 8) | reset_latency;
  else
    data = 
      ((GetCalibLatency() & 0xFF) << 24) |
      ((GetInPathI(MPD_IN_FRONT, MPD_IN_TRIG) & 0x01) << 23) |
      ((GetInPathI(MPD_IN_P0, MPD_IN_TRIG2) & 0x01) << 22) |
      ((GetInPathI(MPD_IN_P0, MPD_IN_TRIG1) & 0x01) << 21) |
      ((GetInPathI(MPD_IN_FRONT, MPD_IN_SYNC) & 0x01) << 17) |
      ((GetInPathI(MPD_IN_P0, MPD_IN_SYNC) & 0x01) << 16) |
      //	  ((test_mode & 0x01) << 15) |
      ((GetTriggerMode() & 0x07) << 12) |
      ((GetTriggerNumber() & 0x0F) << 8) | reset_latency;

  vmeWrite32(&MPDp[id]->ApvDaq.Trig_Gen_Config, data);

  data = (GetOneLevel() << 16) | GetZeroLevel();

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
  unsigned int addr;
  unsigned int data;
  int success;
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
mpdTRIG_GetMissed(int id, unsigned int *missed)
{
  unsigned int base_addr, missed_addr;
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
  unsigned char val;
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


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
 *          Evaristo Cisbani, Paolo Musico
 *          INFN
 *          July 2015
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
#include <stdlib.h>
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

/* */
#define MPD_VERSION_MASK 0xf00f
#define MPD_SUPPORTED_CTRL_FIRMWARE 0x4003

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
/*static*/ int  I2C_SendByte(int id, uint8_t byteval, int start);
/*static*/ int  I2C_ReceiveByte(int id, uint8_t *byteval);
/*static*/ int  I2C_SendStop(int id);
/*static*/ int  I2C_SendNack(int id);

uint32_t
mpdRead32(volatile uint32_t *reg)
{
  uint32_t read=0;
  read = vmeBusRead32(0x09,(uint32_t)reg);
  return read;
}

void
mpdWrite32(volatile uint32_t *reg, uint32_t val)
{
  vmeBusWrite32(0x09,(uint32_t)reg, val);
}

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

STATUS 
mpdInit(UINT32 addr, UINT32 addr_inc, int nmpd, int iFlag)
{

  int i, ii, impd, impd_disc,res, errFlag = 0;
  int boardID = 0;
  int maxSlot = 1;
  int minSlot = 21;
  int trigSrc=0, clkSrc=0, srSrc=0;
  uint32_t csrdata;
  uint32_t csr_const[9]={'C','R',0x08,0x00,0x30,0x00,0x03,0x09,0x04};
  uint32_t rdata, laddr, laddr_inc, laddr_csr, a32addr, a16addr=0;
  volatile struct mpd_struct *mpd;
  volatile struct mpd_struct_csr *mpd_csr;
  uint16_t sdata;
  int noBoardInit=0;
  int useList=0;
  int noFirmwareCheck=0;


  printf("%s: start\n",__FUNCTION__);

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
      // CSR access AM=0x2F or any AM_A24 such as 0x39
#ifdef VXWORKS
      res = sysBusToLocalAdrs(0x2f,(char *)addr,(char **)&laddr_csr);
#else
      res = vmeBusToLocalAdrs(0x2f,(char *)addr,(char **)&laddr_csr); // CSR space
#endif
      if (res != 0) 
	{
#ifdef VXWORKS
	  printf("mpdInit: ERROR in sysBusToLocalAdrs(0x2f,0x%x,&laddr_csr) \n",addr);
#else
	  printf("mpdInit: ERROR in vmeBusToLocalAdrs(0x2f,0x%x,&laddr_csr) \n",addr);
#endif
	  return(ERROR);
	}
      
      //      mpdA24Offset = laddr_csr - addr; //??
    }

  printf("%s: A24 mapping: VME addr=0x%x -> Local 0x%x\n", __FUNCTION__, addr, laddr_csr);
  impd = 0;
  impd_disc = 0;
  for (ii=0;ii<nmpd;ii++) 
    {

      printf("%s: Looking at MPD in slot %d\n",__FUNCTION__,ii);
      if(useList==1)
	{
	  laddr_inc = mpdAddrList[ii] + mpdA24Offset; // not tested yet (EC)
	}
      else
	{
	  laddr_inc = laddr_csr +ii*addr_inc;
	}
      mpd_csr = (struct mpd_struct_csr *) laddr_inc;
      
      // #ifdef NOTSURE
      /* Check if Board exists at that address */

      errFlag = 0;
      for (i=0;i<9; i++) {
#ifdef VXWORKS
	res = vxMemProbe((char *) mpd_csr,VX_READ,sizeof(struct mpd_struct_csr),(char *)csrdata);
#else
	res = vmeMemProbe((char *) &mpd_csr->crcode[0]+4*i,4,(char *) &csrdata);
#endif
	
	if(res < 0) 
	  {
#ifdef VXWORKS
	    printf("mpdInit: WARN: No addressable board at addr=0x%x\n",(UINT32) mpd_csr);
#else
	    printf("mpdInit: WARN: No addressable board at VME addr=0x%x (local 0x%x)\n",
		   (UINT32) addr+ii*addr_inc, (UINT32) mpd_csr);
#endif
	    errFlag = 1;
	    break;
	  }
	else 
	  {
	    if (csrdata != csr_const[i]) {
	      printf("%s: WARN: for board at 0x%x, invalid csr data code 0x%x (expected 0x%x)\n",
		     __FUNCTION__,
		     (UINT32) mpd_csr - mpdA24Offset, csrdata, csr_const[i]);
	      errFlag = 2;
	      break;
	    }
	  }
      } // loop on first csr words

      if (errFlag>0) { continue; } 
      // discovered new board
      impd_disc++;

      boardID=ii;
      
      if((boardID < 0)||(boardID >21)) 
	{
	  printf("%s: ERROR: For Board at 0x%x,  Slot number is not in range: %d\n",
		 __FUNCTION__,(UINT32) mpd_csr - mpdA24Offset, boardID);
	  continue;
	}


      /* read firmware revision */
      rdata = 0;
      for (i=0;i<4;i++) {
	csrdata = vmeRead32( &mpd_csr->revisionID[i]);
	rdata |= ((csrdata&0xff) << (4*(3-i)));
      }

      printf(" MPD Slot %d - Firmware Revision ID = 0x%x\n", boardID, rdata);

      if(!noFirmwareCheck)
	{

	  // Check FPGA firmware version 
	  if( (rdata&MPD_VERSION_MASK) < MPD_SUPPORTED_CTRL_FIRMWARE )
	    {
	      printf("%s: ERROR: Slot %2d: Control FPGA Firmware (0x%02x) not supported by this driver.\n",
		     __FUNCTION__,boardID, rdata & MPD_VERSION_MASK);
	      printf("\tUpdate to 0x%02x to use this driver.\n",MPD_SUPPORTED_CTRL_FIRMWARE);
	      continue;
	    }
	  
	}
      else
	{
	  
	  // Check FPGA firmware version 
	  if( (rdata&MPD_VERSION_MASK) < MPD_SUPPORTED_CTRL_FIRMWARE )
	    {
	      printf("%s: WARN: Slot %2d: Control FPGA Firmware (0x%02x) not supported by this driver (ignored).\n",
		     __FUNCTION__,boardID, rdata & MPD_VERSION_MASK);
	    }
	}

      fMpd[boardID].FpgaRevision = rdata;

      /* time revision */
      rdata = 0;
      for (i=0;i<4;i++) {
	csrdata = vmeRead32( &mpd_csr->revisionTime[i]);
	rdata |= ((csrdata&0xff) << (4*(3-i))); 
      }
	
      fMpd[boardID].FpgaCompileTime |= rdata;
      printf(" MPD Slot %d - Firmware Revision Time: %d (s)\n", boardID, fMpd[boardID].FpgaCompileTime);

      // set it, if it is presents in config file
      if (mpdGetNumberAPV(boardID)<=0) { // not in config file (to be improved)
	printf(" -- MPD in slot %d is NOT in config file, drop it\n",boardID);
	continue;
      }
      printf(" ++ MPD in slot %d is in config file, INIT IT\n",boardID);
      // MPDp[boardID] = (struct mpd_struct *)(laddr_inc);
      //	  mpdRev[boardID] = rdata&MPD_VERSION_MASK;
      //	  mpdProcRev[boardID] = proc_version;
      mpdID[impd] = boardID; 
      if(boardID >= maxSlot) maxSlot = boardID;
      if(boardID <= minSlot) minSlot = boardID;
      

      //      impd++;
      //    } // loop on mpd slots
  

      /* Hard Reset of all MPD boards in the Crate */

      /* Calculate the A32 Offset for MPD mapping*/
      a32addr = mpdA32Base + boardID*mpdA32Offset;
#ifdef VXWORKS
      res = sysBusToLocalAdrs(0x09,(char *)a32addr,(char **)&laddr);
      if (res != 0) 
	{
	  printf("mpdInit: ERROR in sysBusToLocalAdrs(0x09,0x%x,&laddr) \n",mpdA32Base);
	  //	  return(ERROR);
	  continue;
	} 
      else 
	{
	  //	  mpdA32Offset = laddr - mpdA32Base; // ??? (EC)
	}
#else

#ifdef OLDWAY
      res = vmeBusToLocalAdrs(0x09,(char *)a32addr,(char **)&laddr);
      if (res != 0) 
	{
	  printf("mpdInit: ERROR in vmeBusToLocalAdrs(0x09,0x%x,&laddr) \n",a32addr);
	  //	  return(ERROR);
	  continue;
	} 
      else 
	{
	  //	  mpdA32Offset = laddr - mpdA32Base; // ??? (EC)
	}
#else
      laddr = a32addr;
#endif /* OLDWAY */
#endif

      MPDp[boardID] = (struct mpd_struct *)(laddr); // MPD A32 memory map

      printf("Initialized MPD %2d  Slot #%2d at VME address 0x%08x (local 0x%08x) \n",
	     impd,boardID,
	     (UINT32) a32addr, // (UINT32) MPDp[boardID]-mpdA24Offset, // ??
	     (UINT32) MPDp[boardID]);

      
      /* Program an A32 access address for this MPD's FIFO */      
      MPDpd[boardID] = (uint32_t *)(laddr);  /* Set a pointer to the FIFO */



      if(!noBoardInit)
	{
	  //	  vmeWrite32(&(MPDp[mpdID[ii]]->adr32),(a32addr>>16) + 1);  /* Write the register and enable */
	  
	  /* Set Default Block Level to 1 */
	  //  vmeWrite32(&(MPDp[mpdID[ii]]->blk_level),1);
	}
  
      impd++;
    }


  nmpd = impd;
  mpdBlockLevel=1;


  /* If there are more than 1 MPD in the crate then setup the Multiblock Address
     window. This must be the same on each board in the crate */

  if(nmpd > 100)  // not implemented  
    {
#define MPD_MAX_A32_MEM 0x8000000 // should be mpdA32Offset
      a32addr = mpdA32Base + (nmpd+1)*MPD_MAX_A32_MEM; /* set MB base above individual board base */
#ifdef VXWORKS
      res = sysBusToLocalAdrs(0x09,(char *)a32addr,(char **)&laddr);
      if (res != 0) 
	{
	  printf("mpdInit: ERROR in sysBusToLocalAdrs(0x09,0x%x,&laddr)/for multiblock\n",a32addr);
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
	      //	      vmeWrite32(&(MPDp[impd]->adr_mb),
	      //		 (a32addr+MPD_MAX_A32MB_SIZE) + (a32addr>>16) + MPD_A32_ENABLE);
	    }
	}    
      /* Set First Board and Last Board */
      mpdMaxSlot = maxSlot;
      mpdMinSlot = minSlot;
      if(!noBoardInit)
	{
	  //	  vmeWrite32(&(MPDp[minSlot]->ctrl1),
	  //	     mpdRead32(&(MPDp[minSlot]->ctrl1)) | MPD_FIRST_BOARD);
	  // vmeWrite32(&(MPDp[maxSlot]->ctrl1),
	  //	     mpdRead32(&(MPDp[maxSlot]->ctrl1)) | MPD_LAST_BOARD);
	}    
    }

  if(!noBoardInit)
    mpdInited = nmpd;

  if(nmpd > 0) {
    printf("%s: %d MPD(s) initialized, %d discovered\n",__FUNCTION__,nmpd,impd_disc);
  }
  
  if(errFlag > 0) 
    {
      printf("mpdInit: WARN: Unable to initialize all requested MPD Modules (%d)\n",
	     nmpd);
      return(ERROR);
    } 
  else 
    {
      return(OK);
    }
}

int mpdGetNumberMPD() { return mpdInited; };



int mpdGetNumberAPV(int id) { return (uint16_t) fMpd[id].nAPV; };
void mpdSetNumberAPV(int id, uint16_t v) { fMpd[id].nAPV = v; };


int
mpdCheckAddresses(int id)
{
  uint32_t offset=0, expected=0, base=0;
  
  //  if(id==0) id=mpdID[0];
  if((id<0) || (id>21) || (MPDp[id] == NULL)) 
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
  /*
  if(i>=nmpd)
    {
      printf("%s: ERROR: Index (%d) >= MPDs initialized (%d).\n",
	     __FUNCTION__,i,nmpd);
      return ERROR;
    }
  */
  return mpdID[i];
}

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

void mpdSetTriggerMode(int id, int lat, int num) 
{
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

void mpdSetAcqMode(int id, char *name) 
{
  fMpd[id].fAcqMode = 0; // disabled
  if(strcmp(name,"ramtest")==0) fMpd[id].fAcqMode = MPD_DAQ_RAM_TEST;
  if(strcmp(name,"histo")==0) fMpd[id].fAcqMode = MPD_DAQ_HISTO;
  if(strcmp(name,"event")==0) fMpd[id].fAcqMode = MPD_DAQ_EVENT;
  if(strcmp(name,"process")==0) fMpd[id].fAcqMode = MPD_DAQ_PROCESS;
  if(strcmp(name,"sample")==0) fMpd[id].fAcqMode = MPD_DAQ_SAMPLE;
  if(strcmp(name,"sync")==0) fMpd[id].fAcqMode = MPD_DAQ_SYNC;

  printf("%s: Acquisition Mode = 0x%x (%s)\n",
	 __FUNCTION__,fMpd[id].fAcqMode, name);
};
int mpdGetAcqMode(int id) { return fMpd[id].fAcqMode; };

  
void mpdSetInPath0(int id, int t1P0, int t2P0, int tFront, int sP0, int sFront) 
{
  fMpd[id].fInPath[MPD_IN_P0][MPD_IN_TRIG1] = t1P0;
  fMpd[id].fInPath[MPD_IN_P0][MPD_IN_TRIG2] = t2P0;
  fMpd[id].fInPath[MPD_IN_FRONT][MPD_IN_TRIG] = tFront;
  fMpd[id].fInPath[MPD_IN_P0][MPD_IN_SYNC] = sP0;
  fMpd[id].fInPath[MPD_IN_FRONT][MPD_IN_SYNC] = sFront;
};
void mpdSetInPath(int id, int conn, int signal, int val) { fMpd[id].fInPath[conn%2][signal%3] = val; };
int  mpdGetInPath(int id, int conn, int signal) { return fMpd[id].fInPath[conn%2][signal%3]; };
int  mpdGetInPathI(int id, int conn, int signal) { return ((fMpd[id].fInPath[conn%2][signal%3] == 1) ? 1 : 0); };

void mpdSetInputLevel(int id, int conn, short val) {
  if (conn>=0 && conn<2) {
    fMpd[id].fInLevelTTL[conn]=val;
  }
};

short mpdGetInputLevel(int id, int conn) { return fMpd[id].fInLevelTTL[conn]; };

void mpdSetOutputLevel(int id, int conn, short val) {
  if (conn>=0 && conn<2) {
    fMpd[id].fOutLevelTTL[conn]=val;
  }
};

short mpdGetOutputLevel(int id, int conn) { return fMpd[id].fOutLevelTTL[conn];};

uint32_t mpdGetFpgaRevision(int id) 
{
  if (fMpd[id].FpgaRevision == 99999) {
    printf("%s: Fpga revision not set yet, something wrong! exit(%d)",
	   __FUNCTION__,0);
/*     exit(0); */
  }
  return fMpd[id].FpgaRevision; 
}
void mpdSetFpgaRevision(int id, uint32_t r) { fMpd[id].FpgaRevision = r; }

uint32_t mpdGetHWRevision(int id) 
{
  uint32_t d = mpdGetFpgaRevision(id);
  return ((d>>24)&0xff);
}

uint32_t mpdGetFWRevision(int id) 
{
  uint32_t d = mpdGetFpgaRevision(id);
  return (d&0xff);
}

uint32_t mpdGetFpgaCompileTime(int id) {return fMpd[id].FpgaCompileTime; }
void mpdSetFpgaCompileTime(int id, uint32_t t) { fMpd[id].FpgaCompileTime = t; }

int 
mpdLM95235_Read(int id, double *core_t, double *air_t)
{
  const uint8_t LM95235_i2c_addr  = 0x4C;
  const uint8_t Local_TempS_MSB_addr  = 0x00;	// Read only
  const uint8_t Local_TempS_LSB_addr  = 0x30;	// Read only
  const uint8_t Remote_TempU_MSB_addr  = 0x31;	// Read only
  const uint8_t Remote_TempU_LSB_addr  = 0x32;	// Read only
  const uint8_t ConfigReg1_addr  = 0x03;	// also 0x09
  const uint8_t OneShot_addr  = 0x0F;	// Write only
  const uint8_t Status1_addr  = 0x02;	// Read only

  uint8_t val, val2;
  int success, retry_count;
  // if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  val = 0x40; // standby
  success = mpdI2C_ByteWrite(id, (uint8_t)(LM95235_i2c_addr<<1), ConfigReg1_addr, 1, &val);
  success = mpdI2C_ByteWrite(id, (uint8_t)(LM95235_i2c_addr<<1), OneShot_addr, 0, &val);
  success = mpdI2C_ByteWrite(id, (uint8_t)(LM95235_i2c_addr<<1), Status1_addr, 0, &val);
  val = 0x80;
  retry_count = 0;
  //  while( (val & 0x80) && (retry_count++ < 1) )
    success = mpdI2C_ByteRead1(id, (uint8_t)(LM95235_i2c_addr<<1), &val);

  success = mpdI2C_ByteWrite(id, (uint8_t)(LM95235_i2c_addr<<1), Remote_TempU_MSB_addr, 0, &val);
  success = mpdI2C_ByteRead1(id, (uint8_t)(LM95235_i2c_addr<<1), &val);
  success = mpdI2C_ByteWrite(id, (uint8_t)(LM95235_i2c_addr<<1), Remote_TempU_LSB_addr, 0, &val2);
  success = mpdI2C_ByteRead1(id, (uint8_t)(LM95235_i2c_addr<<1), &val2);
  *core_t = (double) val + (double) val2/256.;


  success = mpdI2C_ByteWrite(id, (uint8_t)(LM95235_i2c_addr<<1), Local_TempS_MSB_addr, 0, &val);
  success = mpdI2C_ByteRead1(id, (uint8_t)(LM95235_i2c_addr<<1), &val);
  success = mpdI2C_ByteWrite(id, (uint8_t)(LM95235_i2c_addr<<1), Local_TempS_LSB_addr, 0, &val2);
  success = mpdI2C_ByteRead1(id, (uint8_t)(LM95235_i2c_addr<<1), &val2);
  *air_t = (double) val + (double) val2/256.;
  val = 0x0; //normal operation
  success = mpdI2C_ByteWrite(id, (uint8_t)(LM95235_i2c_addr<<1), ConfigReg1_addr, 1, &val);
  return success;

}


/****************************
 * LOW LEVEL routines 
 ****************************/


/* I2C methods */
void mpdSetI2CSpeed(int id, int val) { fMpd[id].fI2CSpeed = val; };
int  mpdGetI2CSpeed(int id) { return fMpd[id].fI2CSpeed; };
void mpdSetI2CMaxRetry(int id, int val) { fMpd[id].fI2CMaxRetry = val; };
int  mpdGetI2CMaxRetry(int id) { return fMpd[id].fI2CMaxRetry; };

int 
mpdI2C_ApvReset(int id)
{
  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  MPDUNLOCK;
  mpdWrite32(&MPDp[id]->I2C.ApvReset, MPD_I2C_APVRESET_ASYNC_CLEAR);
  mpdWrite32(&MPDp[id]->I2C.ApvReset, MPD_I2C_APVRESET_ASYNC_SET);
  MPDUNLOCK;

  return OK;
}

int 
mpdI2C_Init(int id)
{
  uint32_t data, rdata;
  int success = OK;
  double core_t, air_t;

  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
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
  mpdWrite32(&MPDp[id]->I2C.Control, 0); /* Disable I2C Core and interrupts */

  int ispeed = mpdGetI2CSpeed(id);

  data = (ispeed & 0xFF);
  mpdWrite32(&MPDp[id]->I2C.Clock_Prescaler_low, data);
  
  rdata = mpdRead32(&MPDp[id]->I2C.Clock_Prescaler_low);

  printf("%s: i2c low prescaler register set/read : %d / %d\n",
	 __FUNCTION__,data,rdata);

  data = (ispeed>>8) & 0xff;
  mpdWrite32(&MPDp[id]->I2C.Clock_Prescaler_high, data);
  rdata = mpdRead32(&MPDp[id]->I2C.Clock_Prescaler_high);

  printf("%s: i2c high prescaler register set/read : %d / %d\n",
	 __FUNCTION__,data,rdata);

  printf("%s: i2c speed prescale = %d, (period = %f us, frequency = %f kHz)\n",
	 __FUNCTION__,ispeed, ispeed/10., 10000./ispeed);
  
  mpdWrite32(&MPDp[id]->I2C.Control, MPD_I2C_CONTROL_ENABLE_CORE);


  MPDUNLOCK;
  usleep(500);
  mpdLM95235_Read(id, &core_t, &air_t);
  printf("Board temperatures: core=%.2f air=%.2f (dec celsius)\n",core_t,air_t);

  return success;

}

int 
mpdI2C_ByteWrite(int id, uint8_t dev_addr, uint8_t int_addr, 
		 int ndata, uint8_t *data)
{
  int success, i;
  // if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
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
	  return (success | 1);
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
	    return (success | 2);
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
  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
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
  if((MPDp[id]==NULL) || (id<0) || (id>21))
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
  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  
  //  if( (success = I2C_SendByte(id, (uint8_t)(dev_addr & 0xFE), 1)) != OK )
  if( (success = I2C_SendByte(id, (uint8_t)(dev_addr & 0x1), 1)) != OK )
    return I2C_SendStop(id);
  
  usleep(100);
  if( (success = I2C_ReceiveByte(id, data)) != OK )
    return I2C_SendStop(id);
  usleep(100);

  return I2C_SendStop(id);
}

/*static*/ int 
I2C_SendByte(int id, uint8_t byteval, int start)
{
  int rval=OK, retry_count;
  volatile uint32_t data=0;
  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  MPDLOCK;
  //  printf(" MPD addr I2C.txrx : 0x%x\n", &MPDp[id]->I2C.TxRx - &MPDp[id]->SdramFifo[0]);
  mpdWrite32(&MPDp[id]->I2C.TxRx, byteval);

  if( start )
    mpdWrite32(&MPDp[id]->I2C.CommStat, MPD_I2C_COMMSTAT_START_WRITE);
  else
    mpdWrite32(&MPDp[id]->I2C.CommStat, MPD_I2C_COMMSTAT_WRITE);

  retry_count = 0;
  data = 0x00000002;

  while( (data & 0x00000002) != 0 && retry_count < mpdGetI2CMaxRetry(id) )
    {
      usleep(10);
      data = mpdRead32(&MPDp[id]->I2C.CommStat);

      retry_count++;
    }

  if( retry_count >= mpdGetI2CMaxRetry(id) )
    rval = -10;

  if( data & MPD_I2C_COMMSTAT_NACK_RECV )	/* NACK received */
    rval = -20;

  MPDUNLOCK;

  return rval;
}

/*static*/ int 
I2C_ReceiveByte(int id, uint8_t *byteval)
{
  int retry_count;
  int rval=0;
  uint32_t data=0;
  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  MPDLOCK;
  mpdWrite32(&MPDp[id]->I2C.CommStat, MPD_I2C_COMMSTAT_READ);

  retry_count = 0;
  data = 0x00000002;
  while( (data & 0x00000002) != 0 && retry_count < mpdGetI2CMaxRetry(id) )
    {
      usleep(100);
      data = mpdRead32(&MPDp[id]->I2C.CommStat);
      retry_count++;
    }

  if( retry_count >= mpdGetI2CMaxRetry(id) )
    rval = -10;

  if( data & MPD_I2C_COMMSTAT_NACK_RECV )	/* NACK received */
    rval = -20;

  data = mpdRead32(&MPDp[id]->I2C.TxRx);
  
  *byteval = data;
  MPDUNLOCK;

  return rval;

}

/*static*/ int 
I2C_SendStop(int id)
{
  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  MPDLOCK;
  mpdWrite32(&MPDp[id]->I2C.CommStat, MPD_I2C_COMMSTAT_STOP);
  MPDUNLOCK;

  return OK;
}

/*static*/ int 
I2C_SendNack(int id)
{
  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }
  
  MPDLOCK;
  mpdWrite32(&MPDp[id]->I2C.CommStat, MPD_I2C_COMMSTAT_NACK);
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

int  mpdApvGetSample(int id, int ia) { return fApv[id][ia].fNumberSample; };

void mpdApvSetSampleLeft(int id, int ia) { fApv[id][ia].fReadCount = fApv[id][ia].fNumberSample; };
void mpdApvDecSampleLeft(int id, int ia, int n) { fApv[id][ia].fReadCount -= n; };
int  mpdApvReadDone(int id, int ia) { 
  if (fApv[id][ia].fReadCount <= 0) return TRUE; 
  return FALSE;
};
int  mpdApvGetSampleLeft(int id, int ia) { return (fApv[id][ia].fNumberSample - mpdApvGetBufferSample(id,ia)); };
int  mpdApvGetSampleIdx(int id, int ia) { return (fApv[id][ia].fNumberSample - fApv[id][ia].fReadCount); };

int mpdApvGetAdc(int id, int ia) { return fApv[id][ia].adc; };

int
mpdAPV_Reset101(int id)
{
  uint32_t data;
  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  MPDLOCK;

  data = mpdRead32(&MPDp[id]->ApvDaq.Trig_Gen_Config);

  data |= MPD_APVDAQ_TRIGCONFIG_ENABLE_MACH;	// Enable trig machine
  data |= SOFTWARE_CLEAR_MASK;

  mpdWrite32(&MPDp[id]->ApvDaq.Trig_Gen_Config, data);

  data &= ~SOFTWARE_CLEAR_MASK;
  data &= ~MPD_APVDAQ_TRIGCONFIG_ENABLE_MACH;	// Disable trig machine

  mpdWrite32(&MPDp[id]->ApvDaq.Trig_Gen_Config, data);

  MPDUNLOCK;

  return OK;
}

/*
 * return true if apv is present
 *   (EC: to be improved not yet 100% reliable) 
 */
int
mpdAPV_Try(int id, uint8_t apv_addr) // i2c addr
{

  int timeout;
  uint8_t x = apv_addr+32;
  int ret;

  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  timeout=-1;
  
  do {
    ret = mpdAPV_Write(id, apv_addr, latency_addr,  x);
    timeout++;
  } while (((ret == -20) || (ret == -19) || (ret == -18)) && (timeout<20));

  printf("%s: timeout %d : %d ret = %d\n",__FUNCTION__,timeout,x, ret);

  //  return ((mpdAPV_Read(id, apv_addr, mode_addr, &x) == OK) ? 1 : 0);
  //	return APV_Write(apv_addr, mode_addr, def_Mode);

  return ret;

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
  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }
  
  // print information (debug only, should be done in verbose mode only)
  printf("%s: MPD %d Blind scan on %d apvs: \n",__FUNCTION__,id, MPD_MAX_APV); // in principle can be more

  for(iapv=0;iapv<MPD_MAX_APV;iapv++) 
    {
      if ( mpdAPV_Try(id, iapv)>-1 ) 
	{
	  printf("%s : MPD %d APV i2c = %d found in MPD in slot %d\n",
		 __FUNCTION__, id, iapv, id);
	}
    }
  printf("%s: MPD %d Blind scan done\n",__FUNCTION__,id);

  mpdResetApvEnableMask(id);

  nApv[id]=0;
  for(iapv=0; iapv<fMpd[id].nAPV; iapv++)
    {
      printf("%s: Try %2d %2d : ", __FUNCTION__, fApv[id][iapv].i2c, fApv[id][iapv].adc);
      
      if ( mpdAPV_Try(id, fApv[id][iapv].i2c)>-1 && fApv[id][iapv].adc>-1 ) 
	{
	  printf("%s: %d matched in MPD in slot %d\n",
		 __FUNCTION__, fApv[id][iapv].i2c, id);

	  mpdSetApvEnableMask(id, (1 << fApv[id][iapv].adc));
	  printf("%s: APV enable mask 0x%04x\n", 
		 __FUNCTION__,mpdGetApvEnableMask(id));
	  nApv[id]++;
	}
      else 
	{
	  printf("%s: MPD %d APV i2c = %d does not respond.  It is removed from db\n",
		 __FUNCTION__,id, fApv[id][iapv].i2c);
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
  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
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
  if((MPDp[id]==NULL) || (id<0) || (id>21))
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

  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
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

  //#ifdef DOTHISDIFFERENTLY
  // need improvement
    fApv[id][apv_index].fNumberSample = mpdGetTriggerNumber(id)*mpdApvGetPeakMode(id); // must be set !!
    mpdApvBufferFree(id,apv_index);
    mpdApvBufferAlloc(id,apv_index); // alloc readout buffer
 //#endif /* DOTHISDIFFERENTLY */

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
  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
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
  /*
    if(id==0) id=mpdID[0];
    if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
    printf("%s: ERROR: MPD in slot %d is not initialized.\n",
    __FUNCTION__,id);
    return;
    }
  */

  memcpy((void *)&fApv[id][nApv[id]], &v, sizeof(ApvParameters));
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
  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
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
  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
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

void
mpdApvBufferAlloc(int id, int ia) 
{
  fApv[id][ia].fBufSize = 15*fApv[id][ia].fNumberSample*(EVENT_SIZE+2); // at least 6 times larger @@@ increased to 15 -- need improvement
  fApv[id][ia].fBuffer = (uint32_t *) malloc(fApv[id][ia].fBufSize*sizeof(uint32_t));
  fApv[id][ia].fBi1 = 0;
  MPD_DBG("Fifo %d, buffer allocated with word size %d\n",fApv[id][ia].adc, fApv[id][ia].fBufSize);
}

void 
mpdApvBufferFree(int id, int ia) {
  if (fApv[id][ia].fBuffer != 0) {
    free(fApv[id][ia].fBuffer);
    MPD_DBG("Fifo %d, buffer released\n",fApv[id][ia].adc);
  };
}

void 
mpdApvIncBufferPointer(int id, int ia, int b) {
  fApv[id][ia].fBi1 += b;
}

uint32_t*
mpdApvGetBufferPointer(int id, int ia, int ib) 
{
  MPD_DBG("Fifo %d, retrieved pointer from position %d , size is %d\n",fApv[id][ia].adc, ib, fApv[id][ia].fBufSize);
  if (ib<fApv[id][ia].fBufSize) { // probably not required !!
    return &(fApv[id][ia].fBuffer[ib]);
  }
  MPD_ERR("Fifo %d, index %d is out of range (%d)\n",fApv[id][ia].adc, ib, fApv[id][ia].fBufSize);
  exit(1);
}

int 
mpdApvGetBufferSample(int id, int ia) 
{
  int ix = fApv[id][ia].fBi1 / EVENT_SIZE;
  MPD_DBG("Fifo = %d has %d samples (%d bytes) stored\n",fApv[id][ia].adc, ix, fApv[id][ia].fBi1);
  return ix;

}

uint32_t*
mpdApvGetBufferPWrite(int id, int ia) 
{
  return mpdApvGetBufferPointer(id, ia, fApv[id][ia].fBi1);
}

uint32_t 
mpdApvGetBufferElement(int id, int ia, int ib)
{
  return fApv[id][ia].fBuffer[ib];
}

int 
mpdApvGetBufferAvailable(int id, int ia) 
{
  return fApv[id][ia].fBufSize - fApv[id][ia].fBi1 - 1; // @@@@ Apr/2013 tobe verified
}

int 
mpdApvGetBufferLength(int id, int ia) 
{
  return fApv[id][ia].fBi1;
}

int 
mpdApvGetEventSize(int id, int ia) 
{
  return EVENT_SIZE; // TO BE CHANGED (variable size !!!)
}


/**
 * Set the number of samples / event expected from the single Apv
 */
int 
mpdArmReadout(int id) 
{
  uint32_t iapv=0;
  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
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
  fMpd[id].fReadDone = FALSE;

  return 0;

}

//======================================================
// trigger methods

int 
mpdTRIG_BitSet(int id) 
{ 
  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
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
  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
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
  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  mark_ch = (uint8_t) mpdGetChannelMark(id); // set one of the 128 channels of all apv to 0xfff (mark a single channel of the frame)

  sync_period = (mpdApvGetFrequency(id) == 1) ? 34 : 69; // synch period in number of clock - 1 (34 @ 40 MHz, 69 @ 20 MHz) @@@ To be checked, ask Paolo

  reset_latency = 15 + mpdApvGetMaxLatency(id); // @@@ To be ckecked, ask paolo for meaning

  data = mark_ch << 24 | sync_period << 16 | mpdGetApvEnableMask(id);

  printf("%s: Control Addr = 0x%x\n",__FUNCTION__,data);

  MPDLOCK;
  mpdWrite32(&MPDp[id]->ApvDaq.Control, data);

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

  printf("%s: Trig Gen = 0x%x\n",__FUNCTION__,data);

  mpdWrite32(&MPDp[id]->ApvDaq.Trig_Gen_Config, data);

  data = (mpdGetOneLevel(id) << 16) | mpdGetZeroLevel(id);

  printf("%s: Logic Threshold = 0x%x\n",__FUNCTION__,data);

  mpdWrite32(&MPDp[id]->ApvDaq.Logic_Thresholds, data);
  MPDUNLOCK;

  return OK;
}

int 
mpdTRIG_Disable(int id)
{
  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }
  
  MPDLOCK;
  mpdWrite32(&MPDp[id]->ApvDaq.Control, 0);

  mpdWrite32(&MPDp[id]->ApvDaq.Trig_Gen_Config, 0);
  MPDUNLOCK;

  return OK;

}

int
mpdTRIG_PauseEnable(int id, int time)
{
  mpdTRIG_Enable(id);
  usleep(time);
  MPDLOCK;
  mpdWrite32(&MPDp[id]->ApvDaq.Trig_Gen_Config, 0);
  MPDUNLOCK;
  return OK;
}


int 
mpdTRIG_GetMissed(int id, uint32_t *missed)
{
  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }
  
  MPDLOCK;
  *missed = mpdRead32(&MPDp[id]->ApvDaq.Missed_Trigger);
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
  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  printf("%s: start\n",__FUNCTION__);

  //      mpdI2C_ByteWrite(0xF0 , (apv2_delay & 0x3F) | 0x40, 0, &val);      // CR0: APV2 out
  mpdI2C_ByteWrite(id, 0xF0 , 0x40, 0, &val);    // CR0: APV2 out not delayed
  usleep(10000);
  //      mpdI2C_ByteWrite(0xF2 , 0x40, 0, &val);    // CR1: ADC1 out not delayed
  mpdI2C_ByteWrite(id, 0xF2 , (apv1_delay & 0x3F) | 0x40, 0, &val);      // CR1: ADC1 clock delayed
  usleep(10000);
  //      mpdI2C_ByteWrite(0xF4 , 0x40, 0, &val);    // CR2: ADC2 out not delayed
  mpdI2C_ByteWrite(id, 0xF4 , (apv2_delay & 0x3F) | 0x40, 0, &val);      // CR2: ADC2 clock delayed
  usleep(10000);
  mpdI2C_ByteWrite(id, 0xF6 , 0x00, 0, &val);    // CR3: Not used output
  usleep(10000);
  //      mpdI2C_ByteWrite(0xF8 , (apv1_delay & 0x3F) | 0x40, 0, &val);      // CR4: APV1 out
  mpdI2C_ByteWrite(id, 0xF8 , 0x40, 0, &val);    // CR4: APV1 out not delayed
  usleep(10000);
  mpdI2C_ByteWrite(id, 0xFA , 0x00, 0, &val);    // GCR (40 MHz)
  usleep(10000);

  printf("%s: end\n",__FUNCTION__);

  return 0;
}

//======================================================
// adc methods

#define MPD_ADC_TOUT 1000
#define MPD_ADC_USLEEP 50

/**
 * init and configure ADC 
 */

int mpdADS5281_Config(int id) {

  int j;
  for (j=0;j<2;j++) {
    if (mpdADS5281_SetParameters(id,j) != OK) {
      printf("ERR: adc %d set parameter failed on mpd %d\n",j,id);
    };
    switch( mpdGetAdcPattern(id,j) ) {
    case MPD_ADS5281_PAT_NONE: mpdADS5281_Normal(id,j); break;
    case MPD_ADS5281_PAT_SYNC: mpdADS5281_Sync(id,j); break;
    case MPD_ADS5281_PAT_DESKEW: mpdADS5281_Deskew(id,j); break;
    case MPD_ADS5281_PAT_RAMP: mpdADS5281_Ramp(id,j); break;
    }
    if( mpdGetAdcInvert(id,j) ) {
      mpdADS5281_InvertChannels(id,j);
    } else {
      mpdADS5281_NonInvertChannels(id,j);
    }

    if( mpdADS5281_SetGain(id,j, 
			   mpdGetAdcGain(id,j,0), 
			   mpdGetAdcGain(id,j,1), 
			   mpdGetAdcGain(id,j,2), 
			   mpdGetAdcGain(id,j,3),
			   mpdGetAdcGain(id,j,4), 
			   mpdGetAdcGain(id,j,5), 
			   mpdGetAdcGain(id,j,6), 
			   mpdGetAdcGain(id,j,7)) != OK )
      printf("WRN: Set ADC Gain %d failed on mpd %d\n",j,id);
  }

  return OK;

}

/**
 * adc = 0 or 1 (first or second adc in MPD)
 */
int 
mpdADS5281_Set(int id, int adc, uint32_t val) 
{
/*   int success, retry_count; */
  uint32_t data;

  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  data = (adc == 0) ? 0x40000000 : 0x80000000 ;
  data |= val;

  MPDLOCK;
  mpdWrite32(&MPDp[id]->AdcConfig, data);
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
  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  printf("%s: Board= %d ADC= %d Inverted Polarity\n",
	 __FUNCTION__,fMpd[id].fSlot, adc);
  return mpdADS5281_Set(id, adc, 0x2400FF);
}

int 
mpdADS5281_NonInvertChannels(int id, int adc)	/* adc == 0, 1 */
{
  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  printf("%s: Board= %d ADC= %d Direct Polarity\n",
	 __FUNCTION__, fMpd[id].fSlot, adc);
  return mpdADS5281_Set(id, adc, 0x240000);
  
}

int 
mpdADS5281_SetParameters(int id, int adc)	/* adc == 0, 1 */
{
  int success;
  
  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

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

  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  MPD_MSG("Board= %d ADC= %d No Pattern (Normal Acq)\n",fMpd[id].fSlot, adc);
  if ((success = mpdADS5281_Set(id, adc, 0x450000)) != OK) { return success; }
  usleep(MPD_ADC_USLEEP);
  return mpdADS5281_Set(id, adc, 0x250000);
}

int 
mpdADS5281_Sync(int id, int adc)	/* adc == 0, 1 */
{
  int success;

  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  MPD_MSG("Board= %d ADC= %d Sync Pattern\n",fMpd[id].fSlot, adc);
  if ((success = mpdADS5281_Set(id, adc, 0x250000)) != OK) { return success; }
  usleep(MPD_ADC_USLEEP);
  return mpdADS5281_Set(id, adc, 0x450002);
}

int 
mpdADS5281_Deskew(int id, int adc)	/* adc == 0, 1 */
{
  int success;

  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  MPD_MSG("Board= %d ADC= %d Deskew Pattern\n",fMpd[id].fSlot, adc);
  if ((success = mpdADS5281_Set(id, adc,0x250000)) != OK) { return success; }
  usleep(MPD_ADC_USLEEP);
  return mpdADS5281_Set(id, adc, 0x450001);
}

int 
mpdADS5281_Ramp(int id, int adc)	/* adc == 0, 1 */
{
  int success;

  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

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

  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

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

  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  if( ch >= 8 ) block = 1;

  if (val<0) {
    for(i = 0; i<4096; i++) data[i] = i;
  } else {
    for(i = 0; i<4096; i++) data[i] = val;
  }

  MPDLOCK;
#ifdef BLOCK_TRANSFER
  success = BUS_BlockWrite(addr, 4096, data, &j);
  if( j != (4096) ) success = BUS_GENERIC_ERROR;
#else

  int ntimes = 4096 / 64;
  for(i = 0; i<ntimes; i++) {
    for(j = 0; j<64; j++) {	/* single word transfer */
      mpdWrite32(&MPDp[id]->Histo.block[block].Memory[i*64+j], data[i*64+j]);
    }
  }
#endif
  MPDUNLOCK;
  return success;
}

int 
mpdHISTO_Start(int id, int ch)	/* ch == 0, 15 */
{
  uint32_t data;
  int block=0;

  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  if(ch >= 8) block = 1;

  data = 0x80 | (ch & 0x07);

  MPDLOCK;
  mpdWrite32(&MPDp[id]->Histo.block[block].CSR, data);
  MPDUNLOCK;

  return OK;
}

int 
mpdHISTO_Stop(int id, int ch)	/* ch == 0, 15 */
{
  uint32_t data;
  int block=0;

  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  if(ch >= 8) block = 1;

  data = (ch & 0x07);

  MPDLOCK;
  mpdWrite32(&MPDp[id]->Histo.block[block].CSR, data);
  MPDUNLOCK;

  return OK;
}

int 
mpdHISTO_GetIntegral(int id, int ch, uint32_t *integral)	/* ch == 0, 15 */
{
  int block=0;

  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  if(ch >= 8) block = 1;
  
  MPDLOCK;
  *integral = mpdRead32(&MPDp[id]->Histo.block[block].Histo_Count);
  MPDUNLOCK;

  return OK;
}

int 
mpdHISTO_Read(int id, int ch, uint32_t *histogram)	/* ch == 0, 15; uint32_t histogram[4096] */
{
  int success=OK, i, j;
  int block=0;

  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  if( ch >= 8 ) block=1;
  MPDLOCK;
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
      histogram[i*64+j] = mpdRead32(&MPDp[id]->Histo.block[block].Memory[i*64+j]);
    }
  }
#endif
  MPDUNLOCK;
  return success;
}


//======================================================
// Daq-Readout methods


/**
 * Readout Fifo
 * Standard Event Mode (no zero suppression or pedestal subtraction)
 * Return 0 when something has been read, error otherwise
 */
int 
mpdFIFO_ReadSingle(int id, 
		   int channel,     // apv channel (FIFO)
		   uint32_t *dbuf, // data buffer
		   int *wrec,      // max number of words to get / return words received 
		   int max_retry)   // max number of retry for timeout
//			   int &err)        // return error code
//			   int &n_events)   // number of events
{
  
  int success=OK, i, size;
  int nwords; // words available in fifo
  int wmax; // maximum word acceptable

  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  wmax = *wrec;
  *wrec=0; // returned words

  nwords = 0;

  i = 0;
  while( (nwords <= 0) && (i <= max_retry) ) {
    if( max_retry > 0 ) i++;
    success = mpdFIFO_GetNwords(id, channel, &nwords);
    if( success != OK ) return success;
  }

  printf("%s: number of words to be read %d\n",__FUNCTION__,nwords);
  size = (nwords < wmax) ? nwords : wmax;

  MPD_DBG("fifo ch = %d, words in fifo= %d, retries= %d (max %d)\n",channel, nwords,i, max_retry);

  if( i > max_retry ) {
    MPD_ERR(" max retry = %d, count=%d nword=%d\n", max_retry, i, nwords);
    return ERROR;
  }

  MPDLOCK;
#ifdef BLOCK_TRANSFER
  success = BUS_BlockRead(fifo_addr, size, dbuf, &wrec); // trasfer 4 byte words!!
  MPD_DBG("Block Read fifo ch = %d, words requested = %d, returned = %d\n", channel, size, wrec);
#else
  for(i=0; i<size; i++) {
    dbuf[i] = mpdRead32(&MPDp[id]->ApvDaq.Data_Ch[channel][i*4]);
    *wrec+=1;
  }
  MPD_DBG("Read apv = %d, wrec = %d, success = %d\n", channel, *wrec, success);
#endif
  MPDUNLOCK;

  if( *wrec != size ) {
    MPD_DBG("Count Mismatch: %d expected %d\n", *wrec, size);
    return ERROR;
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

int 
mpdFIFO_ReadSingle0(int id, int channel, int blen, uint32_t *event, int *nread)
{

  int rval, nwords, size;
  int n_part;
  int i=0;

  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  *nread = 0;
  nwords = 0;
  n_part = 0;
  
  rval = mpdFIFO_GetNwords(id, channel, &nwords);
  if( rval != OK ) return 2;
  
  size = (nwords > blen) ? blen : nwords; // cannot exceed memory buffer size

  if (size>0) {

  MPDLOCK;
#ifdef BLOCK_TRANSFER
    rval = BUS_BlockRead(fifo_addr, size, event, &n_part);
#else
    for(i=0; i<size; i++) {
      event[i] = mpdRead32(&MPDp[id]->ApvDaq.Data_Ch[channel][i*4]);
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
    *nread += n_part;

    if( n_part != size ) return 1;

  }

  MPDUNLOCK;
  return *nread;

}


int 
mpdFIFO_Samples(int id, 
		int channel, 
		uint32_t *event, int *nread, int max_samples, int *err)
{
  int success=OK, nwords, i=0;

  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  *err = 1;
  *nread = 0;

  nwords = DAQ_FIFO_SIZE;
  if( nwords > max_samples )
    {
      *err = 2;
      return ERROR;
    }

  MPDLOCK;
#ifdef BLOCK_TRANSFER
  success = BUS_BlockRead(fifo_addr, nwords, event, nread);
#else
  for(i=0; i<nwords; i++)
    event[i] = mpdRead32(&MPDp[id]->ApvDaq.Data_Ch[channel][i*4]);

  *nread = nwords*4;
#endif
  *nread /= 4;
  if( *nread == nwords )
    *err = 0;

  MPDUNLOCK;

  return success;
}

int 
mpdFIFO_IsSynced(int id, int channel, int *synced)
{
  uint32_t data;
  uint32_t channel_mask, synced_mask;
  int success=OK;

  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  channel_mask = 1 << (channel & 0x0F);
  synced_mask = channel_mask << 16;	/* @ error_addr */

  MPDLOCK;
  data = mpdRead32(&MPDp[id]->ApvDaq.Sync_Status);
  MPDUNLOCK;

  if( data & synced_mask )
    *synced = 1;
  else
    *synced = 0;

  return success;
}

/*
 * Return the synch status of each FE, (one bit = one FE, 1 means synched) 
 */
int 
mpdFIFO_AllSynced(int id, int *synced)
{
  uint32_t data;
  int success=OK;

  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  MPDLOCK;
  data = mpdRead32(&MPDp[id]->ApvDaq.Sync_Status);
  MPDUNLOCK;

  *synced = (data>>16);

  return success;
}


int 
mpdFIFO_HasError(int id, int channel, int *error)
{
  uint32_t data;
  uint32_t channel_mask, error_mask;
  int success=OK;

  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  channel_mask = 1 << (channel & 0x0F);
  error_mask = channel_mask;			/* @ error_addr */

  MPDLOCK;
  data = mpdRead32(&MPDp[id]->ApvDaq.Sync_Status);
  MPDUNLOCK;

  if( data & error_mask )
    *error = 1;
  else
    *error = 0;

  return success;
}

int 
mpdFIFO_GetAllFlags(int id, uint16_t *full, uint16_t *empty)
{
  uint32_t data;
  int success=OK;

  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  MPDLOCK;
  data = mpdRead32(&MPDp[id]->ApvDaq.FIFO_Status);
  MPDUNLOCK;

  *full =  data >> 16;
  *empty = data & 0xFFFF;

  return success;
}

int 
mpdFIFO_IsFull(int id, int channel, int *full)
{
  uint32_t data;
  uint32_t channel_mask, full_mask;
  int success=OK;
  
  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  channel_mask = 1 << (channel & 0x0F);
  full_mask = channel_mask << 16;		/* @ flag_addr */

  MPDLOCK;
  data = mpdRead32(&MPDp[id]->ApvDaq.FIFO_Status);
  MPDUNLOCK;

  if( data & full_mask )
    *full = 1;
  else
    *full = 0;

  return success;
}

int 
mpdFIFO_GetNwords(int id, int channel, int *nwords) // can be optimized reading both consecutive channels
{
  uint32_t data;
  //  uint32_t nwords_addr_offset[16] = {0,0,4,4,8,8,12,12,16,16,20,20,24,24,28,28}; // byte unit
  uint32_t nwords_addr_offset[16] = {0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,7}; // uint32_t unit
  int success=OK;
  
  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  MPDLOCK;
  data = mpdRead32(&MPDp[id]->ApvDaq.Used_Word_Ch_Pair[nwords_addr_offset[channel]]);
  MPDUNLOCK;

  printf("EC: nwords from fifo %d 0x%x\n",channel,data);
  if( channel % 2 )	// if odd
    *nwords =  ((data & 0xFFFF0000) >> 16) & 0xFFFF;
  else
    *nwords =  data & 0xFFFF;

  return success;
}

int 
mpdFIFO_IsEmpty(int id, int channel, int *empty)
{
  uint32_t data;
  uint32_t channel_mask, empty_mask;
  int success=OK;

  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  channel_mask = 1 << (channel & 0x0F);
  empty_mask = channel_mask;			/* @ flag_addr */

  MPDLOCK;
  data = mpdRead32(&MPDp[id]->ApvDaq.FIFO_Status);
  MPDUNLOCK;

  if( data & empty_mask )
    *empty = 1;
  else
    *empty = 0;

  return success;
}

int 
mpdFIFO_ClearAll(int id)
{
  uint32_t data, oldval;
  int success=OK;

  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  // Issue a pulse on READOUT_CONFIG[31]

  MPDLOCK;
  oldval = mpdRead32(&MPDp[id]->ApvDaq.Readout_Config);
  data = oldval | 0x80000000;

  mpdWrite32(&MPDp[id]->ApvDaq.Readout_Config, data);

  data = oldval & 0x7FFFFFFF;

  mpdWrite32(&MPDp[id]->ApvDaq.Readout_Config, data);
  MPDUNLOCK;

  return success;
}


int 
mpdFIFO_WaitNotEmpty(int id, int channel, int max_retry)
{
  uint32_t data;
  uint32_t channel_mask, empty_mask;
  int success=OK, retry_count, fifo_empty;

  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  channel_mask = 1 << (channel & 0x0F);
  empty_mask = channel_mask;			/* @ flag_addr */

  retry_count = 0;
  fifo_empty = 1;

  while( fifo_empty && success == OK && retry_count <= max_retry )
    {
      MPDLOCK;
      data = mpdRead32(&MPDp[id]->ApvDaq.FIFO_Status);
      MPDUNLOCK;

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

int
mpdFIFO_ReadAll(int id, int *timeout, int *global_fifo_error) {

  unsigned int k;
  int sample_left;

  int nread, err;

  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  sample_left = 0;
  *global_fifo_error = 0;

  if (fMpd[id].fReadDone == 0) { // at least one MPD FIFO needs to be read
    for(k=0; k<nApv[id]; k++) { // loop on ADC channels on single board

      if (mpdApvReadDone(id, k) == 0) { // APV FIFO has data to be read

	nread = mpdApvGetBufferAvailable(id, k);
	printf(" EC: card %d buffer size available = %d\n",k,nread);
	if (nread>0) { // space in memory buffer
	  err = mpdFIFO_ReadSingle(id, fApv[id][k].adc, 
				   mpdApvGetBufferPWrite(id, k), &nread, 1); 

	  mpdApvIncBufferPointer(id, k, nread);

	  *global_fifo_error |= err; // ???
	  printf(" EC: card %d readsingle done nread=%d, err=%d\n",k,nread,err);
	} else { // no space in memory buffer
	  MPD_ERR("MPD/APV(i2c)/(adc) = %d/%d, no space in memory buffer adc=%d\n", 
		  id, k, fApv[id][k].adc);
	}

	if ((err == ERROR) || (nread == 0)) *timeout++; // timeout

	int n = mpdApvGetBufferSample(id,k);

	MPD_DBG("FIFO= %d, word read= %d, event/sample read= %d, error=%d\n",fApv[id][k].adc,nread ,n, *global_fifo_error);

	sample_left += mpdApvGetSampleLeft(id, k);

      }

      MPD_DBG("Fifo= %d, total sample left= %d (<0 means more samples than requested)\n",k, sample_left);

    } // loop on ADC
    fMpd[id].fReadDone = (sample_left>0) ? 0 : 1;  
  } // if fReadDone
  
  return fMpd[id].fReadDone;

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
int 
mpdSearchEndMarker(uint32_t *b, int i0, int i1) {

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

void 
mpdApvShiftDataBuffer(int id, int k, int i0) {

  uint32_t *b;

  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return;
    }

  b = mpdApvGetBufferPointer(id, k, 0);
  int delta = fApv[id][k].fBi1 - i0;

  MPD_DBG("Move block of %d words from %d to 0\n",delta,i0);

  if (delta>0) {
    memmove(&b[0],&b[i0],sizeof(uint32_t)*delta); // areas may overlap
  }

  fApv[id][k].fBi0 = 0; // to be removed
  fApv[id][k].fBi1 = delta;
  
  MPD_DBG("Fifo= %d cleaned (data shifted) write pointer at=%d\n",fApv[id][k].adc,fApv[id][k].fBi1);

}

#ifdef NOTDONE
int
mpdFIFO_ReadAllNew(int id, int *timeout, int *global_fifo_error) 
{

  int n;
  unsigned int k;
  int sample_left;

  int nread, err;
  uint32_t multiple_events;

  int ii1=0;

  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  n=1; // single event mode

  sample_left = 0;

  if (fMpd[id].fReadDone == 0) { // MPD fifos need to be read
    for(k=0; k<nApv[id]; k++) { // loop on ADC channels on single board

      if (mpdApvReadDone(id,k) == 0) { // APV FIFO has data to be read
  
	//      sample_left += ApvGetSampleLeft(k);

        //      ii0=fApv[k].fBi0;
        //      ii1=fApv[k].fBi1;
        //      idx = SearchEndMarker(ApvGetBufferPointer(k),ii0,ii1);
        //      fApv[k].fBi0 = ii1;
        //      cout << __FUNCTION__ << dec << " " << j << " " << k;

	uint32_t *bptr = mpdApvGetBufferPointer(id,k,ii1);
      
	int bsiz = fApv[k].fBufSize - ii1; // space left in buffer

	err = mpdFIFO_ReadSingle(id, fApv[id][k].adc, bsiz, bptr, nread);

	// ***
	if( nread > 0 ) {
	  for (int i=0; i<nread; i++) {
	    if (mpdIsEndBlock(bptr[i])) { 
	      mpdApvDecSampleLeft(id,k,1);
	      if (mpdApvGetSampleLeft(id,k) == 0) {
		fApv[id][k].fBs = ii1 + i; // pointer to the last sample word
	      }
	    }
	  }

	  fApv[id][k].fBi1=ii1+nread;

	} else {
	  *timeout++;
	}
	
	if( err != 2 )
	  *global_fifo_error |= err;
      
	if (err == 2) *timeout++;
      
	if( n > 1 ) multiple_events++; // not used 
	
      } else { // ...
#ifdef DEBUG_DUMP
	cout << __FUNCTION__ << ": xxxx " << endl;
#endif
      }
      
      sample_left += mpdApvGetSampleLeft(id,k);

    } // loop on ADC FIFOs
    fMpd[id].fReadDone = (sample_left == 0) ? 1 : 0;      
  } 
  
  return fMpd[id].fReadDone;

}
#endif /* NOTDONE */

/**
 *
 */

int 
mpdDAQ_Enable(int id)
{
  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  mpdDAQ_Config(id);
  return mpdTRIG_Enable(id);
}

int 
mpdDAQ_Disable(int id)
{
  uint32_t data;

  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  data = 0;

  MPDLOCK;
  mpdWrite32(&MPDp[id]->ApvDaq.Readout_Config, data);
  MPDUNLOCK;


  return mpdTRIG_Disable(id);
}

int 
mpdDAQ_Config(int id) 
{

  uint32_t data;
  int i;
  short evtbld;

  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  evtbld = mpdGetEventBuilding(id) ? 1 : 0;

  data = (mpdGetAcqMode(id) & 0x07) | // ((test & 0x01) << 15) |
    ((0 & 0x1) << 4) | // FIR enable=1 (to be implemented)
    (( mpdGetInputLevel(id,0) & 0x1) << 8) | // NIM=0/TTL=1 Level LEMO IN0 (TBI)
    (( mpdGetInputLevel(id,1) & 0x1) << 9) | // NIM/TTL Level LEMO IN1 (TBI)
    (( mpdGetOutputLevel(id,0) & 0x1) << 10) | // NIM/TTL Level LEMO OUT0 (TBI)
    (( mpdGetOutputLevel(id,1) & 0x1) << 11) | // NIM/TTL Level LEMO OUT1 (TBI)
    ((mpdGetCommonOffset(id) & 0xfff) << 16) | 
    ((mpdGetCommonNoiseSubtraction(id) & 0x1) << 28) | 
    ((evtbld & 0x1) << 30);

  printf("%s : ReadoutConfig = 0x%x\n",__FUNCTION__,data);
  MPDLOCK;
  mpdWrite32(&MPDp[id]->ApvDaq.Readout_Config, data);
  MPDUNLOCK;

  for (i=0;i<nApv[id];i++) {
    fApv[id][i].fBi0 = 0;
    fApv[id][i].fBi1 = 0;
  }

  return mpdFIFO_ClearAll(id);

}


// ***** Pedestal and Thresholds handling routines

/**
 * ADC has two groups of channels: 0,...7 and 8...15
 * ch refers to the channel index (range is 0...7)
 * even and odd refer to the first and second group
 */

int 
mpdPED_Write0(int id, int ch, int *ped_even, int *ped_odd)	// 
{
  uint32_t data;
  int success=OK, i;

  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  MPDLOCK;
  for(i=0; i<128; i++)
    {
      data = ped_even[i] | (ped_odd[i] << 16);
      mpdWrite32(&MPDp[id]->ApvDaq.Ped[ch].ram[i], data);
    }
  MPDUNLOCK;

  return success;
}

int 
mpdPED_Write(int id, int ch, int v)	// ch = 0..7
{
  int i, data[128];
  
  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  for(i=0; i<128; i++)
    data[i] = v;

  return mpdPED_Write0(id, ch, data, data);
}



int 
mpdPED_Read(int id, int ch, int *ped_even, int *ped_odd)	// TBD
{
  int i;

  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  for(i=0; i<128;  i++)
    ped_even[i] = ped_odd[i] = 0;	// TBD

  return OK;
}


int 
mpdTHR_Write0(int id, int ch, int *thr_even, int *thr_odd)	// ch = 0..7
{
  uint32_t data;
  int success=OK, i;

  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  MPDLOCK;
  for(i=0; i<128; i++)
    {
      data = thr_even[i] | (thr_odd[i] << 16);
      mpdWrite32(&MPDp[id]->ApvDaq.Thres[ch].ram[i], data);
    }
  MPDUNLOCK;

  return success;
}

int 
mpdTHR_Write(int id, int ch, int v)	// ch = 0..7
{
  int i, data[128];
  
  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  for(i=0; i<128; i++)
    data[i] = v;

  return mpdTHR_Write0(id, ch, data, data);
}

int 
mpdTHR_Read(int id, int ch, int *thr_even, int *thr_odd)	// TBD
{
  int i;
  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  for(i=0; i<128;  i++)
    thr_even[i] = thr_odd[i] = 0;	// TBD

  return OK;
}

/**
 * Load pedestal and threshold data into the MPD
 */
int 
mpdPEDTHR_Write(int id) 
{
  int ia;
  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  for (ia=0;ia<8;ia++) 
    { // loop on apv,  odd and even apv are download simultaneously
      mpdPED_Write0(id, ia, mpdGetApvPed(id, 2*ia), mpdGetApvPed(id, 2*ia+1));
      mpdTHR_Write0(id, ia, mpdGetApvThr(id, 2*ia), mpdGetApvThr(id, 2*ia+1));
    }

  return 0;

}

void mpdSetPedThrPath(int id, char *val) { fMpd[id].fPedThrPath = val; };
char *mpdGetPedThrPath(int id) { return fMpd[id].fPedThrPath; };
int mpdSetPedThr(int id, int ch, int p, int t) {
  if (ch>=0 && ch<2048) {
    fMpd[id].fPed[ch] = p;
    fMpd[id].fThr[ch] = t;
    return 1;
  }
  return 0;
};
int mpdGetPed(int id, int ch) { 
  if (ch>=0 && ch<2048) { return fMpd[id].fPed[ch]; } else { return fMpd[id].fPedCommon; }
};
int mpdGetThr(int id, int ch) { 
  if (ch>=0 && ch<2048) { return fMpd[id].fThr[ch]; } else { return fMpd[id].fThrCommon; }
};


int *mpdGetApvPed(int id, int ach) { if (ach>=0 && ach<16) { return &(fMpd[id].fPed[ach*128]); } else { return 0; }};
int *mpdGetApvThr(int id, int ach) { if (ach>=0 && ach<16) { return &(fMpd[id].fThr[ach*128]); } else { return 0; }};

void mpdSetPedThrCommon(int id, int p, int t) {
  fMpd[id].fPedCommon = p;
  fMpd[id].fThrCommon = t;
};


int mpdGetPedCommon(int id) { return fMpd[id].fPedCommon; };
int mpdGetThrCommon(int id) { return fMpd[id].fThrCommon; };


#ifdef NOTDONE
/**
 * Read Pedestals and Thresholds of a single MPD from the file pname
 * if pname is empty, use the stored PedThrPath value
 */
int 
mpdReadPedThr(int id, std::string pname) 
{
  //  if(id==0) id=mpdID[0];
  if((MPDp[id]==NULL) || (id<0) || (id>21))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  string line;

  int ch, ped, thr;
  int count,i;

  if (pname.empty()) {
    pname = mpdGetPedThrPath(id);
  };

  for (i=0;i<2048;i++) {
    mpdSetPedThr(id, i, mpdGetPedCommon(id), mpdGetThrCommon(id));
  };

  std::ifstream infile(pname.data(), std::ifstream::in);
  if (infile.is_open()) {
    count=0;
    while (infile.good()) {
      getline(infile, line);
      if (line.find("#") != 0) { // no comment line
	std::istringstream iss(line);
	iss >> ch >> ped >> thr;
	count += mpdSetPedThr(id, ch, ped, thr);
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


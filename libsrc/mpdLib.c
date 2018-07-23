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
 *
 *          Danning Di
 *          University of Virginia
 *          Email:Danning@jlab.org
 *          Nov 2015
 * </pre>
 *----------------------------------------------------------------------------*/

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

/* Define external Functions */
#ifdef VXWORKS
IMPORT STATUS sysBusToLocalAdrs(int, char *, char **);
IMPORT STATUS intDisconnect(int);
IMPORT STATUS sysIntEnable(int);
IMPORT STATUS sysIntDisable(int);
IMPORT STATUS sysVmeDmaDone(int, int);
IMPORT STATUS sysVmeDmaSend(UINT32, UINT32, int, BOOL);

#define EIEIO    __asm__ volatile ("eieio")
#define SYNC     __asm__ volatile ("sync")
#endif

/* Define Interrupts variables */
BOOL mpdIntRunning = FALSE;	/* running flag */
int mpdIntID = -1;		/* id number of ADC generating interrupts */
LOCAL VOIDFUNCPTR mpdIntRoutine = NULL;	/* user interrupt service routine */
LOCAL int mpdIntArg = 0;	/* arg to user routine */
LOCAL UINT32 mpdIntLevel = 0;	/* default VME interrupt level */
LOCAL UINT32 mpdIntVec = 0;	/* default interrupt Vector */

/* Define global variables */
int nmpd = 0;			/* Number of MPDs in Crate */
int mpdA32Base = 0x09000000;	/* Minimum VME A32 Address for use by MPDs */
int mpdA32Offset = 0x08000000;	/* Difference in CPU A32 Base - VME A32 Base */
int mpdA24Offset = 0x0;		/* Difference in CPU A24 Base - VME A24 Base */
volatile struct mpd_struct *MPDp[(MPD_MAX_BOARDS + 1)];	/* pointers to MPD memory map */
volatile uint32_t *MPDpd[(MPD_MAX_BOARDS + 1)];	/* pointers to MPD FIFO memory */
volatile uint32_t *MPDpmb;	/* pointer to Multblock window */
int mpdID[MPD_MAX_BOARDS];	/* array of slot numbers for MPDs */
uint32_t mpdAddrList[MPD_MAX_BOARDS];	/* array of a24 addresses for MPDs */
int mpdRev[(MPD_MAX_BOARDS + 1)];	/* Board Revision Info for each module */
unsigned short mpdChanDisable[(MPD_MAX_BOARDS + 1)];	/* Disabled Channel Mask for each Module */
int mpdInited = 0;		/* >0 if Library has been Initialized before */
int mpdMaxSlot = 0;		/* Highest Slot hold an MPD */
int mpdMinSlot = 0;		/* Lowest Slot holding an MPD */
int mpdSource = 0;		/* Signal source for MPD system control */
int mpdBlockLevel = 0;		/* Block Level for ADCs */
int mpdIntCount = 0;		/* Count of interrupts from MPD */
int mpdBlockError = 0;		/* Whether (>0) or not (0) Block Transfer had an error */
int mpdOutputBufferBaseAddr = 0x00880000;	/* output buffer base address */
int mpdOutputBufferSpace = 0x800000;	/* output buffer space (8 Mbyte) */
int mpdSdramBaseAddr = 0x0;	/* sdram base address (test only, 0=disabled) */
ApvParameters fApv[(MPD_MAX_BOARDS) + 1][MPD_MAX_APV];
mpdParameters fMpd[(MPD_MAX_BOARDS) + 1];
unsigned short fApvEnableMask[(MPD_MAX_BOARDS) + 1];
int nApv[(MPD_MAX_BOARDS) + 1];
extern GEF_VME_BUS_HDL vmeHdl;
static int mpdSSPMode = 0;
static uint32_t mpdSSPFiberMask[(MPD_SSP_MAX_BOARDS) + 1];	/* index = ssp#, value = fiber port mask of MPDs */
static int mpdSSPFiberMaskUsed = 0;
static int mpdPrintDebug = 0;

/* */
#define MPD_VERSION_MASK 0xf00f
#define MPD_SUPPORTED_CTRL_FIRMWARE 0x4

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
/*static*/ int I2C_SendByte(int id, uint8_t byteval, int start);
/*static*/ int I2C_ReceiveByte(int id, uint8_t * byteval);
/*static*/ int I2C_SendStop(int id);
/*static*/ int I2C_SendNack(int id);

extern int sspID[MPD_SSP_MAX_BOARDS + 1];
extern int nSSP;
extern uint32_t sspMpdReadReg(int id, int impd, unsigned int reg);
extern int sspMpdWriteReg(int id, int impd, unsigned int reg,
			  unsigned int value);

#define CHECKMPD(x) (((int32_t)MPDp[x]==-1) || (x<0) || (x>21))


uint32_t
mpdRead32(volatile uint32_t * reg)
{
  int issp, impd;
  uint32_t newreg;
  uint32_t read = 0;

  if (mpdSSPMode)
    {
      /* SSP stored in bits 29-31, mpd stored in bits 24-28 */
      issp = (int) (((uint32_t) reg & 0xE0000000) >> 29);
      impd = (int) (((uint32_t) reg & 0x1F000000) >> 24);

      /* Check if this is in the mask */
      if ((mpdSSPFiberMask[sspID[issp]] & (1 << impd)) == 0)
	{
	  MPD_ERR("SSP %d, MPD %d, not initializated\n", issp, impd);
	  return ERROR;
	}

      newreg = (uint32_t) ((uint32_t) reg & 0x00FFFFFF);

      read = sspMpdReadReg(issp, impd, newreg);
    }
  else
    read = vmeRead32(reg);


  return read;
}

void
mpdWrite32(volatile uint32_t * reg, uint32_t val)
{
  int issp, impd;
  uint32_t newreg;

  if (mpdSSPMode)
    {
      /* SSP stored in bits 29-31, mpd stored in bits 24-28 */
      issp = (int) (((uint32_t) reg & 0xE0000000) >> 29);
      impd = (int) (((uint32_t) reg & 0x1F000000) >> 24);

      /* Check if this is in the mask */
      if ((mpdSSPFiberMask[sspID[issp]] & (1 << impd)) == 0)
	{
	  MPD_ERR("SSP %d, MPD %d, not initializated\n", issp, impd);
	  return;
	}

      newreg = (uint32_t) ((uint32_t) reg & 0x00FFFFFF);

      sspMpdWriteReg(issp, impd, newreg, val);
    }
  else
    vmeWrite32(reg, val);
}

/**
 * @defgroup Config Initialization/Configuration
 * @defgroup Status Status
 * @defgroup Readout Data Readout
 * @defgroup IntPoll Interrupt/Polling
 * @defgroup Deprec Deprecated - To be removed
 */

int
mpdSetSSPFiberMap_preInit(int ssp, int mpdmask)
{
  int issp, impd;

  if (ssp <= MPD_SSP_MAX_BOARDS)
    {
      MPD_MSG("ERROR: ssp (%d) out of range\n", ssp);
      return ERROR;
    }

  if (mpdSSPFiberMaskUsed == 0)
    {
      /* Initialize fiber mask array */
      memset(&mpdSSPFiberMask, 0, sizeof(mpdSSPFiberMask));
    }

  mpdSSPFiberMask[ssp] = mpdmask;

  /* Count up currently stored ports to use */
  mpdSSPFiberMaskUsed = 0;
  for (issp = 0; issp < nSSP; issp++)
    {
      for (impd = 0; impd < 32; impd++)
	if (mpdSSPFiberMask[issp] & (1 << impd))
	  mpdSSPFiberMaskUsed++;
    }

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Initialize JLAB MPD Library.
 *
 * @param addr
 *  - A24 VME Address of the MPD
 *  - If using SSP mode, this will represent the mask of SSP fiber connections to use.
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
 * @return Number of MPDs initialized, or ERROR if the address is
 * invalid or a board is not present.
 */

STATUS
mpdInit(UINT32 addr, UINT32 addr_inc, int nmpd, int iFlag)
{

  int ii, issp, ibit, impd, impd_disc, res, errFlag = 0;
  int boardID = 0;
  int maxSlot = 1;
  int minSlot = 21;
  uint32_t magic_const = MPD_MAGIC_VALUE;
  uint32_t rdata, laddr, laddr_inc;
  volatile struct mpd_struct *mpd = NULL;
  int noBoardInit = 0;
  int useList = 0;
  int noFirmwareCheck = 0;
  int noConfigFileCheck = 0;
  int *mpdssp_list = NULL, nlist = 0;

  /* Check if we are to exit when pointers are setup */
  noBoardInit = (iFlag & MPD_INIT_SKIP) ? 1 : 0;

  /* Check if we're initializing using a list */
  useList = (iFlag & MPD_INIT_USE_ADDRLIST) ? 1 : 0;

  /* Are we skipping the firmware check? */
  noFirmwareCheck = (iFlag & MPD_INIT_SKIP_FIRMWARE_CHECK) ? 1 : 0;

  /* Are we using the SSP to access the MPD(s)? */
  mpdSSPMode = (iFlag & MPD_INIT_SSP_MODE) ? 1 : 0;

  /* Are we skipping the config file check? */
  noConfigFileCheck = (iFlag & MPD_INIT_NO_CONFIG_FILE_CHECK) ? 1 : 0;

  memset(MPDp, -1, sizeof(MPDp));


  if (mpdSSPMode)
    {				// SSP Mode
      if (nSSP == 0)
	{
	  MPD_MSG("ERROR: Must initialize SSPs first\n");
	  return ERROR;
	}

      if (mpdSSPFiberMaskUsed > 0)
	{
	  MPD_MSG
	    ("WARNING: May Overwrite previously stored SSP Fiber Masks\n");
	}
      else
	{
	  /* Initialize fiber mask array */
	  memset(&mpdSSPFiberMask, 0, sizeof(mpdSSPFiberMask));
	}

      if (addr)
	{
	  mpdSSPFiberMask[sspID[0]] = addr;
	}

      for (issp = 0; issp < nSSP; issp++)
	{
	  if (mpdSSPFiberMask[sspID[issp]])
	    {
	      MPD_MSG("Using SSP (%d) Fibermask (0x%08x) scan for MPDs\n",
		      sspID[issp], mpdSSPFiberMask[sspID[issp]]);
	    }
	}
    }
  else
    {				// VME Mode
      /* Check for valid address */
      if (addr == 0)
	{
	  MPD_ERR("Must specify a Bus (VME-based A24) address for MPD 0\n");
	  return (ERROR);
	}
      else if (addr > 0x00ffffff)
	{			/* A24 Addressing */
	  MPD_ERR("A32 Addressing not allowed for MPD configuration space\n");
	  return (ERROR);
	}
      else
	{			/* A24 Addressing */
	  if (((addr_inc == 0) || (nmpd == 0)) && (useList == 0))
	    nmpd = 1;		/* assume only one MPD to initialize */

	  /* get the MPD address */
#ifdef VXWORKS
	  res = sysBusToLocalAdrs(0x39, (char *) addr, (char **) &laddr);
#else
	  res = vmeBusToLocalAdrs(0x39, (char *) addr, (char **) &laddr);
#endif
	  if (res != 0)
	    {
#ifdef VXWORKS
	      MPD_ERR("ERROR in sysBusToLocalAdrs(0x39,0x%x,&laddr_csr) \n",
		      addr);
#else
	      MPD_ERR("ERROR in vmeBusToLocalAdrs(0x39,0x%x,&laddr_csr) \n",
		      addr);
#endif
	      return (ERROR);
	    }

	  mpdA24Offset = laddr - addr;
	}
      MPD_MSG("A24 mapping: VME addr=0x%x -> Local 0x%x\n", addr, laddr);
    }

  impd = 0;
  impd_disc = 0;

  if (mpdSSPMode)
    {
      printf("****************************************\n");
      /* Make a quick and dirty array to use in the next iteration
         over mpds up to nmpd */
      mpdssp_list = (int *) malloc(nmpd * sizeof(int));
      int value;
      for (issp = 0; issp < nSSP; issp++)
	{
	  value = issp << 28;
	  for (ibit = 0; ibit < 32; ibit++)
	    {
	      if (mpdSSPFiberMask[sspID[issp]] & (1 << ibit))
		{
		  mpdssp_list[nlist++] = value | (ibit << 24);
		  MPD_MSG("Added SSP %2d MPD %2d\n", issp, ibit);
		}
	    }
	}
    }


  for (ii = 0; ii < nmpd; ii++)
    {

      errFlag = 0;

      if (mpdSSPMode)
	{
	  laddr_inc = mpdssp_list[ii];
	  mpd = (struct mpd_struct *) laddr_inc;

	  rdata = mpdRead32(&mpd->magic_value);
	  res = 1;
	}
      else
	{
	  MPD_MSG("Looking at MPD in slot %d\n", ii + 1);
	  if (useList == 1)
	    {
	      laddr_inc = mpdAddrList[ii] + mpdA24Offset;	// not tested yet (EC)
	    }
	  else
	    {
	      laddr_inc = laddr + ii * addr_inc;
	    }
	  mpd = (struct mpd_struct *) laddr_inc;

#ifdef VXWORKS
	  res =
	    vxMemProbe((char *) mpd->magic_value, VX_READ, 4,
		       (char *) &rdata);
#else
	  res = vmeMemProbe((char *) &mpd->magic_value, 4, (char *) &rdata);
#endif
	  if ((res < 0) || (rdata = -1))
	    {
	      /* Turn off fiber mode, and try again */
	      MPD_DBG("Try turning off fiber mode\n");
	      mpd->fiber_status_ctrl = LSWAP(1);
	    }

#ifdef VXWORKS
	  res =
	    vxMemProbe((char *) mpd->magic_value, VX_READ, 4,
		       (char *) &rdata);
#else
	  res = vmeMemProbe((char *) &mpd->magic_value, 4, (char *) &rdata);
#endif
	}



      if (res < 0)
	{
#ifdef VXWORKS
	  MPD_MSG("WARN: No addressable board at addr=0x%x\n", (UINT32) mpd);
#else
	  MPD_MSG
	    ("WARN: No addressable board at VME addr=0x%x (local 0x%x)\n",
	     (UINT32) addr + ii * addr_inc, (UINT32) mpd);
#endif
	  errFlag = 1;
	  continue;
	}

      if (rdata != magic_const)
	{
	  MPD_MSG
	    ("WARN: for board at 0x%x, invalid data code 0x%x (expected 0x%x)\n",
	     (UINT32) mpd - mpdA24Offset, rdata, magic_const);
	  errFlag = 2;
	  continue;
	}

      // discovered new board
      impd_disc++;

      if (mpdSSPMode)
	boardID = ii;
      else
	boardID = mpdRead32(&mpd->a24_bar) >> 3;

      if ((boardID < 0) || (boardID > 21))
	{
	  MPD_ERR("For Board at 0x%x,  Slot number is not in range: %d\n",
		  (UINT32) mpd - mpdA24Offset, boardID);
	  continue;
	}


      /* read firmware revision */
      rdata = mpdRead32(&mpd->revision_id);

      printf(" MPD Slot %d - Firmware Revision ID = 0x%x\n", boardID, rdata);

      if (!noFirmwareCheck)
	{

	  // Check FPGA firmware version
	  if ((rdata & MPD_VERSION_MASK) < MPD_SUPPORTED_CTRL_FIRMWARE)
	    {
	      MPD_ERR
		("Slot %2d: Control FPGA Firmware (0x%02x) not supported by this driver.\n",
		 boardID, rdata & MPD_VERSION_MASK);
	      printf("\tUpdate to 0x%02x to use this driver.\n",
		     MPD_SUPPORTED_CTRL_FIRMWARE);
	      continue;
	    }

	}
      else
	{

	  // Check FPGA firmware version
	  if ((rdata & MPD_VERSION_MASK) < MPD_SUPPORTED_CTRL_FIRMWARE)
	    {
	      MPD_MSG
		("WARN: Slot %2d: Control FPGA Firmware (0x%02x) not supported by this driver (ignored).\n",
		 boardID, rdata & MPD_VERSION_MASK);
	    }
	}

      fMpd[boardID].FpgaRevision = rdata;

      /* time revision */
      rdata = mpdRead32(&mpd->compile_time);

      fMpd[boardID].FpgaCompileTime = rdata;
      printf(" MPD Slot %d - Firmware Revision Time: %s\n",
	     boardID, ctime((const time_t *) &fMpd[boardID].FpgaCompileTime));

      if (!noConfigFileCheck)
	{
	  // set it, if it is presents in config file
	  if (mpdGetNumberAPV(boardID) <= 0)
	    {			// not in config file (to be improved)
	      printf(" -- MPD in slot %d is NOT in config file, drop it\n",
		     boardID);
	      continue;
	    }
	  printf(" ++ MPD in slot %d is in config file, INIT IT\n", boardID);
	}

      mpdID[impd] = boardID;
      printf("#############MPD ID:%d  \n", mpdID[impd]);
      if (boardID >= maxSlot)
	maxSlot = boardID;
      if (boardID <= minSlot)
	minSlot = boardID;

      MPDp[boardID] = (struct mpd_struct *) (laddr_inc);

      if (mpdSSPMode)
	{
	  MPD_MSG("MPD %2d at SSP %d Fiber Connection %d initialized\n",
		  impd, (laddr_inc & 0xE0000000) >> 29,
		  (laddr_inc & 0x1F000000) >> 24);
	  printf(" Local address = 0x%08x \n", (uint32_t) MPDp[boardID]);
	}
      else
	{
	  MPD_MSG
	    ("MPD index %2d Slot #%2d at VME address 0x%08x (local 0x%08x)\n\n",
	     impd, boardID, (UINT32) MPDp[boardID] - mpdA24Offset,
	     (UINT32) MPDp[boardID]);
	}

      if (!noBoardInit)
	{
	  /* Some more initialize here, if needed */
	}

      impd++;
    }


  nmpd = impd;
  mpdBlockLevel = 1;

  if (!noBoardInit)
    mpdInited = nmpd;

  if (nmpd > 0)
    {
      MPD_MSG("%d MPD(s) initialized, %d discovered\n", nmpd, impd_disc);
    }

  if (errFlag > 0)
    {
      MPD_MSG
	("WARN: Unable to initialize all requested MPD Modules (%d found)\n",
	 nmpd);
      return (ERROR);
    }

  return (nmpd);
}

int
mpdGetNumberMPD()
{
  return mpdInited;
};

int
mpdSetPrintDebug(int debug)
{
  mpdPrintDebug = (debug) ? 1 : 0;
  return mpdPrintDebug;
}

int
mpdGetNumberAPV(int id)
{
  return (uint16_t) fMpd[id].nAPV;
};

void
mpdSetNumberAPV(int id, uint16_t v)
{
  fMpd[id].nAPV = v;
};

int
mpdGetNumberConfiguredAPV(int id)
{
  return nApv[id];
};				// return number of enabled APV

int
mpdCheckAddresses(int id)
{
  uint32_t offset = 0, expected = 0, base = 0;
  int rval = OK;

  MPD_MSG("\n\t ---------- Checking mpd address space ---------- \n");

  base = (uint32_t) & MPDp[id]->magic_value;

  offset = ((uint32_t) & MPDp[id]->reset_reg) - base;
  expected = 0x100;
  if (offset != expected)
    {
      MPD_ERR(" MPDp[id]->reset_reg \n not at offset = 0x%x (@ 0x%x)\n",
	      expected, offset);
      rval = ERROR;
    }

  offset = ((uint32_t) & MPDp[id]->a24_bar) - base;
  expected = 0x180;
  if (offset != expected)
    {
      MPD_ERR(" MPDp[id]->a24_bar \n not at offset = 0x%x (@ 0x%x)\n",
	      expected, offset);
      rval = ERROR;
    }

  offset = ((uint32_t) & MPDp[id]->ob_status.evb_fifo_word_count) - base;
  expected = 0x200;
  if (offset != expected)
    {
      MPD_ERR
	(" MPDp[id]->ob_status.evb_fifo_word_count \n not at offset = 0x%x (@ 0x%x)\n",
	 expected, offset);
      rval = ERROR;
    }

  offset = ((uint32_t) & MPDp[id]->adc_config) - base;
  expected = 0x300;
  if (offset != expected)
    {
      MPD_ERR(" MPDp[id]->adc_config \n not at offset = 0x%x (@ 0x%x)\n",
	      expected, offset);
      rval = ERROR;
    }

  offset = ((uint32_t) & MPDp[id]->i2c.clock_prescaler_low) - base;
  expected = 0x400;
  if (offset != expected)
    {
      MPD_ERR
	(" MPDp[id]->i2c.clock_prescaler_low \n not at offset = 0x%x (@ 0x%x)\n",
	 expected, offset);
      rval = ERROR;
    }

  offset = ((uint32_t) & MPDp[id]->histo[0].csr) - base;
  expected = 0x1000;
  if (offset != expected)
    {
      MPD_ERR(" MPDp[id]->histo[0].csr \n not at offset = 0x%x (@ 0x%x)\n",
	      expected, offset);
      rval = ERROR;
    }

  offset = ((uint32_t) & MPDp[id]->histo_memory[0][0]) - base;
  expected = 0x4000;
  if (offset != expected)
    {
      MPD_ERR
	(" MPDp[id]->histo_memory[0][0] \n not at offset = 0x%x (@ 0x%x)\n",
	 expected, offset);
      rval = ERROR;
    }

  offset = ((uint32_t) & MPDp[id]->data_ch[0][0]) - base;
  expected = 0x10000;
  if (offset != expected)
    {
      MPD_ERR(" MPDp[id]->data_ch[0][0] \n not at offset = 0x%x (@ 0x%x)\n",
	      expected, offset);
      rval = ERROR;
    }

  offset = ((uint32_t) & MPDp[id]->ch_flags.used_word_ch_pair[0]) - base;
  expected = 0x30000;
  if (offset != expected)
    {
      MPD_ERR
	(" MPDp[id]->ch_flags.used_word_ch_pair[0] \n not at offset = 0x%x (@ 0x%x)\n",
	 expected, offset);
      rval = ERROR;
    }

  offset = ((uint32_t) & MPDp[id]->ped[0][0]) - base;
  expected = 0x34000;
  if (offset != expected)
    {
      MPD_ERR(" MPDp[id]->ped[0][0] \n not at offset = 0x%x (@ 0x%x)\n",
	      expected, offset);
      rval = ERROR;
    }

  offset = ((uint32_t) & MPDp[id]->thres[0][0]) - base;
  expected = 0x36000;
  if (offset != expected)
    {
      MPD_ERR(" MPDp[id]->thres[0][0] \n not at offset = 0x%x (@ 0x%x)\n",
	      expected, offset);
      rval = ERROR;
    }


  return rval;
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
     MPD_ERR("Index (%d) >= MPDs initialized (%d).\n",
     i,nmpd);
     return ERROR;
     }
   */
  return mpdID[i];
}

void
mpdSetZeroLevel(int id, uint16_t level)
{
  fMpd[id].fLevel_0 = level;
}

int
mpdGetZeroLevel(int id)
{
  return fMpd[id].fLevel_0;
}

void
mpdSetOneLevel(int id, uint16_t level)
{
  fMpd[id].fLevel_1 = level;
}

int
mpdGetOneLevel(int id)
{
  return fMpd[id].fLevel_1;
}

void
mpdSetChannelMark(int id, int v)
{
  fMpd[id].fChannelMark = v;
}				// channel mark 0-127 for frame alignment, mark>127 disabled

int
mpdGetChannelMark(int id)
{
  return fMpd[id].fChannelMark;
}

void
mpdSetCommonNoiseSubtraction(int id, short val)
{
  fMpd[id].fCommonNoiseSubtraction = val;
};

short
mpdGetCommonNoiseSubtraction(int id)
{
  return fMpd[id].fCommonNoiseSubtraction;
};

void
mpdSetEventBuilding(int id, int val)
{
  fMpd[id].fEventBuilding = val;
};

int
mpdGetEventBuilding(int id)
{
  return fMpd[id].fEventBuilding;
};

void
mpdSetEventPerBlock(int id, int val)
{
  fMpd[id].fEventPerBlock = val;
};

int
mpdGetEventPerBlock(int id)
{
  return fMpd[id].fEventPerBlock;
};

void
mpdSetUseSdram(int id, int val)
{
  fMpd[id].fUseSdram = val;
};

int
mpdGetUseSdram(int id)
{
  return fMpd[id].fUseSdram;
};

void
mpdSetFastReadout(int id, int val)
{
  fMpd[id].fFastReadout = val;
};

int
mpdGetFastReadout(int id)
{
  return fMpd[id].fFastReadout;
};

void
mpdSetCommonOffset(int id, int val)
{
  fMpd[id].fCommonOffset = val;
};

int
mpdGetCommonOffset(int id)
{
  return fMpd[id].fCommonOffset;
};

void
mpdSetCalibLatency(int id, int val)
{
  fMpd[id].fCalibLatency = val;
};

int
mpdGetCalibLatency(int id)
{
  return fMpd[id].fCalibLatency;
};

void
mpdSetTriggerLatency(int id, int val)
{
  fMpd[id].fTriggerLatency = val;
};

int
mpdGetTriggerLatency(int id)
{
  return fMpd[id].fTriggerLatency;
};

void
mpdSetTriggerNumber(int id, int val)
{
  fMpd[id].fTriggerNumber = val;
};

int
mpdGetTriggerNumber(int id)
{
  return fMpd[id].fTriggerNumber;
};

void
mpdSetTriggerMode(int id, int lat, int tlat, int num)
{
  if (num == 0)
    fMpd[id].fTriggerMode = MPD_TRIG_MODE_NONE;
  else
    {
      if (lat > 0)
	{
	  fMpd[id].fTriggerMode = MPD_TRIG_MODE_CALIB;
	}
      else
	{
	  if (num == 1)
	    fMpd[id].fTriggerMode = MPD_TRIG_MODE_APV;
	  else if (num > 1)
	    fMpd[id].fTriggerMode = MPD_TRIG_MODE_MULTI;
	}
    }
  mpdSetTriggerNumber(id, num);
  mpdSetCalibLatency(id, lat);
  mpdSetTriggerLatency(id, tlat);
  MPD_MSG("Calib / Trigger Latency = %d %d, Trigger Mode = 0x%x\n",
	  lat, tlat, fMpd[id].fTriggerMode);
};

int
mpdGetTriggerMode(int id)
{
  return fMpd[id].fTriggerMode;
};

void
mpdSetAcqMode(int id, char *name)
{
  fMpd[id].fAcqMode = 0;	// disabled
  if (strcmp(name, "ramtest") == 0)
    fMpd[id].fAcqMode = MPD_DAQ_RAM_TEST;
  if (strcmp(name, "histo") == 0)
    fMpd[id].fAcqMode = MPD_DAQ_HISTO;
  if (strcmp(name, "event") == 0)
    fMpd[id].fAcqMode = MPD_DAQ_EVENT;
  if (strcmp(name, "process") == 0)
    fMpd[id].fAcqMode = MPD_DAQ_PROCESS;
  if (strcmp(name, "sample") == 0)
    fMpd[id].fAcqMode = MPD_DAQ_SAMPLE;
  if (strcmp(name, "sync") == 0)
    fMpd[id].fAcqMode = MPD_DAQ_SYNC;

  MPD_MSG("Acquisition Mode = 0x%x (%s)\n", fMpd[id].fAcqMode, name);
};

int
mpdGetAcqMode(int id)
{
  return fMpd[id].fAcqMode;
};


void
mpdSetInPath0(int id, int t1P0, int t2P0, int tFront, int sP0, int sFront)
{
  fMpd[id].fInPath[MPD_IN_P0][MPD_IN_TRIG1] = t1P0;
  fMpd[id].fInPath[MPD_IN_P0][MPD_IN_TRIG2] = t2P0;
  fMpd[id].fInPath[MPD_IN_FRONT][MPD_IN_TRIG] = tFront;
  fMpd[id].fInPath[MPD_IN_P0][MPD_IN_SYNC] = sP0;
  fMpd[id].fInPath[MPD_IN_FRONT][MPD_IN_SYNC] = sFront;
};

void
mpdSetInPath(int id, int conn, int signal, int val)
{
  fMpd[id].fInPath[conn % 2][signal % 3] = val;
};

int
mpdGetInPath(int id, int conn, int signal)
{
  return fMpd[id].fInPath[conn % 2][signal % 3];
};

int
mpdGetInPathI(int id, int conn, int signal)
{
  return ((fMpd[id].fInPath[conn % 2][signal % 3] == 1) ? 1 : 0);
};

void
mpdSetInputLevel(int id, int conn, short val)
{
  if (conn >= 0 && conn < 2)
    {
      fMpd[id].fInLevelTTL[conn] = val;
    }
};

short
mpdGetInputLevel(int id, int conn)
{
  return fMpd[id].fInLevelTTL[conn];
};

void
mpdSetOutputLevel(int id, int conn, short val)
{
  if (conn >= 0 && conn < 2)
    {
      fMpd[id].fOutLevelTTL[conn] = val;
    }
};

short
mpdGetOutputLevel(int id, int conn)
{
  return fMpd[id].fOutLevelTTL[conn];
};

uint32_t
mpdGetFpgaRevision(int id)
{
  if (fMpd[id].FpgaRevision == 99999)
    {
      MPD_MSG("Fpga revision not set yet, something wrong! exit(%d)", 0);
/*     exit(0); */
    }
  return fMpd[id].FpgaRevision;
}

void
mpdSetFpgaRevision(int id, uint32_t r)
{
  fMpd[id].FpgaRevision = r;
}

uint32_t
mpdGetHWRevision(int id)
{
  uint32_t d = mpdGetFpgaRevision(id);
  return ((d >> 24) & 0xff);
}

uint32_t
mpdGetFWRevision(int id)
{
  uint32_t d = mpdGetFpgaRevision(id);
  return (d & 0xff);
}

uint32_t
mpdGetFpgaCompileTime(int id)
{
  return fMpd[id].FpgaCompileTime;
}

void
mpdSetFpgaCompileTime(int id, uint32_t t)
{
  fMpd[id].FpgaCompileTime = t;
}

int
mpdLM95235_Read(int id, double *core_t, double *air_t)
{
  const uint8_t LM95235_i2c_addr = 0x4C;
  const uint8_t Local_TempS_MSB_addr = 0x00;	// Read only
  const uint8_t Local_TempS_LSB_addr = 0x30;	// Read only
  const uint8_t Remote_TempU_MSB_addr = 0x31;	// Read only
  const uint8_t Remote_TempU_LSB_addr = 0x32;	// Read only
  const uint8_t ConfigReg1_addr = 0x03;	// also 0x09
  const uint8_t OneShot_addr = 0x0F;	// Write only
  const uint8_t Status1_addr = 0x02;	// Read only

  uint8_t val, val2;
  int success, retry_count;
  // if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  val = 0x40;			// standby
  success =
    mpdI2C_ByteWrite(id, (uint8_t) (LM95235_i2c_addr << 1), ConfigReg1_addr,
		     1, &val);
  success =
    mpdI2C_ByteWrite(id, (uint8_t) (LM95235_i2c_addr << 1), OneShot_addr, 0,
		     &val);
  success =
    mpdI2C_ByteWrite(id, (uint8_t) (LM95235_i2c_addr << 1), Status1_addr, 0,
		     &val);
  val = 0x80;
  retry_count = 0;
  //  while( (val & 0x80) && (retry_count++ < 1) )
  success = mpdI2C_ByteRead1(id, (uint8_t) (LM95235_i2c_addr << 1), &val);

  success =
    mpdI2C_ByteWrite(id, (uint8_t) (LM95235_i2c_addr << 1),
		     Remote_TempU_MSB_addr, 0, &val);
  success = mpdI2C_ByteRead1(id, (uint8_t) (LM95235_i2c_addr << 1), &val);
  success =
    mpdI2C_ByteWrite(id, (uint8_t) (LM95235_i2c_addr << 1),
		     Remote_TempU_LSB_addr, 0, &val2);
  success = mpdI2C_ByteRead1(id, (uint8_t) (LM95235_i2c_addr << 1), &val2);
  *core_t = (double) val + (double) val2 / 256.;


  success =
    mpdI2C_ByteWrite(id, (uint8_t) (LM95235_i2c_addr << 1),
		     Local_TempS_MSB_addr, 0, &val);
  success = mpdI2C_ByteRead1(id, (uint8_t) (LM95235_i2c_addr << 1), &val);
  success =
    mpdI2C_ByteWrite(id, (uint8_t) (LM95235_i2c_addr << 1),
		     Local_TempS_LSB_addr, 0, &val2);
  success = mpdI2C_ByteRead1(id, (uint8_t) (LM95235_i2c_addr << 1), &val2);
  *air_t = (double) val + (double) val2 / 256.;
  val = 0x0;			//normal operation
  success =
    mpdI2C_ByteWrite(id, (uint8_t) (LM95235_i2c_addr << 1), ConfigReg1_addr,
		     1, &val);
  return success;

}


/****************************
 * LOW LEVEL routines
 ****************************/


/* I2C methods */
void
mpdSetI2CSpeed(int id, int val)
{
  fMpd[id].fI2CSpeed = val;
};

int
mpdGetI2CSpeed(int id)
{
  return fMpd[id].fI2CSpeed;
};

void
mpdSetI2CMaxRetry(int id, int val)
{
  fMpd[id].fI2CMaxRetry = val;
};

int
mpdGetI2CMaxRetry(int id)
{
  return fMpd[id].fI2CMaxRetry;
};

int
mpdI2C_ApvReset(int id)
{
  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  MPDUNLOCK;
  mpdWrite32(&MPDp[id]->i2c.apv_reset, MPD_I2C_APVRESET_ASYNC_CLEAR);
  mpdWrite32(&MPDp[id]->i2c.apv_reset, MPD_I2C_APVRESET_ASYNC_SET);
  MPDUNLOCK;

  return OK;
}

int
mpdI2C_Init(int id)
{
  uint32_t data, rdata;
  int success = OK;
  /* double core_t, air_t; */

  //  if(id==0) id=mpdID[0];
  if (((int32_t) MPDp[id] == -1) || (id < 0) || (id > 21))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  /*
     clock_prescaler = prescaler_high * 256 + prescaler_low
     period = clock_prescale/10 us
   */

  MPDLOCK;
  mpdWrite32(&MPDp[id]->i2c.control, 0);	/* Disable I2C Core and interrupts */

  int ispeed = mpdGetI2CSpeed(id);

  data = (ispeed & 0xFF);
  mpdWrite32(&MPDp[id]->i2c.clock_prescaler_low, data);

  rdata = mpdRead32(&MPDp[id]->i2c.clock_prescaler_low);

  MPD_MSG("i2c low prescaler register set/read : %d / %d\n", data, rdata);

  data = (ispeed >> 8) & 0xff;
  mpdWrite32(&MPDp[id]->i2c.clock_prescaler_high, data);
  rdata = mpdRead32(&MPDp[id]->i2c.clock_prescaler_high);

  MPD_MSG("i2c high prescaler register set/read : %d / %d\n", data, rdata);

  MPD_MSG("i2c speed prescale = %d, (period = %f us, frequency = %f kHz)\n",
	  ispeed, ispeed / 10., 10000. / ispeed);

  mpdWrite32(&MPDp[id]->i2c.control, MPD_I2C_CONTROL_ENABLE_CORE);

  MPDUNLOCK;

  usleep(500);

  //  mpdLM95235_Read(id, &core_t, &air_t);

  //  printf("%s: Board temperatures: core=%.2f air=%.2f (dec celsius)\n",__FUNCTION__,core_t,air_t);

  return success;

}

int
mpdI2C_ByteWrite(int id, uint8_t dev_addr, uint8_t int_addr,
		 int ndata, uint8_t * data)
{
  int success, i;
  // if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  if ((success = I2C_SendByte(id, (uint8_t) (dev_addr & 0xFE), 1)) != OK)
    {
      //      if (success < 0) printf("SendByte DEVadd ret %d\n",success);
      if (success <= -10)
	{
	  I2C_SendStop(id);
	  return success;
	}
      else
	return I2C_SendStop(id);
    }
  // usleep(300);

  if ((success = I2C_SendByte(id, int_addr, 0)) != OK)
    {
      //      if (success < 0) printf("SendByte INTadd ret %d\n",success);
      if (success <= -10)
	{
	  I2C_SendStop(id);
	  return (success | 1);
	}
      else
	return I2C_SendStop(id);
    }
  // usleep(300);
  for (i = 0; i < ndata; i++)
    if ((success = I2C_SendByte(id, data[i], 0)) != OK)
      {
	//      if (success < 0) printf("SendByte data ret %d\n",success);
	if (success <= -10)
	  {
	    I2C_SendStop(id);
	    return (success | 2);
	  }
	else
	  return I2C_SendStop(id);
      }

  success = I2C_SendStop(id);

  return success;

}

int
mpdI2C_ByteRead(int id, uint8_t dev_addr, uint8_t int_addr,
		int ndata, uint8_t * data)
{
  int success, i;
  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  if ((success = I2C_SendByte(id, (uint8_t) (dev_addr), 1)) != OK)
    return I2C_SendStop(id);

  if ((success = I2C_SendByte(id, int_addr, 0)) != OK)
    return I2C_SendStop(id);

  if ((success = I2C_SendByte(id, (uint8_t) (dev_addr | 0x01), 1)) != OK)
    return I2C_SendStop(id);

  for (i = 0; i < ndata; i++)
    if ((success = I2C_ReceiveByte(id, data + i)) != OK)
      return I2C_SendStop(id);

  return I2C_SendStop(id);
}

int
mpdI2C_ByteWriteRead(int id, uint8_t dev_addr, uint8_t int_addr,
		     int ndata, uint8_t * data)
{
  int success, i;
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  if ((success = I2C_SendByte(id, (uint8_t) (dev_addr & 0xFE), 1)) != OK)
    return I2C_SendStop(id);

  if ((success = I2C_SendByte(id, int_addr, 0)) != OK)
    return I2C_SendStop(id);

  for (i = 0; i < ndata; i++)
    if ((success = I2C_ReceiveByte(id, data + i)) != OK)
      return I2C_SendStop(id);

  return I2C_SendStop(id);
}


int
mpdI2C_ByteRead1(int id, uint8_t dev_addr, uint8_t * data)
{
  int success;
  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }


  //  if( (success = I2C_SendByte(id, (uint8_t)(dev_addr & 0xFE), 1)) != OK )
  if ((success = I2C_SendByte(id, (uint8_t) (dev_addr & 0x1), 1)) != OK)
    return I2C_SendStop(id);

  usleep(100);
  if ((success = I2C_ReceiveByte(id, data)) != OK)
    return I2C_SendStop(id);
  usleep(100);

  return I2C_SendStop(id);
}

/*static*/ int
I2C_SendByte(int id, uint8_t byteval, int start)
{
  int rval = OK, retry_count;
  volatile uint32_t data = 0;
  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  MPDLOCK;
  //  printf(" MPD addr I2C.txrx : 0x%x\n", &MPDp[id]->i2c.tx_rx - &MPDp[id]->magic_value);
  mpdWrite32(&MPDp[id]->i2c.tx_rx, byteval);
  //  printf("mpdWrite TxRx:%d %d %08x \n",byteval,mpdRead32(&MPDp[id]->i2c.tx_rx),mpdRead32(&MPDp[id]->i2c.tx_rx));

  if (start)
    mpdWrite32(&MPDp[id]->i2c.comm_stat, MPD_I2C_COMMSTAT_START_WRITE);
  else
    mpdWrite32(&MPDp[id]->i2c.comm_stat, MPD_I2C_COMMSTAT_WRITE);

  //  usleep(500);
  retry_count = 0;
  data = 0x00000002;
  // printf("mpdWrite TxRx:%d",mpdRead32(&MPDp[id]->i2c.tx_rx));

  while ((data & 0x00000002) != 0 && retry_count < mpdGetI2CMaxRetry(id))
    {
      usleep(10);
      data = mpdRead32(&MPDp[id]->i2c.comm_stat);
      if (retry_count > 0)
	printf("mpdRead commstat:%08x %d (%d)\n", data, retry_count,
	       mpdGetI2CMaxRetry(id));
      retry_count++;
    }

  if (retry_count >= mpdGetI2CMaxRetry(id))
    rval = -10;

  if (data & MPD_I2C_COMMSTAT_NACK_RECV)	/* NACK received */
    rval = -20;

  MPDUNLOCK;

  return rval;
}

/*static*/ int
I2C_ReceiveByte(int id, uint8_t * byteval)
{
  int retry_count;
  int rval = 0;
  uint32_t data = 0;
  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  MPDLOCK;
  mpdWrite32(&MPDp[id]->i2c.comm_stat, MPD_I2C_COMMSTAT_READ);

  retry_count = 0;
  data = 0x00000002;
  while ((data & 0x00000002) != 0 && retry_count < mpdGetI2CMaxRetry(id))
    {
      usleep(100);
      data = mpdRead32(&MPDp[id]->i2c.comm_stat);
      retry_count++;
    }

  if (retry_count >= mpdGetI2CMaxRetry(id))
    rval = -10;

  if (data & MPD_I2C_COMMSTAT_NACK_RECV)	/* NACK received */
    rval = -20;

  data = mpdRead32(&MPDp[id]->i2c.tx_rx);

  *byteval = data;
  MPDUNLOCK;

  return rval;

}

/*static*/ int
I2C_SendStop(int id)
{
  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  MPDLOCK;
  usleep(100);
  mpdWrite32(&MPDp[id]->i2c.comm_stat, MPD_I2C_COMMSTAT_STOP);
  usleep(100);
  MPDUNLOCK;

  return OK;
}

/*static*/ int
I2C_SendNack(int id)
{
  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  MPDLOCK;
  mpdWrite32(&MPDp[id]->i2c.comm_stat, MPD_I2C_COMMSTAT_NACK);
  MPDUNLOCK;

  return OK;
}

/* ADC set/get methods */
void
mpdSetAdcClockPhase(int id, int adc, int phase)
{
  fMpd[id].fAdcClockPhase[adc] = phase;
};

int
mpdGetAdcClockPhase(int id, int adc)
{
  int clock_phase;
  clock_phase = fMpd[id].fAdcClockPhase[adc];
  if (mpdGetFpgaRevision(id) < 2)
    {				// no delay line
      switch (clock_phase)
	{
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
	  MPD_MSG("Warning: apv_clock phase %d out of range for Fpga rev. %d",
		  clock_phase, mpdGetFpgaRevision(id));
	}
    }
  return clock_phase;
};


void
mpdSetFIRenable(int id, int flag)
{
  fMpd[id].fFIR = flag;
}

int
mpdGetFIRenable(int id)
{
  return fMpd[id].fFIR;
}

void
mpdSetFIRcoeff(int id, int idx, int val)
{
  fMpd[id].fFIRcoeff[idx] = val;
}

int
mpdGetFIRcoeff(int id, int idx)
{
  return fMpd[id].fFIRcoeff[idx];
}

void
mpdSetAdcGain(int id, int adc, int ch, int g)
{
  fMpd[id].fAdcGain[adc][ch] = g;
};

int
mpdGetAdcGain(int id, int adc, int ch)
{
  return fMpd[id].fAdcGain[adc][ch];
};

void
mpdSetAdcInvert(int id, int adc, int val)
{
  fMpd[id].fAdcInvert[adc] = val;
};

int
mpdGetAdcInvert(int id, int adc)
{
  return fMpd[id].fAdcInvert[adc];
};

void
mpdSetAdcPattern(int id, int adc, int p)
{
  fMpd[id].fAdcPattern[adc] = p;
};

int
mpdGetAdcPattern(int id, int adc)
{
  return fMpd[id].fAdcPattern[adc];
};


/* APV methods */
int
mpdApvGetLatency(int id, int ia)
{
  return fApv[id][ia].Latency;
};

int
mpdApvGetCalibrationMode(int id, int ia)
{
  return (1 - ((fApv[id][ia].Mode >> 2) & 0x1));
};

int
mpdApvGetMode(int id, int ia)
{
  return (fApv[id][ia].Mode & 0x3F);
};

int
mpdApvGetSample(int id, int ia)
{
  return fApv[id][ia].fNumberSample;
};

void
mpdApvSetSampleLeft(int id, int ia)
{
  fApv[id][ia].fReadCount = fApv[id][ia].fNumberSample;
};

void
mpdApvDecSampleLeft(int id, int ia, int n)
{
  fApv[id][ia].fReadCount -= n;
};

int
mpdApvReadDone(int id, int ia)
{
  if (fApv[id][ia].fReadCount <= 0)
    return TRUE;
  return FALSE;
};

int
mpdApvGetSampleLeft(int id, int ia)
{
  return (fApv[id][ia].fNumberSample - mpdApvGetBufferSample(id, ia));
};

int
mpdApvGetSampleIdx(int id, int ia)
{
  return (fApv[id][ia].fNumberSample - fApv[id][ia].fReadCount);
};

int
mpdApvGetAdc(int id, int ia)
{
  return fApv[id][ia].adc;
};

int
mpdAPV_Reset101(int id)
{
  uint32_t data;
  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  MPDLOCK;

  data = mpdRead32(&MPDp[id]->trigger_config);

  data |= MPD_APVDAQ_TRIGCONFIG_ENABLE_MACH;	// Enable trig machine
  data |= SOFTWARE_CLEAR_MASK;

  mpdWrite32(&MPDp[id]->trigger_config, data);

  usleep(100);

  data &= ~SOFTWARE_CLEAR_MASK;
  data &= ~MPD_APVDAQ_TRIGCONFIG_ENABLE_MACH;	// Disable trig machine

  mpdWrite32(&MPDp[id]->trigger_config, data);

  MPDUNLOCK;

  usleep(100);

  return OK;
}

/*
// generate software trigger
int
mpdAPV_SoftTrigger(int id)
{
  uint32_t data;
  //  if(id==0) id=mpdID[0];
  if(CHECKMPD(id))
    {
      printf("%s: ERROR: MPD in slot %d is not initialized.\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  MPDLOCK;

   data = mpdRead32(&MPDp[id]->ApvDaq.Trig_Gen_Config);


   //   data |= MPD_APVDAQ_TRIGCONFIG_ENABLE_MACH;	// Enable trig machine
  data |= SOFTWARE_TRIGGER_MASK;

  mpdWrite32(&MPDp[id]->ApvDaq.Trig_Gen_Config, data);

  data &= ~SOFTWARE_TRIGGER_MASK;
  //   data &= ~MPD_APVDAQ_TRIGCONFIG_ENABLE_MACH;	// Disable trig machine

    mpdWrite32(&MPDp[id]->ApvDaq.Trig_Gen_Config, data);

  MPDUNLOCK;

  return OK;
}
*/
/*
 * return true if apv is present
 *   (EC: to be improved not yet 100% reliable)
 */
int
mpdAPV_Try(int id, uint8_t apv_addr)	// i2c addr
{

  int timeout;
  uint8_t x = apv_addr + 32;	//96
  /* uint8_t y; */
  int ret;

  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  timeout = -1;

  do
    {
      ret = mpdAPV_Write(id, apv_addr, latency_addr, x);
      timeout++;
    }
  while (((ret == -20) || (ret == -19) || (ret == -18)) && (timeout < 20));

  if (ret < 0)
    MPD_MSG("MPD %d i2c %d w %d tout %d ret %d\n", id, apv_addr, x, timeout,
	    ret);

  // mpdAPV_Read(id, apv_addr, latency_addr, &y);
  // printf("%s: latency read: %0x %0x\n",__FUNCTION__,x,y);

  //  return APV_Write(apv_addr, mode_addr, def_Mode);

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
  int iapv = 0;
  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  // print information (debug only, should be done in verbose mode only)
  MPD_MSG("MPD %d Blind scan on %d apvs: \n", id, MPD_MAX_APV);	// in principle can be more

  for (iapv = 0; iapv < MPD_MAX_APV; iapv++)
    {
      //      printf("%s MPD %d I2C addr %d:\n",__FUNCTION__,id,iapv);
      if (mpdAPV_Try(id, iapv) > -1)
	{
	  MPD_MSG("MPD %d found candidate card at i2c addr %2d\n", id, iapv);
	}
    }
  MPD_MSG("MPD %d Blind scan done\n", id);

  mpdResetApvEnableMask(id);

  nApv[id] = 0;
  for (iapv = 0; iapv < fMpd[id].nAPV; iapv++)
    {
      MPD_MSG("Try i2c=%2d adc=%2d : ", fApv[id][iapv].i2c,
	      fApv[id][iapv].adc);

      if (mpdAPV_Try(id, fApv[id][iapv].i2c) > -1 && fApv[id][iapv].adc > -1)
	{
	  MPD_MSG("%d matched in MPD in slot %d\n", fApv[id][iapv].i2c, id);

	  mpdSetApvEnableMask(id, (1 << fApv[id][iapv].adc));
	  //printf("%s: APV enable mask 0x%04x\n", __FUNCTION__,mpdGetApvEnableMask(id));
	  fApv[id][iapv].enabled = 1;
	  nApv[id]++;
	}
      else
	{
	  MPD_MSG("MPD %d APV i2c = %d does not respond.  It is disabled\n",
		  id, fApv[id][iapv].i2c);
	  fApvEnableMask[id] &= ~(1 << fApv[id][iapv].adc);
	  fApv[id][iapv].enabled = 0;
	}
    }

  MPD_MSG("%d APV found matching settings\n", nApv[id]);

  return nApv[id];
}


int
mpdAPV_Write(int id, uint8_t apv_addr, uint8_t reg_addr, uint8_t val)
{
  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }
  //  usleep(500);
  return mpdI2C_ByteWrite(id, (uint8_t) ((0x20 | apv_addr) << 1), reg_addr, 1,
			  &val);

}

int
mpdAPV_Read(int id, uint8_t apv_addr, uint8_t reg_addr, uint8_t * val)
{
  int success;
  uint8_t rval = 0;

  if (id == 0)
    id = mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  usleep(500);

  // success = mpdI2C_ByteWrite(id, (uint8_t)((0x20 | apv_addr)<<1), (reg_addr|0x01), 1, val);
  // if( success != OK )
  //   return success;

  usleep(500);
  success =
    mpdI2C_ByteRead(id, (uint8_t) ((0x20 | apv_addr) << 1), reg_addr, 1, val);

  MPD_MSG("Get / Return = 0x%x 0x%x\n", *val, rval);

  return success;
}

int
mpdAPV_Config(int id, int apv_index)
{
  int success, i;
  uint8_t apv_addr, reg_addr = 0, val = 0;

  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  apv_addr = fApv[id][apv_index].i2c;

  printf("APV card i2c=%d to ADC (fifo)=%d (from config file)\n",
	 (int) apv_addr, fApv[id][apv_index].adc);

  for (i = 0; i < 18; i++)
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
	  MPD_ERR
	    ("This message should not appear, please check code consistency");
	}

      usleep(300);
      success = mpdAPV_Write(id, apv_addr, reg_addr, val);

      if (success != OK)
	{
	  MPD_ERR("I2C Bus Error: i/addr/reg/val/err %d/ %d 0x%x 0x%x 0x%x",
		  i, apv_addr, reg_addr, val, success);
	  return success;
	}
    }				// end loop

  //#ifdef DOTHISDIFFERENTLY
  // need improvement
  fApv[id][apv_index].fNumberSample = mpdGetTriggerNumber(id) * mpdApvGetPeakMode(id);	// must be set !!
  mpdApvBufferFree(id, apv_index);
  mpdApvBufferAlloc(id, apv_index);	// alloc readout buffer
  //#endif /* DOTHISDIFFERENTLY */

  return success;

}

/**
 * return 0 if card is disable
 */

short
mpdApvEnabled(int id, int ia)
{
  return fApv[id][ia].enabled;
}

/**
 * Return the setting value of the number of samples per trigger (1 or 3)
 * this is the same for all Apvs
 * return -1 if there are no APV connected
 */
int
mpdApvGetPeakMode(int id)
{

  int c = -1;
  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  if (nApv[id] > 0)
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
     if(CHECKMPD(id))
     {
     MPD_ERR("MPD in slot %d is not initialized.\n",
     id);
     return;
     }
   */

  memcpy((void *) &fApv[id][nApv[id]], &v, sizeof(ApvParameters));
  fApv[id][nApv[id]].fBuffer = 0;

  MPD_MSG("APV %d added to list of FECs\n", nApv[id]);
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
  int c = 0;
  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }


  if (nApv[id] > 0)
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
  uint8_t c = 0;
  uint32_t iapv = 0;
  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  for (iapv = 0; iapv < fMpd[id].nAPV; iapv++)
    {
      if (fApv[id][iapv].enabled)
	{
	  c = (fApv[id][iapv].Latency > c) ? fApv[id][iapv].Latency : c;
	}
    }
  return c;
}

void
mpdApvBufferAlloc(int id, int ia)
{
  GEF_STATUS status;
  GEF_MAP_PTR mapPtr;
  GEF_VME_DMA_HDL dma_hdl;

  fApv[id][ia].fBufSize = 15 * fApv[id][ia].fNumberSample * (EVENT_SIZE + 2);	// at least 6 times larger @@@ increased to 15 -- need improvement

#define PHYSMEM
#ifdef PHYSMEM
  status = gefVmeAllocDmaBuf(vmeHdl, fApv[id][ia].fBufSize,
			     &dma_hdl, &mapPtr);
  if (status != GEF_STATUS_SUCCESS)
    {
      MPD_ERR("id=%d, ia=%d\n\tgefVmeAllocDmaBuf returned 0x%x\n", id, ia,
	      status);
    }
  fApv[id][ia].fBuffer = (uint32_t *) mapPtr;
  fApv[id][ia].physMemBase = dmaHdl_to_PhysAddr(dma_hdl);
  fApv[id][ia].dmaHdl = dma_hdl;
  // fApv[id][ia].fBuffer = (uint32_t *) malloc(fApv[id][ia].fBufSize*sizeof(uint32_t));
  fApv[id][ia].fBi1 = 0;
  printf
    ("Fifo %d, buffer allocated with word size %d\n\t id=%d  ia=%d  dmaHdl = 0x%08x  physMemBase = 0x%08x  fBuffer = 0x%08x\n",
     fApv[id][ia].adc, fApv[id][ia].fBufSize, id, ia,
     (uint32_t) fApv[id][ia].dmaHdl, (uint32_t) fApv[id][ia].physMemBase,
     (uint32_t) fApv[id][ia].fBuffer);
#else
  fApv[id][ia].fBuffer =
    (uint32_t *) malloc(fApv[id][ia].fBufSize * sizeof(uint32_t));
  fApv[id][ia].fBi1 = 0;
  MPD_DBG("id=%d  ia=%d  Fifo %d, buffer allocated with word size %d\n", id,
	  ia, fApv[id][ia].adc, fApv[id][ia].fBufSize);
#endif
}

void
mpdApvBufferFree(int id, int ia)
{
#ifdef PHYSMEM
  GEF_STATUS status;
#endif
  if (fApv[id][ia].fBuffer != 0)
    {
#ifdef PHYSMEM
      status = gefVmeFreeDmaBuf(fApv[id][ia].dmaHdl);
      if (status != GEF_STATUS_SUCCESS)
	{
	  MPD_ERR("id=%d, ia=%d\n\tgefVmeFreeDmaBuf returned 0x%x\n", id, ia,
		  status);
	}
#else
      free(fApv[id][ia].fBuffer);
#endif
      MPD_DBG("Fifo %d, buffer released\n", fApv[id][ia].adc);
    }
}

void
mpdApvIncBufferPointer(int id, int ia, int b)
{
  fApv[id][ia].fBi1 += b;
}

uint32_t *
mpdApvGetBufferPointer(int id, int ia, int ib)
{
  //MPD_DBG("Fifo %d, retrieved pointer from position %d , size is %d\n",fApv[id][ia].adc, ib, fApv[id][ia].fBufSize);
  if (ib < fApv[id][ia].fBufSize)
    {				// probably not required !!
      return &(fApv[id][ia].fBuffer[ib]);
    }
  MPD_ERR("MPD %d Fifo %d, index %d is out of range (buf size = %d)\n", id,
	  fApv[id][ia].adc, ib, fApv[id][ia].fBufSize);
  exit(1);
}

int
mpdApvGetBufferSample(int id, int ia)
{
  int ix = fApv[id][ia].fBi1 / EVENT_SIZE;
  //MPD_DBG("Fifo = %d has %d samples (%d bytes) stored\n",fApv[id][ia].adc, ix, fApv[id][ia].fBi1);
  return ix;

}

uint32_t *
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
  return fApv[id][ia].fBufSize - fApv[id][ia].fBi1 - 1;	// @@@@ Apr/2013 tobe verified
}

int
mpdApvGetBufferLength(int id, int ia)
{
  return fApv[id][ia].fBi1;
}

int
mpdApvGetEventSize(int id, int ia)
{
  return EVENT_SIZE;		// TO BE CHANGED (variable size !!!)
}


/**
 * Set the number of samples / event expected from the single Apv
 */
int
mpdArmReadout(int id)
{
  uint32_t iapv = 0;
  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  for (iapv = 0; iapv < fMpd[id].nAPV; iapv++)
    {
      if (fApv[id][iapv].enabled)
	{
	  mpdApvSetSampleLeft(id, iapv);	// improve peak mode
	  fApv[id][iapv].fBi0 = 0;	// begin of buffer (should be always 0)
	  fApv[id][iapv].fBs = 0;	// end of event (last sample end mark)
	  fApv[id][iapv].fBi1 = 0;	// end of buffer
	}
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
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
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
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
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
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  mark_ch = (uint8_t) mpdGetChannelMark(id);	// set one of the 128 channels of all apv to 0xfff (mark a single channel of the frame)

  sync_period = (mpdApvGetFrequency(id) == 1) ? 34 : 69;	// synch period in number of clock - 1 (34 @ 40 MHz, 69 @ 20 MHz) @@@ To be checked, ask Paolo

  reset_latency = 15 + mpdApvGetMaxLatency(id);	// @@@ To be ckecked, ask paolo for meaning

  MPDLOCK;

  mpdWrite32(&MPDp[id]->marker_channel, mark_ch);
  mpdWrite32(&MPDp[id]->sync_period, sync_period);
  mpdWrite32(&MPDp[id]->channel_enable, mpdGetApvEnableMask(id));

  data = mpdGetTriggerLatency(id);
  mpdWrite32(&MPDp[id]->trigger_delay, data);

  mpdWrite32(&MPDp[id]->zero_threshold, mpdGetZeroLevel(id));
  mpdWrite32(&MPDp[id]->one_threshold, mpdGetOneLevel(id));

  data =
    ((mpdGetCalibLatency(id) & 0xFF) << 24) |
    ((mpdGetInPathI(id, MPD_IN_FRONT, MPD_IN_TRIG) & 0x01) << 23) |
    ((mpdGetInPathI(id, MPD_IN_P0, MPD_IN_TRIG2) & 0x01) << 22) |
    ((mpdGetInPathI(id, MPD_IN_P0, MPD_IN_TRIG1) & 0x01) << 21) |
    ((mpdGetInPathI(id, MPD_IN_FRONT, MPD_IN_SYNC) & 0x01) << 17) |
    ((mpdGetInPathI(id, MPD_IN_P0, MPD_IN_SYNC) & 0x01) << 16) |
    //    ((test_mode & 0x01) << 15) |
    ((mpdGetTriggerMode(id) & 0x07) << 12) |
    ((mpdGetTriggerNumber(id) & 0x0F) << 8) | reset_latency;

  mpdWrite32(&MPDp[id]->trigger_config, data);

  MPDUNLOCK;


  return OK;
}

int
mpdTRIG_Disable(int id)
{
  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  MPDLOCK;
  mpdWrite32(&MPDp[id]->channel_enable, 0);
  mpdWrite32(&MPDp[id]->trigger_config, 0);
  MPDUNLOCK;

  return OK;

}

int
mpdTRIG_PauseEnable(int id, int time)
{
  mpdTRIG_Enable(id);
  usleep(time);
  MPDLOCK;
  mpdWrite32(&MPDp[id]->trigger_config, 0);
  MPDUNLOCK;
  return OK;
}


int
mpdTRIG_GetMissed(int id, uint32_t * missed)
{
  if (id == 0)
    id = mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  MPDLOCK;
  *missed = mpdRead32(&MPDp[id]->ob_status.missed_trigger);
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
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  MPD_DBG("start (%d: %d %d)\n", id, apv1_delay, apv2_delay);

  //      mpdI2C_ByteWrite(0xF0 , (apv2_delay & 0x3F) | 0x40, 0, &val);      // CR0: APV2 out
  mpdI2C_ByteWrite(id, 0xF0, 0x40, 0, &val);	// CR0: APV2 out not delayed
  usleep(10000);
  //      mpdI2C_ByteWrite(0xF2 , 0x40, 0, &val);    // CR1: ADC1 out not delayed
  mpdI2C_ByteWrite(id, 0xF2, (apv1_delay & 0x3F) | 0x40, 0, &val);	// CR1: ADC1 clock delayed
  usleep(10000);
  //      mpdI2C_ByteWrite(0xF4 , 0x40, 0, &val);    // CR2: ADC2 out not delayed
  mpdI2C_ByteWrite(id, 0xF4, (apv2_delay & 0x3F) | 0x40, 0, &val);	// CR2: ADC2 clock delayed
  usleep(10000);
  mpdI2C_ByteWrite(id, 0xF6, 0x00, 0, &val);	// CR3: Not used output
  usleep(10000);
  //      mpdI2C_ByteWrite(0xF8 , (apv1_delay & 0x3F) | 0x40, 0, &val);      // CR4: APV1 out
  mpdI2C_ByteWrite(id, 0xF8, 0x40, 0, &val);	// CR4: APV1 out not delayed
  usleep(10000);
  mpdI2C_ByteWrite(id, 0xFA, 0x00, 0, &val);	// GCR (40 MHz)
  usleep(10000);

  mpdI2C_ByteWrite(id, 0xFA, 0x40, 0, &val);	// resync DDL
  usleep(10000);

  mpdI2C_ByteWrite(id, 0xFA, 0x00, 0, &val);
  usleep(10000);

  MPD_DBG("end\n");

  return 0;
}

//======================================================
//

int
mpdFIR_Config(int id)
{

  uint32_t data, rdata;
  int i;
  int coeff0, coeff1;

  int npar = 16;

  // set coeff
  MPD_DBG("FIR coefficients (FIRenable=%d):\n", mpdGetFIRenable(id));
  for (i = 0; i < npar / 2; i++)
    {
      coeff0 = mpdGetFIRcoeff(id, i * 2);
      coeff1 = mpdGetFIRcoeff(id, i * 2 + 1);
      data = ((coeff1 << 16) & 0xffff0000) | (coeff0 & 0xffff);
      printf(" %2d: W 0x%4x 0x%4x", i * 2, coeff0 & 0xffff, coeff1 & 0xffff);

      mpdWrite32(&MPDp[id]->fir_coefficients[i], data);

      rdata = mpdRead32(&MPDp[id]->fir_coefficients[i]);
      printf(" R 0x%4x 0x%4x\n", rdata & 0xffff, (rdata >> 16) & 0xffff);
    }

  return OK;

}

//======================================================
// adc methods

#define MPD_ADC_TOUT 1000
#define MPD_ADC_USLEEP 50

/**
 * init and configure ADC
 */

int
mpdADS5281_Config(int id)
{

  int j;
  for (j = 0; j < 2; j++)
    {
      if (mpdADS5281_SetParameters(id, j) != OK)
	{
	  printf("ERR: adc %d set parameter failed on mpd %d\n", j, id);
	};
      switch (mpdGetAdcPattern(id, j))
	{
	case MPD_ADS5281_PAT_NONE:
	  mpdADS5281_Normal(id, j);
	  break;
	case MPD_ADS5281_PAT_SYNC:
	  mpdADS5281_Sync(id, j);
	  break;
	case MPD_ADS5281_PAT_DESKEW:
	  mpdADS5281_Deskew(id, j);
	  break;
	case MPD_ADS5281_PAT_RAMP:
	  mpdADS5281_Ramp(id, j);
	  break;
	}
      if (mpdGetAdcInvert(id, j))
	{
	  mpdADS5281_InvertChannels(id, j);
	}
      else
	{
	  mpdADS5281_NonInvertChannels(id, j);
	}

      if (mpdADS5281_SetGain(id, j,
			     mpdGetAdcGain(id, j, 0),
			     mpdGetAdcGain(id, j, 1),
			     mpdGetAdcGain(id, j, 2),
			     mpdGetAdcGain(id, j, 3),
			     mpdGetAdcGain(id, j, 4),
			     mpdGetAdcGain(id, j, 5),
			     mpdGetAdcGain(id, j, 6),
			     mpdGetAdcGain(id, j, 7)) != OK)
	printf("WRN: Set ADC Gain %d failed on mpd %d\n", j, id);
    }

  // FIR filter configuration (april/2015)
  if (mpdGetFpgaCompileTime(id) >= 1429878298)
    {				// Apr 24 14:24:58 2015
      mpdFIR_Config(id);	// configure FIR coefficients (FIR is enabled in DAQ_Config)
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
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  data = (adc == 0) ? 0x40000000 : 0x80000000;
  data |= val;

  MPDLOCK;
  mpdWrite32(&MPDp[id]->adc_config, data);
  usleep(MPD_ADC_USLEEP);
  MPDUNLOCK;

  return OK;

  // BM: Retain this stuff if there's a problem writing to the adc_config
/* #ifdef DUM */
/*   MPD_DUM("Write Adc= %d: value= 0x%x, success= 0x%x\n",adc, data, success); */
/* #endif */

/*   retry_count = 0; */
/*   data = 0xC0000000; */
/*   while( ((data & 0xFFFFFFF) != val) && (success == BUS_OK) && (retry_count < MPD_ADC_TOUT) ) { */
/*     success = BUS_Read(adc_configOffset, &data); */
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
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  MPD_DBG("Board= %d ADC= %d Inverted Polarity\n", fMpd[id].fSlot, adc);
  return mpdADS5281_Set(id, adc, 0x2400FF);
}

int
mpdADS5281_NonInvertChannels(int id, int adc)	/* adc == 0, 1 */
{
  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  MPD_MSG("Board= %d ADC= %d Direct Polarity\n", fMpd[id].fSlot, adc);
  return mpdADS5281_Set(id, adc, 0x240000);

}

int
mpdADS5281_SetParameters(int id, int adc)	/* adc == 0, 1 */
{
  int success;

  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  if (mpdGetHWRevision(id) >= 4)
    {
      if ((success = mpdADS5281_Set(id, adc, 0x428021)) != OK)
	{
	  return success;
	}			// Differential clock
    }
  else
    {
      if ((success = mpdADS5281_Set(id, adc, 0x428020)) != OK)
	{
	  return success;
	}			// Single ended clock
    }
  usleep(MPD_ADC_USLEEP);
  return mpdADS5281_Set(id, adc, 0x110000);
}

int
mpdADS5281_Normal(int id, int adc)	/* adc == 0, 1 */
{
  int success;

  if (id == 0)
    id = mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  MPD_MSG("Board= %d ADC= %d No Pattern (Normal Acq)\n", fMpd[id].fSlot, adc);
  if ((success = mpdADS5281_Set(id, adc, 0x450000)) != OK)
    {
      return success;
    }
  usleep(MPD_ADC_USLEEP);
  return mpdADS5281_Set(id, adc, 0x250000);
}

int
mpdADS5281_Sync(int id, int adc)	/* adc == 0, 1 */
{
  int success;

  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  MPD_MSG("Board= %d ADC= %d Sync Pattern\n", fMpd[id].fSlot, adc);
  if ((success = mpdADS5281_Set(id, adc, 0x250000)) != OK)
    {
      return success;
    }
  usleep(MPD_ADC_USLEEP);
  return mpdADS5281_Set(id, adc, 0x450002);
}

int
mpdADS5281_Deskew(int id, int adc)	/* adc == 0, 1 */
{
  int success;

  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  MPD_MSG("Board= %d ADC= %d Deskew Pattern\n", fMpd[id].fSlot, adc);
  if ((success = mpdADS5281_Set(id, adc, 0x250000)) != OK)
    {
      return success;
    }
  usleep(MPD_ADC_USLEEP);
  return mpdADS5281_Set(id, adc, 0x450001);
}

int
mpdADS5281_Ramp(int id, int adc)	/* adc == 0, 1 */
{
  int success;

  if (id == 0)
    id = mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  MPD_MSG("Board= %d ADC= %d Ramp Pattern\n", fMpd[id].fSlot, adc);
  if ((success = mpdADS5281_Set(id, adc, 0x450000)) != OK)
    {
      return success;
    }
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
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  MPD_MSG("Board= %d ADC= %d Set Gain %d %d %d %d %d %d %d %d\n", id, adc,
	  gain0, gain1, gain2, gain3, gain4, gain5, gain6, gain7);

  data = 0x2A0000;
  data |= gain0 | (gain1 << 4) | (gain2 << 8) | (gain3 << 12);
  if ((success = mpdADS5281_Set(id, adc, data)) != OK)
    {
      return success;
    }
  usleep(MPD_ADC_USLEEP);
  data = 0x2B0000;
  data |= gain7 | (gain6 << 4) | (gain5 << 8) | (gain4 << 12);
  return mpdADS5281_Set(id, adc, data);

}

//======================================================
// histogramming methods

int
mpdHISTO_MemTest(int id)
{
  uint32_t *hdata;
  const int MAX_HDATA = 4096;
  int idata = 0, error_count = 0;

  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  hdata = (uint32_t *) malloc(MAX_HDATA * sizeof(uint32_t));

  if (!hdata)
    {
      MPD_ERR("Unable to allocate memory for histogram memory write/read\n");
      return ERROR;
    }

  memset((void *) &hdata, 0, MAX_HDATA * sizeof(uint32_t));

  mpdHISTO_Clear(id, 0, -1);
  mpdHISTO_Read(id, 0, hdata);
  error_count = 0;

  for (idata = 0; idata < MAX_HDATA; idata++)
    {
      if (hdata[idata] != idata)
	{
	  error_count++;
	}
    }
  if (error_count)
    {
      MPD_ERR("HISTO Test fail %d time / %d attempts\n",
	      error_count, MAX_HDATA);
    }
  else
    {
      MPD_MSG("HISTO Read/Write test SUCCESS on MPD slot %d\n", id);
    }

  return OK;
}

int
mpdHISTO_Clear(int id, int ch, int val)	/* ch == 0, 15 */
{
  uint32_t data[4096];
  int success = OK, i, j;
  int block = 0;

  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  if (ch >= 8)
    block = 1;

  if (val < 0)
    {
      for (i = 0; i < 4096; i++)
	data[i] = i;
    }
  else
    {
      for (i = 0; i < 4096; i++)
	data[i] = val;
    }

  MPDLOCK;
#ifdef BLOCK_TRANSFER
  success = BUS_BlockWrite(addr, 4096, data, &j);
  if (j != (4096))
    success = BUS_GENERIC_ERROR;
#else

  int ntimes = 4096 / 64;
  for (i = 0; i < ntimes; i++)
    {
      for (j = 0; j < 64; j++)
	{			/* single word transfer */
	  mpdWrite32(&MPDp[id]->histo_memory[block][i * 64 + j],
		     data[i * 64 + j]);
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
  int block = 0;

  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  if (ch >= 8)
    block = 1;

  data = 0x80 | (ch & 0x07);

  MPDLOCK;
  mpdWrite32(&MPDp[id]->histo[block].csr, data);
  MPDUNLOCK;

  return OK;
}

int
mpdHISTO_Stop(int id, int ch)	/* ch == 0, 15 */
{
  uint32_t data;
  int block = 0;

  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  if (ch >= 8)
    block = 1;

  data = (ch & 0x07);

  MPDLOCK;
  mpdWrite32(&MPDp[id]->histo[block].csr, data);
  MPDUNLOCK;

  return OK;
}

int
mpdHISTO_GetIntegral(int id, int ch, uint32_t * integral)	/* ch == 0, 15 */
{
  int block = 0;

  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  if (ch >= 8)
    block = 1;

  MPDLOCK;
  *integral = mpdRead32(&MPDp[id]->histo[block].count);
  MPDUNLOCK;

  return OK;
}

int
mpdHISTO_Read(int id, int ch, uint32_t * histogram)	/* ch == 0, 15; uint32_t histogram[4096] */
{
  int success = OK, i, j;
  int block = 0;

  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  if (ch >= 8)
    block = 1;
  MPDLOCK;
#ifdef BLOCK_TRANSFER
  success = BUS_BlockRead(addr, 4096, histogram, &j);
  if (j != 4096)
    {
      MPD_ERR("Block Transfer returned %d 32bit words, 4096 expected\n", j);
      success = BUS_GENERIC_ERROR;
    }
#else
  int ntimes = 4096 / 64;
  for (i = 0; i < ntimes; i++)
    {
      for (j = 0; j < 64; j++)
	{			/* single word transfer */
	  histogram[i * 64 + j] =
	    mpdRead32(&MPDp[id]->histo_memory[block][i * 64 + j]);
	}
    }
#endif
  MPDUNLOCK;
  return success;
}


//======================================================
// Daq-Readout methods

int
mpdOBUF_GetFlags(int id, int *empty, int *full, int *nwords)
{

  uint32_t data;

  MPDLOCK;
  data = mpdRead32(&MPDp[id]->ob_status.output_buffer_flag_wc);
  MPDUNLOCK;

  if (data & 0x80000000)
    *full = 1;
  else
    *full = 0;
  if (data & 0x40000000)
    *empty = 1;
  else
    *empty = 0;

  *nwords = data & 0xFFFF;

  return OK;
}

int
mpdOBUF_Read(int id, volatile uint32_t * data, int size, int *wrec)
{

  uint32_t vmeAdrs;
  int retVal = 0;
  int dummy;
  volatile uint32_t *laddr;

  MPDLOCK;
  if ((unsigned long) (data) & 0x7)
    {
      dummy = 1;
      *data = 0;		/* Data word added for byte alignment */
      laddr = (data + 1);
    }
  else
    {
      dummy = 0;
      laddr = data;
    }

  switch (mpdGetFastReadout(id))
    {
    case 0:
      vmeDmaConfig(2, 2, 0);	// A32 BLT
      break;
    case 1:
      vmeDmaConfig(2, 3, 0);	// MBLT
      break;
    case 2:
      vmeDmaConfig(2, 4, 0);	// 2eVME
      break;
    case 3:
      vmeDmaConfig(2, 5, 0);
      break;
    case 4:
      vmeDmaConfig(2, 5, 1);	// 2esst266
      break;
    case 5:
      vmeDmaConfig(2, 5, 2);	// 2esst320
      break;
    default:
      vmeDmaConfig(2, 2, 0);	// A32 BLT
      break;
    }

  MPDLOCK;

  vmeAdrs = (uint32_t) mpdOutputBufferBaseAddr + mpdOutputBufferSpace * id;

  retVal = vmeDmaSend((unsigned long) laddr, vmeAdrs, (size << 2));

  if (retVal != 0)
    {
      MPD_ERR("ERROR in DMA transfer start (returned 0x%x)\n", retVal);
      MPD_ERR("  id=%d apv laddr = 0x%08x  vmeAdrs = 0x%08x  size = %d\n",
	      id, (uint32_t) laddr, vmeAdrs, size);
      *wrec = 0;
      MPDUNLOCK;
      return (retVal);
    }

  /* Wait until Done or Error */
  retVal = vmeDmaDone();
  MPDUNLOCK;
  if (retVal == 0)
    {
      *wrec = 0;
      MPD_ERR("vmeDmaDone returned zero word count\n");

      return ERROR;
    }
  else if (retVal == ERROR)
    {
      *wrec = 0;
      MPD_ERR("vmeDmaDone returned ERROR\n");

      return ERROR;
    }
  else
    {
      *wrec = (retVal >> 2) + dummy;
      MPD_DBG("vmeDmaDone returned 0x%x (%d)  wrec = %d\n",
	      retVal, retVal, *wrec);

#ifdef DEBUG_BLOCKREAD
      int iword = 0;
      for (iword = 0; iword < *wrec; iword++)
	{
	  if ((iword % 4) == 0)
	    printf("\n%4d:  ", iword);

	  printf("0x%08x   ", LSWAP(fApv[id][0].fBuffer[iword]));
	}
      printf("\n");
#endif
    }

  return OK;
}

int
mpdSDRAM_GetParam(int id, int *init, int *overrun, int *rdaddr, int *wraddr,
		  int *nwords)
{


  uint32_t data;

  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  MPDLOCK;
  *wraddr =
    (int) mpdRead32(&MPDp[id]->ob_status.sdram_fifo_wr_addr) & 0xffffff;
  data = mpdRead32(&MPDp[id]->ob_status.sdram_fifo_rd_addr);
  *rdaddr = (int) data & 0xffffff;
  *init = (int) (data >> 31) & 0x1;
  data = mpdRead32(&MPDp[id]->ob_status.sdram_flag_wc);
  *nwords = (int) data & 0xffffff;
  *overrun = (int) (data >> 31) & 0x1;
  MPDUNLOCK;

  return OK;

}

/**
 * Readout Fifo
 * Standard Event Mode (no zero suppression or pedestal subtraction)
 * Return 0 when something has been read, error otherwise
 */
int
mpdFIFO_ReadSingle(int id, int channel,	// apv channel (FIFO)
		   uint32_t * dbuf,	// data buffer
		   int *wrec,	// max number of words to get / return words received
		   int max_retry)	// max number of retry for timeout
//                         int &err)        // return error code
//                         int &n_events)   // number of events
{

  int success = OK, i, size;
  int nwords;			// words available in fifo
  int wmax;			// maximum word acceptable
  uint32_t vmeAdrs = 0;		// Vme address of channel data
  int retVal = 0;
  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  wmax = *wrec;
  *wrec = 0;			// returned words

  nwords = 0;

  i = 0;
  while ((nwords <= 0) && (i <= max_retry))
    {
      if (max_retry > 0)
	i++;
      success = mpdFIFO_GetNwords(id, fApv[id][channel].adc, &nwords);
      if (success != OK)
	return success;
    }

  //printf("\n\n%s: number of words to be read %d\n %d",__FUNCTION__,nwords,max_retry);
  size = (nwords < wmax) ? nwords : wmax;

  MPD_MSG("\n\nmpdFIFO_GetNwords:number of words to be read %d size: %d\n",
	  nwords, size);

  //MPD_DBG("fifo ch = %d, words in fifo= %d, retries= %d (max %d)\n",channel, nwords,i, max_retry);
  MPD_DBG("  id=%d  channel=%d  physMemBase = 0x%08x   dbuf = 0x%08x\n\n",
	  id, channel, (uint32_t) fApv[id][channel].physMemBase,
	  (uint32_t) & dbuf[0]);

  if (i > max_retry)
    {
      MPD_ERR(" max retry = %d, count=%d nword=%d\n", max_retry, i, nwords);
      return ERROR;
    }

  MPDLOCK;
#define BLOCK_TRANSFER1
#ifdef BLOCK_TRANSFER1
  /*   unsigned long offset = ((unsigned long)&dbuf - (unsigned long)&fApv[id][0].fBuffer); */
  unsigned long offset = 0;

  vmeDmaConfig(1, 2, 0);	//A24 BLT32

  vmeAdrs =
    (uint32_t) & MPDp[id]->data_ch[fApv[id][channel].adc][0] - mpdA24Offset;

  MPD_DBG
    ("dbuf addr = 0x%lx  fBuffer = 0x%lx  offset = 0x%lx  vmeAdrs = 0x%08x\n",
     (unsigned long) dbuf, (unsigned long) fApv[id][channel].fBuffer,
     (unsigned long) offset, vmeAdrs);

  retVal =
    vmeDmaSendPhys(fApv[id][channel].physMemBase, vmeAdrs, (size << 2));

  if (retVal != 0)
    {
      MPD_ERR("ERROR in DMA transfer Initialization (returned 0x%x)\n",
	      retVal);
      MPD_ERR("  id=%d  channel=%d  physMemBase = 0x%08x\n", id, channel,
	      (uint32_t) fApv[id][channel].physMemBase);
      *wrec = 0;
      MPDUNLOCK;
      return (retVal);
    }

  /* Wait until Done or Error */
  retVal = vmeDmaDone();

  if (retVal == 0)
    {
      *wrec = 0;
      MPD_ERR("vmeDmaDone returned zero word count retVal=0x%x (%d)\n",
	      retVal, retVal);
      MPDUNLOCK;
      return ERROR;
    }
  else if (retVal == ERROR)
    {
      *wrec = 0;
      MPD_ERR("vmeDmaDone returned ERROR retVal=0x%x (%d)\n", retVal, retVal);
      MPDUNLOCK;
      return ERROR;
    }
  else
    {
      *wrec = (retVal >> 2);
      MPD_DBG("vmeDmaDone returned 0x%x (%d)  wrec = %d\n",
	      retVal, retVal, *wrec);
      int iword = 0;
      for (iword = 0; iword < *wrec; iword++)
	{
#define DEBUG_BLOCKREAD
#ifdef DEBUG_BLOCKREAD
	  if ((iword % 4) == 0)
	    printf("\n%4d:  ", iword);

	  printf("0x%08x   ", LSWAP(fApv[id][channel].fBuffer[iword]));
#endif
	  /* Byte swap necessary for block transfers */
	  fApv[id][channel].fBuffer[iword] =
	    LSWAP(fApv[id][channel].fBuffer[iword]);
	}
#ifdef DEBUG_BLOCKREAD
      printf("\n");
#endif
    }

#else
  for (i = 0; i < size; i++)
    {
      dbuf[i] = mpdRead32(&MPDp[id]->data_ch[channel][i]);	//changed from [channel][i*4] to [channel][i]--Danning_Sep_30_2015
      *wrec += 1;
    }
  //MPD_DBG("Read apv = %d, wrec = %d, success = %d\n", channel, *wrec, success);
#endif
  MPDUNLOCK;

  if (*wrec != size)
    {
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
mpdFIFO_ReadSingle0(int id, int channel, int blen, uint32_t * event,
		    int *nread)
{

  int rval, nwords, size;
  int n_part;
  int i = 0;

  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  *nread = 0;
  nwords = 0;
  n_part = 0;

  rval = mpdFIFO_GetNwords(id, channel, &nwords);
  if (rval != OK)
    return 2;

  size = (nwords > blen) ? blen : nwords;	// cannot exceed memory buffer size

  if (size > 0)
    {

      MPDLOCK;
#ifdef BLOCK_TRANSFER
      rval = BUS_BlockRead(fifo_addr, size, event, &n_part);
#else
      for (i = 0; i < size; i++)
	{
	  event[i] = mpdRead32(&MPDp[id]->data_ch[channel][i * 4]);
	  n_part++;
	}
#endif

#ifdef DEBUG_DUMP
      printf("\nReadData: %x %d\n", fifo_addr, channel);
      for (i = 0; i < n_part; i++)
	{
	  if ((i % 16) == 0)
	    printf("\n%4d", i);
	  printf(" %6x", event[i]);
	}

      printf(" -> %d\n", n_part);
#endif
      *nread += n_part;

      if (n_part != size)
	return 1;

    }

  MPDUNLOCK;
  return *nread;

}


int
mpdFIFO_Samples(int id,
		int channel,
		uint32_t * event, int *nread, int max_samples, int *err)
{
  int success = OK, nwords, i = 0;

  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  *err = 1;
  *nread = 0;

  nwords = DAQ_FIFO_SIZE;
  if (nwords > max_samples)
    {
      *err = 2;
      return ERROR;
    }

  MPDLOCK;
#ifdef BLOCK_TRANSFER
  success = BUS_BlockRead(fifo_addr, nwords, event, nread);
#else
  printf("addr: %08x \n",
	 (&MPDp[id]->data_ch[0][0] - &MPDp[id]->histo_memory[0][0]));

  for (i = 0; i < nwords; i++)
    {
      event[i] = mpdRead32(&MPDp[id]->data_ch[channel][i]);
    }
  *nread = nwords * 4;
#endif
  *nread /= 4;
  if (*nread == nwords)
    *err = 0;

  MPDUNLOCK;

  return success;
}

int
mpdFIFO_IsSynced(int id, int channel, int *synced)
{
  uint32_t data;
  uint32_t channel_mask, synced_mask;
  int success = OK;

  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  channel_mask = 1 << (channel & 0x0F);
  synced_mask = channel_mask << 16;	/* @ error_addr */

  MPDLOCK;
  data = mpdRead32(&MPDp[id]->ch_flags.sync_status);
  MPDUNLOCK;

  if (data & synced_mask)
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
  int success = OK;

  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  MPDLOCK;
  data = mpdRead32(&MPDp[id]->ch_flags.sync_status);
  MPDUNLOCK;

  *synced = (data >> 16);

  return success;
}


int
mpdFIFO_HasError(int id, int channel, int *error)
{
  uint32_t data;
  uint32_t channel_mask, error_mask;
  int success = OK;

  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  channel_mask = 1 << (channel & 0x0F);
  error_mask = channel_mask;	/* @ error_addr */

  MPDLOCK;
  data = mpdRead32(&MPDp[id]->ch_flags.sync_status);
  MPDUNLOCK;

  if (data & error_mask)
    *error = 1;
  else
    *error = 0;

  return success;
}

int
mpdFIFO_GetAllFlags(int id, uint16_t * full, uint16_t * empty)
{
  uint32_t data;
  int success = OK;

  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  MPDLOCK;
  data = mpdRead32(&MPDp[id]->ch_flags.fifo_status);
  MPDUNLOCK;

  *full = data >> 16;
  *empty = data & 0xFFFF;

  return success;
}

int
mpdFIFO_IsFull(int id, int channel, int *full)
{
  uint32_t data;
  uint32_t channel_mask, full_mask;
  int success = OK;

  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  channel_mask = 1 << (channel & 0x0F);
  full_mask = channel_mask << 16;	/* @ flag_addr */

  MPDLOCK;
  data = mpdRead32(&MPDp[id]->ch_flags.fifo_status);
  MPDUNLOCK;

  if (data & full_mask)
    *full = 1;
  else
    *full = 0;

  return success;
}

int
mpdFIFO_GetNwords(int id, int channel, int *nwords)
{
  uint32_t data;
  int success = OK;

  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  MPDLOCK;
  *nwords = data =
    mpdRead32(&MPDp[id]->ch_flags.used_word_ch_pair[channel]) & 0xffff;
  MPDUNLOCK;

  //printf("EC: nwords from fifo %d 0x%x\n",channel,data);

  return success;
}

int
mpdFIFO_IsEmpty(int id, int channel, int *empty)
{
  uint32_t data;
  uint32_t channel_mask, empty_mask;
  int success = OK;

  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  channel_mask = 1 << (channel & 0x0F);
  empty_mask = channel_mask;	/* @ flag_addr */

  MPDLOCK;
  data = mpdRead32(&MPDp[id]->ch_flags.fifo_status);
  MPDUNLOCK;

  if (data & empty_mask)
    *empty = 1;
  else
    *empty = 0;

  return success;
}

int
mpdFIFO_ClearAll(int id)
{
  uint32_t data, oldval;
  int success = OK;

  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  // Issue a pulse on READOUT_CONFIG[31] clear fifos, sdram, output buffer

  MPDLOCK;
  oldval = mpdRead32(&MPDp[id]->readout_config);
  data = oldval | 0x80000000;

  mpdWrite32(&MPDp[id]->readout_config, data);
  usleep(100);			// seems to be important

  data = oldval & 0x7FFFFFFF;

  mpdWrite32(&MPDp[id]->readout_config, data);
  MPDUNLOCK;

  usleep(100);

  return success;
}


int
mpdFIFO_WaitNotEmpty(int id, int channel, int max_retry)
{
  uint32_t data;
  uint32_t channel_mask, empty_mask;
  int success = OK, retry_count, fifo_empty;

  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  channel_mask = 1 << (channel & 0x0F);
  empty_mask = channel_mask;	/* @ flag_addr */

  retry_count = 0;
  fifo_empty = 1;

  while (fifo_empty && success == OK && retry_count <= max_retry)
    {
      MPDLOCK;
      data = mpdRead32(&MPDp[id]->ch_flags.fifo_status);
      MPDUNLOCK;

      if (max_retry > 0)
	retry_count++;
      if (data & empty_mask)
	fifo_empty = 1;
      else
	fifo_empty = 0;
      if (retry_count > max_retry)
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
mpdFIFO_ReadAll(int id, int *timeout, int *global_fifo_error)
{

  unsigned int k;
  int sample_left;

  int nread, err = OK;


  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  sample_left = 0;
  *global_fifo_error = 0;

  if (fMpd[id].fReadDone == 0)
    {				// at least one MPD FIFO needs to be read
      for (k = 0; k < fMpd[id].nAPV; k++)
	{			// loop on ADC channels on single board

	  if (fApv[id][k].enabled == 0)
	    {
	      continue;
	    }

	  if (mpdApvReadDone(id, k) == 0)
	    {			// APV FIFO has data to be read

	      nread = mpdApvGetBufferAvailable(id, k);
	      // printf(" EC: card %d %d buffer size available = %d\n",id, k,nread);
	      if (nread > 0)
		{		// space in memory buffer
		  err = mpdFIFO_ReadSingle(id, k /*fApv[id][k].adc */ , mpdApvGetBufferPWrite(id, k), &nread, 20);	//not this

		  mpdApvIncBufferPointer(id, k, nread);

		  *global_fifo_error |= err;	// ???
		  //printf(" EC: card %d readsingle done nread=%d, err=%d\n",k,nread,err);
		}
	      else
		{		// no space in memory buffer
		  MPD_ERR
		    ("MPD/APV(i2c)/(adc) = %d/%d, no space in memory buffer adc=%d\n",
		     id, k, fApv[id][k].adc);
		}

	      if ((err == ERROR) || (nread == 0))
		*timeout = *timeout + 1;	// timeout

	      int n = mpdApvGetBufferSample(id, k);

	      MPD_DBG
		("MPD: %d APV_idx= %d, ADC_FIFO= %d, word read= %d, event/sample read= %d, error=%d\n",
		 id, k, fApv[id][k].adc, nread, n, *global_fifo_error);

	      sample_left += mpdApvGetSampleLeft(id, k);

	    }

	  //MPD_DBG("Fifo= %d, total sample left= %d (<0 means more samples than requested)\n",k, sample_left);

	}			// loop on ADC
      fMpd[id].fReadDone = (sample_left > 0) ? 0 : 1;
    }				// if fReadDone
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
mpdSearchEndMarker(uint32_t * b, int i0, int i1)
{

  int i;

  if (i0 >= i1)
    {
      return -1;
    }
  for (i = i0; i < i1; i++)
    {
      if ((b[i] & 0x180000) == 0x180000)
	{
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
mpdApvShiftDataBuffer(int id, int k, int i0)
{

  uint32_t *b;

  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return;
    }

  b = mpdApvGetBufferPointer(id, k, 0);
  int delta = fApv[id][k].fBi1 - i0;

  //MPD_DBG("Move block of %d words from %d to 0\n",delta,i0);

  if (delta > 0)
    {
      memmove(&b[0], &b[i0], sizeof(uint32_t) * delta);	// areas may overlap
    }

  fApv[id][k].fBi0 = 0;		// to be removed
  fApv[id][k].fBi1 = delta;

  //MPD_DBG("Fifo= %d cleaned (data shifted) write pointer at=%d\n",fApv[id][k].adc,fApv[id][k].fBi1);

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

  int ii1 = 0;

  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  n = 1;			// single event mode

  sample_left = 0;

  if (fMpd[id].fReadDone == 0)
    {				// MPD fifos need to be read
      for (k = 0; k < fMpd[id].nAPV; k++)
	{			// loop on ADC channels on single board

	  if (fApv[id][k].enabled == 0)
	    {
	      continune;
	    }
	  if (mpdApvReadDone(id, k) == 0)
	    {			// APV FIFO has data to be read

	      uint32_t *bptr = mpdApvGetBufferPointer(id, k, ii1);

	      int bsiz = fApv[k].fBufSize - ii1;	// space left in buffer

	      err =
		mpdFIFO_ReadSingle(id, fApv[id][k].adc, bsiz, bptr, nread);

	      // ***
	      if (nread > 0)
		{
		  for (int i = 0; i < nread; i++)
		    {
		      if (mpdIsEndBlock(bptr[i]))
			{
			  mpdApvDecSampleLeft(id, k, 1);
			  if (mpdApvGetSampleLeft(id, k) == 0)
			    {
			      fApv[id][k].fBs = ii1 + i;	// pointer to the last sample word
			    }
			}
		    }

		  fApv[id][k].fBi1 = ii1 + nread;

		}
	      else
		{
		  *timeout++;
		}

	      if (err != 2)
		*global_fifo_error |= err;

	      if (err == 2)
		*timeout++;

	      if (n > 1)
		multiple_events++;	// not used

	    }

	  sample_left += mpdApvGetSampleLeft(id, k);

	}			// loop on ADC FIFOs
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
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
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
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  data = 0;

  MPDLOCK;
  mpdWrite32(&MPDp[id]->readout_config, data);
  MPDUNLOCK;


  return mpdTRIG_Disable(id);
}

int
mpdDAQ_Config(int id)
{

  uint32_t data;
  int i;
  short evtbld, UseSdram, FastReadout;

  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  evtbld = mpdGetEventBuilding(id) ? 1 : 0;
  UseSdram = mpdGetUseSdram(id) ? 1 : 0;
  FastReadout = mpdGetFastReadout(id) ? 1 : 0;
  if (evtbld == 0)
    {
      UseSdram = 0;
    }
  if (evtbld == 0 || (evtbld == 1 && UseSdram == 0))
    FastReadout = 0;

  if (evtbld)
    {
      data = mpdGetTriggerNumber(id) * mpdApvGetPeakMode(id);

      mpdWrite32(&MPDp[id]->sample_per_event, data);

      MPD_MSG("Sample_per_event = 0x%x\n", data);
      data = mpdGetEventPerBlock(id) & 0xff;
      if (data == 0)
	data = 1;

      mpdWrite32(&MPDp[id]->event_per_block, data);

      MPD_MSG("Event_per_block = 0x%x\n", data);
    }

  data = (mpdGetAcqMode(id) & 0x07) |	// ((test & 0x01) << 15) |
    ((mpdGetFIRenable(id) & 0x1) << 4) |	// FIR enable=1
    (FastReadout << 14) |
    (UseSdram << 15) |
    ((mpdGetCommonOffset(id) & 0xfff) << 16) |
    ((mpdGetCommonNoiseSubtraction(id) & 0x1) << 28) | ((evtbld & 0x1) << 30);

  mpdWrite32(&MPDp[id]->readout_config, data);

  MPD_MSG("ReadoutConfig = 0x%x\n", data);

  data = ((mpdGetInputLevel(id, 0) & 0x1) << 0) |	// NIM=1/TTL=0 Level LEMO IN0
    ((mpdGetInputLevel(id, 1) & 0x1) << 1) |	// NIM/TTL Level LEMO IN1
    ((mpdGetOutputLevel(id, 0) & 0x1) << 2) |	// NIM/TTL Level LEMO OUT0
    ((mpdGetOutputLevel(id, 1) & 0x1) << 3);	// NIM/TTL Level LEMO OUT1

  mpdWrite32(&MPDp[id]->io_config, data);

  MPD_MSG("IOConfig = 0x%x\n", data);

  if (UseSdram)
    {
      data = (mpdOutputBufferBaseAddr + mpdOutputBufferSpace * id) >> 2;
    }
  else
    {
      data = 0;
    }

  mpdWrite32(&MPDp[id]->obuf_base_addr, data);
  MPD_MSG("Output buffer base address = 0x%x (0=no SDRAM)\n", data);

  data = mpdSdramBaseAddr;
  mpdWrite32(&MPDp[id]->sdram_base_addr, data);
  MPD_MSG("Sdram base address = 0x%x (test only)\n", data);

  for (i = 0; i < fMpd[id].nAPV; i++)
    {
      fApv[id][i].fBi0 = 0;
      fApv[id][i].fBi1 = 0;
    }

  MPDUNLOCK;

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
  int success = OK, i;

  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  MPDLOCK;
  for (i = 0; i < 128; i++)
    {
      mpdWrite32(&MPDp[id]->ped[ch][i], ped_even[i]);
      mpdWrite32(&MPDp[id]->ped[ch + 1][i], ped_odd[i]);
    }
  MPDUNLOCK;

  return success;
}

int
mpdPED_Write(int id, int ch, int v)	// ch = 0..7
{
  int i, data[128];

  if (id == 0)
    id = mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  for (i = 0; i < 128; i++)
    data[i] = v;

  return mpdPED_Write0(id, ch, data, data);
}



int
mpdPED_Read(int id, int ch, int *ped_even, int *ped_odd)	// TBD
{
  int i;

  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  for (i = 0; i < 128; i++)
    {
      ped_even[i] = mpdRead32(&MPDp[id]->ped[ch][i]);
      ped_odd[i] = mpdRead32(&MPDp[id]->ped[ch + 1][i]);
    }

  return OK;
}


int
mpdTHR_Write0(int id, int ch, int *thr_even, int *thr_odd)	// ch = 0..7
{
  int success = OK, i;

  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  MPDLOCK;
  for (i = 0; i < 128; i++)
    {
      mpdWrite32(&MPDp[id]->thres[ch][i], thr_even[i]);
      mpdWrite32(&MPDp[id]->thres[ch + 1][i], thr_odd[i]);
    }
  MPDUNLOCK;

  return success;
}

int
mpdTHR_Write(int id, int ch, int v)	// ch = 0..7
{
  int i, data[128];

  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  for (i = 0; i < 128; i++)
    data[i] = v;

  return mpdTHR_Write0(id, ch, data, data);
}

int
mpdTHR_Read(int id, int ch, int *thr_even, int *thr_odd)	// TBD
{
  int i;
  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  for (i = 0; i < 128; i++)
    {
      thr_even[i] = mpdRead32(&MPDp[id]->thres[ch][i]);
      thr_odd[i] = mpdRead32(&MPDp[id]->thres[ch + 1][i]);
    }

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
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  for (ia = 0; ia < 8; ia++)
    {				// loop on apv,  odd and even apv are download simultaneously
      mpdPED_Write0(id, ia, mpdGetApvPed(id, 2 * ia),
		    mpdGetApvPed(id, 2 * ia + 1));
      mpdTHR_Write0(id, ia, mpdGetApvThr(id, 2 * ia),
		    mpdGetApvThr(id, 2 * ia + 1));
    }

  return 0;

}

void
mpdSetPedThrPath(int id, char *val)
{
  fMpd[id].fPedThrPath = val;
};

char *
mpdGetPedThrPath(int id)
{
  return fMpd[id].fPedThrPath;
};

int
mpdSetPedThr(int id, int ch, int p, int t)
{
  if (ch >= 0 && ch < 2048)
    {
      fMpd[id].fPed[ch] = p;
      fMpd[id].fThr[ch] = t;
      return 1;
    }
  return 0;
};

int
mpdGetPed(int id, int ch)
{
  if (ch >= 0 && ch < 2048)
    {
      return fMpd[id].fPed[ch];
    }
  else
    {
      return fMpd[id].fPedCommon;
    }
};

int
mpdGetThr(int id, int ch)
{
  if (ch >= 0 && ch < 2048)
    {
      return fMpd[id].fThr[ch];
    }
  else
    {
      return fMpd[id].fThrCommon;
    }
};


int *
mpdGetApvPed(int id, int ach)
{
  if (ach >= 0 && ach < 16)
    {
      return &(fMpd[id].fPed[ach * 128]);
    }
  else
    {
      return 0;
    }
};

int *
mpdGetApvThr(int id, int ach)
{
  if (ach >= 0 && ach < 16)
    {
      return &(fMpd[id].fThr[ach * 128]);
    }
  else
    {
      return 0;
    }
};

void
mpdSetPedThrCommon(int id, int p, int t)
{
  fMpd[id].fPedCommon = p;
  fMpd[id].fThrCommon = t;
};


int
mpdGetPedCommon(int id)
{
  return fMpd[id].fPedCommon;
};

int
mpdGetThrCommon(int id)
{
  return fMpd[id].fThrCommon;
};


#ifdef NOTDONE
/**
 * Read Pedestals and Thresholds of a single MPD from the file pname
 * if pname is empty, use the stored PedThrPath value
 */
int
mpdReadPedThr(int id, std::string pname)
{
  //  if(id==0) id=mpdID[0];
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  string line;

  int ch, ped, thr;
  int count, i;

  if (pname.empty())
    {
      pname = mpdGetPedThrPath(id);
    };

  for (i = 0; i < 2048; i++)
    {
      mpdSetPedThr(id, i, mpdGetPedCommon(id), mpdGetThrCommon(id));
    };

  std::ifstream infile(pname.data(), std::ifstream::in);
  if (infile.is_open())
    {
      count = 0;
      while (infile.good())
	{
	  getline(infile, line);
	  if (line.find("#") != 0)
	    {			// no comment line
	      std::istringstream iss(line);
	      iss >> ch >> ped >> thr;
	      count += mpdSetPedThr(id, ch, ped, thr);
	    }
	}
      cout << __FUNCTION__ << ": " << count << " channels read from file " <<
	pname << " and set" << endl;
      infile.close();
    }
  else
    {
      cout << __FUNCTION__ << ": Warning, unable to open file " << pname <<
	endl;
      cout << __FUNCTION__ <<
	": Warning, set pedestal and threshold to common values" << endl;
    }

  return count;

}
#endif /* NOTDONE */

int
mpdGetEBWordCount(int id)
{
  int rval = 0;
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  MPDLOCK;
  rval = mpdRead32(&MPDp[id]->ob_status.evb_fifo_word_count);
  MPDUNLOCK;

  return rval;
}

int
mpdGetEventCount(int id)
{
  int rval = 0;
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  MPDLOCK;
  rval = mpdRead32(&MPDp[id]->ob_status.event_count);
  MPDUNLOCK;

  return rval;
}

int
mpdGetBlockCount(int id)
{
  int rval = 0;
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  MPDLOCK;
  rval = mpdRead32(&MPDp[id]->ob_status.block_count);
  MPDUNLOCK;

  return rval & 0xff;
}

int
mpdOBUF_GetBlockCount(int id)
{
  int rval = 0;
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  MPDLOCK;
  rval = mpdRead32(&MPDp[id]->ob_status.block_count);
  MPDUNLOCK;

  return (rval >> 16) & 0xFF;
}

int
mpdGetTriggerCount(int id)
{
  int rval = 0;
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  MPDLOCK;
  rval = mpdRead32(&MPDp[id]->ob_status.trigger_count);
  MPDUNLOCK;

  return rval;
}

int
mpdGetTriggerReceivedCount(int id)
{
  int rval = 0;
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  MPDLOCK;
  rval = mpdRead32(&MPDp[id]->ob_status.incoming_trigger);
  MPDUNLOCK;

  return rval;
}

int
mpdSetFiberTestMode(int id, int enable, int period)
{
  unsigned int data = 0;
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  if ((period < 0) || (period > 0xff))
    {
      MPD_ERR("Invalid period (%d)", period);
      return ERROR;
    }


  if (enable)
    {
      data |= (1 << 8);

      if (period)
	data |= (period << 16);
    }


  MPDLOCK;
  mpdWrite32(&MPDp[id]->io_config, data);
  MPDUNLOCK;

  return OK;
}

int
mpdSetSamplesPerEvent(int id, int samples)
{
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  if ((samples < 0) || (samples > 31))
    {
      MPD_ERR("Invalid samples (%d)", samples);
      return ERROR;
    }

  MPDLOCK;
  mpdWrite32(&MPDp[id]->sample_per_event, samples);
  MPDUNLOCK;

  return OK;
}

int
mpdSetBlocklevel(int id, int blocklevel)
{
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  if ((blocklevel < 0) || (blocklevel > 255))
    {
      MPD_ERR("Invalid blocklevel (%d)", blocklevel);
      return ERROR;
    }

  MPDLOCK;
  mpdWrite32(&MPDp[id]->event_per_block, blocklevel);
  MPDUNLOCK;

  return OK;
}

int
mpdSetBusyThreshold(int id, int thres)
{
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  if ((thres < 0) || (thres > 0xffff))
    {
      MPD_ERR("Invalid thres (%d)", thres);
      return ERROR;
    }

  MPDLOCK;
  mpdWrite32(&MPDp[id]->busy_thr, thres);
  MPDUNLOCK;

  return OK;
}

int
mpdSetLocalBusyThreshold(int id, int thres)
{
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  if ((thres < 0) || (thres > 0xffff))
    {
      MPD_ERR("Invalid thres (%d)", thres);
      return ERROR;
    }

  MPDLOCK;
  mpdWrite32(&MPDp[id]->busy_thr_local, thres);
  MPDUNLOCK;

  return OK;
}

int
mpdSetTriggerDelay(int id, int delay)
{
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  if ((delay < 0) || (delay > 0xffff))
    {
      MPD_ERR("Invalid delay (%d)", delay);
      return ERROR;
    }

  MPDLOCK;
  mpdWrite32(&MPDp[id]->trigger_delay, delay);
  MPDUNLOCK;

  return OK;
}

int
mpdFiberStatus(int id)
{
  uint32_t fiber_status_ctrl = 0;
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  MPDLOCK;
  fiber_status_ctrl = mpdRead32(&MPDp[id]->fiber_status_ctrl);
  MPDUNLOCK;

  printf("Fiber Status for SSP in slot %d\n", id);
  printf
    ("--------------------------------------------------------------------------------\n");
  printf("\n");
  printf(" Register = 0x%08x\n", fiber_status_ctrl);
  printf("\n");

  printf("  Fiber %s\n",
	 (fiber_status_ctrl & MPD_FIBER_DISABLED) ? "Disabled" : "Enabled");

  printf("  SFP transmit %s\n",
	 (fiber_status_ctrl & MPD_SFP_TRANSMIT_DISABLED) ? "Disabled" :
	 "Enabled");

  printf("\n");


  printf("  SFP %s\n",
	 (fiber_status_ctrl & MPD_SFP_PRESENT) ? "Present" : "NOT Present");

  if (fiber_status_ctrl & MPD_SFP_LOS)
    printf("  SFP -Loss of Signal- DETECTED\n");
  if (fiber_status_ctrl & MPD_FIBER_FRAME_ERROR)
    printf("  Frame Error DETECTED\n");
  if (fiber_status_ctrl & MPD_FIBER_HARD_ERROR)
    printf("  Hard  Error DETECTED\n");

  printf("  Fiber Channel %s\n",
	 (fiber_status_ctrl & MPD_FIBER_CHANNEL_UP) ? "UP" : "DOWN");

  printf("  Error Count  = %d\n",
	 (fiber_status_ctrl & MPD_FIBER_ERROR_COUNT_MASK) >> 4);

  printf("\n");
  printf
    ("--------------------------------------------------------------------------------\n");
  printf("\n\n");

  return OK;
}

int
mpdFiberEnable(int id)
{
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  if (mpdSSPMode)
    {
      MPD_ERR("Cannot Enable/Disable Fiber in SSP Mode\n");
      return ERROR;
    }

  MPDLOCK;
  mpdWrite32(&MPDp[id]->fiber_status_ctrl, 1);
  MPDUNLOCK;

  return OK;
}

int
mpdFiberDisable(int id)
{
  if (CHECKMPD(id))
    {
      MPD_ERR("MPD in slot %d is not initialized.\n", id);
      return ERROR;
    }

  if (mpdSSPMode)
    {
      MPD_ERR("Cannot Enable/Disable Fiber in SSP Mode\n");
      return ERROR;
    }

  MPDLOCK;
  mpdWrite32(&MPDp[id]->fiber_status_ctrl, 0);
  MPDUNLOCK;

  return OK;
}

/* ASMI Commands */
void
mpdASMI_reset(int id)
{
  uint32_t x;

  x = 7;

  MPDLOCK;
  mpdWrite32(&MPDp[id]->serial_memory_if[1], x);

  usleep(1);

  x = 0x80000000;
  while (x & 0x80000000)
    {
      x = mpdRead32(&MPDp[id]->serial_memory_if[1]);
    }
  MPDUNLOCK;
}

int
mpdASMI_rdid(int id)
{
  uint32_t x, iy;

  x = 1;

  MPDLOCK;
  mpdWrite32(&MPDp[id]->serial_memory_if[1], x);

  usleep(1);

  x = 0x80000000;
  while (x & 0x80000000)
    {
      x = mpdRead32(&MPDp[id]->serial_memory_if[1]);
    }
  iy = ((x & 0x0000FF00) >> 8);

  MPDUNLOCK;
  return iy;
}

int
mpdASMI_rdstatus(int id)
{
  uint32_t x, status;

  x = 5;

  MPDLOCK;
  mpdWrite32(&MPDp[id]->serial_memory_if[1], x);

  usleep(1);

  x = 0x80000000;
  while (x & 0x80000000)
    {
      x = mpdRead32(&MPDp[id]->serial_memory_if[1]);
    }
  status = ((x & 0x00FF0000) >> 16);
  MPDUNLOCK;
  return status;
}

void
mpdASMI_sec_erase(int id, uint32_t addr)	/* addr must be in the sector to erase */
{
  uint32_t x;

  x = (addr & 0x00FFFFFF) << 8;
//printf("asmi_sec_erase(): x = 0x%08x\n", x);

  MPDLOCK;
  mpdWrite32(&MPDp[id]->serial_memory_if[0], x);
  x = 4;
  mpdWrite32(&MPDp[id]->serial_memory_if[1], x);

  usleep(1);

  x = 0x80000000;
  while (x & 0x80000000)
    {
      x = mpdRead32(&MPDp[id]->serial_memory_if[1]);
    }

  MPDUNLOCK;
}

int
mpdASMI_rd(int id, uint32_t addr)
{
  uint32_t x;

  x = (addr & 0x00FFFFFF) << 8;
//printf("asmi_rd(): x = 0x%08x\n", x);
  MPDLOCK;
  mpdWrite32(&MPDp[id]->serial_memory_if[0], x);
  x = 2;
  mpdWrite32(&MPDp[id]->serial_memory_if[1], x);
  mpdRead32(&MPDp[id]->serial_memory_if[1]);	// DUMMY read to slow down a little bit

  x = 0x80000000;
  while (x & 0x80000000)
    {
      x = mpdRead32(&MPDp[id]->serial_memory_if[1]);
    }

  MPDUNLOCK;
  return (x & 0xFF);
}


void
mpdASMI_wr(int id, uint32_t addr, uint32_t data)
{
  uint32_t x;

  x = ((addr & 0x00FFFFFF) << 8) | (data & 0xFF);
//printf("asmi_wr(): x = 0x%08x\n", x);
  MPDLOCK;
  mpdWrite32(&MPDp[id]->serial_memory_if[0], x);
  x = 3;
  mpdWrite32(&MPDp[id]->serial_memory_if[1], x);
  mpdRead32(&MPDp[id]->serial_memory_if[1]);	// DUMMY read to slow down a little bit

  x = 0x80000000;
  while (x & 0x80000000)
    {
      x = mpdRead32(&MPDp[id]->serial_memory_if[1]);
    }

  MPDUNLOCK;
}

/* Remote update functions */
int
mpdRUPD_reconfigure(int id, int pgm)
{
  uint32_t x;
  int par0, par5;

  mpdRUPD_setup(id, pgm);

  x = 0x80;
  MPDLOCK;
  mpdWrite32(&MPDp[id]->remote_update[1], x);

  x = 0x80000000;
  while (x & 0x80000000)
    {
      usleep(1000);
      x = mpdRead32(&MPDp[id]->remote_update[1]);
    }
  MPDUNLOCK;

  par0 = mpdRUPD_rd_param(id, 0);
  par5 = mpdRUPD_rd_param(id, 5);
  printf("par0 = %d, par5 = %d\n", par0, par5);
  return par5;
}

void
mpdRUPD_setup(int id, int pgm)
{
  mpdRUPD_wr_param(id, 3, 0);	// Watchdog disable
  usleep(20000);
  mpdRUPD_wr_param(id, 5, pgm ? 1 : 0);	// Application(1) - Factory(0)
  usleep(20000);
  mpdRUPD_wr_param(id, 4, pgm);	// Image page address
  usleep(20000);
}

void
mpdRUPD_wr_param(int id, int par, int val)
{
  uint32_t x;

  x = (val & 0x3FF) | (par & 0x7) << 16;
  MPDLOCK;
  mpdWrite32(&MPDp[id]->remote_update[0], x);

  x = 2;
  usleep(5000);

  mpdWrite32(&MPDp[id]->remote_update[1], x);

  x = 0x80000000;
  while (x & 0x80000000)
    {
      usleep(1000);
      MPDLOCK;
      x = mpdRead32(&MPDp[id]->remote_update[1]);
    }

  MPDUNLOCK;
}

int
mpdRUPD_rd_param(int id, int par)
{
  volatile uint32_t x;

  x = (par & 0x7) << 16;
  MPDLOCK;
  mpdWrite32(&MPDp[id]->remote_update[0], x);

  x = 1;
  usleep(5000);

  mpdWrite32(&MPDp[id]->remote_update[1], x);

  x = 0x80000000;
  printf("x = 0x%x\n", x);
  while (x & 0x80000000)
    {
      usleep(100000);

      x = mpdRead32(&MPDp[id]->magic_value);
      x = mpdRead32(&MPDp[id]->remote_update[1]);

      printf("x = 0x%x\n", x);
    }
  printf("x = 0x%x\n", x);

  MPDUNLOCK;
  return x & 0x3FF;
}

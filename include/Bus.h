#ifndef __GI_BUS__
#define __GI_BUS__


/*
 * Interface to the bus control functions 
 * Pure virtual class, specific class must be implemented for each bus that
 * manages the MPDs
 * Bus.h
 */


#include <stdint.h>

#define BUS_OK			 0
#define	BUS_ERROR		-1
#define	BUS_COMM_ERROR		-2
#define	BUS_GENERIC_ERROR	-3
#define	BUS_INVALID_PARAM	-4
#define	BUS_TIMEOUT		-5
#define BUS_MISMATCH            -6
#define BUS_TBC                -10

enum giDT {
  giD8  = 0x01,                   /*  8 bit                                       */
  giD16 = 0x02,                   /* 16 bit                                       */
  giD32 = 0x04,                   /* 32 bit                                       */
  giD64 = 0x08,                   /* 64 bit                                       */
  giD16s = 0x12,           /* 16 bit swapped                               */
  giD32s = 0x14,           /* 32 bit swapped                               */
  giD64s = 0x18            /* 64 bit swapped                               */
};

enum giAM {
  A16S         = 0x2D,         /* A16 supervisory access                       */
  A16U         = 0x29,         /* A16 non-privileged                           */
  A16_LCK      = 0x2C,         /* A16 lock command                             */

  A24S_BLT     = 0x3F,         /* A24 supervisory block transfer               */
  A24S_PGM     = 0x3E,         /* A24 supervisory program access               */
  A24S_DATA    = 0x3D,         /* A24 supervisory data access                  */
  A24S_MBLT    = 0x3C,         /* A24 supervisory 64-bit block trnsfer         */
  A24U_BLT     = 0x3B,         /* A24 non-privileged block transfer            */
  A24U_PGM     = 0x3A,         /* A24 non-privileged program access            */
  A24U_DATA    = 0x39,         /* A24 non-privileged data access               */
  A24U_MBLT    = 0x38,         /* A24 non-privileged 64-bit block trnsfer      */
  A24_LCK      = 0x32,         /* A24 lock command                             */

  A32S_BLT     = 0x0F,         /* A32 supervisory block transfer               */
  A32S_PGM     = 0x0E,         /* A32 supervisory program access               */
  A32S_DATA    = 0x0D,         /* A32 supervisory data access                  */
  A32S_MBLT    = 0x0C,         /* A32 supervisory 64-bit block trnsfer         */
  A32U_BLT     = 0x0B,         /* A32 non-privileged block transfer            */
  A32U_PGM     = 0x0A,         /* A32 non-privileged program access            */
  A32U_DATA    = 0x09,         /* A32 non-privileged data access               */
  A32U_MBLT    = 0x08,         /* A32 non-privileged 64-bit block trnsfer      */
  A32_LCK      = 0x05,         /* A32 lock command                             */
  CR_CSR       = 0x2F,         /* CR/CSR space                                 */
};


class Bus {

 private:
  static int fBus_Idx; // number of allocated busses; int Bus::fBus_Idx=-1; must be in the main!
  int fLocalIdx;

 public:

  Bus() {
    fBus_Idx++;
    fLocalIdx = fBus_Idx;
  };

  ~Bus() { fBus_Idx--; };

  int getIndex() { return fLocalIdx; }; // return the index of the allocated bus

  virtual int Open(void) = 0;
  virtual int Close(void) = 0;
  virtual int Status(void) = 0;

  virtual int Write(uint32_t Address, void *Data) = 0;
  virtual int Read(uint32_t Address, void *Data) = 0;

  //
  // AM is the standard AM modifier code
  // DT = 0x01 : 8 bit
  //    = 0x02 : 16 bit
  //    = 0x04 : 32 bit
  //    = 0x08 : 64 bit  
  virtual int Write(int AM, int DT, uint32_t Address, void *Data) = 0;
  virtual int Read(int AM, int DT, uint32_t Address, void *Data) = 0;
  
  virtual int ReadCR(uint32_t Address, void *Data) = 0;

  // size = number of 32bit words
  virtual int BlockWrite(uint32_t Address, int Size, void *Buffer, int *Transferred) = 0;
  virtual int BlockRead(uint32_t Address, int Size, void *Buffer, int *Transferred) = 0;

  //  virtual int FastBlockRead(uint32_t Address, int Size, void *Buffer, int *Transferred) = 0;

  /**
   * specific methods of the CAEN Vx718 device used as i/o register and scaler
   * not required to be defined in other drivers
   */
  virtual int SetOutConf(const int osel) { return (0); };
  virtual int SetOutReg(int osel) { return (0); };
  virtual int ClearOutReg(int osel) { return (0); };
  virtual int PulseOutReg(int osel) { return (0); };
  virtual int SetInConf(const int isel) { return (0); };
  virtual int ConfScaler(const int isel) { return (0); };
  virtual int ResetScaler() { return (0); };
  virtual int EnableScaler() { return (0); };
  virtual int DisableScaler() { return (0); };
  virtual int GetScalerCount() { return (0); };

// Specific Methods for Struck SIS3104 controller
  virtual int GetInLatch(int *x) { return (0); };
  virtual int ClearInLatch(int x) { return (0); };
  virtual int GetInTransparent(int *x) { return (0); };

};

#endif

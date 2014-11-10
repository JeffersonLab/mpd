
#ifndef __GI_INTEL__
#define __GI_INTEL__

#include "Bus.h"

#include <unistd.h>
#include <stdint.h>

//#include "jvme.h"

extern "C" {
#include "jvme.h"
}

class VmeXVB : public Bus
{
 private:
  long V17x8Handle;
  long * vmeBoard;
  int Vme_status;
  
  int VmeOpen(void);
  int VmeClose(void);
  //  int VmeCRScan(int slot, uint32_t *rev, uint32_t *timec);
  int VmeWrite(uint32_t Address, void *Data);
  int VmeRead(uint32_t Address, void *Data);
  
  int VmeWrite(int AM, int DT, uint32_t Address, void *Data);
  int VmeRead(int AM, int DT, uint32_t Address, void *Data);
  
  int VmeBlockWrite(uint32_t Address, int Size, void *Buffer, int *Transferred);
  int VmeBlockRead(uint32_t Address, int Size, void *Buffer, int *Transferred);

  int VmeReadCR(uint32_t vme_adr, void* vme_data); // EC
  
 public:

  int Open(void) { return VmeOpen(); }
  int Close(void) { return VmeClose(); }
  int Status(void) { return Vme_status; }
  // int Scan(int slot, uint32_t *rev) { return VmeCRScan(slot, rev); }
  //  int Scan(int slot, uint32_t *rev, uint32_t *timec=0) { return VmeCRScan(slot, rev, timec); }
  
  int Write(uint32_t Address, void *Data) { return VmeWrite(Address, Data); }
  int Read(uint32_t Address, void *Data) { return VmeRead(Address, Data); }
  
  int Write(int AM, int DT, uint32_t Address, void *Data) { return VmeWrite(AM, DT, Address, Data); }
  int Read(int AM, int DT, uint32_t Address, void *Data) { return VmeRead(AM, DT, Address, Data); }
  
  int BlockWrite(uint32_t Address, int Size, void *Buffer, int *Transferred)
  { return VmeBlockWrite(Address, Size, Buffer, Transferred); }
  int BlockRead(uint32_t Address, int Size, void *Buffer, int *Transferred)
  { return VmeBlockRead(Address, Size, Buffer, Transferred); }
  int ReadCR(uint32_t Address, void *Data) {return VmeReadCR(Address, Data); } /* EC */
  VmeXVB() { Vme_status = 0; VmeOpen();  }
  ~VmeXVB() { VmeClose(); Vme_status = 0; }
  
};

#endif

/*
 * Vme JLab Intel Controller
 */

#include <iostream>
#include <unistd.h>
#include <stdint.h>

#include "MPDlo.h"
#include <Vme_gexvb.h>

#include <string.h>
#include <pthread.h>


//#include "CAENVMElib.h"
//#include "Vme_intel.h"
//#include "jvme.h"
//#include <AdcVme64x.h>

/* Mutex to guard flexio read/writes */
pthread_mutex_t   giMutex = PTHREAD_MUTEX_INITIALIZER;
#define DSCLOCK     pthread_mutex_lock(&giMutex);
#define DSCUNLOCK   pthread_mutex_unlock(&giMutex);

#define cvSuccess 1;

#define BUFFER_SIZE 20480
#define NBUFFERS 10
DMA_MEM_ID vmeIN; 
DMA_MEM_ID vmeOUT;

 extern DMANODE *the_event;
 extern unsigned int *dma_dabufp;

using namespace std;

int VmeXVB::VmeOpen(void)
{
   vmeOpenDefaultWindows();
vmeIN  = dmaPCreate("vmeIN", BUFFER_SIZE, NBUFFERS, 0);
vmeOUT = dmaPCreate("vmeOUT", 0, 0, 0);
dmaPReInitAll();
 /* Setup Address and data modes for DMA transfers
   *   
   *  vmeDmaConfig(addrType, dataType, sstMode);
   *
   *  addrType = 0 (A16)    1 (A24)    2 (A32)
   *  dataType = 0 (D16)    1 (D32)    2 (BLK32) 3 (MBLK) 4 (2eVME) 5 (2eSST)
   *  sstMode  = 0 (SST160) 1 (SST267) 2 (SST320)
   */
vmeDmaConfig(2,4,1); 

 // 	CVBoardTypes VMEController;
// 	short Link;
// 	short Device;
 	int success = 1;

// 	VMEController = cvV1718;
// 	Device = 0;
// 	Link = 0;
// 	success = CAENVME_Init(VMEController, Device, Link, &V17x8Handle);
// 	if( success == cvSuccess )
// 	{
// 		Sleep(500);
// 		success = CAENVME_SystemReset(V17x8Handle);
// 		Sleep(500);
// 		if( success == cvSuccess )
		  //		Vme_status = 1;
	//		else
 	//		Vme_status = 0;
	// 	}
// 	else
	  // 		Vme_status = 0;
//	return success;
   Vme_status = 1;
   return BUS_OK;
}

int VmeXVB::VmeClose(void)
{
	// return CAENVME_End(V17x8Handle);
dmaPFreeAll();
    vmeCloseDefaultWindows(); 
  return BUS_OK;
}

int VmeXVB::VmeReadCR(uint32_t Address, void *Data)
{
  long int * ptr = 0;
  int res;
  unsigned int * dest;
  dest = ( unsigned int * ) Data;
   res = vmeBusToLocalAdrs(0x39,(char *) Address , (char **)&ptr);
   //   *dest = *ptr;
   *dest = vmeRead32((volatile unsigned int *)ptr);
  return BUS_OK;
}


int VmeXVB::VmeWrite(uint32_t Address, void *  Data)
{
  VmeWrite(0x09,4,Address,Data);
  return BUS_OK;
}

int VmeXVB::VmeRead(uint32_t Address, void * Data)
{
  VmeRead(0x09,4,Address,Data);
  return BUS_OK;
}

int VmeXVB::VmeWrite(int AM, int DT, uint32_t Address, void *Data)
{

  // to be extended
  long int * dest;
  long int * ptr = 0;
  int res;
   dest = ( long int * ) Data;
   res = vmeBusToLocalAdrs(AM,(char *) Address , (char **)&ptr);
   if (ptr!=0)
     {
       if (DT==4) {
	 *ptr = LSWAP((unsigned int) *dest);
       } else
	 {
	   *ptr = LSWAP((unsigned short) *dest);
	 };
     }
       else
     {
       cout<<" Error writing value "<<*dest <<" at address :"<<hex<<Address<<endl;
     }
   //      cout<<"Written value at"<<hex<<Address<< " local "<<ptr<<"value :"<<*ptr<<endl;
   // if (DT==4) {
   //   vmeWrite32((volatile unsigned int *)ptr, (unsigned int) *dest);
   // } else { // assume 16 bit
   //   vmeWrite16((volatile unsigned short *)ptr, (unsigned short) *dest);
   

   //  }
    return BUS_OK;
}

int VmeXVB::VmeRead(int AM, int DT, uint32_t Address, void *Data)
{
  long int * ptr = 0;
  int res;
  long int * dest;
  dest = ( long int * ) Data;
  // printf ("VME addr 0x%x AM : 0x%x",Address,AM);
  res = vmeBusToLocalAdrs(AM,(char *) Address , (char **)&ptr);
 if (res != 0)
     {
      printf("%s: ERROR: Error in vmeBusToLocalAdrs res=%d \n",__FUNCTION__,res);
      return -1;
      exit(0);
     };


/////////////////////////////////////////////////////////////
//  long int * ptr = 0;
//   int res;
//   long int * dest;
//   dest = ( long int * ) Data;
//    res = vmeBusToLocalAdrs(AM,(char *) Address , (char **)&ptr);
   if (ptr!=0)
     {
   
//   if (DT==4) {
//       *dest = vmeRead32((volatile unsigned int *)ptr);
//   } else { // assume 16 bits read
//      *dest = vmeRead16((volatile unsigned short *)ptr);
//   }



 if (DT==4) {
      *dest = LSWAP((unsigned int) *ptr);
    }
    else
      {
	*dest = LSWAP((unsigned short) *ptr);
      }
     }
   else {
     cout<<" Error reading address :"<<hex<<Address<<endl;
     // exit(5);
   }
//   if (DT==4) {
//       *dest = vmeRead32((volatile unsigned int *)ptr);
//   } else { // assume 16 bits read
//      *dest = vmeRead16((volatile unsigned short *)ptr);
//   }
//       *dest = *ptr;
   return BUS_OK;
}

int VmeXVB::VmeBlockWrite(uint32_t Address, int Size, void *Buffer, int *Transferred)
{
  int i,retval,nbytes; 
  // vmeDmaSend(unsigned int locAdrs, unsigned int vmeAdrs, int size);
  //	return CAENVME_BLTWriteCycle(V17x8Handle, Address, (unsigned char *)Buffer, Size, cvA32_U_BLT, cvD32, Transferred);
 unsigned int event_number = 1234; /* change this to whatever you want */
 GETEVENT(vmeIN,event_number);
 retval=vmeDmaSend((unsigned int )Address,(unsigned int) dma_dabufp,(unsigned int) Size);
 nbytes = vmeDmaDone();
 *Transferred = nbytes;
 return retval;
}

// int VmeXVB::VmeBlockRead(uint32_t Address, int Size, void *Buffer, int *Transferred)
// {
//  int i,retval,nbytes; 
//  // vmeDmaSend(unsigned int locAdrs, unsigned int vmeAdrs, int size);
//   //	return CAENVME_BLTWriteCycle(V17x8Handle, Address, (unsigned char *)Buffer, Size, cvA32_U_BLT, cvD32, Transferred);
//  extern DMANODE *the_event;
//  extern unsigned int *dma_dabufp;
//  unsigned int event_number = 1234; /* change this to whatever you want */
//  GETEVENT(vmeIN,event_number);
//  retval=vmeDmaSend((unsigned int )Address,(unsigned int) dma_dabufp,(unsigned int) Size);
//  nbytes = vmeDmaDone();
//  *Transferred = nbytes;
//  return retval;
// }
int VmeXVB::VmeBlockRead(uint32_t Address, int Size, void *Buffer, int *Transferred)
{
  //return CAENVME_BLTReadCycle(V17x8Handle, Address, (void *)Buffer, Size, cvA32_U_BLT, cvD32, Transferred);

   


 extern DMANODE *the_event;
 extern unsigned int *dma_dabufp;
 unsigned int event_number =0001;
 GETEVENT(vmeIN,event_number);
//////Attempt at dma block transfer
// printf("\n dmaconfig \n");
 vmeDmaConfig(2,2,1);
 

 // *dma_dabufp++ =0x12345678;
 // Buffer=dma_dabufp;
 int retVal = vmeDmaSend((unsigned int)dma_dabufp,Address,Size*4);
 //printf("retval = %d \n",retVal);
 int cnt=0;
int testval= vmeDmaDone();
 while(cnt<1000&&(testval==0))
   {
     cnt++;
    testval= vmeDmaDone();
   }

 //printf("testval = %d \n",testval);
dma_dabufp += (testval)>>2;
PUTEVENT(vmeOUT);
/* Grab a buffer from the vmeOUT queue */
DMANODE *outEvent = dmaPGetItem(vmeOUT);

/* Get it's length, type, and event number */
int length       = outEvent->length;
int type         = outEvent->type;
int event_number2 = outEvent->nevent; 
 int *temp=(int *)Buffer;

/* Do something with the data... I'm just printing it to stdout */

int i;
//printf("this is the length %d",length);

 // memcpy(outEvent->data,Buffer,length*4);

for(i=0; i<length; i++)
{
  temp[i]= LSWAP(outEvent->data[i]);
  //  cout<<"buf "<<hex<<outEvent->data[i]<<" "<<temp[i]<<endl;
}
// printf("this is the length %d retval %d",length, retVal);
/* put the buffer back into the vmeIN queue */
dmaPFreeItem(outEvent);
 *Transferred= testval/4 ;

 return retVal;
}

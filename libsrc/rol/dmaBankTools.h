
#ifndef __DMABANKTOOLS__
#define __DMABANKTOOLS__

#define BANKOPEN(bnum, btype, code) {					\
    long *StartOfBank;							\
    StartOfBank = (dma_dabufp);						\
    *(++(dma_dabufp)) = LSWAP((((bnum) << 16) | (btype##_ty) << 8) | (code)); \
    ((dma_dabufp))++;


#define BANKCLOSE							\
  long banklen;								\
  banklen = (long) (((char *) (dma_dabufp)) - ((char *) StartOfBank));	\
  *StartOfBank = LSWAP(banklen);					\
  if ((banklen & 1) != 0) {						\
    (dma_dabufp) = ((long *)((char *) (dma_dabufp))+1);			\
    banklen += 1;							\
    *StartOfBank = LSWAP(banklen);					\
  };									\
  if ((banklen & 2) !=0) {						\
    banklen = banklen + 2;						\
    *StartOfBank = LSWAP(banklen);					\
    (dma_dabufp) = ((long *)((short *) (dma_dabufp))+1);;		\
  };									\
  *StartOfBank = LSWAP(( (banklen) >> 2) - 1);				\
  };

#endif

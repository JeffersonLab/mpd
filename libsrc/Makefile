#
# Description:  Makefile for mpdLib.o, mpdLib.so
#   Depending on ARCH:
#
#  ARCH = VXWORKSPPC:
#   This driver is specific to VxWorks BSPs and must be compiled
#   with access to vxWorks headers.
#
#  ARCH = Linux:
#   This driver requires access to the jvme headers (JLab specific Linux routine header)
#
# Uncomment DEBUG line for debugging info ( -g and -Wall )
DEBUG=1
#
ifndef ARCH
	ifdef LINUXVME_LIB
		ARCH=Linux
	else
		ARCH=VXWORKSPPC
	endif
endif


#Check Operating system we are using
ifndef OSNAME
  OSNAME := $(subst -,_,$(shell uname))
endif

ifndef ARCH
  ARCH = VXWORKSPPC
endif

ifndef WIND_BASE
  WIND_BASE = /site/vxworks/5.5/ppc
endif


ifeq ($(OSNAME),SunOS)
CODALIB		= $(CODA)/$(OSNAME)/lib
CODAINC		= $(CODA)/common/include
CFLAGS		= -O $(DEFS)
endif

ifeq ($(ARCH),Linux)

LINUXVME_LIB	?= ${CODA}/linuxvme/lib
LINUXVME_INC	?= ${CODA}/linuxvme/include

CODALIB 	= $(CODA)/$(OSNAME)/lib
CODAINC 	= $(CODA)/common/include
CFLAGS 		= -O $(DEFS) \
		-I. -I${LINUXVME_INC}  \
		-L. -L${LINUXVME_LIB} -DJLAB
ifdef DEBUG
CFLAGS		+= -Wall -g
else
CFLAGS		+= -O2
endif
AR		= ar
RANLIB		= ranlib
endif

ifeq ($(ARCH),VXWORKSPPC)
INCDIR		= $(WIND_BASE)/target/h -I../universeDma
CC		= ccppc
LD		= ldppc
DEFS		= -mcpu=604 -DCPU=PPC604 -DVXWORKS -D_GNU_TOOL -DVXWORKSPPC
CFLAGS		= -O $(DEFS)
INCS		= -fno-for-scope -fno-builtin -fvolatile -fstrength-reduce -mlongcall \
		 -I. -I$(CODAINC) -I$(INCDIR)
endif

ifeq ($(ARCH),VXWORKS68K51)
INCDI		= /site/vxworks/5.3/68k/target/h
CC		= cc68k
DEFS		= -DCPU=MC68040 -DVXWORKS -DVXWORKS68K51
CFLAGS		= -O $(DEFS)
INCS		= -Wall -mc68020 -fvolatile -fstrength-reduce -nostdinc -I. -I$(INCDIR)
endif


ifeq ($(ARCH),Linux)
all: echoarch libmpd.a
else 
all: echoarch mpdLib.o
endif

mpdLib.o: mpdLib.c mpdLib.h
	$(CC) -c $(CFLAGS) $(INCS) -o $@ mpdLib.c

clean:
	rm -f mpdLib.o libmpd.{a,so} *~

echoarch:
	echo "Make for $(ARCH)"

ifeq ($(OSNAME),Linux)
libmpd.a: mpdLib.o
	$(CC) -fpic -shared $(CFLAGS) -o libmpd.so mpdLib.c
	$(AR) ruv libmpd.a mpdLib.o
	$(RANLIB) libmpd.a

links: libmpd.a
	ln -sf $(PWD)/libmpd.a $(LINUXVME_LIB)/libmpd.a
	ln -sf $(PWD)/libmpd.so $(LINUXVME_LIB)/libmpd.so
	ln -sf $(PWD)/mpdLib.h $(LINUXVME_INC)/mpdLib.h

install: libmpd.a
	@cp -v $(PWD)/$< $(LINUXVME_LIB)/$<
	@cp -v $(PWD)/$(<:%.a=%.so) $(LINUXVME_LIB)/$(<:%.a=%.so)
	@cp -v ${PWD}/*Lib.h $(LINUXVME_INC)

endif


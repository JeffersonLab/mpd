#
# File:
#    Makefile
#
# Description:
#    Makefile for the JLAB extensions to the GEFANUC Controller API
#
#
# $Date$
# $Rev$
#
DEBUG=1

ifndef ARCH
	ifdef LINUXVME_LIB
		ARCH=Linux
	else
		ARCH=VXWORKSPPC
	endif
endif

ifdef CODA_VME
CODA_VME_INC = -I${CODA_VME}/include
endif
ifdef CODA_VME_LIB
CODA_LIB = -L${CODA_VME_LIB}
endif

# linuxvme defaults, if they're not already defined
LINUXVME_INC	?= .
LINUXVME_LIB	?= .

CROSS_COMPILE		=
CC			= $(CROSS_COMPILE)gcc
AR                      = ar
RANLIB                  = ranlib
CFLAGS			= -I. -I../ -I../libconfig -I${LINUXVME_INC} ${CODA_VME_INC}\
			  -L. -L../ -L../libconfig -L${LINUXVME_LIB} -L../libconfig ${CODA_LIB}
ifdef DEBUG
CFLAGS			+= -Wall -g
endif

PROGS			= mpdLibTest i2c_test mpd_histo

all: $(PROGS)

install: all

clean distclean:
	@rm -f $(PROGS) *~ *.so

%: %.c
	echo "Making $@"
	$(CC) $(CFLAGS) -o $@ $(@:%=%.c) -I../ -L../ -lrt -ljvme -lmpd -lconfig -lssp

.PHONY: all clean distclean

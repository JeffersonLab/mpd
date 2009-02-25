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

LINUXVME_LIB	?= ${CODA}/extensions/linuxvme/libs
LINUXVME_INC	?= ${CODA}/extensions/linuxvme/include

CROSS_COMPILE		=
CC			= $(CROSS_COMPILE)gcc
AR                      = ar
RANLIB                  = ranlib
CFLAGS			= -I${LINUXVME_INC} -I. -I/usr/include -I../libconfig -I../\
			  -L${LINUXVME_LIB} -L. -L../libconfig
ifdef DEBUG
CFLAGS			+= -Wall -g
endif

PROGS			= mpdLibTest

all: $(PROGS)

install: all

clean distclean:
	@rm -f $(PROGS) *~ *.so

%: %.c
	echo "Making $@"
	$(CC) $(CFLAGS) -o $@ $(@:%=%.c) -I../ -L../ -lrt -ljvme -lmpd -lconfig

.PHONY: all clean distclean

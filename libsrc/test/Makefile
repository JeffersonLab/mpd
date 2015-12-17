#
# File:
#    Makefile
#
# Description:
#    Makefile for MPD library test programs
#
#
# $Date$
# $Rev$
#
DEBUG=1

LINUXVME_LIB	?= ${CODA}/extensions/linuxvme/libs
LINUXVME_INC	?= ${CODA}/extensions/linuxvme/include

CROSS_COMPILE		=
CC			= $(CROSS_COMPILE)gcc
AR                      = ar
RANLIB                  = ranlib
CFLAGS			= -I. -I../ -I../libconfig/ -I${LINUXVME_INC} \
			  -L. -L../libconfig -L${LINUXVME_LIB}
ifdef DEBUG
CFLAGS			+= -Wall -g
endif

PROGS			= mpdLibTest mpdSSPReadoutTest

all: $(PROGS)

install: all

clean distclean:
	@rm -f $(PROGS) *~ *.so

%: %.c
	echo "Making $@"
	$(CC) $(CFLAGS) -o $@ $(@:%=%.c) -lrt -ljvme -lti -lmpd -lconfig -lssp

.PHONY: all clean distclean

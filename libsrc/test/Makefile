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

DEBUG	?= 1
QUIET	?= 1
#
ifeq ($(QUIET),1)
        Q = @
else
        Q =
endif

ARCH	?= $(shell uname -m)
OS	?= LINUX

ifdef CODA_VME
CODA_VME_INC	= -I${CODA_VME}/include
CODA_LIB	= -L${CODA_VME_LIB}
endif

# linuxvme defaults, if they're not already defined
LINUXVME_INC	?= .
LINUXVME_LIB	?= .


CROSS_COMPILE		=
CC			= $(CROSS_COMPILE)gcc
AR                      = ar
RANLIB                  = ranlib
INCS			= -I. -I../ -I${LINUXVME_INC} ${CODA_VME_INC}
CFLAGS			= -L. -L../ -L${LINUXVME_LIB} ${CODA_LIB} \
			 -lrt -ljvme -lmpd -lconfig
ifdef DEBUG
CFLAGS			+= -Wall -Wno-unused -g
endif

ifeq ($(ARCH),armv7l)
CC			+= -DARCH_armv7l -DVTP
CFLAGS			+= -lvtp -li2c
else
CFLAGS			+= -lssp
endif

SRC			= $(wildcard *.c)
DEPS			= $(SRC:.c=.d)
OBJ			= $(SRC:.c=.o)
PROGS			= $(SRC:.c=)

all: echoarch $(PROGS)

clean distclean:
	@rm -f $(PROGS) *~ $(OBJS) $(DEPS)

%: %.c
	@echo " CC     $@"
	${Q}$(CC) $(CFLAGS) $(INCS) -o $@ $<

%.d: %.c
	@echo " DEP    $@"
	@set -e; rm -f $@; \
	$(CC) -MM -shared $(INCS) $< > $@.$$$$; \
	sed 's,\($*\)\.o[ :]*,\1 $@ : ,g' < $@.$$$$ > $@; \
	rm -f $@.$$$$

-include $(DEPS)

.PHONY: all clean distclean

echoarch:
	@echo "Make for $(OS)-$(ARCH)"

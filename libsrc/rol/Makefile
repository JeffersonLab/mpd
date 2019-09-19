#
# File:
#    Makefile
#
# Description:
#    Makefile for the coda primary and secondary readout lists
#    running on an Intel-based controller running Linux
#
# Uncomment DEBUG line for debugging info ( -g and -Wall )
DEBUG=1
#

# Plug in your primary readout lists here..
VMEROL			= mpd_list.so mpd_ssp_list.so mpd_obuf_list.so
# Add shared library dependencies here.  (jvme, ti, are already included)
ROLLIBS			=  -lmpd -lssp -lconfig #-lremex

LINUXVME_LIB	?= ${CODA}/extensions/linuxvme/libs
LINUXVME_INC	?= ${CODA}/extensions/linuxvme/include

# DEFs for compiling primary readout lists
CC			= gcc
AR                      = ar
RANLIB                  = ranlib
ifdef DEBUG
CFLAGS			= -Wall -g
else
CFLAGS			= -O3
endif
CFLAGS			+= -I. -isystem${LINUXVME_INC} -isystem${CODA}/common/include\
			  -L. -L${LINUXVME_LIB} -DJLAB
CFLAGS			+= -lrt -lpthread -ljvme -lti -lremex $(ROLLIBS)
CFLAGS			+= -DLINUX -DDAYTIME=\""`date`"\"

# DEFs for compiling CODA readout lists
CCRL			= ${CODA_BIN}/ccrl
CODA_INCS		= -I. -I${LINUXVME_INC} -I${CODA}/common/include
CODA_LIBDIRS            = -L. -L${LINUXVME_LIB}
CODA_LIBS		= -ljvme -lti
CODA_DEFS		= -DLINUX -DDAYTIME=\""`date`"\"
CODA_CFLAGS		= -O -w -fpic -shared ${CODA_INCS} ${CODA_LIBDIRS} \
			  ${CODA_LIBS} ${CODA_DEFS}
ifdef DEBUG
CODA_CFLAGS		+= -Wall -g
endif
CRLFILES		= $(wildcard *.crl)
CFILES			= $(CRLFILES:.crl=.c)
SOBJS			= $(CRLFILES:.crl=.so)


all: $(VMEROL) $(SOBJS)

event_list.c: event_list.crl Makefile
	@echo
	@echo "Converting $< to $@"
	@${CCRL} $<

event_list.so: event_list.c
	@echo
	@echo "Building $@ from $<"
	@echo
	${CC} ${CODA_CFLAGS} -o $@ $<

v965_mpd.c: v965_mpd.crl
	@echo
	@echo "Converting $< to $@"
	@${CCRL} $<

v965_mpd.so: v965_mpd.c
	@echo
	@echo "Building $@ from $<"
	@echo
	${CC} ${CODA_CFLAGS} -o $@ $<

%.so: %.c Makefile
	@echo
	@echo "Building $@ from $<"
	@echo
	$(CC) -fpic -shared  $(CFLAGS) -DINIT_NAME=$(@:.so=__init) -DINIT_NAME_POLL=$(@:.so=__poll) -o $@ $<

mpd_slave_list.so: mpd_obuf_list.c Makefile
	@echo
	@echo "Building $@ from $<"
	@echo
	$(CC) -fpic -shared  $(CFLAGS) -DTI_SLAVE -DINIT_NAME=$(@:.so=__init) -DINIT_NAME_POLL=$(@:.so=__poll) -o $@ $<


clean distclean:
	@rm -f  $(VMEROL) $(SOBJS) $(CFILES) *~

.PHONY: all

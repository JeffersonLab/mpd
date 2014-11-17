MPD_OBJ  = MPDhi.o MPDlo.o  
MPD_DEPS = MPDhi.h MPDlo.h
MPD_LIB  = libmpd.so

OBJ      = Vme_gexvb.o

DEPS     = Bus.h Vme_gexvb.h

LINUXVME_LIB	?= ${CODA}/extensions/linuxvme/libs
LINUXVME_INC	?= ${CODA}/extensions/linuxvme/include

CFLAGS	= -g -c -Wall -DLINUX -I../include
LDFLAGS	= -ljvme
DEFS	= -DJLAB
INCS	= -I. -I${LINUXVME_INC} -I../include
CFLAGS += -O ${DEFS} -O2  $(INCS)
LDFLAGS	= -L${LINUXVME_LIB}  -ljvme

LIB	= libjgi.so
CC	= g++ 
LD	= g++

LDFLAGS	= -shared -fPIC -L${LINUXVME_LIB} -ljvme

$(MPD_LIB): $(LIB) $(MPD_OBJ)
#	ar cr $(MPD_LIB) $(MPD_OBJ)
	$(LD) $(LDFLAGS)  $(MPD_OBJ) -o  $@

$(LIB): $(OBJ)
#	ar cr $(LIB) $(OBJ)
	$(LD) $(LDFLAGS)  $(OBJ) -o  $@

.cpp.o: $(DEPS) $(MPD_DEPS)
	$(CC) -o $@ $< $(CFLAGS)

clean:
	rm -f $(MPD_OBJ) $(MPD_LIB) $(EXE) $(OBJ) $(LIB) *~ *.log

testxvb: testxvb.o Vme_gexvb.o
	$(LD) $(LDFLAGS2)  $^ -o  $@
/*
 * Test decoding MPD data 
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define VERSION_TAG 0xE0000000
#define EVENT_TAG   0x10000000
#define MPD_TAG     0x20000000
#define ADC_TAG     0x30000000
#define HEADER_TAG  0x40000000
#define DATA_TAG    0x0
#define TRAILER_TAG 0x50000000

int main(int argc, char *argv[]) {

  char fn_in[1000];
  int ret;
  int val;
  int state;
  int dcount, bcount;
  uint32_t tag;
  int version;

  if (argc<2) {
    printf("SYNTAX: %s input_file_name_hex\n",argv[0]);
    exit(-1);
  }
  
  sprintf(fn_in,"%s",argv[1]);

  FILE *fin;

  fin = fopen(fn_in,"r");

  if (fin == NULL) {
    printf("ERROR: cannot open file %s\n",fn_in);
    exit(-2);
  }

  state=0;
  dcount=0;
  bcount=0;
  version = -1;

  do {

    ret = fscanf(fin, "%x",&val);

    if (ret == EOF) {
      break;
    }

    tag = val & 0xF0000000;

    if (version == -1) { // first line
      version = 0;
      if (tag == VERSION_TAG) {
	version = (val &0xffff); 
	printf("File version = %x\n",version);
	continue; 
      }
    } 

    if (version>0) {
      switch (tag) {
      case EVENT_TAG:
	printf("#EVENT = %d\n", val&0xfffffff);
	break;
      case MPD_TAG:
	printf(" MPD   = %d\n", val&0xffff);
	break;
      case ADC_TAG:
	printf("   ADC = %d\n", val&0xff); 
	break;
      case HEADER_TAG:
	printf("    FRAME-HEADER %d (block=%d)", val&0x1ff, bcount);
	if ((val & 0xE00) == 0xE00) {
	  printf(" >>>\n");
	} else {
	  printf(" APV internal memory error >>>\n");
	}
	break;
      case DATA_TAG:
	printf(" %d", val&0xfff);
	if ((dcount % 16) == 15) { printf("\n"); }
	dcount++;
	break;
      case TRAILER_TAG:
	if ((dcount % 16) != 0) { printf(" >> missing data ? \n"); }
	dcount = 0;
	printf("    FRAME-TRAILER, SAMPLE index=%d <<<\n",val&0xfff);
	state=0;
	bcount++;
	break;
      default:
	printf("WARNING: wrong tag %x\n",tag);
	break;
      }
    } else {
      switch (state) {
      case 0: // header
	printf("SAMPLE-BLOCK %d HEADER %d", bcount, val&0x1ff);
	if ((val & 0xE00) == 0xE00) {
	  printf(" >>>\n");
	} else {
	  printf(" APV internal memory error >>>\n");
	}
	state = 1;
	break;
      case 1: // data
	printf(" %d", val&0xfff);
	if ((dcount % 16) == 15) { printf("\n"); }
	if (dcount == 127) { state=2; dcount=0; } else { dcount++; }
	break;
      case 2: // trailer
	printf("SAMPLE IDX %d <<<\n",val&0xfff);
	state=0;
	bcount++;
	break;
      defaut: // error
	printf("ERROR: something wrong in decoding!\n");
	break;
      }
    }  
  } while (1==1);
    
  fclose(fin);

};

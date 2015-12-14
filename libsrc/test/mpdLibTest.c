/*
 * File:
 *    mpdLibTest.c
 *
 * Description:
 *    Simply evolving program to test the mpd library
 *
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "jvme.h"
#include "mpdLib.h"
#include "../mpdConfig.h"

int 
main(int argc, char *argv[]) 
{

  printf("\nMPD Library Tests\n");
  printf("----------------------------\n");

  mpdCheckAddresses(0);


 CLOSE:

  exit(0);
}


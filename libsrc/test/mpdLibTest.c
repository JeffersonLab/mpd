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
#include "jvme.h"
#include "libconfig/libconfig.h"
#include "mpdConfig.h"

int 
main(int argc, char *argv[]) 
{

  printf("\nMPD Library Tests\n");
  printf("----------------------------\n");

  if(vmeOpenDefaultWindows()!=OK)
    {
/*       exit(-1); */
    }

  mpdConfigInit("cfg/config_apv.txt");
  mpdConfigLoad();


 CLOSE:

  vmeCloseDefaultWindows();

  exit(0);
}


/*
 * File:
 *    sspMpdConfigTest
 *
 * Description:
 *    Test loading of the sspMpdconfig file
 *
 *
 */


#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "jvme.h"
#include "mpdLib.h"
#include "sspMpdConfig.h"

int
main(int argc, char *argv[])
{
  char configfn[128];

  if(argc > 1)
    {
      strncpy(configfn,argv[1],128);
    }
  else
    {
      strncpy(configfn,"/home/sbs-onl/cfg/ssp_config.cfg",128);
    }

  mpdSetPrintDebug(0xffffffff);
  sspMpdConfigInit(configfn);
  sspMpdConfigLoad();

  exit(0);
}

/*
  Local Variables:
  compile-command: "make -k -B sspMpdConfigTest"
  End:
 */

/*
 * File:
 *    mpdStatus.c
 *
 * Description:
 *    Show status of MPDs
 *
 *
 */


#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "jvme.h"
#include "mpdLib.h"

int
main(int argc, char *argv[])
{

  /* mpdGStatus(2); */
  mpdApvStatus(0, 0xffff);

  exit(0);
}

/*
  Local Variables:
  compile-command: "make -k -B mpdStatus"
  End:
 */

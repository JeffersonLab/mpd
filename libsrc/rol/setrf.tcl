#  TCL command to set randy_factor (max events per output buffer)
#  in the coda_roc
#
#  You must include the following arguments
#     rocname  - Name of the ROC
#     val      - value to set randy_factor to
#     filename - Name of file including path that the set val should be written to
#
# 
#  To include this function in the coda_roc at boot
#
#     coda_roc -i -n ROC1 -t ROC -f setrf.tcl
#

proc setrf {rocname val filename} {
  global env

  $rocname configure -randy_factor $val

  set fileId [open $filename a]
  set myvar [$rocname cget -randy_factor]
  puts $fileId "randy_factor=$myvar"
  close $fileId

  return $myvar
}

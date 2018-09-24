#  TCL command to set randy_factor (max events per output buffer)
#  in the coda_roc
#
#  You must include the following arguments
#     val      - value to set randy_factor to
#
#  To include this function in the coda_roc at boot
#
#     coda_roc -i -n ROC1 -t ROC -f setrf.tcl
#

proc setrf {val} {
    global env

    set rocname [info objects]
    $rocname configure -randy_factor $val

    set myvar [$rocname cget -randy_factor]
    puts ""
    puts "************** setrf.tcl ***************"
    puts "*"
    puts "* Max Events per output Buffer = $myvar"
    puts "*"
    puts "************** setrf.tcl ***************"
    puts ""

    return $myvar
}

proc getrf {} {
    global env

    set rocname [info objects]

    set myvar [$rocname cget -randy_factor]
    puts ""
    puts "************** setrf.tcl ***************"
    puts "*"
    puts "* Max Events per output Buffer = $myvar"
    puts "*"
    puts "************** setrf.tcl ***************"
    puts ""

    return $myvar
}

# Set when this script is loaded by the ROC with "-f setrf.tcl"
setrf 5

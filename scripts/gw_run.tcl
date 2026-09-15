# Drives gw_sh through the Gowin synthesis / place-and-route / bitstream
# flow without needing the GUI. The flow stage is selected by the
# `STAGE` environment variable set in the wrapper script:
#   syn  - synthesis only
#   pnr  - place-and-route + bitstream generation
#   all  - both, in sequence
# The project file path is taken from the GPRJ environment variable.

set gprj $::env(GPRJ)
set stage $::env(STAGE)

puts "Opening project: $gprj"
open_project $gprj

# Optional explicit top module (TOP env var). Left unset here: the .gprj's
# top-module detection picks acm9767_top.
if {[info exists ::env(TOP)]} {
    puts "set_option -top_module $::env(TOP)"
    set_option -top_module $::env(TOP)
}

# The RTL is SystemVerilog 2012/2017 (logic, typed enums, $signed casts); the
# gw_sh default is Verilog-2001, which rejects it.
set_option -verilog_std sysv2017

# Header pins 54/55/56 (DAC ch1 clock, ch2 data[1:0]) double as the SSPI
# configuration pins; release them as ordinary IO (the Sipeed LCD examples and
# the camera template do the same).
set_option -use_sspi_as_gpio 1

# Placement / routing effort (Gowin place_option / route_option). 2 = high-effort
# timing-driven; both default to 2 and are overridable via PLACE_OPTION / ROUTE_OPTION.
set place_opt 2
set route_opt 2
if {[info exists ::env(PLACE_OPTION)]} { set place_opt $::env(PLACE_OPTION) }
if {[info exists ::env(ROUTE_OPTION)]} { set route_opt $::env(ROUTE_OPTION) }
puts "set_option -place_option $place_opt -route_option $route_opt"
set_option -place_option $place_opt
set_option -route_option $route_opt

switch -- $stage {
    syn { run syn }
    pnr { run pnr }
    all { run all }
    default {
        puts "Unknown STAGE: $stage (expected syn|pnr|all)"
        exit 1
    }
}

exit

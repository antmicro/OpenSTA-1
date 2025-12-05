read_liberty asap7_small.lib.gz

# Read verilog file and link design
read_verilog read_vcd.v
link_design dut

# Create clock
set_units -time ps
set clock_period 1000
create_clock -name clk -period $clock_period

# Read VCD file
read_vcd -scope dut read_vcd.vcd

# Helper proc to report pin activity for a list of pins
proc report_pin_activity {pins} {
  set pin_list {}
  foreach pin $pins {
    set name [get_property -object_type pin $pin full_name]
    lappend pin_list [list $name $pin]
  }
  set pin_list [lsort -index 0 $pin_list]
  foreach pair $pin_list {
    set name [lindex $pair 0]
    set pin [lindex $pair 1]
    set activity [get_property -object_type pin $pin activity]
    puts "$name transitions [lindex $activity 0] duty [lindex $activity 1]"
  }
}

# Report activity for hierarchical pins
set pins [get_pins * -hierarchical]
report_pin_activity $pins

# Report activity for input/output pins
set pins [concat [all_inputs] [all_outputs]]
report_pin_activity $pins

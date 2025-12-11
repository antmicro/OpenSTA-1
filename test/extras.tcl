read_liberty ../examples/sky130hd_tt.lib.gz
read_verilog ../examples/gcd_sky130hd.v
link_design gcd

set_dont_use sky130_fd_sc_hd__a2111o_1
set_dont_touch sky130_fd_sc_hd__a2111o_1
echo [get_db program_short_name]
all_fanin -to [get_ports resp_rdy]
all_fanout -from [get_ports req_rdy]

check_units -time ps -resistance kOhm -capacitance fF -voltage V -current mA
set_units -time ps -resistance kOhm -capacitance fF -voltage V -current mA
check_units -time ps -resistance kOhm -capacitance fF -voltage V -current mA

create_clock -name clk -period 10
puts "Clock period: [get_attribute period [get_clocks clk]]"
puts "Clock period: [get_attribute [get_clocks clk] period]"

# get_attribute -quiet support and warning message
set dummy [get_attribute -quiet dummy [get_clocks clk]]
if { $dummy == "" } {
  puts "dummy is empty"
} else {
  puts "dummy is not empty"
}
puts [get_attribute dummy [get_clocks clk]]

# get_object_name on invalid returns warning

# get_object_name on lists

# unset_output_delay, unset_input_delay reset

# get_pins -q support

# set_false_path, set_max_delay -th support

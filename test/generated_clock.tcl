read_liberty generated_clock.lib
read_verilog generated_clock.v
link_design generated_clock
create_clock -name clk -period 10 [get_ports SYS_CLK_IN] 

# Should see 3 clocks
puts [get_object_name [get_clocks]]

# TODO figure this out
# foreach_in_collection pin [get_ports *] {
#   puts "[get_object_name $pin] - [get_object_name [get_property -object_type port $pin clocks]]"
# }
foreach_in_collection clk [get_clocks] {
  puts "[get_object_name $clk] period: [get_attribute $clk period]"
}

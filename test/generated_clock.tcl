read_liberty generated_clock.lib
read_verilog generated_clock.v
link_design generated_clock
create_clock -name clk -period 10 [get_ports SYS_CLK_IN] 

# Should see 5 clocks
puts "Number of clocks: [ llength [get_clocks]]"

foreach_in_collection clk [get_clocks] {
  puts "[get_object_name $clk] period: [get_attribute $clk period]"
}

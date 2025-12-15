read_liberty generated_clock.lib
read_verilog generated_clock.v
link_design generated_clock
create_clock -name clk -period 10 [get_ports CLK_IN_1] 
create_clock -name clk2 -period 100 [get_ports CLK_IN_2] 

# Should see 7 clocks
puts "Number of clocks: [ llength [get_clocks]]"

foreach_in_collection clk [get_clocks] {
  puts "[get_object_name $clk] period: [get_attribute $clk period]"
}

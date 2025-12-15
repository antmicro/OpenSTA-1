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

create_generated_clock \
  -name clk_div4 \
  -source [get_ports CLK_IN_1] \
  -master_clock clk \
  -divide_by 4 \
  [get_pins u_second_hierarchy/clk_gen2/CLK_OUT_DIV2]

# Should see 8 clocks now
puts "Number of clocks: [ llength [get_clocks]]"

# Period of this clock should be a divide by 4 overall
puts "[get_object_name [get_clocks clk_div4]] period: [get_attribute [get_clocks clk_div4] period]"

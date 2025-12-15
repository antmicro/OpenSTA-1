read_liberty generated_clock.lib
read_verilog generated_clock.v
link_design generated_clock
create_clock -name clk -period 10 [get_ports CLK_IN_1] 
create_clock -name clk2 -period 100 [get_ports CLK_IN_2] 

# Should see 7 clocks
puts "Number of clocks: [ llength [get_clocks]]"

# Divide a generated clock (virtual) by 2 and should divide the original master by 4
create_generated_clock \
  -name clk_div4 \
  -source [get_ports CLK_IN_1] \
  -master_clock u_second_hierarchy/clk_gen2/CLK_OUT_DIV2 \
  -divide_by 2 \
  -invert \
  [get_pins u_second_hierarchy/clk_gen2/CLK_OUT_DIV2]

# Update generated clocks to compute their waveforms/periods, 8 clock
report_clock_properties

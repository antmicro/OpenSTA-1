read_liberty generated_clock.lib

read_verilog generated_clock.v
link_design generated_clock
create_clock -name clk -period 10 [get_ports SYS_CLK_IN]

# Should see two clocks clk, CLK_OUT_DIV2
puts [get_object_name [get_clocks]]

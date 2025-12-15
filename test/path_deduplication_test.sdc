set period 5
create_clock -period $period [get_ports CLK]
set clk_period_factor .2
set delay [expr $period * $clk_period_factor]
set_input_delay $delay -clock CLK {d[*]}
set_output_delay $delay -clock CLK [all_outputs]
set_input_transition .1 [all_inputs]

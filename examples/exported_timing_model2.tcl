read_liberty nangate45_slow.lib.gz
read_verilog exported_timing_model.v
link_design top
create_clock -name clk -period 10 {clk}
set_input_delay -clock clk 0 {in}
write_timing_models exported_timing_model2.lib -instances [list [sta::find_instance i1] [sta::find_instance i2]]

read_liberty nangate45_slow.lib.gz
read_verilog exported_timing_model.v
link_design top
write_timing_models exported_timing_model2.lib -instances [list [sta::find_instance i1] [sta::find_instance i2]]

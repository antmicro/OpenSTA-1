read_liberty nangate45_slow.lib.gz
read_verilog exported_timing_model.v
link_design top
write_timing_model exported_timing_model1.lib -instance [sta::find_instance i1]

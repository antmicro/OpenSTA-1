# timing cell example
read_liberty nangate45_slow.lib.gz
read_verilog timing_cell.v
link_design timing_cell
create_clock -name clk -period 10 {clk}
set_input_delay -clock clk 0 {in}
write_timing_model -paths timing_cell.lib -internal_path_count 1

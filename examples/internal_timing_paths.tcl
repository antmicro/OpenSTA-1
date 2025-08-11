# internal timing paths example
read_liberty timing_cell.lib
read_verilog internal_timing_paths.v
link_design top
create_clock -name clk -period 10 {clk}
set_input_delay -clock clk 0 {in}
find_internal_timing_paths
report_checks

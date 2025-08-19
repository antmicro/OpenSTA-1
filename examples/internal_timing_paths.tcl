# internal timing paths example
read_liberty nangate45_slow.lib.gz
read_liberty timing_cell.lib
read_verilog internal_timing_paths.v
link_design top
create_clock -name clk -period 10 {clk}
set_input_delay -clock clk 0 {in}
group_path -name custom -from {in}
report_checks -path_group custom -path_delay max_fall -group_path_count 1 -format json

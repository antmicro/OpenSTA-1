# internal timing paths example
read_liberty nangate45_slow.lib.gz
read_liberty timing_cell.lib
read_verilog internal_timing_paths.v
link_design top
create_clock -name clk -period 10 {clk}
set_input_delay -clock clk 0 {in}
group_path -name custom -to {t2 t3}
group_path -name long -to {t4 t5}
report_checks -group_path_count 2

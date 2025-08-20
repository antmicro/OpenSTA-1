# timing cell example
read_liberty nangate45_slow.lib.gz
read_verilog timing_cell.v
link_design timing_cell
create_clock -name clk -period 10 {clk}
set_input_delay -clock clk 0 {in}
group_path -name custom -to {r3 r4}
group_path -name long -to r5
write_timing_model -paths timing_cell.lib -internal_path_count 4
report_checks -group_path_count 4 -path_delay max_rise -from r1 -sort_by_slack

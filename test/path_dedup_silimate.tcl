read_liberty ../examples/sky130hd_tt.lib.gz
read_verilog ./path_deduplication_test.v
link_design path_deduplication_test
read_sdc path_deduplication_test.sdc

source ./path_dedup_common.tcl

set checks_rpt [make_checks_rpt\
    -group_path_count 999\
    -fields {input_pins slew}\
    -silimate_dedup_endpoints_rx "_reg\\\[\d+\\\]/D$"]

# store 1 has the highest delay so it will be first
puts "store[1] count: [count_pattern "store\\\[1\\\]" $checks_rpt]"

# store_reg[0] matches store_reg[1,2,3] after the regex. this one happens to be first
puts "store_reg[0]/D count: [count_pattern "store_reg\\\[0\\\]/D" $checks_rpt]"

file delete -force $checks_rpt

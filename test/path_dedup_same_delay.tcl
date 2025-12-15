read_liberty ../examples/sky130hd_tt.lib.gz
read_verilog ./path_deduplication_test.v
link_design path_deduplication_test
read_sdc path_deduplication_test.sdc

source ./path_dedup_common.tcl

set checks_rpt [make_checks_rpt\
    -group_path_count 999\
    -fields {input_pins slew}\
    -dedup_same_delay]

# store[1] has unique delay because of inverter
puts "store[1] count: [count_pattern "store\\\[1\\\]" $checks_rpt]"

# store[0] is the first of the rest of the bits
puts "store[0] count: [count_pattern "store\\\[0\\\]" $checks_rpt]"

# all flip-flops have identical delay
puts "store_reg count: [count_pattern "store_reg" $checks_rpt]"

file delete -force $checks_rpt

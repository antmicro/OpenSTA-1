read_liberty ../examples/sky130hd_tt.lib.gz
read_verilog ../examples/gcd_sky130hd.v
link_design gcd
read_sdc ../examples/gcd_sky130hd.sdc

source ./path_dedup_common.tcl

set checks_rpt [make_checks_rpt\
    -group_path_count 999\
    -fields {input_pins slew}\
    -dedup_by_word]

puts "resp_msg count: [count_pattern "resp_msg" $checks_rpt]"
puts "resp_msg[15] count: [count_pattern "resp_msg\\\[15\\\]" $checks_rpt]"

file delete -force $checks_rpt

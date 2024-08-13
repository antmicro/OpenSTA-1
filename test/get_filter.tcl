# Read in design and libraries
read_liberty asap7_invbuf.lib.gz
read_liberty asap7_seq.lib.gz
read_liberty asap7_simple.lib.gz
read_verilog reg1_asap7.v
link_design top
create_clock -name clk -period 500 {clk1 clk2 clk3}
create_clock -name vclk -period 1000

# Test filters for each SDC command
puts "get_cells"
puts [report_object_full_names [get_cells -filter liberty_cell==BUFx2_ASAP7_75t_R *]]
puts "get_clocks"
puts [report_object_full_names [get_clocks -filter is_virtual==0 *]]
puts [report_object_full_names [get_clocks -filter is_virtual==1 *]]
puts "get_lib_cells"
puts [report_object_full_names [get_lib_cells -filter is_buffer==1 *]]
puts [report_object_full_names [get_lib_cells -filter is_inverter==1 *]]
puts "get_lib_pins"
puts [report_object_full_names [get_lib_pins -filter direction==input BUFx2_ASAP7_75t_R/*]]
puts [report_object_full_names [get_lib_pins -filter direction==output BUFx2_ASAP7_75t_R/*]]
puts "get_libs"
puts [report_object_full_names [get_libs -filter name==asap7sc7p5t_INVBUF_RVT_TT_ccs_211120 *]]
puts "get_nets"
puts [report_object_full_names [get_nets -filter name=~*q *]]
puts "get_pins"
puts [report_object_full_names [get_pins -filter direction==input *]]
puts [report_object_full_names [get_pins -filter direction==output *]]
puts "get_ports"
puts [report_object_full_names [get_ports -filter direction==input *]]
puts [report_object_full_names [get_ports -filter direction==output *]]

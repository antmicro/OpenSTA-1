read_liberty get_cells_crash.lib
read_verilog get_cells_crash.v
link_design top

report_object_names [get_cells * -hierarchical -filter liberty_cell==lib_cell]

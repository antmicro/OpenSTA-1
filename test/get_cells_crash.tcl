read_liberty get_cells_crash.lib
read_verilog get_cells_crash.v
link_design top

get_cells * -filter liberty_cell==hi

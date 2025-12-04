# set_units with floating point numbers

# start by setting the units
set_units -capacitance 1000.0fF
set_units -resistance 1.0kohm
set_units -time 1.0ns
set_units -voltage 1.0v
set_units -current 1.0A
set_units -distance 1.0um

# then check the units
check_units -capacitance 1.0pF
check_units -resistance 1.0kohm
check_units -time 1.0ns
check_units -voltage 1.0v
check_units -current 1.0A
check_units -power 1.0W
check_units -distance 1.0um

# finally report the units
report_units

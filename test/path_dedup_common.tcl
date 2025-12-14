proc count_pattern {pattern filename} {
    set count 0
    set fh [open $filename r]
    while {[gets $fh line] >= 0} {
        if {[regexp $pattern $line]} {
            incr count
        }
    }
    close $fh
    return $count
}

proc make_checks_rpt {args} {
    set temp_file_id [file tempfile temp_filename]
    close $temp_file_id

    report_checks\
        -format end\
        {*}$args > $temp_filename

    return $temp_filename
}

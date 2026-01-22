module nested
(
  input wire i,
  output wire o
);

  lib_cell u_macro (
    .i(i),
    .o(o)
  );

endmodule

module top
(
  input wire inp,
  output wire out
);

  nested i_nested (
    .i(inp),
    .o(out)
  );

endmodule

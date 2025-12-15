module generated_clock (

  // Inputs/outputs for first test
  input wire CLK_IN_1,
  output wire slow_clk_int,
  output wire fast_clk_int,
  output wire slow_clk_out2,
  output wire fast_clk_out2,

  // Inputs/outputs for second test
  input wire CLK_IN_2,
  output wire CLK_OUT_2
);

  // Tests basic clock generation
  second_hierarchy u_second_hierarchy (
    .clk_in(CLK_IN_1),
    .slow_clk_out(slow_clk_int),
    .fast_clk_out(fast_clk_int),
    .slow_clk_out2(slow_clk_out2),
    .fast_clk_out2(fast_clk_out2)
  );

  // Test edges/shifts
  CLK_EDGE_SHIFT clk_edge_shift (
    .CLK_IN(CLK_IN_2),
    .CLK_OUT(CLK_OUT_2)
  );

endmodule

module second_hierarchy (
  input wire clk_in,
  output wire slow_clk_out,
  output wire fast_clk_out,
  output wire slow_clk_out2,
  output wire fast_clk_out2
);

  CLK_GEN clk_gen (
    .CLK_IN(clk_in),
    .CLK_OUT_DIV2(slow_clk_out),
    .CLK_OUT_MUL2(fast_clk_out)
  );

  CLK_GEN clk_gen2 (
    .CLK_IN(clk_in),
    .CLK_OUT_DIV2(slow_clk_out2),
    .CLK_OUT_MUL2(fast_clk_out2)
  );

endmodule
module generated_clock (
  input wire SYS_CLK_IN,
  output wire slow_clk_int,
  output wire fast_clk_int
);

  second_hierarchy u_second_hierarchy (
    .clk_in(SYS_CLK_IN),
    .slow_clk_out(slow_clk_int),
    .fast_clk_out(fast_clk_int)
  );

endmodule

module second_hierarchy (
  input wire clk_in,
  output wire slow_clk_out,
  output wire fast_clk_out
);

  CLK_GEN clk_gen (
    .clk_in(clk_in),
    .CLK_OUT_DIV2(slow_clk_out),
    .CLK_OUT_MUL2(fast_clk_out)
  );

endmodule
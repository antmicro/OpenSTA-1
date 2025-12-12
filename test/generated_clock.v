module generated_clock (
  input wire SYS_CLK_IN,
  output wire slow_clk_int
);

  second_hierarchy u_second_hierarchy (
    .slow_clk_int(SYS_CLK_IN),
    .slow_clk_out(slow_clk_int)
  );

endmodule

module second_hierarchy (
  input wire slow_clk_int,
  output wire slow_clk_out
);

  CLK_DIV_BY_2 my_divider (
    .CLK_IN(slow_clk_int),
    .CLK_OUT_DIV2(slow_clk_out)
  );

endmodule
module generated_clock (
  input wire SYS_CLK_IN,
  output wire slow_clk_int
);

  CLK_DIV_BY_2 my_divider (
    .CLK_IN(SYS_CLK_IN),
    .CLK_OUT_DIV2(slow_clk_int)
  );

endmodule
module inner1 (input clk_inner, input in_inner, output out_inner);
  DFF_X1 r(.D(in_inner), .CK(clk_inner), .Q(out_inner));
endmodule

module inner2 (input clk_inner, input in_inner, output out_inner);
  DFF_X2 r(.D(in_inner), .CK(clk_inner), .Q(out_inner));
endmodule

module top (input clk, input in, output out);
  wire rq;

  DFF_X1 r (.D(in), .CK(clk), .Q(rq));
  inner1 i1 (.clk_inner(clk), .in_inner(rq), .out_inner(out));
  inner2 i2 (.clk_inner(clk), .in_inner(rq), .out_inner(out));
endmodule

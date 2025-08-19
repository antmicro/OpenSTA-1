module timing_cell (input in, input clk, output out);

  wire w1, w2, w3, w4;

  DFF_X1 r1 (.D(in), .CK(clk), .Q(w1));

  BUF_X1 u2 (.A(w1), .Z(w2));
  BUF_X1 u3 (.A(w2), .Z(w3));
  BUF_X1 u5 (.A(w3), .Z(w4));
  
  DFF_X1 r2 (.D(w4), .CK(clk), .Q(out));

  DFF_X1 r3 (.D(w3), .CK(clk), .Q(out));

endmodule

module internal_paths_cell (input in, input clk, output out);

  wire w1, w2, w3, w4, w5, w6, w7;

  DFF_X1 r1 (.D(in), .CK(clk), .Q(w1));

  BUF_X1 u2 (.A(w1), .Z(w2));
  BUF_X1 u3 (.A(w2), .Z(w3));
  BUF_X1 u5 (.A(w3), .Z(w4));
  BUF_X1 u6 (.A(w4), .Z(w5));
  BUF_X1 u7 (.A(w5), .Z(w6));
  BUF_X1 u8 (.A(w6), .Z(w7));
  
  // r1 - r2 with one delay buffer
  // path group: clk
  DFF_X1 r2 (.D(w2), .CK(clk), .Q(out));

  // r1 - r3 with two delay buffers
  // path group: custom
  DFF_X1 r3 (.D(w3), .CK(clk), .Q(out));

  // r1 - r4 with three delay buffers
  // path group: custom
  DFF_X1 r4 (.D(w4), .CK(clk), .Q(out));

  // r1 - r5 with four delay buffers
  // path group: long
  DFF_X1 r5 (.D(w5), .CK(clk), .Q(out));

  // r1 - r6 with six delay buffers
  // path group: long
  DFF_X1 r6 (.D(w7), .CK(clk), .Q(out));

endmodule

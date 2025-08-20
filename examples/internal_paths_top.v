module internal_paths_top (input in, input clk, output out);

  wire w1, w2, w3, w4, w5, w6;

  internal_paths_cell t1 (.in(in), .clk(clk), .out(w1));

  BUF_X1 b1 (.A(w1), .Z(w2));
  BUF_X1 b2 (.A(w2), .Z(w3));

  internal_paths_cell t2 (.in(w2), .clk(clk), .out(out));

  internal_paths_cell t3 (.in(w3), .clk(clk), .out(out));

  BUF_X1 b3 (.A(w3), .Z(w4));
  BUF_X1 b4 (.A(w4), .Z(w5));

  internal_paths_cell t4 (.in(w4), .clk(clk), .out(out));

  internal_paths_cell t5 (.in(w5), .clk(clk), .out(out));

  BUF_X1 b4 (.A(w5), .Z(w6));

  internal_paths_cell t6 (.in(w6), .clk(clk), .out(out));
  
endmodule
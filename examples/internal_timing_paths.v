module top (input in, input clk, output out);

  wire w1, w2;

  timing_cell t1 (.in(in), .clk(clk), .out(w1));

  BUF_X1 b1 (.A(w1), .Z(w2));

  timing_cell t2 (.in(w2), .clk(clk), .out(out));
  
endmodule
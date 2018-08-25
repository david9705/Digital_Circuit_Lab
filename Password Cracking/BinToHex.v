module BinToHex(input [31:0] A, output [63:0] B);
    
  Bin1ToHex1 D8 (.A(A[31:28]), .B(B[63:56]));
  Bin1ToHex1 D7 (.A(A[27:24]), .B(B[55:48]));
  Bin1ToHex1 D6 (.A(A[23:20]), .B(B[47:40]));
  Bin1ToHex1 D5 (.A(A[19:16]), .B(B[39:32]));
  
  Bin1ToHex1 D1 (.A(A[15:12]), .B(B[31:24]));
  Bin1ToHex1 D2 (.A(A[11:8]), .B(B[23:16]));
  Bin1ToHex1 D3 (.A(A[7:4]), .B(B[15:8]));
  Bin1ToHex1 D4 (.A(A[4:0]), .B(B[7:0]));

endmodule

module Bin1ToHex1(input [3:0] A, output [7:0] B);

  assign B = (A >=0 && A < 10)? A+48 : A+55;

endmodule

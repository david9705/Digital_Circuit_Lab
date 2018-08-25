module BinToHex(input [19:0] A, output [39:0] B);

  Bin1ToHex1 D0 (.A(A[19:16]), .B(B[39:32]));
  Bin1ToHex1 D1 (.A(A[15:12]), .B(B[31:24]));
  Bin1ToHex1 D2 (.A(A[11:8]), .B(B[23:16]));
  Bin1ToHex1 D3 (.A(A[7:4]), .B(B[15:8]));
  Bin1ToHex1 D4 (.A(A[3:0]), .B(B[7:0]));

endmodule

module Bin1ToHex1(input [3:0] A, output [7:0] B);

  assign B = (A >=0 && A < 10)? A+48 : A+55;

endmodule

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2017/09/19 11:25:45
// Design Name: 
// Module Name: mmult
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module mmult(
  input  clk,                 // Clock signal
  input  reset_n,             // Reset signal (negative logic)
  input  enable,              // Activation signal for matrix multiplication
  input  [0:9*8-1] A_mat,     // A matrix
  input  [0:9*8-1] B_mat,     // B matrix
  output valid,               // Signals that the output is valid to read
  output reg [0:9*17-1] C_mat // The result of A x B
);

endmodule

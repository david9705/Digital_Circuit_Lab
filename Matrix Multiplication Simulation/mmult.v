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

integer idx =  0 ;

assign valid = (idx == 3)? 1:0 ;

always @ (posedge clk) begin
	if(!reset_n) begin
		
		C_mat <= 0 ;
		
		end
	else if(enable && idx < 3) begin
			if(idx == 0) begin
				C_mat[0 : 16] <= A_mat[0 : 7] * B_mat[0 : 7]
								+ A_mat[8 : 15] * B_mat[24 : 31]
								+ A_mat[16 : 23] * B_mat[48 : 55] ;   //done
								
				C_mat[17 : 33] <= A_mat[0 : 7] * B_mat[8 : 15]
									+ A_mat[8 : 15] * B_mat[32 : 39]
									+ A_mat[16 : 23] * B_mat[56 : 63] ;
									
				C_mat[34 : 50] <= A_mat[0 : 7] * B_mat[16 : 23]
									+ A_mat[8 : 15] * B_mat[40 : 47]
									+ A_mat[16 : 23] * B_mat[64 : 71] ;
				idx <= idx + 1 ;
			
				end
			else if(idx == 1) begin

	
				C_mat[51 : 67] <= A_mat[24 : 31] * B_mat[0 : 7]
									+ A_mat[32 : 39] * B_mat[24 : 31]
									+ A_mat[40 : 47] * B_mat[48 : 55] ;   //done
									
				C_mat[68 : 84] <= A_mat[24 : 31] * B_mat[8 : 15]
									+ A_mat[32 : 39] * B_mat[32 : 39]
									+ A_mat[40 : 47] * B_mat[56 : 63] ;
									
				C_mat[83 : 101] <= A_mat[24 : 31] * B_mat[16 : 23]
									+ A_mat[32 : 39] * B_mat[40 : 47]
									+ A_mat[40 : 47] * B_mat[64 : 71] ;
				idx <= idx + 1 ;
				
				end
			else if(idx == 2) begin
				
				C_mat[102 : 118] <= A_mat[48 : 55] * B_mat[0 : 7]
									+ A_mat[56 : 63] * B_mat[24 : 31]
									+ A_mat[64 : 71] * B_mat[48 : 55] ;   //done
									
				C_mat[119 : 135] <= A_mat[48 : 55] * B_mat[8 : 15]
									+ A_mat[56 : 63] * B_mat[32 : 39]
									+ A_mat[64 : 71] * B_mat[56 : 63] ;
									
				C_mat[136 : 152] <= A_mat[48 : 55] * B_mat[16 : 23]
									+ A_mat[56 : 63] * B_mat[40 : 47]
									+ A_mat[64 : 71] * B_mat[64 : 71] ;
				idx <= idx + 1 ;
				
				end 
								
				
				
				else ;
				
			end 
			
		else ;
			
	


end







endmodule

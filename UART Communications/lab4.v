`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Dept. of CS, National Chiao Tung University
// Engineer: Chun-Jen Tsai
//
// Create Date: 2017/04/27 15:06:57
// Design Name: UART I/O example for Arty
// Module Name: lab4
// Project Name:
// Target Devices: Xilinx FPGA @ 100MHz
// Tool Versions:
// Description:
//
// The parameters for the UART controller are 9600 baudrate, 8-N-1-N
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////

module lab4(
  input  clk,
  input  reset_n,
  input  [3:0] usr_btn,
  output [3:0] usr_led,
  input  uart_rx,
  output uart_tx
);

localparam [2:0] S_MAIN_INIT = 0, S_MAIN_PROMPT = 1,
                 S_MAIN_WAIT_KEY = 2, S_MAIN_PROMPT2 = 3 ,
				 S_MAIN_WAIT_KEY2 = 4 , S_MAIN_HELLO = 5 ;

localparam [1:0] S_UART_IDLE = 0, S_UART_WAIT = 1,
                 S_UART_SEND = 2, S_UART_INCR = 3;

// declare system variables
wire print_enable, print_done;
reg [6:0] send_counter;
reg [2:0] P, P_next;
reg [1:0] Q, Q_next;
reg [23:0] init_counter;

// declare UART signals
wire transmit;
wire received;
wire [7:0] rx_byte;
reg  [7:0] rx_temp;
wire [7:0] tx_byte;
wire is_receiving;
wire is_transmitting;
wire recv_error;
reg [15:0]num1 , num2 , swap;
reg [15:0]ans ;
reg [3:0]dex ;
reg done  ;

wire [7:0] temp;     //new
reg [2:0] cnt ;
wire [31:0] HEX ;

/* The UART device takes a 100MHz clock to handle I/O at 9600 baudrate */
uart uart(
  .clk(clk),
  .rst(~reset_n),
  .rx(uart_rx),
  .tx(uart_tx),
  .transmit(transmit),
  .tx_byte(tx_byte),
  .received(received),
  .rx_byte(rx_byte),
  .is_receiving(is_receiving),
  .is_transmitting(is_transmitting),
  .recv_error(recv_error)
);

// Initializes some strings.
// System Verilog has an easier way to initialize an array,
// but we are using Verilog 2005 :(
//
localparam MEM_SIZE = 92;
localparam PROMPT_STR = 0;
localparam HELLO_STR = 35;  // prompt 1
localparam PROMPT2 = 70 ;
reg [7:0] data[0:MEM_SIZE-1];

initial begin
  { data[ 0], data[ 1], data[ 2], data[ 3], data[ 4], data[ 5], data[ 6], data[ 7],
    data[ 8], data[ 9], data[10], data[11], data[12], data[13], data[14], data[15],
	data[16], data[17], data[18], data[19], data[20], data[21], data[22], data[23],
	data[24], data[25], data[26], data[27], data[28], data[29], data[30], data[31],
	data[32], data[33], data[34]}
  <= { 8'h0D, 8'h0A, "Enter the first decimal number: ", 8'h00 };

  { data[35], data[36], data[37], data[38], data[39], data[40], data[41], data[42],
    data[43], data[44], data[45], data[46], data[47], data[48], data[49], data[50],
	data[51], data[52], data[53], data[54], data[55], data[56], data[57], data[58],
	data[59], data[60], data[61], data[62], data[63], data[64], data[65], data[65],
	data[66], data[67], data[68], data[69]}
  <= { 8'h0D, 8'h0A, "Enter the second decimal number: ", 8'h00 };

  { data[70], data[71], data[72], data[73], data[74], data[75], data[76], data[77],
    data[78], data[79], data[80], data[81], data[82], data[83], data[84], data[85],
	data[86], data[87], data[88], data[89], data[90], data[91]}
  <= {8'h0D, 8'h0A ,"The GCD is : 0x" ,"0000", 8'h00 };

  /*{ data[16], data[17], data[18], data[19], data[20], data[21], data[22], data[23],
    data[24], data[25], data[26], data[27], data[28], data[29], data[30], data[31] }
  <= { "Hello, World!", 8'h0D, 8'h0A, 8'h00 };*/
end

// Combinational I/O logics
assign usr_led = usr_btn;
assign enter_pressed = (rx_temp == 8'h0D);
assign tx_byte = (rx_byte >= 48 && rx_byte <= 57 && received)? rx_byte: data[send_counter];

//assign temp = (rx_byte >= 48 && rx_byte <= 57 && cnt <5)? rx_byte : 0 ;  //new

// ------------------------------------------------------------------------
// Main FSM that reads the UART input and triggers
// the output of the string "Hello, World!".
always @(posedge clk) begin
  if (~reset_n)
      P <= S_MAIN_INIT;
  else
      P <= P_next;
end

always @(*) begin // FSM next-state logic

  case (P)
    S_MAIN_INIT: // Delay 10 us.
	   if (init_counter < 1000)
	       P_next = S_MAIN_INIT;
	   else
	       P_next = S_MAIN_PROMPT;
    S_MAIN_PROMPT: // Print the prompt message.
      if (print_done)
           P_next = S_MAIN_WAIT_KEY;
      else
           P_next = S_MAIN_PROMPT;
    S_MAIN_WAIT_KEY: // wait for <Enter> key.
      if (enter_pressed)
           P_next = S_MAIN_PROMPT2;
      else
           P_next = S_MAIN_WAIT_KEY;
	S_MAIN_PROMPT2:   //print msg 2
	  if(print_done)
	      P_next = S_MAIN_WAIT_KEY2;
	  else
	      P_next = S_MAIN_PROMPT2 ;
	S_MAIN_WAIT_KEY2: // wait and print enter number
	  if(enter_pressed)
	      P_next = S_MAIN_HELLO ;
	  else
	      P_next = S_MAIN_WAIT_KEY2 ;
	S_MAIN_HELLO: // Print the ANS message.
      if (print_done)
          P_next = S_MAIN_INIT;
      else
          P_next = S_MAIN_HELLO;
  endcase
end

// FSM output logics: print string control signals.
assign print_enable = (P != S_MAIN_PROMPT && P_next == S_MAIN_PROMPT) ||
				  (P != S_MAIN_PROMPT2 && P_next == S_MAIN_PROMPT2) ||
                  (P == S_MAIN_WAIT_KEY && P_next == S_MAIN_PROMPT2) ||
				  (P == S_MAIN_WAIT_KEY2 && P_next == S_MAIN_HELLO)  ;
assign print_done = (tx_byte == 8'h0);

// Initialization counter.
always @(posedge clk) begin
  if (P == S_MAIN_INIT)
       init_counter <= init_counter + 1;
  else
       init_counter <= 0;
end
// End of the FSM of the print string controller
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
// FSM of the controller to send a string to the UART.
always @(posedge clk) begin
  if (~reset_n) Q <= S_UART_IDLE;
  else Q <= Q_next;
end

always @(*) begin // FSM next-state logic
  case (Q)
    S_UART_IDLE: // wait for the print_string flag
      if (print_enable) Q_next = S_UART_WAIT;
      else Q_next = S_UART_IDLE;
    S_UART_WAIT: // wait for the transmission of current data byte begins
      if (is_transmitting == 1) Q_next = S_UART_SEND;
      else Q_next = S_UART_WAIT;
    S_UART_SEND: // wait for the transmission of current data byte finishes
      if (is_transmitting == 0) Q_next = S_UART_INCR; // transmit next character
      else Q_next = S_UART_SEND;
    S_UART_INCR:
      if (tx_byte == 8'h0) Q_next = S_UART_IDLE; // string transmission ends
      else Q_next = S_UART_WAIT;
  endcase
end

// FSM output logics
assign transmit = (Q_next == S_UART_WAIT || received);
//assign tx_byte = data[send_counter];

// UART send_counter control circuit
always @(posedge clk) begin
  case (P_next)
    S_MAIN_INIT: send_counter <= PROMPT_STR;
    S_MAIN_WAIT_KEY: send_counter <= HELLO_STR;
	S_MAIN_WAIT_KEY2: send_counter <= PROMPT2 ;
    default: send_counter <= send_counter + (Q_next == S_UART_INCR);
  endcase
end
// End of the FSM of the print string controller
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
// The following logic stores the UART input in a temporary buffer.
// The input character will stay in the buffer for one clock cycle.
always @(posedge clk) begin
  rx_temp <= (received)? rx_byte : 8'h0;
end
// ------------------------------------------------------------------------

always @(posedge clk ) begin
	if(P == S_MAIN_INIT)
  begin
	    done =0;
      ans = 0;
      num1 = 0;
      num2 = 0;
      swap = 0;
  end
	if(P == S_MAIN_WAIT_KEY)
	begin
		num1 = (received && rx_byte>=48 && rx_byte <= 57) ? (num1*10) + (rx_byte - 48 ) : num1 ;
	end
	else if(P == S_MAIN_WAIT_KEY2)
	begin
		num2 = (received && rx_byte>=48 && rx_byte <= 57) ? (num2*10) + (rx_byte - 48 ) : num2 ;
	end
	else
	begin
		if(num1 < num2)
		begin
			swap = num1 ;
			num1 = num2 ;
			num2 = swap ;
		end
		else if(num2 != 0)
		begin
			num1 = num1 - num2 ;
		end
		else
		begin
			done = 1 ;
		end
		ans = num1 ;


		/*if(done == 1) begin
			dex[3] = ((ans[15] * 8 + ans[14] * 4 + ans[13] * 2 + ans[12]) >= 10 ) ?  ans[15] * 8 + ans[14] * 4 + ans[13] * 2 + ans[12] + 65 :
				ans[15] * 8 + ans[14] * 4 + ans[13] * 2 + ans[12] +48;
			dex[2] = ((ans[11] * 8 + ans[10] * 4 + ans[9] * 2 + ans[8]) >= 10 ) ?  ans[11] * 8 + ans[10] * 4 + ans[9] * 2 + ans[8] + 65 :
				ans[11] * 8 + ans[10] * 4 + ans[9] * 2 + ans[8] + 48;
			dex[1] = ((ans[7] * 8 + ans[6] * 4 + ans[5] * 2 + ans[4]) >= 10 ) ?  ans[7] * 8 + ans[6] * 4 + ans[5] * 2 + ans[4] + 65 :
				ans[7] * 8 + ans[6] * 4 + ans[5] * 2 + ans[4] +48;
			dex[0] = ((ans[3] * 8 + ans[2] * 4 + ans[1] * 2 + ans[0]) >= 10 ) ?  ans[3] * 8 + ans[2] * 4 + ans[1] * 2 + ans[0] + 65 :
				ans[3] * 8 + ans[2] * 4 + ans[1] * 2 + ans[0] +48;

		end*/
	end



end

always@(posedge clk) begin
	//{data[87], data[88], data[89], data[90]} <= dex ;
	data[87] <= HEX[31:24];
    data[88] <= HEX[23:16];
    data[89] <= HEX[15:8];
    data[90] <= HEX[7:0];
end
/*
always@(posedge clk) begin
	if(~reset_n && P != S_MAIN_WAIT_KEY && P != S_MAIN_WAIT_KEY2) begin
		num1 <= 0 ;
		num2 <= 0 ;
		swap <= 0 ;
		//ans <= 0 ;
	end
end*/

BinToHex H2A1(.A(ans), .B(HEX));


endmodule

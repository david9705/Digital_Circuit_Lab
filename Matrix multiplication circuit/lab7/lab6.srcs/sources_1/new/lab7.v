`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Dept. of Computer Science, National Chiao Tung University
// Engineer: Chun-Jen Tsai
// 
// Create Date: 2017/05/08 15:29:41
// Design Name: 
// Module Name: lab6
// Project Name: 
// Target Devices: 
// Tool Versions:
// Description: The sample top module of lab 6: sd card reader. The behavior of
//              this module is as follows
//              1. When the SD card is initialized, display a message on the LCD.
//                 If the initialization fails, an error message will be shown.
//              2. The user can then press usr_btn[2] to trigger the sd card
//                 controller to read the super block of the sd card (located at
//                 block # 8192) into the SRAM memory.
//              3. During SD card reading time, the four LED lights will be turned on.
//                 They will be turned off when the reading is done.
//              4. The LCD will then displayer the sector just been read, and the
//                 first byte of the sector.
//              5. Everytime you press usr_btn[2], the next byte will be displayed.
// 
// Dependencies: clk_divider, LCD_module, debounce, sd_card
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module lab7(
  // General system I/O ports
  input  clk,
  input  reset_n,
  input  [3:0] usr_btn,
  output [3:0] usr_led,

  // SD card specific I/O ports
  output spi_ss,
  output spi_sck,
  output spi_mosi,
  input  spi_miso,

  // 1602 LCD Module Interface
  output LCD_RS,
  output LCD_RW,
  output LCD_E,
  output [3:0] LCD_D,
  
  input  uart_rx,
  output uart_tx
);

localparam [2:0] S_MAIN_INIT = 3'b000, S_MAIN_IDLE = 3'b001,
                 S_MAIN_WAIT = 3'b010, S_MAIN_READ = 3'b011,
                 S_MAIN_FIND = 3'b100, S_MAIN_END = 3'b101;

// Declare system variables
wire btn_level, btn_pressed;
reg  prev_btn_level;
reg  [8:0] send_counter;
reg  [2:0] P, P_next;
reg  [9:0] sd_counter;
reg  [7:0] data_byte;
reg  [31:0] blk_addr;
reg  [127:0] row_A = "SD card cannot  ";
reg  [127:0] row_B = "be initialized! ";
//reg  done_flag; // Signals the completion of reading one SD sector.

// Declare SD card interface signals
wire clk_sel;
wire clk_500k;
reg  rd_req;
reg  [31:0] rd_addr;
wire init_finished;
wire [7:0] sd_dout;
wire sd_valid;

// Declare the control/data signals of an SRAM memory block
wire [7:0] data_in;
wire [7:0] data_out;
wire [8:0] sram_addr;
wire       sram_we, sram_en;

assign clk_sel = (init_finished)? clk : clk_500k; // clock for the SD controller

clk_divider#(200) clk_divider0(
  .clk(clk),
  .reset(~reset_n),
  .clk_out(clk_500k)
);

debounce btn_db0(
  .clk(clk),
  .btn_input(usr_btn[1]),
  .btn_output(btn_level)
);

LCD_module lcd0( 
  .clk(clk),
  .reset(~reset_n),
  .row_A(row_A),
  .row_B(row_B),
  .LCD_E(LCD_E),
  .LCD_RS(LCD_RS),
  .LCD_RW(LCD_RW),
  .LCD_D(LCD_D)
);

sd_card sd_card0(
  .cs(spi_ss),
  .sclk(spi_sck),
  .mosi(spi_mosi),
  .miso(spi_miso),

  .clk(clk_sel),
  .rst(~reset_n),
  .rd_req(rd_req),
  .block_addr(rd_addr),
  .init_finished(init_finished),
  .dout(sd_dout),
  .sd_valid(sd_valid)
);

sram ram0(
  .clk(clk),
  .we(sram_we),
  .en(sram_en),
  .addr(sram_addr),
  .data_i(data_in),
  .data_o(data_out)
);

//
// Enable one cycle of btn_pressed per each button hit
//
always @(posedge clk) begin
  if (~reset_n)
    prev_btn_level <= 0;
  else
    prev_btn_level <= btn_level;
end

assign btn_pressed = (btn_level == 1 && prev_btn_level == 0)? 1 : 0;

// ------------------------------------------------------------------------
// The following code sets the control signals of an SRAM memory block
// that is connected to the data output port of the SD controller.
// Once the read request is made to the SD controller, 512 bytes of data
// will be sequentially read into the SRAM memory block, one byte per
// clock cycle (as long as the sd_valid signal is high).
assign sram_we = sd_valid;          // Write data into SRAM when sd_valid is high.
assign sram_en = 1;                 // Always enable the SRAM block.
assign data_in = sd_dout;           // Input data always comes from the SD controller.
assign sram_addr = sd_counter[8:0]; // Set the driver of the SRAM address signal.
// End of the SRAM memory block
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
// FSM of the SD card reader that reads the super block (512 bytes)
always @(posedge clk) begin
  if (~reset_n) begin
    P <= S_MAIN_INIT;
  end
  else begin
    P <= P_next;
  end
end

always @(*) begin // FSM next-state logic
  case (P)
    S_MAIN_INIT: // wait for SD card initialization
      if (init_finished == 1) P_next = S_MAIN_IDLE;
      else P_next = S_MAIN_INIT;
    S_MAIN_IDLE: // wait for button click
      if (btn_pressed == 1) P_next = S_MAIN_WAIT;
      else P_next = S_MAIN_IDLE;
    S_MAIN_WAIT: // issue a rd_req to the SD controller until it's ready
      P_next = S_MAIN_READ;
    S_MAIN_READ: // wait for the input data to enter the SRAM buffer
      if (sd_counter == 512) P_next = S_MAIN_FIND;
      else P_next = S_MAIN_READ;
	S_MAIN_FIND: // search 512 bytes within a block in the SRAM buffer
	  if(cal) P_next = S_MAIN_END; // find dlab_end
	  else if(sd_counter == 512) P_next = S_MAIN_WAIT; // read a new block
	  else P_next = S_MAIN_FIND; 
	S_MAIN_END:
	  P_next = S_MAIN_END;
  endcase
end

// FSM output logic: controls the 'rd_req' and 'rd_addr' signals.
always @(*) begin
  rd_req = (P == S_MAIN_WAIT);
  rd_addr = blk_addr;
end

always @(posedge clk) begin
  if (~reset_n) blk_addr <= 32'h2000;
  else blk_addr <= blk_addr + (P == S_MAIN_FIND && P_next == S_MAIN_WAIT); 
end

// FSM output logic: controls the 'sd_counter' signal.
// SD card read address incrementer
always @(posedge clk) begin
  if (~reset_n || P == S_MAIN_WAIT || (P == S_MAIN_READ && P_next == S_MAIN_FIND))
    sd_counter <= 0;
  else if ((P == S_MAIN_READ && sd_valid) || (P == S_MAIN_FIND))
    sd_counter <= sd_counter + 1;
  else
	sd_counter <= sd_counter;
end

// FSM ouput logic: Retrieves the content of sram[] for display
always @(posedge clk) begin
  if (~reset_n) data_byte<=8'b0;
  else if (sram_en && P == S_MAIN_FIND) data_byte <= data_out;
end
// End of the FSM of the SD card reader

// -----------------------------------------------------------------------------------
reg findtag=0;
reg done=0;
reg cal=0;
reg [2:0]idx=0;
reg [5:0] cnt=0;
reg [3:0] tagcounter=0;
reg [7:0] pre;
reg [0:16*8-1] A_mat;
reg [0:16*8-1] B_mat;
reg [0:16*20-1] C_mat;
reg in;

assign usr_led[0]=findtag;
assign usr_led[1]=done;
assign usr_led[2]=cal;

always @(posedge clk) begin
	if(~reset_n) begin
		pre<=0;
		findtag<=0;
		tagcounter<=0;
		A_mat<=0;
		B_mat<=0;
		cnt<=0;
		done<=0;
		idx<=0;
		cal<=0;
		in<=0;
	end 
	if(!findtag) begin 
        if (data_byte == "M") begin tagcounter <= 1; pre<="M"; end
        else if (pre=="M" && (tagcounter == 1) && (data_byte == "A")) begin tagcounter <= tagcounter+1; pre<="A"; end
        else if (pre=="A" && (tagcounter == 2) && (data_byte == "T")) begin tagcounter <= tagcounter+1; pre<="T"; end
        else if (pre=="T" && (tagcounter == 3) && (data_byte == "X")) begin tagcounter <= tagcounter+1; pre<="X"; end
        else if (pre=="X" && (tagcounter == 4) && (data_byte == "_")) begin tagcounter <= tagcounter+1; pre<="_"; end
        else if (pre=="_" && (tagcounter == 5) && (data_byte == "T")) begin tagcounter <= tagcounter+1; pre<="T"; end
        else if (pre=="T" && (tagcounter == 6) && (data_byte == "A")) begin tagcounter <= tagcounter+1; pre<="A"; end
        else if (pre=="A" && (tagcounter == 7) && (data_byte == "G")) begin findtag <= 1; tagcounter <= 0; pre<="G";end
        else tagcounter <= 0;
	end
	
	if(findtag && (~done)) begin
		if(data_byte >= "0" && data_byte <= "9")begin
			if(in==0) begin
				if(cnt <= 15)begin
					A_mat[(cnt*8)+:8]=(data_byte-48) * 16;  //(cnt*8)+:8
					//cnt<=cnt+1;	
				end
				else if(cnt < 32 && cnt > 15) begin
					B_mat[((cnt-16)*8)+:8]=(data_byte-48) * 16; 
				end
				in = in + 1 ;
			end
			else begin
				if(cnt <= 15)begin
					A_mat[(cnt*8)+:8]=A_mat[(cnt*8)+:8]+(data_byte-48);  
					//cnt<=cnt+1;	
				end
				else if(cnt < 32 && cnt > 15) begin
					B_mat[((cnt-16)*8)+:8]=B_mat[((cnt-16)*8)+:8]+(data_byte-48); 
				end
				in=0;
				cnt = cnt + 1 ;
			end
		end
		
		else if(data_byte >= "A" && data_byte <= "F") begin
			if(in == 0) begin
				if(cnt <= 15)begin
					A_mat[(cnt*8)+:8]=(data_byte-55)*16; 
				
				end
				else if(cnt < 32 && cnt > 15) begin
					B_mat[((cnt-16)*8)+:8]=(data_byte-55)*16; 
				end
				in=in+1;
			end
			else begin
				if(cnt <= 15)begin
					A_mat[(cnt*8)+:8]=A_mat[(cnt*8)+:8] + (data_byte-55);  
					//cnt<=cnt+1;	
				end
				else if(cnt < 32 && cnt > 15) begin
					B_mat[((cnt-16)*8)+:8]=B_mat[((cnt-16)*8)+:8]+ (data_byte-55); 
				end
				in=0;
				cnt = cnt + 1 ;
			end
		end
		
		if(cnt>=32) begin done<=1 ; end
		
	end
	
	if(done && idx < 4)begin
		if(idx==0) begin
		C_mat[0:19] <= A_mat[0:7] * B_mat[0:7] +
					A_mat[32:39] * B_mat[8:15] +
					A_mat[64:71] * B_mat[16:23]+
					A_mat[96:103] * B_mat[24:31] ;
					
		C_mat[80:99] <= A_mat[0:7] * B_mat[32:39] +
					A_mat[32:39] * B_mat[40:47] +
					A_mat[64:71] * B_mat[48:55]+
					A_mat[96:103] * B_mat[56:63] ;
					
		C_mat[160:179] <= A_mat[0:7] * B_mat[64:71] +
					A_mat[32:39] * B_mat[72:79] +
					A_mat[64:71] * B_mat[80:87]+
					A_mat[96:103] * B_mat[88:95] ;
					
		C_mat[240:259] <= A_mat[0:7] * B_mat[96:103] +
					A_mat[32:39] * B_mat[104:111] +
					A_mat[64:71] * B_mat[112:119]+
					A_mat[96:103] * B_mat[120:127] ;
		idx <= idx + 1;
		
		end
		else if(idx==1) begin
		C_mat[20:39] <= A_mat[8:15] * B_mat[0:7] +
					A_mat[40:47] * B_mat[8:15] +
					A_mat[72:79] * B_mat[16:23]+
					A_mat[104:111] * B_mat[24:31] ;
		
		C_mat[100:119] <= A_mat[8:15] * B_mat[32:39] +
					A_mat[40:47] * B_mat[40:47] +
					A_mat[72:79] * B_mat[48:55]+
					A_mat[104:111] * B_mat[56:63] ;
					
		C_mat[180:199] <= A_mat[8:15] * B_mat[64:71] +
					A_mat[40:47] * B_mat[72:79] +
					A_mat[72:79] * B_mat[80:87]+
					A_mat[104:111] * B_mat[88:95] ;
					
		C_mat[260:279] <= A_mat[8:15] * B_mat[96:103] +
					A_mat[40:47] * B_mat[104:111] +
					A_mat[72:79] * B_mat[112:119]+
					A_mat[104:111] * B_mat[120:127] ;
		idx <= idx + 1 ;

		end
		else if(idx==2) begin
		C_mat[40:59] <= A_mat[16:23] * B_mat[0:7] +
					A_mat[48:55] * B_mat[8:15] +
					A_mat[80:87] * B_mat[16:23]+
					A_mat[112:119] * B_mat[24:31] ;
		
		C_mat[120:139] <= A_mat[16:23] * B_mat[32:39] +
					A_mat[48:55] * B_mat[40:47] +
					A_mat[80:87] * B_mat[48:55]+
					A_mat[112:119] * B_mat[56:63] ;
					
		C_mat[200:219] <= A_mat[16:23] * B_mat[64:71] +
					A_mat[48:55] * B_mat[72:79] +
					A_mat[80:87] * B_mat[80:87]+
					A_mat[112:119] * B_mat[88:95] ;
					
		C_mat[280:299] <= A_mat[16:23] * B_mat[96:103] +
					A_mat[48:55] * B_mat[104:111] +
					A_mat[80:87] * B_mat[112:119]+
					A_mat[112:119] * B_mat[120:127] ;
		idx <= idx + 1 ;
		
		end
		else if(idx==3) begin
		C_mat[60:79] <= A_mat[24:31] * B_mat[0:7] +
					A_mat[56:63] * B_mat[8:15] +
					A_mat[88:95] * B_mat[16:23]+
					A_mat[120:127] * B_mat[24:31] ;
		
		C_mat[140:159] <= A_mat[24:31] * B_mat[32:39] +
					A_mat[56:63] * B_mat[40:47] +
					A_mat[88:95] * B_mat[48:55]+
					A_mat[120:127] * B_mat[56:63] ;
					
		C_mat[220:239] <= A_mat[24:31] * B_mat[64:71] +
					A_mat[56:63] * B_mat[72:79] +
					A_mat[88:95] * B_mat[80:87]+
					A_mat[120:127] * B_mat[88:95] ;
					
		C_mat[300:319] <= A_mat[24:31] * B_mat[96:103] +
					A_mat[56:63] * B_mat[104:111] +
					A_mat[88:95] * B_mat[112:119]+
					A_mat[120:127] * B_mat[120:127] ;
		idx <= idx + 1 ;
		end
		//else if(idx==4) cal<= 1;
		else ;
		
	
	end
	if(idx==4) cal<=1;
	
end
// -----------------------------------------------------------------------------------

// ------------------------------------------------------------------------
// LCD Display function.

always @(posedge clk) begin
  if (~reset_n) begin
    row_A = "SD card cannot  ";
    row_B = "be initialized! ";
  end else if(P == S_MAIN_IDLE) begin
    row_A = "Hit BTN1 to read";
    row_B = "the SD card ... ";	
  end else if (P == S_MAIN_END) begin
    row_A <= "Now reading.....";
	row_B <= "Open Tera Term..";
  end
end
// End of the LCD display function
// ------------------------------------------------------------------------

//  ---------------------------------------------------------------------------

//UART  

localparam [1:0] S_MAIN_INIT_UART = 0, S_MAIN_PROMPT = 1,
                 S_MAIN_WAIT_KEY = 2, S_MAIN_HELLO = 3;
localparam [1:0] S_UART_IDLE = 0, S_UART_WAIT = 1,
                 S_UART_SEND = 2, S_UART_INCR = 3;
				 
				 
// declare system variables
wire print_enable, print_done;
//reg [8:0] send_counter;
reg [1:0] U, U_next;
reg [1:0] Q, Q_next;


// declare UART signals
wire transmit;
wire received;
wire [7:0] rx_byte;
reg  [7:0] rx_temp;
wire [7:0] tx_byte;
wire is_receiving;
wire is_transmitting;
wire recv_error;

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
localparam MEM_SIZE = 148;
localparam PROMPT_STR = 0;
localparam HELLO_STR = 16;
reg [7:0] data[0:MEM_SIZE-1];

initial begin
  { data[ 0], data[ 1], data[ 2], data[ 3], data[ 4], data[ 5], data[ 6], data[ 7],
    data[ 8], data[ 9], data[10], data[11], data[12], data[13], data[14], data[15],
    data[16], data[17], data[18], data[19], data[20], data[21], data[22], data[23], 
	data[24], data[25], data[26], data[27], data[28], data[29], data[30], data[31],
	data[32], data[33], data[34], data[35], data[36], data[37], data[38], data[39], 
	data[40], data[41], data[42], data[43], data[44], data[45], data[46], data[47], 
	data[48], data[49], data[50], data[51], data[52], data[53], data[54], data[55], 
	data[56], data[57], data[58], data[59], data[60], data[61], data[62], data[63],
	data[64], data[65], data[66], data[67], data[68], data[69], data[70], data[71], 
	data[72], data[73], data[74], data[75], data[76], data[77], data[78], data[79], 
	data[80], data[81], data[82], data[83], data[84], data[85], data[86], data[87], 
	data[88], data[89], data[90], data[91], data[92], data[93], data[94], data[95], 
	data[96], data[97], data[98], data[99], data[100], data[101], data[102], data[103], 
	data[104], data[105], data[106], data[107], data[108], data[109], data[110], data[111], 
	data[112], data[113], data[114], data[115], data[116], data[117], data[118], data[119], 
	data[120], data[121], data[122], data[123], data[124], data[125], data[126], data[127],
	data[128], data[129], data[130], data[131], data[132], data[133], data[134], data[135],
	data[136], data[137], data[138], data[139], data[140], data[141], data[142], data[143],
	data[144], data[145], data[146], data[147] }
  <= { 8'h0D, 8'h0A, "The result is: ", 8'h0D , 8'h0A , // 19
		"[ 00000, 00000, 00000, 00000 ]",8'h0D, 8'h0A,   // 32  //3~7 10~14 17~21 24~28  //sum=51
		"[ 00000, 00000, 00000, 00000 ]",8'h0D, 8'h0A,  // 32  //3~7 10~14 17~21 24~28   //sum=83
		"[ 00000, 00000, 00000, 00000 ]",8'h0D, 8'h0A,  // 32  //3~7 10~14 17~21 24~28   //sum=115
		"[ 00000, 00000, 00000, 00000 ]",8'h0D, 8'h0A, 8'h00}; // 33  //3~7 10~14 17~21 24~28

  
end

// Combinational I/O logics
//assign usr_led = usr_btn;
assign tx_byte = data[send_counter];

// ------------------------------------------------------------------------
// Main FSM that reads the UART input and triggers
// the output of the string "Hello, World!".
always @(posedge clk) begin
  if (~reset_n) U <= S_MAIN_INIT_UART;
  else U <= U_next;
end



always @(*) begin // FSM next-state logic
  case (U)
    S_MAIN_INIT_UART: // Delay 10 us.
	   if (cal) U_next = S_MAIN_PROMPT;
		else U_next = S_MAIN_INIT_UART;
    S_MAIN_PROMPT: // Print the prompt message.
      if (print_done) U_next = S_MAIN_WAIT_KEY;
      else U_next = S_MAIN_PROMPT;
	S_MAIN_WAIT_KEY: // wait for <Enter> key.
      U_next = S_MAIN_WAIT_KEY;
    endcase
end

// FSM output logics: print string control signals.
assign print_enable = (U == S_MAIN_INIT_UART && U_next == S_MAIN_PROMPT) ;

assign print_done = (tx_byte == 8'h0);

// Initialization counter.


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
assign transmit = (Q_next == S_UART_WAIT || print_enable);
//assign tx_byte = data[send_counter];

// UART send_counter control circuit
always @(posedge clk) begin
  case (U_next)
    S_MAIN_INIT_UART: send_counter <= PROMPT_STR;
    default: send_counter <= send_counter + (Q_next == S_UART_INCR);
  endcase
end
// End of the FSM of the print string controller
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
// The following logic stores the UART input in a temporary buffer.
// The input character will stay in the buffer for one clock cycle.
// ------------------------------------------------------------------------

//start to chane to ASCII //start to chane to ASCII //start to chane to ASCII //start to chane to ASCII //start to chane to ASCII //start to chane to ASCII //start to chane to ASCII //start to cha


wire [39:0] HEX0 ;
wire [39:0] HEX1 ;
wire [39:0] HEX2 ;
wire [39:0] HEX3 ;
wire [39:0] HEX4 ;
wire [39:0] HEX5 ;
wire [39:0] HEX6 ;
wire [39:0] HEX7 ;
wire [39:0] HEX8 ;
wire [39:0] HEX9 ;
wire [39:0] HEX10 ;
wire [39:0] HEX11 ;
wire [39:0] HEX12 ;
wire [39:0] HEX13 ;
wire [39:0] HEX14 ;
wire [39:0] HEX15 ;
reg [4:0] cntt;


BinToHex H0(.A(C_mat[0:19]), .B(HEX0));  //1
BinToHex H1(.A(C_mat[20:39]), .B(HEX1));  // 5
BinToHex H2(.A(C_mat[40:59]), .B(HEX2));  //9
BinToHex H3(.A(C_mat[60:79]), .B(HEX3));  //13

BinToHex H4(.A(C_mat[80:99]), .B(HEX4));  //2
BinToHex H5(.A(C_mat[100:119]), .B(HEX5));  //6
BinToHex H6(.A(C_mat[120:139]), .B(HEX6));  //10
BinToHex H7(.A(C_mat[140:159]), .B(HEX7));  //14

BinToHex H8(.A(C_mat[160:179]), .B(HEX8));  //3
BinToHex H9(.A(C_mat[180:199]), .B(HEX9));   //7
BinToHex H10(.A(C_mat[200:219]), .B(HEX10));  //11
BinToHex H11(.A(C_mat[220:239]), .B(HEX11));   //15

BinToHex H12(.A(C_mat[240:259]), .B(HEX12));  //4
BinToHex H13(.A(C_mat[260:279]), .B(HEX13));   //8
BinToHex H14(.A(C_mat[280:299]), .B(HEX14));   //12
BinToHex H15(.A(C_mat[300:319]), .B(HEX15));   //16

always @(posedge clk) begin
    if(~reset_n) begin
        cntt= 0;
    end
	else if(cal) begin
        if(cntt==0) begin {data[21], data[22], data[23], data[24], data[25]} = HEX0; cntt=cntt+1; end
        else if(cntt==1) begin {data[28], data[29], data[30], data[31], data[32]} = HEX4; cntt=cntt+1; end
        else if(cntt==2) begin {data[35], data[36], data[37], data[38], data[39]} = HEX8; cntt=cntt+1; end
        else if(cntt==3) begin {data[42], data[43], data[44], data[45], data[46]} = HEX12; cntt=cntt+1; end
		
        else if(cntt==4) begin {data[53], data[54], data[55], data[56], data[57]} = HEX1; cntt=cntt+1; end
		else if(cntt==5) begin {data[60], data[61], data[62], data[63], data[64]} = HEX5; cntt=cntt+1; end
        else if(cntt==6) begin {data[67], data[68], data[69], data[70], data[71]} = HEX9; cntt=cntt+1; end
        else if(cntt==7) begin {data[74], data[75], data[76], data[77], data[78]} = HEX13; cntt=cntt+1; end
		
        else if(cntt==8) begin {data[85], data[86], data[87], data[88], data[89]} = HEX2; cntt=cntt+1; end
		else if(cntt==9) begin {data[92], data[93], data[94], data[95], data[96]} = HEX6; cntt=cntt+1; end
        else if(cntt==10) begin {data[99], data[100], data[101], data[102], data[103]} = HEX10; cntt=cntt+1; end
        else if(cntt==11) begin {data[106], data[107], data[108], data[109], data[110]} = HEX14; cntt=cntt+1; end
		
        else if(cntt==12) begin {data[117], data[118], data[119], data[120], data[121]} = HEX3; cntt=cntt+1; end
        else if(cntt==13) begin {data[124], data[125], data[126], data[127], data[128]} = HEX7; cntt=cntt+1; end
        else if(cntt==14) begin {data[131], data[132], data[133], data[134], data[135]} = HEX11; cntt=cntt+1; end
        else if(cntt==15) begin {data[138], data[139], data[140], data[141], data[142]} = HEX15; end;
    end
end



endmodule

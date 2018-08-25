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

module lab6(
  // General system I/O ports
  input  clk,
  input  reset_n,
  input  [3:0] usr_btn,
  output reg [3:0] usr_led,

  // SD card specific I/O ports
  output spi_ss,
  output spi_sck,
  output spi_mosi,
  input  spi_miso,

  // 1602 LCD Module Interface
  output LCD_RS,
  output LCD_RW,
  output LCD_E,
  output [3:0] LCD_D
);

localparam [2:0] S_MAIN_INIT = 3'b000, S_MAIN_IDLE = 3'b001,
                 S_MAIN_WAIT = 3'b010, S_MAIN_READ = 3'b011,
                 S_MAIN_FIND = 3'b100, S_MAIN_FIN = 3'b101,
				 S_NOT_FIND = 3'b110;

// Declare system variables
wire btn_level, btn_pressed;
reg  prev_btn_level;
reg  [5:0] send_counter;
reg  [2:0] P, P_next;
reg  [9:0] sd_counter;
reg  [7:0] data_byte;
reg  [31:0] blk_addr;
reg  [127:0] row_A = "SD card cannot  ";
reg  [127:0] row_B = "be initialized! ";
reg a_tag = 0 ;
reg add = 0 ;

reg tag = 0;
reg find_end = 0;
reg [7:0] the = 0;
reg [3:0] counter1 = 0;
reg [3:0] counter2 = 0;
reg [3:0] counter3 = 0;
reg [7:0] pre_word;
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
  .btn_input(usr_btn[2]),
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

/*
always @(*) begin // FSM next-state logic
	//if(flag) usr_led[3] = 1 ;
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
      if (sd_counter == 512 && (~TAG_done)) P_next = S_MAIN_DONE;
	  else if(sd_counter == 512 && (~end_TAG_done) && TAG_done) P_next = S_MAIN_SHOW ;
      else P_next = S_MAIN_READ;
    S_MAIN_DONE: begin//  read dlab tag    // read byte 0 of the superblock from sram[] 
      //usr_led[0] = 1;	
      if (sd_counter == 512 && (~TAG_done)) P_next = S_NOT_FIND;
      else if(TAG_done)P_next = S_MAIN_SHOW;
	  else P_next = S_MAIN_DONE;
	  end
	S_NOT_FIND: 
		P_next = S_MAIN_WAIT ;
	S_MAIN_SHOW: begin //  read dlab end  & find the 
      //usr_led[1] = 1;	
	  if(sd_counter == 512 && (~end_TAG_done)) P_next = S_NOT_FIND ;
	  else if(end_TAG_done) P_next = S_FIN ;
	  else P_next = S_MAIN_SHOW;
	end
	S_FIN: begin
		//usr_led[2] <= 1 ;
		P_next <= S_FIN;
	end
    default:
      P_next = S_MAIN_IDLE;
  endcase
end
*/


/*
always @(posedge clk) begin
  if (~reset_n) begin
    R <= R_init;
    
  end
  else begin
    R <= R_next;
    end
end



always @(posedge clk) begin
	if(~reset_n) begin 
		start = 0 ;
		TAG_done = 0 ;
		//pre_word = 0 ;
		//R = R_TAG_D ;
	end 
	else if((~TAG_done) && ((P==S_MAIN_DONE && P_next == S_MAIN_DONE)||(P== S_MAIN_DONE && P_next == S_MAIN_SHOW))) begin
		case(R)
		R_init: begin
			if(data_byte == "D" && start == 0) begin
				pre_word = "D" ;
				start = 1;
				R_next = R_TAG_D ;
			end
			else begin
			    pre_word = data_byte;
				R_next = R_init ;
				start = 0 ;
			end 
		end
		R_TAG_D: begin
			if(data_byte == "L" && start == 1 && pre_word == "D") begin
				pre_word = "L" ;
				start = 2;
				R_next = R_TAG_L ;
			end
			else begin 
			    pre_word = data_byte;
				R_next = R_init ;
				start = 0 ;
			end
		end
		R_TAG_L:  begin
		if(data_byte == "A" && start == 2 && pre_word == "L") begin
				pre_word = "A" ;
				start = 3;
				R_next = R_TAG_A ;
			end
			else begin 
				pre_word = data_byte;
				R_next = R_init ;
				start = 0 ;
			end
		end
		R_TAG_A:  begin
		if(data_byte == "B" && start == 3 && pre_word == "A") begin
				pre_word = "B" ;
				start = 4;
				R_next = R_TAG_B ;
			end
			else begin 
			    pre_word = data_byte;
				R_next = R_init ;
				start = 0 ;
			end
		end
		R_TAG_B:  begin
		if(data_byte == "_" && start == 4 && pre_word == "B") begin
				pre_word = "_" ;
				start = 5;
				R_next = R_TAG_1 ;
			end
			else begin 
			    pre_word = data_byte;
				R_next = R_init ;
				start = 0 ;
			end
		end
		R_TAG_1:  begin
		if(data_byte == "T" && start == 5 && pre_word == "_") begin
				pre_word = "T" ;
				start =  6;
				R_next = R_TAG_T ;
			end
			else begin 
				pre_word = data_byte;
				R_next = R_init ;
				start = 0 ;
			end
		end
		R_TAG_T:  begin
		if(data_byte == "A" && start == 6 && pre_word == "T") begin
				pre_word = "A" ;
				start = 7;
				R_next = R_TAG_aa ;
			end
			else begin 
				pre_word = data_byte;
				R_next = R_init ;
				start = 0 ;
			end
		end
		R_TAG_aa:  begin
		if(data_byte == "G" && start == 7 && pre_word == "A") begin
				pre_word = "G" ;
				start = 8;
				R_next = R_TAG_G ;
			end
			else begin 
				pre_word = data_byte;
				R_next = R_init ;
				start = 0 ;
			end
		end

		R_TAG_G:  begin
		if( start == 8 && pre_word == "G") begin
				TAG_done = 1 ;
			end
		else begin 
		        pre_word = data_byte;
				R_next = R_init ;
				start = 0 ;
			end
		end
			
		
		
		endcase
	end
end*/

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
	  if(find_end) P_next = S_MAIN_FIN; 
	  else if(sd_counter == 512) P_next = S_MAIN_WAIT; 
	  else if(a_tag == 1) P_next = S_NOT_FIND ;
	  else P_next = S_MAIN_FIND;
	S_NOT_FIND:
		if(add) P_next = S_MAIN_FIND ;
		else P_next = S_NOT_FIND ;
	S_MAIN_FIN:
	  P_next = S_MAIN_FIN;
  endcase
end

// FSM output logic: controls the 'rd_req' and 'rd_addr' signals.
always @(*) begin
  rd_req = (P == S_MAIN_WAIT);
  rd_addr = blk_addr;
end

always @(posedge clk) begin
  if (~reset_n) blk_addr <= 32'h2000;
  else if((P == S_MAIN_FIND && P_next == S_MAIN_WAIT)||P == S_NOT_FIND)blk_addr <= blk_addr + 1;
  else blk_addr <= blk_addr ;
end

// FSM output logic: controls the 'sd_counter' signal.
// SD card read address incrementer
always @(posedge clk) begin
  if (~reset_n || P == S_MAIN_WAIT || (P == S_MAIN_READ && P_next == S_MAIN_FIND))
    sd_counter <= 0;
  else 
    sd_counter <= sd_counter + ((P == S_MAIN_READ && sd_valid) || (P == S_MAIN_FIND));
  
end

// FSM ouput logic: Retrieves the content of sram[] for display
always @(posedge clk) begin
  if (~reset_n) data_byte<=8'b0;
  else if (sram_en && P == S_MAIN_FIND) data_byte <= data_out;
end
// End of the FSM of the SD card reader

// -----------------------------------------------------------------------------------


always @(posedge clk) begin
	if(~reset_n) begin
		pre_word <= 0;
		tag <= 0;
		counter1 <= 0;
		find_end <= 0;
		counter2 <= 0;
		the <= 0;
		counter3 <= 0;
	end 
	
	if(!tag) begin 
        if (data_byte == "D") 
		begin 
			counter1 <= 1; 
			pre_word <= "D"; 
		end
        else if ((data_byte == "L") && (counter1 == 1) && pre_word=="D") 
		begin 
			counter1 <= counter1+1; 
			pre_word <= "L"; 
		end
        else if ((data_byte == "A") && (counter1 == 2) && pre_word=="L") 
		begin 
			counter1 <= counter1+1; 
			pre_word <= "A"; 
		end
        else if ((data_byte == "B") && (counter1 == 3) && pre_word=="A") 
		begin 
			counter1 <= counter1+1; 
			pre_word <= "B"; 
		end
        else if ( (data_byte == "_") && (counter1 == 4) && pre_word=="B") 
		begin 
			counter1 <= counter1+1; 
			pre_word <= "_"; 
		end
        else if ((data_byte == "T") && (counter1 == 5) &&  pre_word=="_" ) 
		begin 
			counter1 <= counter1+1; 
			pre_word <= "T"; 
		end
        else if ( (data_byte == "A") && (counter1 == 6) && pre_word=="T") 
		begin 
			counter1 <= counter1+1; 
			pre_word <= "A"; 
		end
        else if ((data_byte == "G") && (counter1 == 7) && pre_word=="A") 
		begin 
			tag <= 1; 
			counter1 <= 0; 
			pre_word <= "G";
		end
        else counter1 <= 0;
	end
	
	if(tag) begin
        if ((counter2 != 7) && data_byte == "D") 
		begin 
			counter2 <= 1; 
			pre_word <= "D"; 
		end
        else if ((data_byte == "L") && (counter2 == 1) && pre_word=="D") 
		begin 
			counter2 <= counter2+1; 
			pre_word <= "L"; 
		end
        else if ((data_byte == "A") && (counter2 == 2) && pre_word=="L") 
		begin 
			counter2 <= counter2+1; 
			pre_word <= "A"; 
		end
        else if ((data_byte == "B") && (counter2 == 3) && pre_word=="A") 
		begin 
			counter2 <= counter2+1; 
			pre_word <= "B"; 
		end
        else if ((data_byte == "_") && (counter2 == 4) && pre_word=="B") 
		begin 
			counter2 <= counter2+1; 
			pre_word <= "_"; 
		end
        else if ( (data_byte == "E") && (counter2 == 5) && pre_word=="_") 
		begin 
			counter2 <= counter2+1; 
			pre_word <= "E"; 
		end
        else if ((data_byte == "N") && (counter2 == 6) && pre_word=="E") 
		begin 
			counter2 <= counter2+1; 
			pre_word <= "N"; 
		end
        else if ((data_byte == "D") && (counter2 == 7) && pre_word=="N") 
		begin 
			counter2 <= 0;
			find_end <= 1; 
			pre_word <= "D"; 
		end
        else counter2 <= 0;		
	end
	
	if(tag && !find_end) begin
        if ((counter3 != 4) && ((data_byte == 10) || (data_byte == 32)|| (data_byte==46))) 
		begin 
			counter3 <= 1; 
		end
        else if ((counter3 == 1) && (data_byte == "t" || data_byte == "T" )) 
		begin 
			counter3 <= counter3+1; 
		end
        else if ((counter3 == 2) && (data_byte == "h" || data_byte == "H")) 
		begin 
			counter3 <= counter3+1; 
		end
        else if ((counter3 == 3) && (data_byte == "e" || data_byte == "E")) 
		begin 
			counter3 <= counter3+1; 
		end
        else if ((counter3 == 4) && ((data_byte == 10) || (data_byte == 32)||(data_byte==46)) 
		begin 
			the <= the + 1; 
			counter3 <= 1; 
		end
        else counter3 <= 0;
	end
end
// -----------------------------------------------------------------------------------

// ------------------------------------------------------------------------
// LCD Display function.
wire [7:0] ten;
wire [7:0] signal;
assign ten = the/10;
assign signal = the - (ten*10);
always @(posedge clk) begin
  if (~reset_n) begin
    row_A = "SD card cannot  ";
    row_B = "be initialized! ";
  end else if(P == S_MAIN_IDLE) begin
    row_A = "Hit BTN2 to read";
    row_B = "the SD card ... ";	
  end else if (P == S_MAIN_FIN) begin
    row_A <= {"Found ", ten+"0", signal+"0", " matches"};
	row_B <= "in the text file";
  end 
  else if(P == S_MAIN_READ) begin
    row_A <= "S_MAIN_READ";
    row_B <= "S_MAIN_READ";
  end
  else if(P == S_MAIN_DONE) begin
    row_A <= "S_MAIN_DONE";
    row_B <= "S_MAIN_DONE";
  end
  else if(P == S_MAIN_SHOW) begin
    row_A <= "S_MAIN_SHOW";
    row_B <= "S_MAIN_SHOW";
  end
end
// End of the LCD display function
// ------------------------------------------------------------------------
endmodule

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Dept. of Computer Science, National Chiao Tung University
// Engineer: Chun-Jen Tsai
// 
// Create Date: 2017/12/06 20:44:08
// Design Name: 
// Module Name: lab9
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: This is a sample circuit to show you how to initialize an SRAM
//              with a pre-defined data file. Hit BTN0/BTN1 let you browse
//              through the data.
// 
// Dependencies: LCD_module, debounce
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module lab9(
  // General system I/O ports
  input  clk,
  input  reset_n,
  input  [3:0] usr_btn,
  output [3:0] usr_led,

  // 1602 LCD Module Interface
  output LCD_RS,
  output LCD_RW,
  output LCD_E,
  output [3:0] LCD_D
);

localparam [2:0] S_MAIN_ADDR = 3'b000, S_MAIN_READ = 3'b001,
                 S_MAIN_SHOW = 3'b010, S_MAIN_WAIT = 3'b011,
				 S_MAIN_C = 3'b100, S_MAIN_FIN = 3'b101,
                 S_MAIN_INIT = 3'b110;

// declare system variables
wire [1:0]        btn_level, btn_pressed;
reg  [1:0]        prev_btn_level;
reg  [2:0]        P, P_next;
reg  [11:0]       sample_addr;
reg  signed [7:0] sample_data;
wire [7:0]        abs_data;

reg  [127:0] row_A, row_B;

// declare SRAM control signals
wire [10:0] sram_addr;
wire [7:0]  data_in;
wire [7:0]  data_out;
wire        sram_we, sram_en;

reg [10:0] i ;
reg signed [7:0] f[0:1023];
reg signed [7:0] g[0:63];
reg [23:0] init_counter;

reg signed [21:0] sum = 0 ,max=0;
reg [10:0] max_pos=0;
reg read_done;
reg done;
reg valid;


//assign usr_led = 4'h00;

assign usr_led[0]=read_done;
assign usr_led[1]=done;
assign usr_led[2]=valid;

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
  
debounce btn_db0(
  .clk(clk),
  .btn_input(usr_btn[0]),
  .btn_output(btn_level[0])
);

debounce btn_db1(
  .clk(clk),
  .btn_input(usr_btn[1]),
  .btn_output(btn_level[1])
);

//
// Enable one cycle of btn_pressed per each button hit
//
always @(posedge clk) begin
  if (~reset_n)
    prev_btn_level <= 2'b00;
  else
    prev_btn_level <= btn_level;
end

assign btn_pressed = (btn_level & ~prev_btn_level);

// ------------------------------------------------------------------------
// The following code describes an initialized SRAM memory block that
// stores an 1024+64 8-bit signed data samples.
sram ram0(.clk(clk), .we(sram_we), .en(sram_en),
          .addr(sram_addr), .data_i(data_in), .data_o(data_out));

assign sram_we = usr_btn[3]; // In this demo, we do not write the SRAM. However,
                             // if you set 'we' to 0, Vivado fails to synthesize
                             // ram0 as a BRAM -- this is a bug in Vivado.
assign sram_en = (P == S_MAIN_ADDR || P == S_MAIN_READ); // Enable the SRAM block.
assign sram_addr = sample_addr[11:0] ;
assign data_in = 8'b0; // SRAM is read-only so we tie inputs to zeros.
// End of the SRAM memory block.
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
// FSM of the main controller
always @(posedge clk) begin
  if (~reset_n) begin
    P <= S_MAIN_INIT; // read samples at 000 first
  end
  else begin
    P <= P_next;
  end
end

always @(posedge clk) begin
    if (P == S_MAIN_INIT) init_counter <= init_counter + 1;
    else init_counter <= 0;
end

always @(*) begin // FSM next-state logic
  case (P)
    S_MAIN_INIT:
      if (init_counter < 5000) P_next <= S_MAIN_INIT;
      else P_next <= S_MAIN_ADDR;
    S_MAIN_ADDR: // send an address to the SRAM 
      P_next <= S_MAIN_READ;
    S_MAIN_READ: // fetch the sample from the SRAM
      P_next <= S_MAIN_SHOW;
    S_MAIN_SHOW:
      P_next <= S_MAIN_WAIT;
    S_MAIN_WAIT: // wait for a button click
      if (sample_addr < 1087) P_next <= S_MAIN_ADDR;
      else P_next <= S_MAIN_C;
    S_MAIN_C:
	  if(btn_pressed && done) P_next=S_MAIN_FIN;
	  else P_next=S_MAIN_C;
	S_MAIN_FIN:
		P_next=S_MAIN_FIN;
  endcase
end

// FSM ouput logic: Fetch the data bus of sram[] for display
always @(posedge clk) begin
  if (~reset_n) sample_data <= 8'b0;
  else if (sram_en && !sram_we) sample_data <= data_out;
end
// End of the main controller
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
// The following code updates the 1602 LCD text messages.
/*always @(posedge clk) begin
  if (~reset_n) begin
    row_A <= "Sample at [---h]";
  end
  else if (P == S_MAIN_SHOW) begin
    row_A[39:32] <= ((sample_addr[11:08] > 9)? "7" : "0") + sample_addr[11:08];
    row_A[31:24] <= ((sample_addr[07:04] > 9)? "7" : "0") + sample_addr[07:04];
    row_A[23:16] <= ((sample_addr[03:00] > 9)? "7" : "0") + sample_addr[03:00];
  end
end



always @(posedge clk) begin
  if (~reset_n) begin
    row_B <= "is equal to ---h";
  end
  else if (P == S_MAIN_SHOW) begin
    row_B[31:24] <= (sample_data[7])? "-" : "+";
    row_B[23:16] <= ((abs_data[7:4] > 9)? "7" : "0") + abs_data[7:4];
    row_B[15: 8] <= ((abs_data[3:0] > 9)? "7" : "0") + abs_data[3:0];
  end
end*/

reg [10:0] cnt=0;
//assign cnt = (btn_pressed[0])? cnt+1 ; */
always @(posedge clk) begin
	if(btn_pressed[0]) cnt<= cnt+1;
end


always @(posedge clk) begin
  if (~reset_n) begin
    row_A <= "Press BTN0 to do";
	row_B <= "x-correlation...";
	//cnt<=0;
  end
  else if (P == S_MAIN_FIN) begin
  
	row_A[127:48] <= "Max value ";
    row_A[47:40] <= ((max[21:20] > 9)? "7" : "0") + max[21:20];
	row_A[39:32] <= ((max[19:16] > 9)? "7" : "0") + max[19:16];
	row_A[31:24] <= ((max[15:12] > 9)? "7" : "0") + max[15:12];
	row_A[23:16] <= ((max[11:8] > 9)? "7" : "0") + max[11:8];
	row_A[15:8] <= ((max[7:4] > 9)? "7" : "0") + max[7:4];
	row_A[7:0] <= ((max[3:0] > 9)? "7" : "0") + max[3:0];
	
	row_B[127:24]<="Max location ";
	row_B[23:16] <= ((max_pos[10:8] > 9) ? "7" : "0" ) + max_pos[10:8];
	//row_B[23:16] <= ((max_pos[10:8] > 9) ? "7" : "9" ) ? + max_pos[10:8];
	row_B[15:8] <= ((max_pos[7:4] > 9) ? "7" : "0" ) + max_pos[7:4];
	row_B[7:0] <= ((max_pos[3:0] > 9) ? "7" : "0" ) + max_pos[3:0];
    end
	else if(P==S_MAIN_WAIT) begin
		row_A<= "aaaa";
		//row_B<= "aaaaaaaa";
        row_B[15:8] <= ((sample_addr[7:4] > 9) ? "7" : "0" ) + sample_addr[7:4];
	row_B[7:0] <= ((sample_addr[3:0] > 9) ? "7" : "0" ) + sample_addr[3:0];
	end
	else if(P==S_MAIN_C) begin
		/*row_A<= "bbbb";
		row_B<= "bbbbbb";*/
		/*row_A<={(((f[0+cnt][7:4] > 9)? "7":"0")+f[0+cnt][7:4]) , (((f[0+cnt][3:0] > 9)? "7":"0")+f[0+cnt][3:0]) , (((f[1+cnt][7:4] > 9)? "7":"0")+f[1+cnt][7:4]), (((f[1+cnt][3:0] > 9)? "7":"0")+f[1+cnt][3:0])} ;
		
		/*row_B<={(((g[0+cnt][7:4] > 9)? "7":"0")+g[0+cnt][7:4]) , (((g[0+cnt][3:0] > 9)? "7":"0")+g[0+cnt][3:0]) , (((g[1+cnt][7:4] > 9)? "7":"0")+g[1+cnt][7:4]), (((g[1+cnt][3:0] > 9)? "7":"0")+g[1+cnt][3:0])} ;*/
		row_A<= "Press BTN0 to do";
		row_B <= "x-correlation...";
		
		
	end
	
	
end
    
	
// End of the 1602 LCD text-updating code.
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
// The circuit block that processes the user's button event.
/*always @(posedge clk) begin
  if (~reset_n)
    sample_addr <= 12'h000;
  else if (btn_pressed[1])
    sample_addr <= (sample_addr < 2048)? sample_addr + 1 : sample_addr;
  else if (btn_pressed[0])
    sample_addr <= (sample_addr > 0)? sample_addr - 1 : sample_addr;
end*/
// End of the user's button control.
// ------------------------------------------------------------------------

always @(posedge clk) begin
  if (~reset_n) begin
    sample_addr <= 12'd0;
  end
  else if ( P_next == S_MAIN_ADDR && P == S_MAIN_WAIT) begin
    sample_addr <= sample_addr + 1;
  end
end

always @(posedge clk) begin
    if(P == S_MAIN_SHOW) begin
        if(sample_addr < 1024) begin
            f[sample_addr] <= sample_data;
        end
        else begin
            g[sample_addr - 1024] <= sample_data;
        end
    end
end
  
reg [10:0] x,k;



always @(posedge clk) begin
	if(~reset_n) begin
		max<=0;
		max_pos<=0;
		x<=0;
		k<=0;
		sum<=0;
		valid<=0;
		done<=0;
	end
	else if(P==S_MAIN_C) begin
		if(x < (1024-64)) begin
			if(k < 64 && (~valid)) begin
				sum= sum + (f[x+k] * g[k]) ;
				k = k+ 1;
				if(k==64) begin 
					valid=1;
					//k=0;
				end
			end
			
			else if(valid) begin
			
				if (sum > max) begin 
					max = sum; 
					max_pos = x;
				end
				sum=0;
				x=x+1;
				k=0;
				valid=0;
                if(x>=(1024-64)) done=1;
			end
		
		end
	
	end
	

end

endmodule

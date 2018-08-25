`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Dept. of CS, National Chiao Tung University
// Engineer: Chun-Jen Tsai
// 
// Create Date: 2017/10/16 14:21:33
// Design Name: 
// Module Name: lab5
// Project Name: 
// Target Devices: Xilinx FPGA @ 100MHz 
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

// total prime is 172
// 10^-8 a cycle -> 7 * 10^ 7
module lab5(
  input clk,
  input reset_n,
  input [3:0] usr_btn,
  output [3:0] usr_led,
  output LCD_RS,
  output LCD_RW,
  output LCD_E,
  output [3:0] LCD_D
);

// turn off all the LEDs
//assign usr_led = 4'b0000;

localparam LIMIT = 1024 ;

wire btn_level, btn_pressed;
reg prev_btn_level;
reg [127:0] row_A = "Press BTN3 to   "; // Initialize the text of the first row. 
reg [127:0] row_B = "show a message.."; // Initialize the text of the second row.
reg [1023:0] primes ;
reg [11:0] idx , idx_next ;
reg [10:0] jdx ,jdx_next;
reg [10:0] kdx ;
reg scroll ;  // scroll direction 0: up  1: down
reg pre_scroll ;
reg [7:0] counter ;
reg [7:0] b_counter ;
reg [9:0] p_cnt ;
reg [26:0] add ;
reg flag ;
reg cal ;
reg done ;
reg init ;
reg [9:0] init_i ;
reg [9:0] all_primes [172:0] ;
reg [7:0] i ;
reg sieve , sieve_next ;


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
  .btn_input(usr_btn[3]),
  .btn_output(btn_level)
);
    
always @(posedge clk) begin
  if (~reset_n)
    prev_btn_level <= 1;
  else
    prev_btn_level <= btn_level;
end

assign btn_pressed = (btn_level == 1 && prev_btn_level == 0);
/*
always @ (posedge clk) begin
	if(~reset_n) begin
		for(idx = 2; idx < LIMIT ; idx = idx + 1) begin
			primes[idx] = 1 ;
		end
	end
	else  begin
		for(idx = 2 ; idx < LIMIT ; idx = idx + 1) begin
			if(primes[idx]) begin
				for(jdx = idx + idx ; jdx < LIMIT ; jdx = idx + jdx) begin
					primes[jdx] = 0 ;
				end
				
			end
		
		end
        cal = 1 ;
	end
end */

initial  begin

	primes = { {1022{1'b1}} ,{2'b00} } ;

end

always @(posedge clk) begin
	if (~reset_n) begin
		idx <= 2;
		jdx <= 2;
		sieve <= 0;
	end
	else begin
		idx <= idx_next;
		jdx <= jdx_next;
		sieve <= sieve_next;
	end

end

always @(posedge clk) begin
	idx_next <= idx;
	jdx_next <= jdx;
	sieve_next <= sieve;
	
	if ((idx + jdx) < LIMIT) begin
		idx_next <= idx + jdx;
		primes[idx + jdx] <= 0;
	end
	else begin
		if (jdx < 500) begin
			jdx_next <= jdx + 1;
			idx_next <= jdx + 1;
		end
		else begin
			sieve_next <= 1;
		end
	end

end
 

/*
always @ (posedge clk) begin
    if(~reset_n) begin
        idx = 2 ;
        //primes = 1024b'1 ;
		init = 0;
		init_i = 2 ;
		jdx = 2 ;
    end
	
    else begin
        if(idx < LIMIT) begin
            if(primes[idx]) begin
                //jdx = jdx + idx ;
                if( jdx < LIMIT) begin
					jdx = idx + idx ;
                    primes[jdx] = 0;
                end
                else begin
					idx = idx + 1;
					jdx = idx;
				end
            end
			
			
//            if(jdx >= LIMIT) begin 
//				idx = idx + 1 ;
	//			jdx = idx ;
		//	end
        end
		else begin  
			cal = 1;
		end
    end

end*/


assign usr_led = primes[5:2] ;
//assign usr_led = {primes[5:3], done} ; 
//assign usr_led = all_primes[1][3:0] ;

always @(posedge clk) begin
  if (~reset_n) begin
    // Initialize the text when the user hit the reset button
    row_A = "Prime #01 is 002";
    row_B = "Prime #02 is 003";
	counter = 1 ;
	b_counter = 2 ;
  end else if (flag && scroll && done) begin
	row_A[127:72] <= "Prime #" ;
	row_A[71:64]  <= (counter[7:4] < 10) ? counter[7:4] + 48 :counter[7:4] + 55;
	row_A[63:56]  <= (counter[3:0] < 10) ? counter[3:0] + 48 :counter[3:0] + 55;
	row_A[55:24]  <= " is " ;
	/*while(primes[p_cnt] == 0)begin
		p_cnt <= p_cnt + 1 ;
	end*/
	row_A[23:16]  <= (all_primes[counter][9:8] < 10) ? all_primes[counter][9:8] + 48 :all_primes[counter][9:8] + 55 ;
	row_A[15:8]   <= (all_primes[counter][7:4] < 10) ? all_primes[counter][7:4] + 48 :all_primes[counter][7:4] + 55 ;
	row_A[7:0]    <= (all_primes[counter][3:0] < 10) ? all_primes[counter][3:0] +48 :all_primes[counter][3:0] +55 ;
	
	//p_cnt <= p_cnt +1 ;
	if(counter == 172) counter <= 1;
	else counter <= counter + 1 ;
	
	//b_counter <= counter + 1 ;
	row_B[127:72] <= "Prime #" ;
	row_B[71:64]  <= (b_counter[7:4] < 10) ? b_counter[7:4] + 48 :b_counter[7:4] + 55;
	row_B[63:56]  <= (b_counter[3:0] < 10) ? b_counter[3:0] + 48 :b_counter[3:0] + 55;
	row_B[55:24]  <= " is " ;
	//while(primes[p_cnt] == 0) p_cnt <= p_cnt + 1 ;
	row_B[23:16]  <= (all_primes[b_counter][9:8] < 10) ? all_primes[b_counter][9:8] + 48 :all_primes[b_counter][9:8] + 55 ;
	row_B[15:8]   <= (all_primes[b_counter][7:4] < 10) ? all_primes[b_counter][7:4] + 48 :all_primes[b_counter][7:4] + 55 ;
	row_B[7:0]    <= (all_primes[b_counter][3:0] < 10) ? all_primes[b_counter][3:0] +48 :all_primes[b_counter][3:0] +55 ;
	
	//b_counter <= 0;
	//counter <= counter + 1 ;
	//p_cnt <= p_cnt - 1;
	if(b_counter == 172) b_counter <= 1 ;
	else b_counter <= b_counter + 1 ;
		
	
	
	end
	
  else if(flag && !scroll && done) begin
	row_A[127:72] <= "Prime #" ;
	row_A[71:64]  <= (counter[7:4] < 10) ? counter[7:4] + 48 :counter[7:4] + 55;
	row_A[63:56]  <= (counter[3:0] < 10) ? counter[3:0] + 48 :counter[3:0] + 55;
	row_A[55:24]  <= " is " ;
	//while(primes[p_cnt] == 0) p_cnt <= p_cnt - 1 ;
	row_A[23:16]  <= (all_primes[counter][9:8] < 10) ? all_primes[counter][9:8] + 48 :all_primes[counter][9:8] + 55 ;
	row_A[15:8]   <= (all_primes[counter][7:4] < 10) ? all_primes[counter][7:4] + 48 :all_primes[counter][7:4] + 55 ;
	row_A[7:0]    <= (all_primes[counter][3:0] < 10) ? all_primes[counter][3:0] +48 :all_primes[counter][3:0] +55 ;
	
	//p_cnt <= p_cnt -1 ;
	if(counter == 1) counter <= 172;
	else counter <= counter - 1 ;
	
	
	b_counter <= counter - 1 ;
	row_B[127:72] <= "Prime #" ;
	row_B[71:64]  <= (b_counter[7:4] < 10) ? b_counter[7:4] + 48 :b_counter[7:4] + 55;
	row_B[63:56]  <= (b_counter[3:0] < 10) ? b_counter[3:0] + 48 :b_counter[3:0] + 55;
	row_B[55:24]  <= " is " ;
	//while(primes[p_cnt] == 0) p_cnt <= p_cnt - 1 ;
	row_B[23:16]  <= (all_primes[b_counter][9:8] < 10) ? all_primes[b_counter][9:8] + 48 :all_primes[b_counter][9:8] + 55 ;
	row_B[15:8]   <= (all_primes[b_counter][7:4] < 10) ? all_primes[b_counter][7:4] + 48 :all_primes[b_counter][7:4] + 55 ;
	row_B[7:0]    <= (all_primes[b_counter][3:0] < 10) ? all_primes[b_counter][3:0] +48 :all_primes[b_counter][3:0] +55 ;
	
	//b_counter <= 0;
	/*counter <= counter - 1 ;
	p_cnt <= p_cnt + 1 ;
	if(counter == 0) begin
		counter <= 172;
		p_cnt <= 1021;
	end*/
	if(b_counter == 1) b_counter <= 172 ;
	else b_counter <= b_counter - 1 ;
	
	
	
   end
    /*row_A <= "Hello, World!   ";
    row_B <= "Demo of the LCD.";*/
  
end

always @(posedge clk) begin
	if(add > 70000000) begin
		add <= 0 ;
		flag <= 1 ;
		
	end	
	else begin
		add <= add + 1 ;
		flag <= 0;
	end 
	
end



always @(posedge clk)  begin
	if(~reset_n) scroll <= 0 ;
	else if(btn_pressed) scroll = !scroll ;
	else scroll <= scroll ;
end

/*
always @(posedge clk) begin
    if(cal) begin
        for(kdx = 2 ; kdx < LIMIT ; kdx = kdx + 1) begin
            if(primes[kdx]) begin
                all_primes[i] = kdx ;
                i = i + 1 ;
            end
        end
    end
    
end*/

always @(posedge clk) begin
    if(~reset_n) begin
        kdx = 2 ;
		i = 1 ;
		done = 0 ;
    end
	else if(sieve) begin
		if(kdx < LIMIT) begin
			if(primes[kdx]) begin
				all_primes[i] = kdx ;
				i = i + 1 ;
			end
			kdx = kdx + 1 ;
		end
		if(i == 173) done = 1; 
	end
    
end



endmodule

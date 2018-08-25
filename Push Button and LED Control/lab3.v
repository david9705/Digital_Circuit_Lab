`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2017/09/26 15:03:31
// Design Name: 
// Module Name: lab3
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


module lab3(
  input  clk,            // System clock at 100 MHz
  input  reset_n,        // System reset signal, in negative logic
  input  [3:0] usr_btn,  // Four user pushbuttons
  output [3:0] usr_led   // Four yellow LEDs
);

	reg [29:0] counter ;
    reg signed[3:0]value ;
	reg signed[3:0]value_tmp;
	reg [3:0] light_tmp = 4'b1111;
    wire bounce ;
    wire stability ;
    reg [20:0] cnt ;
    integer light = 5 ;
	integer i ;
    
    always@(posedge clk) begin
        cnt <= cnt + 1;
        if(cnt > 1000000)begin
            cnt <= 0;
        end
    end
    
    assign bounce = usr_btn[0] | usr_btn[1] | usr_btn[2] | usr_btn[3];
    //deal with counter;
    always@(posedge clk ,negedge bounce)
    begin
      if(!bounce) 
          counter <= 0 ;
      else if(counter <= 987988)
          counter <= counter + 1 ;
    end
    
   //when counter is all 1 , stability go 1 ;
    assign stability = counter == 987987 ;
    //deal with debouncing and increment
    always@(posedge clk )  begin
      if(!reset_n) begin
        value <= 0 ;
        value_tmp <= 0 ;
        end
      else if(usr_btn[0] && stability)
        begin
          if(value > -8)  begin
            value <= value - 1;
            value_tmp <= value_tmp - 1 ;
          end
          else begin
            value <= value ;
            value_tmp <= value_tmp ;
           end
			
		  //value_tmp <= value ; 
         end
      else if(usr_btn[1] && stability)
        begin
           if(value < 7) begin
            value <= value + 1;
            value_tmp <= value_tmp + 1 ;
            end
           else  begin
            value <= value ;
            value_tmp <= value_tmp ;
            end
		  //value_tmp <= value ;
        end
        
       else if(usr_btn[2] && stability) begin
       
            if(light == 1) begin
                light <= light;
            end
            else  begin
                light <= light - 1 ;
            end
            
 
       end
       
       else if(usr_btn[3] && stability) begin
       
       
            if(light == 5) begin
                light <= light;
            end
            else  begin
                light <= light + 1 ;
            end
        end
        
        
        case(light)
            1:begin
                if(cnt <= 50000) begin
					//value <= value_tmp ;
					light_tmp <= 4'b1111 ;
				end
				else  begin
					light_tmp <= 0 ;
					
				end
                
            end
                
            2:begin
                if(cnt <= 250000) begin
                    //value <= value_tmp ;
					light_tmp <= 4'b1111 ;
				end
				else  begin
					light_tmp <= 0 ;
				end
                
            end
                
            3:begin
                if(cnt <= 500000) begin
                    //value <= value_tmp ;
					light_tmp <= 4'b1111 ;
				end
				else  begin
					light_tmp <= 0 ;
				end
            end
                
            4:begin
                if(cnt <= 750000) begin
                    light_tmp <= 4'b1111 ;
                end
				else  begin
					light_tmp <= 0 ;
				end
                     
            end
                 
            5:begin
                if(cnt <= 1000000) begin
                    light_tmp <= 4'b1111 ;
				end
				else  begin
					light_tmp <= 0 ;
				end
                 
            end
                
                
            
        endcase
      
    end
    
    

    
    
assign usr_led = value & light_tmp;

endmodule

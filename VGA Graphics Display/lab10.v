`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Dept. of Computer Science, National Chiao Tung University
// Engineer: Chun-Jen Tsai
// 
// Create Date: 2017/08/25 14:29:54
// Design Name: 
// Module Name: lab10
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: A circuit that show the animation of a moon moving across a city
//              night view on a screen through the VGA interface of Arty I/O card.
// 
// Dependencies: vga_sync, clk_divider, sram
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module lab10(
    input  clk,
    input  reset_n,
    input  [3:0] usr_btn,
    output [3:0] usr_led,

    // VGA specific I/O ports
    output VGA_HSYNC,
    output VGA_VSYNC,
    output [3:0] VGA_RED,
    output [3:0] VGA_GREEN,
    output [3:0] VGA_BLUE
    );



// Declare system variables
reg  [33:0] moon_clock;
wire [9:0]  pos;
wire        moon_region;
wire        firework_region;

// declare SRAM control signals
wire [16:0] sram_addr;
wire [11:0] data_in;
wire [11:0] data_out;
wire        sram_we, sram_en;

// General VGA control signals
wire vga_clk;       // 50MHz clock for VGA control
wire video_on;      // when video_on is 0, the VGA controller is sending
                    // synchronization signals to the display device.
  
wire pixel_tick;    // when pixel tick is 1, we must update the RGB value
                    // based for the new coordinate (pixel_x, pixel_y)
  
wire [9:0] pixel_x; // x coordinate of the next pixel (between 0 ~ 639) 
wire [9:0] pixel_y; // y coordinate of the next pixel (between 0 ~ 479)
  
reg  [11:0] rgb_reg;  // RGB value for the current pixel
reg  [11:0] rgb_next; // RGB value for the next pixel
  
// Application-specific VGA signals
reg  [16:0] pixel_addr;
reg  [1:0] frames;
reg  [26:0] firework_clock;

// Declare the video buffer size
localparam VBUF_W = 320; // video buffer width
localparam VBUF_H = 240; // video buffer height
  
// Instiantiate a VGA sync signal generator
vga_sync vs0(
  .clk(vga_clk), .reset(~reset_n), .oHS(VGA_HSYNC), .oVS(VGA_VSYNC),
  .visible(video_on), .p_tick(pixel_tick),
  .pixel_x(pixel_x), .pixel_y(pixel_y)
);

clk_divider#(2) clk_divider0(
  .clk(clk),
  .reset(~reset_n),
  .clk_out(vga_clk)
);

// ------------------------------------------------------------------------
// The following code describes an initialized SRAM memory block that
// stores an 320x240 12-bit city image, plus a 64x40 moon image.
sram #(.DATA_WIDTH(12), .ADDR_WIDTH(17), .RAM_SIZE(VBUF_W*VBUF_H+64*40+3*80*80))
  ram0 (.clk(clk), .we(sram_we), .en(sram_en),
          .addr(sram_addr), .data_i(data_in), .data_o(data_out));

assign sram_we = usr_btn[3]; // In this demo, we do not write the SRAM. However,
                             // if you set 'we' to 0, Vivado fails to synthesize
                             // ram0 as a BRAM -- this is a bug in Vivado.
assign sram_en = 1;          // Here, we always enable the SRAM block.
assign sram_addr = pixel_addr;
assign data_in = 12'h000; // SRAM is read-only so we tie inputs to zeros.
// End of the SRAM memory block.
// ------------------------------------------------------------------------

// VGA color pixel generator
assign {VGA_RED, VGA_GREEN, VGA_BLUE} = rgb_reg;

// ------------------------------------------------------------------------
// An animation clock for the motion of the moon, upper bits of the
// moon clock is the x position of the moon in the VGA screen
assign pos = moon_clock[33:24];

//fire work clock
always @(posedge clk) begin
  if (~reset_n && frames == 3)  // &&  ??
    frames <= 0 ;
  else
	firework_clock <= firework_clock + 1;
	if(firework_clock == 50000000) begin
		frames <= frames + 1;
		firework_clock <= 0;
	end
 
end

always @(posedge clk) begin
  if (~reset_n || moon_clock[33:25] > VBUF_W + 64)
    moon_clock <= 0;
  else
    moon_clock <= moon_clock + 1;
end
// End of the animation clock code.
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
// Video frame buffer address generation unit (AGU) with scaling control
// Note that the width x height of the moon image is 64x40, when scaled
// up to the screen, it becomes 128x80
assign moon_region = pixel_y >= 0 && pixel_y < 80 &&
                     (pixel_x + 127) >= pos && pixel_x < pos + 1;
assign firework_region = pixel_y >= 100 && pixel_y < 180 && pixel_x < 380 && pixel_x >=300; 



always @ (*) begin
  if (~reset_n) begin
    pixel_addr <= 0;
  end
  else if (moon_region && (data_out[11:0] != 12'h0F0)) begin
    pixel_addr <= VBUF_W*VBUF_H + ((pixel_y&10'h2FE)<<5) + ((pixel_x-pos+127)>>1);
  end
  else if(firework_region && (data_out[11:0] != 12'h000)) begin
	pixel_addr <= VBUF_W*VBUF_H + 64*40 + 80*80*frames + (pixel_y - 100)*80 + (pixel_x - 300);
  end
  else begin
    // Scale up a 320x240 image for the 640x480 display.
    // (pixel_x, pixel_y) ranges from (0,0) to (639, 379)
    pixel_addr <= (pixel_y >> 1) * VBUF_W + (pixel_x >> 1);
  end
end
// End of the AGU code.
// ------------------------------------------------------------------------


	
// ------------------------------------------------------------------------
// Send the video data in the sram to the VGA controller
always @(posedge clk) begin
  if (pixel_tick) rgb_reg <= rgb_next;
end

always @(posedge clk) begin
  if (~video_on) begin
    rgb_next = 12'h000; // Synchronization period, must set RGB values to zero.
  end
  else begin
    rgb_next = data_out; // RGB value at (pixel_x, pixel_y)
  end
end
// End of the video data display code.
// ------------------------------------------------------------------------

endmodule


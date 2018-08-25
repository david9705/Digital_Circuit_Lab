module md5(
 
  
  //input [31:0] start,
  //input [31:0] fin,
  //input [127:0] msg_i,
  input  clk,
  input  reset_n,
  
  
  //output [31:0] ans,
 //output find,
  
  input  [3:0] usr_btn,
  output [3:0] usr_led,

  

  // 1602 LCD Module Interface
  output LCD_RS,
  output LCD_RW,
  output LCD_E,
  output [3:0] LCD_D
  
  
 
);

wire btn_level, btn_pressed;
reg prev_btn_level;
   
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


localparam r_size = 64;
localparam k_size = 48;
reg [31:0] r[0:r_size-1];
reg [31:0] k[0:k_size-1];
reg [31:0] h0=32'h67452301, h1=32'hefcdab89, h2=32'h98badcfe, h3=32'h10325476 ;
reg [31:0] a,b,c,d,f,g,temp,x,y;
reg [5:0] pad_len=56;
reg [5:0] offset=0;
reg [6:0] i=0; 
reg valid=0;
// ??„æ?‰w??„æ?’å®£???
reg [7:0] msg[0:(56+64-1)];
reg [31:0] num;  // now is which number //from start to end
reg [3:0] len,num_cnt;
reg [31:0] bits_len;
reg len_done;
reg assign_done;
reg pre;
reg find_i;
reg msg_done;
reg go;

reg  [127:0] row_A = "SD card cannot  ";
reg  [127:0] row_B = "be initialized! ";

assign usr_led[0]=msg_done;
assign usr_led[1]=assign_done;
assign usr_led[2]=valid;
assign usr_led[3]=go;

reg [127:0] msg_i = 128'hE9982EC5CA981BD365603623CF4B2277;
reg [31:0] fin=4000000;


initial begin
	{ r[ 0], r[ 1], r[ 2], r[ 3], r[ 4], r[ 5], r[ 6], r[ 7],
	  r[ 8], r[ 9], r[10], r[11], r[12], r[13], r[14], r[15],
	  r[16], r[17], r[18], r[19], r[20], r[21], r[22], r[23],
	  r[24], r[25], r[26], r[27], r[28], r[29], r[30], r[31],
	  r[32], r[33], r[34], r[35], r[36], r[37], r[38], r[39],
	  r[40], r[41], r[42], r[43], r[44], r[45], r[46], r[47],
	  r[48], r[49], r[50], r[51], r[52], r[53], r[54], r[55],
	  r[56], r[57], r[58], r[59], r[60], r[61], r[62], r[63]}
	  <= {7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22,
		  5,  9, 14, 20, 5,  9, 14, 20, 5,  9, 14, 20, 5,  9, 14, 20,
		  4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23,
		  6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21};
	
	{ k[ 0], k[ 1], k[ 2], k[ 3], k[ 4], k[ 5], k[ 6], k[ 7], k[ 8], k[ 9], k[10],
	  k[11], k[12], k[13], k[14], k[15], k[16], k[17], k[18], k[19], k[20], k[21],
	  k[22], k[23], k[24], k[25], k[26], k[27], k[28], k[29], k[30], k[31], k[32],
	  k[33], k[34], k[35], k[36], k[37], k[38], k[39], k[40], k[41], k[42], k[43],
	  k[44], k[45], k[46], k[47]}
	  <={ 32'hd76aa478, 32'he8c7b756, 32'h242070db, 32'hc1bdceee,
    32'hf57c0faf, 32'h4787c62a, 32'ha8304613, 32'hfd469501,
    32'h698098d8, 32'h8b44f7af, 32'hffff5bb1, 32'h895cd7be,
    32'h6b901122, 32'hfd987193, 32'ha679438e, 32'h49b40821,
    32'hf61e2562, 32'hc040b340, 32'h265e5a51, 32'he9b6c7aa,
    32'hd62f105d, 32'h02441453, 32'hd8a1e681, 32'he7d3fbc8,
    32'h21e1cde6, 32'hc33707d6, 32'hf4d50d87, 32'h455a14ed,
    32'ha9e3e905, 32'hfcefa3f8, 32'h676f02d9, 32'h8d2a4c8a,
    32'hfffa3942, 32'h8771f681, 32'h6d9d6122, 32'hfde5380c,
    32'ha4beea44, 32'h4bdecfa9, 32'hf6bb4b60, 32'hbebfbc70,
    32'h289b7ec6, 32'heaa127fa, 32'hd4ef3085, 32'h04881d05,
    32'hd9d4d039, 32'he6db99e5, 32'h1fa27cf8, 32'hc4ac5665,
    32'hf4292244, 32'h432aff97, 32'hab9423a7, 32'hfc93a039,
    32'h655b59c3, 32'h8f0ccc92, 32'hffeff47d, 32'h85845dd1,
    32'h6fa87e4f, 32'hfe2ce6e0, 32'ha3014314, 32'h4e0811a1,
    32'hf7537e82, 32'hbd3af235, 32'h2ad7d2bb, 32'heb86d391};
		  
end

always @(posedge clk) begin
	if(~reset_n) begin
		offset<=0;
		a<=h0;
		b<=h1;
		c<=h2;
		d<=h3;
		i<=0;
		valid<=0;
		temp<=0;
		x<=0;
		y<=0;
		find_i<=0;
		
		// deal with msg
		
		//msg<=0;
        msg[0]<=0; msg[1]<=0; msg[2]<=0; msg[3]<=0; msg[4]<=0; msg[5]<=0; msg[6]<=0; msg[7]<=0; msg[8]<=0; msg[9]<=0; msg[10]<=0; msg[11]<=0;
        msg[12]<=0; msg[13]<=0; msg[14]<=0; msg[15]<=0; msg[16]<=0; msg[17]<=0; msg[18]<=0; msg[19]<=0; msg[20]<=0; msg[21]<=0; msg[22]<=0; msg[23]<=0;
        msg[24]<=0; msg[25]<=0; msg[26]<=0; msg[27]<=0; msg[28]<=0; msg[29]<=0; msg[30]<=0; msg[31]<=0; msg[32]<=0; msg[33]<=0; msg[34]<=0; msg[35]<=0;
        msg[36]<=0; msg[37]<=0; msg[38]<=0; msg[39]<=0; msg[40]<=0; msg[41]<=0; msg[42]<=0; msg[43]<=0; msg[44]<=0; msg[45]<=0; msg[46]<=0; msg[47]<=0;
        msg[48]<=0; msg[49]<=0; msg[50]<=0; msg[51]<=0; msg[52]<=0; msg[53]<=0; msg[54]<=0; msg[55]<=0; msg[56]<=0; msg[57]<=0; msg[58]<=0; msg[59]<=0;
        msg[60]<=0; msg[61]<=0; msg[62]<=0; msg[63]<=0; msg[64]<=0; msg[65]<=0; msg[66]<=0; msg[67]<=0; msg[68]<=0; msg[69]<=0; msg[70]<=0; msg[71]<=0;
        msg[72]<=0; msg[73]<=0; msg[74]<=0; msg[75]<=0; msg[76]<=0; msg[77]<=0; msg[78]<=0; msg[79]<=0; msg[80]<=0; msg[81]<=0; msg[82]<=0; msg[83]<=0;
        msg[84]<=0; msg[85]<=0; msg[86]<=0; msg[87]<=0; msg[88]<=0; msg[89]<=0; msg[90]<=0; msg[91]<=0; msg[92]<=0; msg[93]<=0; msg[94]<=0; msg[95]<=0;
        msg[96]<=0; msg[97]<=0; msg[98]<=0; msg[99]<=0; msg[100]<=0; msg[101]<=0; msg[102]<=0; msg[103]<=0; msg[104]<=0; msg[105]<=0; msg[106]<=0; msg[107]<=0;
        msg[108]<=0; msg[109]<=0; msg[110]<=0; msg[111]=0; msg[112]<=0; msg[113]<=0; msg[114]<=0; msg[115]<=0; msg[116]<=0; msg[117]<=0; msg[118]<=0; msg[119]<=0;
		//num<=0;
		len<=0;
		len_done<=0;
		assign_done<=0;
		num_cnt<=0;
		msg_done<=0;
        num<=31415926;
        go<=0;
	end
	else if(~msg_done)begin  // deal with msg
		if((~len_done) & (~assign_done))begin
			
			if(num<10) len=1;
			else if(num<100) len=2;
			else if(num<1000) len=3;
			else if(num<10000) len=4;
			else if(num<100000) len=5;
			else if(num<1000000) len=6;
			else if(num<10000000) len=7;
			else if(num<100000000) len=8;
			len_done=1;
		end
		else if(len_done && ~assign_done) begin  // do memcpy
			msg[num_cnt]= (num%10);
			num=num/10;
			num_cnt=num_cnt+1;
			if(num==0) assign_done=1;
		end
		else if(len_done && assign_done) begin
		
			bits_len=len*8;
			msg[len]=128;
			msg[56]=bits_len[7:0];
			msg[57]=bits_len[15:8];
			msg[58]=bits_len[23:16];
			msg[59]=bits_len[31:24];
			msg_done=1;
			//pre =1;
		end
	end
	
	
	else if(msg_done)begin
		if((~find_i) && (num<fin)) begin
			if(~valid) begin
				a=h0;
				b=h1;
				c=h2;
				d=h3;
				valid=1;
                go=1;
			end
			
			if(i<64 && valid && go) begin  // enter second for 
               
				if(i<16) begin
					f = (b & c) | ((~b) & d);
					g = i;
                    //go = 0;
				end
				else if(i<32) begin
					f = (d & b) | ((~d) & c);
					g = (5*i + 1) % 16; 
                    //go = 0;
				end
				else if(i<48) begin
					f = b ^ c ^ d;
					g = (3*i + 5) % 16;
                     //go = 0;
				
				end
				else begin
					f = c ^ (b | (~d));
					g = (7*i) % 16;
                     //go = 0;
				end
				
				temp = d;
				d = c ;
				c = b ;
				x=a+f+k[i]+msg[g];
				y=(((x) << (r[i])) | ((x) >> (32 - (r[i]))));
				b = b + y;  // left rotate
				a = temp ;
                 go = 0;
				
				
				/*if(btn_pressed) begin 
                    i = i + 1;
                    //go=1;
                   end*/
				if(i==64) valid=0;
				
			end  //end second for
            
            if(btn_pressed) begin 
                    i = i + 1;
                    go=1;
            end
			
			if(~valid) begin
				h0=h0+a;
				h1=h1+b;
				h2=h2+c;
				h3=h3+d;
				
				if(msg_i == {h0,h1,h2,h3}) begin
					find_i=1;
					//ans = num;
					//pre=1;
					
				end
				else begin
					num = num + 1;
					msg_done=0;
					//msg=0;
					num=0;
					len=0;
					len_done=0;
					assign_done=0;
					num_cnt=0;
					//pre<=0;
                    msg[0]=0; msg[1]=0; msg[2]=0; msg[3]=0; msg[4]=0; msg[5]=0; msg[6]=0; msg[7]=0; msg[8]=0; msg[9]=0; msg[10]=0; msg[11]=0;
                    msg[12]=0; msg[13]=0; msg[14]=0; msg[15]=0; msg[16]=0; msg[17]=0; msg[18]=0; msg[19]=0; msg[20]=0; msg[21]=0; msg[22]=0; msg[23]=0;
                    msg[24]=0; msg[25]=0; msg[26]=0; msg[27]=0; msg[28]=0; msg[29]=0; msg[30]=0; msg[31]=0; msg[32]=0; msg[33]=0; msg[34]=0; msg[35]=0;
                    msg[36]=0; msg[37]=0; msg[38]=0; msg[39]=0; msg[40]=0; msg[41]=0; msg[42]=0; msg[43]=0; msg[44]=0; msg[45]=0; msg[46]=0; msg[47]=0;
                    msg[48]=0; msg[49]=0; msg[50]=0; msg[51]=0; msg[52]=0; msg[53]=0; msg[54]=0; msg[55]=0; msg[56]=0; msg[57]=0; msg[58]=0; msg[59]=0;
                    msg[60]=0; msg[61]=0; msg[62]=0; msg[63]=0; msg[64]=0; msg[65]=0; msg[66]=0; msg[67]=0; msg[68]=0; msg[69]=0; msg[70]=0; msg[71]=0;
                    msg[72]=0; msg[73]=0; msg[74]=0; msg[75]=0; msg[76]=0; msg[77]=0; msg[78]=0; msg[79]=0; msg[80]=0; msg[81]=0; msg[82]=0; msg[83]=0;
                    msg[84]=0; msg[85]=0; msg[86]=0; msg[87]=0; msg[88]=0; msg[89]=0; msg[90]=0; msg[91]=0; msg[92]=0; msg[93]=0; msg[94]=0; msg[95]=0;
                    msg[96]=0; msg[97]=0; msg[98]=0; msg[99]=0; msg[100]=0; msg[101]=0; msg[102]=0; msg[103]=0; msg[104]=0; msg[105]=0; msg[106]=0; msg[107]=0;
                    msg[108]=0; msg[109]=0; msg[110]=0; msg[111]=0; msg[112]=0; msg[113]=0; msg[114]=0; msg[115]=0; msg[116]=0; msg[117]=0; msg[118]=0; msg[119]=0;
              
				end
			
			end
		
		end 
	end
	

end 

//assign find=(find_i)? 1:0 ;
//assign ans=(find_i)? num:0;

always @(posedge clk) begin
  if (~reset_n) begin
    row_A = "SD card cannot  ";
    row_B = "be initialized! ";
  end 
  else if(find_i) begin
    row_A = "AAAAA" ;
    row_B = "fuck you  ";
  end
  
  else if(msg_done) begin
    row_A =/* (i>9)? i+"7" : i +"0" } */{hex0,hex1} ;
    row_B = {hex2,hex3};
  end
  
  else if(assign_done) begin
    row_A = "48654" ;
    row_B = "sewtg  ";
  end
  else if(len_done) begin
    row_A = "qqqq" ;
    row_B = "77777  ";
  end
  
end

wire [8*8-1:0]hex0,hex1,hex2,hex3 ;

BinToHex s0(.A(a), .B(hex0));
BinToHex s1(.A(b), .B(hex1));
BinToHex s2(.A(c), .B(hex2));
BinToHex s3(.A(d), .B(hex3));
  
  
endmodule

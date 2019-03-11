//8255A PPI chip with BSR Mode and I/O mode zero

//---------------------------------PORTA------------------------------------------//
module porta (mode, inout_port, data_in, data_out);
output [7 : 0] data_out;
input [7 : 0] data_in;
input mode;
inout [7 : 0] inout_port;
assign data_out = (mode == 1) ? inout_port : 8'bzzzzzzzz; // input to the chip from the external peripheral 
assign inout_port = (mode == 0) ? data_in : 8'bzzzzzzzz; // output from the chip to the external peripheral
endmodule

//---------------------------------PORTB------------------------------------------//
module portb (mode, inout_port, data_in, data_out);
output [7 : 0] data_out;
input [7 : 0] data_in;
input mode;
inout [7 : 0] inout_port;
assign data_out = (mode == 1) ? inout_port : 8'bzzzz_zzzz; // input to the chip from the external peripheral 
assign inout_port = (mode == 0) ? data_in : 8'bzzzz_zzzz; // output from the chip to the external peripheral
endmodule

//---------------------------------PORTC------------------------------------------//

//in portc i assigned bit by bit (so i can use portc as 8-bit or 2 4-bit) 
module portc (mode, inout_port, data_in, data_out);
output [7 : 0] data_out;
input [7 : 0] data_in;
input mode;
inout [7 : 0] inout_port;
assign data_out = (mode == 1) ? inout_port : 8'bzzzzzzzz; // input to the chip from the external peripheral 
assign inout_port = (mode == 0) ? data_in : 8'bzzzzzzzz; // output from the chip to the external peripheral
endmodule

//---------------------------------GROUPA------------------------------------------//
module groupa (inout_port_a, inout_port_c, change, cw, data_in ,data_out);
output wire [7:0] data_out;
input [7:0] data_in;
inout [7:0] inout_port_a; //external port 
inout [7:0] inout_port_c; // external port
input [3:0] cw;  //control word
input change;  // indicator that any change will cause a transaction (3shan el-always block bimonitor ay change fa excute ely inside it)

wire [7:0] porta_data_in;
reg [7:0] porta_data_out;
wire [7:0] portc_data_in;
reg [7:0] portc_data_out;

reg mode_a; //whether it's input or output
reg mode_c; // control each bit in portc whether it's input or output 
//reg a_flag; // flag if we read from porta
//reg c_flag; // flag if we read from portc
reg one_or_zero;
reg [7 : 0] bit_in_portc; // holds 4 bits from portc and through this variable we can set or reset any bit
//reg [7 : 0] a_buffer; // for porta because an error occured when i uesd data_out directly(line 110)
//reg [3 : 0] c_buffer;
reg [7 : 0] buffer;
reg in;
reg  temp;
porta my_porta (mode_a, inout_port_a, porta_data_out, porta_data_in);
portc my_portc (mode_c, inout_port_c, portc_data_out, portc_data_in);
/*
assign data_out = (a_flag) ? porta_data_in : a_buffer;
assign data_out = (c_flag) ? portc_data_in : c_buffer;*/
assign inout_port_c = (mode_c) ? bit_in_portc : 8'hzz;
assign data_out = (in == 1'b1) ? buffer : 8'hzz;

always @ (*)
begin
	casez (cw)
		4'b0001 : 
			begin
				//a_flag <= 0; c_flag <= 0;
				//-----------BSR MODE-------------//
				//---PORTC ONLY
				if(data_in[7] == 0)
				begin
					if(data_in[0] == 1) //set bits in portc
					begin one_or_zero = 1;/*input*/ end
					else begin one_or_zero = 0;/*output*/ end
					casez (data_in[3 : 1])
						3'b000 : begin bit_in_portc[0] = one_or_zero; bit_in_portc[7 : 1] = 0; mode_c = 1; end
						3'b001 : begin bit_in_portc[1] = one_or_zero; bit_in_portc[7 : 2] = 0; bit_in_portc[0] = 0; mode_c = 1; end
						3'b010 : begin bit_in_portc[2] = one_or_zero; bit_in_portc[7 : 3] = 0; bit_in_portc[1 : 0] = 0; mode_c = 1; end
						3'b011 : begin bit_in_portc[3] = one_or_zero; bit_in_portc[7 : 4] = 0; bit_in_portc[2 : 0] = 0; mode_c = 1; end
						3'b100 : begin bit_in_portc[4] = one_or_zero; bit_in_portc[7 : 5] = 0; bit_in_portc[3 : 0] = 0; mode_c = 1; end
						3'b101 : begin bit_in_portc[5] = one_or_zero; bit_in_portc[7 : 6] = 0; bit_in_portc[4 : 0] = 0; mode_c = 1; end
						3'b110 : begin bit_in_portc[6] = one_or_zero; bit_in_portc[7] = 0; bit_in_portc[5 : 0] = 0; mode_c = 1; end
						3'b111 : begin bit_in_portc[7] = one_or_zero; bit_in_portc[6 : 0] = 0; mode_c = 1; end
						default : bit_in_portc = 0;
					endcase
				end
				//---------I/O MODE-----------//
				else
				begin
					// Mode 0
					if (data_in[6 : 5] == 2'b00)begin mode_a = data_in[4]; mode_c = data_in[3]; end

				end
			end
		4'b0010 : //CPU writes to porta out to the external device
			begin porta_data_out = data_in; mode_a = 0; in = 0; end
		4'b0011 : //CPU reads from porta from the external device
			begin assign buffer = porta_data_in; mode_a = 1; in = 1;end // replace data_out with a buffer because there is an error
		4'b0100 : //CPU writes to portc out to the external device
			begin portc_data_out = data_in[7 : 4]; mode_c = 0; in = 0; end
		4'b0101 : //CPU reads from porta from the external device
			begin assign buffer = portc_data_in; mode_c = 1; in = 1; end
	endcase
end
endmodule

//---------------------------------GROUPB------------------------------------------//
module groupb (inout_port_b, inout_port_c, change, cw, data_in ,data_out);
output [7:0] data_out;
input [7:0] data_in;
inout [7:0] inout_port_b; //external port 
inout [7:0] inout_port_c; // external port
input [3:0] cw;  //control word
input change;  // indicator that any change will cause a transaction (3shan el-always block bimonitor ay change fa excute ely inside it)

wire [7:0] portb_data_in;
reg [7:0] portb_data_out;
wire [7:0] portc_data_in;
reg [7:0] portc_data_out;
reg mode_b; //whether it's input or output
reg mode_c; // control each bit in portc whether it's input or output 
//reg b_flag; // flag if porta has an input
//reg c_flag; // flag if portc has an input
reg one_or_zero;
reg [7 : 0] bit_in_portc; // holds 4 bits from portc and through this variable we can set or reset any bit
//reg [7 : 0] b_buffer; // for porta because an error occured when i uesd data_out directly(line 110)
//reg [3 : 0] c_buffer;
reg [7 : 0] buffer;
reg in;
portb my_portb (mode_b, inout_port_b, portb_data_out, portb_data_in);
//portc my_portc (mode_c, inout_port_c, portc_data_out, portc_data_in);
/*
assign data_out = (b_flag) ? portb_data_in : b_buffer;
assign data_out = (c_flag) ? portc_data_in : c_buffer;*/
assign data_out = (in) ? buffer : 8'hzz; 


always @ (*)
begin
	casez (cw)
		4'b0001 : 
			begin
				//b_flag <= 0; c_flag <= 0;
				//-----------BSR MODE-------------//
				//---PORTC ONLY
				if(data_in[7] == 0)
				begin
					if(data_in[0] == 1) //set bits in portc
					begin one_or_zero = 1; end
					else begin one_or_zero = 0; end
					casez (data_in[3 : 1])
						3'b000 : begin bit_in_portc[0] = one_or_zero; bit_in_portc[7 : 1] = 0; mode_c = 1; end
						3'b001 : begin bit_in_portc[1] = one_or_zero; bit_in_portc[7 : 2] = 0; bit_in_portc[0] = 0; mode_c = 1; end
						3'b010 : begin bit_in_portc[2] = one_or_zero; bit_in_portc[7 : 3] = 0; bit_in_portc[1 : 0] = 0; mode_c = 1; end
						3'b011 : begin bit_in_portc[3] = one_or_zero; bit_in_portc[7 : 4] = 0; bit_in_portc[2 : 0] = 0; mode_c = 1; end
						3'b100 : begin bit_in_portc[4] = one_or_zero; bit_in_portc[7 : 5] = 0; bit_in_portc[3 : 0] = 0; mode_c = 1; end
						3'b101 : begin bit_in_portc[5] = one_or_zero; bit_in_portc[7 : 6] = 0; bit_in_portc[4 : 0] = 0; mode_c = 1; end
						3'b110 : begin bit_in_portc[6] = one_or_zero; bit_in_portc[7] = 0; bit_in_portc[5 : 0] = 0; mode_c = 1; end
						3'b111 : begin bit_in_portc[7] = one_or_zero; bit_in_portc[6 : 0] = 0; mode_c = 1; end
						default : bit_in_portc = 0;
					endcase
				end
				//---------I/O MODE-----------//
				else
				begin // mode 0
					if (data_in[2] == 0) begin mode_b = data_in[1]; mode_c = data_in[0]; end
				end
			end
		4'b0010 : //CPU writes to portb out to the external device
			begin portb_data_out = data_in; mode_b = 0; in = 0; end
		4'b0011 : //CPU reads from portb from the external device
			begin assign buffer = portb_data_in; mode_b = 1; in = 1; end // replace data_out with a buffer because there is an error
		4'b0100 : //CPU writes to portc out to the external device
			begin portc_data_out = data_in[7 : 4]; mode_c = 0; in = 0; end
		4'b0101 : //CPU reads from portc from the external device
			begin assign buffer = portc_data_in; mode_c = 1; in = 1; end
	endcase
end
endmodule

//---------------------------------8255 PPI------------------------------------------//
module PPI_8255 (reset, cs, rd, wr, a0_a1, data, pa, pb, pcu, pcl);
input reset, cs, rd, wr;
input [1 : 0] a0_a1;
inout [7 : 0] data;
inout [7 : 0] pa, pb;
inout [3 : 0] pcu, pcl;
reg [7 : 0] latch_out; // for latched output
wire [7 : 0] buffer_in_a; // for buffered input (not latched)
wire [7 : 0] buffer_in_b; // for buffered input (not latched)
reg [7 : 0] mode; // buffer for the data
reg [7 : 0] data_in;
reg [3 : 0] cw_a; // control word for porta 
reg [3 : 0] cw_b; // control word for portc
reg read; // if read is set to one means that data from external device (buffer_in) is ready to be put in the data that will be read by the CPU
reg change;

assign data = (read) ? ( (cw_a == 3) ? buffer_in_a :( (cw_b == 3) ? buffer_in_b : 8'hzz)): 8'hzz;


groupa group_a(pa, {pcu, pcl}, change, cw_a, latch_out ,buffer_in_a);
groupb group_b(pb, {pcu, pcl}, change, cw_b, latch_out ,buffer_in_b);
//-------------reset---------------//
always @ (posedge reset)
begin
data_in <= 8'b10011011; // initialize all ports as input
read <= 0; // so that data is cleared
end

//------------write (wr)----------//
always @ (*)
begin
if (wr == 0)
begin
	if(cs == 0) // if chip is selected
	begin
		casez(a0_a1)
			//-----------write to control register-----------//
			2'b11 :
			begin
				if (data[7] == 0) begin cw_a <= 1; latch_out = data; end
				else begin /*data_in= data;*/ end
			end
			//-----------------write to porta---------------//
			2'b00 :
			begin
				cw_a <= 4'b0010;
				latch_out <= data;
				//change <= 1;
			end
			//-----------------write to portb---------------//
			2'b01 :
			begin
				cw_b <= 4'b0010;
				latch_out <= data;
				//change <= 1;
			end
			//-----------------write to portc---------------//
			2'b10 :
			begin
				cw_a <= 4'b0100; // fill in upper c
				cw_b <= 4'b0100; // fill in lower c
				latch_out <= data;
				//change <= 1;
			end 
		endcase
	end
end
if (rd == 0)
begin
	if (cs == 0) // if chip is selected
	begin
		casez(a0_a1)
			
			//-----------------read from porta---------------//
			2'b00 :
			begin
				cw_a <= 4'b0011;
				read <= 1; //data is given a value up in line 216
				//change <= 1;
			end
			//-----------------read from portb---------------//
			2'b01 :
			begin
				cw_b <= 4'b0011;
				read <= 1;
				//change <= 1;
			end
			//-----------------read from portc---------------//
			2'b10 :
			begin
				cw_a <= 4'b0101; // fill in upper c
				cw_b <= 4'b0101; // fill in lower c
				read <= 1;
				//change <= 1;
			end 
		endcase
	end
end
end
endmodule

//--------------------------------------Test bench-------------------------------------//
module PPI_8255_tb();
reg reset, cs, rd, wr;
reg [1 : 0] a0_a1;
wire [7 : 0] data; // bidirectional signal
wire [7 : 0] pa, pb; // bidirectional signal
wire [3 : 0] pcu, pcl; // bidirectional signal

//data
wire [7:0] in_data;
reg [7:0] out_data;
reg out_valid_data;
//pa
wire [7 : 0] in_pa;
reg [7 : 0] out_pa;
reg out_valid_pa;
//pb
wire [7 : 0] in_pb;
reg [7 : 0] out_pb;
reg out_valid_pb;
//pcu
wire [7 : 0] in_pcu;
reg [7 : 0] out_pcu;
reg out_valid_pcu;
//pcl
wire [7 : 0] in_pcl;
reg [7 : 0] out_pcl;
reg out_valid_pcl;

//data
assign in_data = data;
assign data = (out_valid_data)? out_data : 8'hzz;
//pa
assign in_pa = pa;
assign pa = (out_valid_pa) ? out_pa : 8'hzz;
//pb
assign in_pb = pb;
assign pb = (out_valid_pb) ? out_pb : 8'hzz;
//pcu
assign in_pcu = pcu;
assign pcu = (out_valid_pcu) ? out_pcu : 8'hzz;
//pcl
assign in_pcl = pcl;
assign pcl = (out_valid_pcl) ? out_pcl : 8'hzz;

PPI_8255 test (reset, cs, rd, wr, a0_a1, data, pa, pb, pcu, pcl);
initial
begin
$monitor($time,,"%b %b %b %b %b %b %b %b %b %b",reset, cs, rd, wr, a0_a1, data, pa, pb, pcu, pcl);

/*
//write
#5
reset = 1;
cs = 0;
#10
reset = 0;
cs = 0;
rd = 1;
wr = 0;
a0_a1 = 2'b00;
out_valid_data = 1;
out_valid_pa = 0;
out_data = 8'h54;
#10
a0_a1 = 2'b01;
out_data = 8'h90;
out_valid_pb = 0;
#10
a0_a1 = 2'b10;
out_valid_pcu = 0;
out_valid_pcl = 0;
*/

/*
//read
reset = 1;
cs = 0;
#10
reset = 0;
rd = 0;
wr = 1;
a0_a1 = 2'b00; //read from port A
out_valid_data = 0;
out_valid_pa = 1;
out_pa = 8'h45;*/
/*

//BSR
#5
reset = 1;
cs = 0;
#10
reset = 0;
cs = 0;
rd = 1;
wr = 0;
a0_a1 = 2'b11;
out_valid_data = 1;
out_data = 8'h01; // set bit number zero in portc (BSR mode)
out_valid_pcu = 0;
out_valid_pcl = 0;
#5
out_data = 8'h0f; // set bit number seven in portc
#5
out_data = 8'h02; // clear bit number one in portc 
*/
end
endmodule

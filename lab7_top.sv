`define MREAD 2'b01
`define MWRITE 2'b10

module lab7_top(KEY,SW,LEDR,HEX0,HEX1,HEX2,HEX3,HEX4,HEX5);
    input [3:0] KEY;
    input [9:0] SW;
    output [9:0] LEDR;
    output [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;

    //clk？
    wire clk; 
    wire reset;

    assign clk = ~KEY[0];
    assign reset = ~KEY[1];

    wire[1:0] mem_cmd;
    wire[8:0] mem_addr;
    wire[15:0] read_data, dout, din, write_data; 
    wire msel, addr_sel, write, readOut, writeOut, enable, enable140, loadLED;
    wire N, V, Z;
    wire[7:0] ledLights;

    //stage 2
    RAM MEM(clk,mem_addr[7:0],mem_addr[7:0],write,write_data,dout);

    cpu CPU(clk,reset,write_data,N,V,Z,read_data,mem_cmd,mem_addr);

    equal writeMEM({`MWRITE},mem_cmd,writeOut);

    equal read({`MREAD},mem_cmd,readOut);

    ANDs writeAND(writeOut,msel,write);

    ANDs readAND(readOut,msel,enable);

    equal2 addr(mem_addr[8],1'b0,msel); 

    triState triBuffer(dout,read_data,enable);

    //stage 3
    //h140 

    assign enable140 = (mem_cmd == `MREAD) && (mem_addr == 9'h140); 

    triState140 switch(SW[7:0],read_data[7:0],enable140);
    triState140 zeros(8'h00,read_data[15:8],enable140);

    assign loadLED = (mem_cmd == `MWRITE) && (mem_addr == 9'h100); 

    loadLED LED(clk,loadLED,write_data[7:0],ledLights);

    assign LEDR[7:0] = ledLights;

    assign HEX4 = 7'b1111111;
    assign HEX5 = 7'b1111111;

    assign LEDR[9] = 1'b0;
    assign LEDR[8] = 1'b0;

     // fill in sseg to display 4-bits in hexidecimal 0,1,2...9,A,B,C,D,E,F
    sseg H0(write_data[3:0],   HEX0);
    sseg H1(write_data[7:4],   HEX1);
    sseg H2(write_data[11:8],  HEX2);
    sseg H3(write_data[15:12], HEX3);

endmodule 

module equal(in1,in2,out);

    input[1:0] in1,in2;
    output reg out;

    always @(*) begin
        if(in1 == in2) begin
            out = 1'b1;
        end
        else begin
            out = 1'b0;
        end
    end
endmodule

module equal2(in1,in2,out);

    input in1,in2;
    output reg out;

    always @(*) begin
        if(in1 == in2) begin
            out = 1'b1;
        end
        else begin
            out = 1'b0;
        end
    end
endmodule

module ANDs(in1,in2,out);
    input in1, in2;
    output reg out;

    always @(*) begin
        if(in1 == 1 && in2 == 1) begin
            out = 1'b1;
        end
        else begin 
            out = 1'b0;
        end
    end
endmodule  

module triState(in,out,enable);
    input enable;
    input[15:0] in;

    output wire[15:0] out;
 
    assign out = enable ? in : {16{1'bz}};
endmodule

module RAM(clk,read_address,write_address,write,din,dout);
  parameter data_width = 32; 
  parameter addr_width = 4;
  parameter filename = "data.txt";

  input clk;
  input [addr_width-1:0] read_address, write_address;
  input write;
  input [data_width-1:0] din;
  output [data_width-1:0] dout;
  reg [data_width-1:0] dout;

  reg [data_width-1:0] mem [2**addr_width-1:0];

  initial $readmemb(filename, mem);

  always @ (posedge clk) begin
    if (write)
      mem[write_address] <= din;
    dout <= mem[read_address]; // dout doesn't get din in this clock cycle 
                               // (this is due to Verilog non-blocking assignment "<=")
  end 
endmodule

module triState140(in,out,enable140);

    input[7:0] in;
    input enable140;

    output wire[7:0] out;

    assign out = enable140? in: {8'bzzzzzzzz};
endmodule

module loadLED(clk,loadLED,in,out);

    input[7:0] in;
    input clk, loadLED;

    output reg[7:0] out;

    always @(posedge clk) begin
        if (loadLED == 1'b1) begin
            out = in;
        end
        else begin
            out = out;
        end
    end
endmodule 

module sseg(in,segs);
  input [3:0] in;
  output [6:0] segs;

  // NOTE: The code for sseg below is not complete: You can use your code from
  // Lab4 to fill this in or code from someone else's Lab4.  
  //
  // IMPORTANT:  If you *do* use someone else's Lab4 code for the seven
  // segment display you *need* to state the following three things in
  // a file README.txt that you submit with handin along with this code: 
  //
  //   1.  First and last name of student providing code
  //   2.  Student number of student providing code
  //   3.  Date and time that student provided you their code
  //
  // You must also (obviously!) have the other student's permission to use
  // their code.
  //
  // To do otherwise is considered plagiarism.
  //
  // One bit per segment. On the DE1-SoC a HEX segment is illuminated when
  // the input bit is 0. Bits 6543210 correspond to:
  //
  //    0000
  //   5    1
  //   5    1
  //    6666
  //   4    2
  //   4    2
  //    3333
  //
  // Decimal value | Hexadecimal symbol to render on (one) HEX display
  //             0 | 0
  //             1 | 1
  //             2 | 2
  //             3 | 3
  //             4 | 4
  //             5 | 5
  //             6 | 6
  //             7 | 7
  //             8 | 8
  //             9 | 9
  //            10 | A
  //            11 | b
  //            12 | C
  //            13 | d
  //            14 | E
  //            15 | F

  //assign segs = 7'b0001110;  // this will output "F" 
  reg[6:0] segs;
  
  always@(*) begin 
		case(in)
			4'b0000 : segs = 7'b1000000;
			4'b0001 : segs = 7'b1111001;
			4'b0010 : segs = 7'b0100100;
			4'b0011 : segs = 7'b0110000;
			4'b0100 : segs = 7'b0011001;
			4'b0101 : segs = 7'b0010010;
			4'b0110 : segs = 7'b0000011;
			4'b0111 : segs = 7'b1111000;
			4'b1000 : segs = 7'b0000000;
			4'b1001 : segs = 7'b0011000;
			4'b1010 : segs = 7'b0001000;
			4'b1011 : segs = 7'b0000011;
			4'b1100 : segs = 7'b1000110;
			4'b1101 : segs = 7'b0100001;
			4'b1110 : segs = 7'b0000110;
			4'b1111 : segs = 7'b0001110;
			default: segs = 7'bxxxxxxx;
		endcase
	end

endmodule
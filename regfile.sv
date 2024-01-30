module regfile(data_in,writenum,write,readnum,clk,data_out);

input[15:0] data_in; 
input[2:0] writenum, readnum;
input write, clk; 
output[15:0] data_out; 
//fill out the rest 

wire[15:0] R0,R1,R2,R3,R4,R5,R6,R7;
wire load0,load1,load2,load3,load4,load5,load6,load7;

wire[7:0] writenumdecoded;
wire[7:0] readnumdecoded;

decoder decodewritenum(writenum,writenumdecoded);
decoder decodereadnum(readnum,readnumdecoded); 

//check which register to be updated
assign load0 = (write && writenumdecoded[0])? 1'b1 : 1'b0;
assign load1 = (write && writenumdecoded[1])? 1'b1 : 1'b0;
assign load2 = (write && writenumdecoded[2])? 1'b1 : 1'b0;
assign load3 = (write && writenumdecoded[3])? 1'b1 : 1'b0;
assign load4 = (write && writenumdecoded[4])? 1'b1 : 1'b0;
assign load5 = (write && writenumdecoded[5])? 1'b1 : 1'b0;
assign load6 = (write && writenumdecoded[6])? 1'b1 : 1'b0;
assign load7 = (write && writenumdecoded[7])? 1'b1 : 1'b0;

//updates the register with load that is 1
loadRegister register0(clk, R0, load0, data_in);
loadRegister register1(clk, R1, load1, data_in);
loadRegister register2(clk, R2, load2, data_in);
loadRegister register3(clk, R3, load3, data_in);
loadRegister register4(clk, R4, load4, data_in);
loadRegister register5(clk, R5, load5, data_in);
loadRegister register6(clk, R6, load6, data_in);
loadRegister register7(clk, R7, load7, data_in);

//select the register to read from and update data_out
muxSwitch updateDataOut(R0,R1,R2,R3,R4,R5,R6,R7,readnumdecoded,data_out);
endmodule

//decoder 3:8 usable for read and write
module decoder(threeBits, eightOneHot);

//3 bit binary input
input [2:0] threeBits;
//8 bit one hot output
output [7:0] eightOneHot;

//shifts the placement of 1 according to value of input
assign eightOneHot = 1 << threeBits;
endmodule

//mux switch for data_out
module muxSwitch(r0,r1,r2,r3,r4,r5,r6,r7,eightOneHotSelect,out);

input[15:0] r0,r1,r2,r3,r4,r5,r6,r7;
input [7:0] eightOneHotSelect;
output [15:0] out;
reg[15:0] out; 

always @(*) begin
    case(eightOneHotSelect)
        8'b00000001: out = r0;
        8'b00000010: out = r1; 
        8'b00000100: out = r2;
        8'b00001000: out = r3;
        8'b00010000: out = r4;
        8'b00100000: out = r5;
        8'b01000000: out = r6;
        8'b10000000: out = r7;
        default: out = 16'bxxxxxxxxxxxxxxxx;
    endcase
end

endmodule 

//updates the register values with either data_in or old out depending on load 
module loadRegister(clk, out, load, data_in);

input clk, load; 
input [15:0] data_in; 
output [15:0] out; 
wire[15:0] new_out; 
reg[15:0] out;

//checks if load is 1. If 1, register is updated, else stays same
assign new_out = (load == 1'b1)? data_in : out; 

//updates register value at rising edge
always @(posedge clk) begin
    out = new_out; 
end

endmodule 
module datapath(clk, 
                readnum, //id = provided by instruction decoder
                vsel, //sm = provided by state machine
                loada, //sm
                loadb, //sm
                shift, //id
                asel, //sm
                bsel, //sm
                ALUop, //id 
                loadc, //sm
                loads, //sm
                writenum, //id
                write, //sm
                sximm8, //id
                sximm5, //id
                mdata, //pre assigned below
                PC, //pre assigned below
                C,
                Z_out
             );

input[2:0] readnum, writenum; 
input[7:0] PC; //new for lab6
input[15:0] sximm5, sximm8, mdata; //new for lab6
input clk, loada, loadb, asel, bsel, loadc, loads, write;
input[1:0] shift, ALUop, vsel; //vsel is now 2 bits

output[15:0] C; //datapath_out now called C
output[2:0] Z_out; //Z_out is now 3 bits

wire[2:0] Z; // Z is now 3 bits
wire[15:0] data_in, data_out, rA, rB, Ain, Bin, sout, ALUout;

//select between 4 inputs mdata, sximm8, PC, C - lab6 
multiplexer4 multi9(vsel,mdata,sximm8,{8'b00000000,PC},C,data_in);

regfile REGFILE(data_in,writenum,write,readnum,clk,data_out);

loading loadA(clk,rA,loada,data_out);

loading loadB(clk,rB,loadb,data_out);

shifter shifting(rB,shift,sout);

multiplexer multi6(bsel,sximm5,sout,Bin);

multiplexer multi7(asel,{16'b0000000000000000},rA,Ain); 

ALU ALU(Ain,Bin,ALUop,ALUout,Z);

loading loadC(clk,C,loadc,ALUout);

loadingStatus status(clk,Z_out,loads,Z); 

endmodule

//multiplexer for 4 inputs
module multiplexer4(sel, in3, in2, in1, in0, out);

    input[1:0] sel; 
    input[15:0] in3, in2, in1, in0;
    output[15:0] out;
    reg[15:0] out; 

    always @(*) begin
        case(sel)
            2'b00: out = in0;
            2'b01: out = in1;
            2'b10: out = in2;
            2'b11: out = in3;
            default: out = 16'bxxxxxxxxxxxxxxxx;
        endcase 
    end 

endmodule

//multiplexer for 2 inputs
module multiplexer(sel,in1,in0,out);

    input sel;
    input[15:0] in1, in0;
    output[15:0] out; 
    reg[15:0] out;
    
    always @(*) begin
        case(sel)
            1'b1: out = in1;
            1'b0: out = in0;
            default: out = 16'bxxxxxxxxxxxxxxxx;
        endcase
    end

endmodule

module loading(clk, out, load, in);

input clk, load; 
input [15:0] in; 
output [15:0] out; 
wire[15:0] new_out; 
reg[15:0] out;

//checks if load is 1. If 1, register is updated, else stays same
assign new_out = (load == 1'b1)? in : out; 

//updates register value at rising edge
always @(posedge clk) begin
    out = new_out; 
end

endmodule 

module loadingStatus(clk, out, load, in);

input clk, load; 
input[2:0] in; 
output[2:0] out; 
wire[2:0] new_out; 
reg[2:0] out;

//checks if load is 1. If 1, register is updated, else stays same
assign new_out = (load == 1'b1)? in : out; 

//updates register value at rising edge
always @(posedge clk) begin
    out = new_out; 
end

endmodule 




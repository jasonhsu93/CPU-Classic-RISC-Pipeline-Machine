module datapath_tb();

reg[2:0] readnum, writenum;
reg[15:0] datapath_in;
reg clk, vsel, loada, loadb, asel, bsel, loadc, loads, write;
reg[1:0] shift, ALUop;
reg[15:0] sximm5, sximm8, mdata;
reg[7:0] PC;

wire[2:0] Z_out;
wire[15:0] C;
reg err;

datapath DUT(clk, 
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

initial begin

vsel = 2'b01; //selects data_in = C
PC = 8'b00000111;
datapath_in = 16'b0000000000000111; //set to decimal 7
writenum = 3'b000; //write to R0
write = 1'b1; //enable write
readnum = 3'b000; //read R0 

#1;
clk = 1'b1; //posedge of clk to update R0
#1;
clk = 1'b0;
#1;

loadb = 1'b1; //enable RB to be loaded
#1;
clk = 1'b1; //posedge of clk to update RB
#1;
shift = 2'b01; //shift left of RB vale
clk = 1'b0; 
#1;

writenum = 3'b001; //write to R1
write = 1'b1; //enable write
readnum = 3'b001; //read R1
loadb = 1'b0; //turn off RB
loada = 1'b1; //enable RA to be loaded

#1;
datapath_in = 16'b0000000000000010; //input decimal 2
#1;
clk = 1'b1; //posedge of clk
#1;
clk = 1'b0;
#1;
asel = 1'b0; //selects RA as path to Ain
bsel = 1'b0; //selects RB as path to Bin 
#1; 
ALUop = 2'b00; //selects + operator
loadc = 1'b1;  //enables RC to be loaded
loads = 1'b1; //enables RS to be loaded
#1;
clk = 1'b1; //posedge of clk to update status
#1;
clk = 1'b0;
#1;
clk = 1'b1; //posedge of clk to update RC
#1;
clk = 1'b0;

//checks if datapath_out and Z_out is the expected value
if(datapath_tb.DUT.C !== 16'bxxxxxxxxxxxxxxxx || datapath_tb.DUT.Z_out !== 3'b000) begin
    $display("ERROR your C is %b, expected %b", datapath_tb.DUT.C, 16'bxxxxxxxxxxxxxxxx);
    $display("your Z_out is %b, expected %b", datapath_tb.DUT.Z_out, 3'b000);
    err = 1'b1;
end
else begin 
    $display("Z_out and datapath_out is correct!");
end


$stop;
end

endmodule 
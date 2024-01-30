`define sR 5'b00000 // reset state, w = 1, s waiting to be changed to 1
`define sDecode 5'b00001 //decode state
`define sMOV1 5'b00010 //MOV state 
`define sMOV2 5'b00011 //MOV state
`define sA 5'b00100
`define sB 5'b00101
`define sC 5'b00110
`define sD 5'b00111
`define sM 5'b01000
`define sW 5'b01001
`define sIF1 5'b01010
`define sIF2 5'b01011 
`define sUpdatePC 5'b01100 
`define sLDRa 5'b01101 
`define sLDRb 5'b01110 
`define sLDRc 5'b01111 
`define sLDRd 5'b10000 
`define sLDRe 5'b10001 
`define sSTRa 5'b10010 
`define sSTRb 5'b10011 
`define sSTRc 5'b10100 
`define sSTRd 5'b10101 
`define sSTRe 5'b10110 
`define sSTRf 5'b10111 
`define sHALT 5'b11000 
//described further below

module cpu(clk,reset,out,N,V,Z,mdata,mem_cmd,mem_addr);

    input clk, reset; //load = instruction register load 
    input [15:0] mdata;

    output[1:0] mem_cmd;
    output[8:0] mem_addr; 

    output [15:0] out;
    output N, V, Z;

    wire[2:0] nsel;
    wire[15:0] data_in, sximm5, sximm8;
    wire[2:0] opcode, Rd, Rm, Rn, Z_out, readnum, writenum;
    wire[1:0] op, shift, vsel;
    wire[15:0] mdata;
    wire write, loada, loadb, loadc, loads, asel, bsel;

    //lab7 additions
    wire[8:0] next_pc, PC, mem_addr, dataAddressOut;
    wire[8:0] plusOneOut;
    wire load_pc, reset_pc, addr_sel, load_ir, load_addr;
    wire[15:0] read_data;
    wire[1:0] mem_cmd;
    

    instructionRegister step1(mdata,load_ir,clk,data_in); //loads instruction 

    //program counter
    ProgramCounter programC(next_pc,load_pc,clk,PC); 

    //plus one 
     plusOne plus(PC,plusOneOut);

    /*always@(*)begin

    plusOneOut = PC + 1'b1; //just adds one if nothing as been set

    if(PC == 0) begin plusOneOut = 9'b000000001; end //makes sure when PC is initialized to 0, the next PC will be 1

    end*/

    //data address
    dataAddress updateDataAddress(load_addr,clk,out[8:0],dataAddressOut);

    //mux with reset_pc
    twoInputMultiplexer resetPC(reset_pc,{9'b000000000},plusOneOut,next_pc);

    //mux with addr_sel
    twoInputMultiplexer addressMUX(addr_sel,PC,dataAddressOut,mem_addr);

    instructionDecoder step2(data_in, nsel, op, opcode, sximm8, sximm5, shift, readnum, writenum, Rn, Rm, Rd); 

    stateMachine step3(reset, opcode, op, nsel,  vsel, asel, bsel, loada, loadb, loadc, loads, reset_pc, write, clk, load_ir, load_pc, addr_sel, mem_cmd, load_addr);

    datapath DP(clk, 
                readnum, //id = provided by instruction decoder
                vsel, //sm = provided by state machine
                loada, //sm
                loadb, //sm
                shift, //id
                asel, //sm
                bsel, //sm
                op, //id 
                loadc, //sm
                loads, //sm
                writenum, //id
                write, //sm
                sximm8, //id
                sximm5, //id
                mdata, //pre assigned below
                PC, //pre assigned below
                out,
                Z_out
             );

    assign V = Z_out[2]; //Overflow
    assign N = Z_out[1]; //Negative
    assign Z = Z_out[0]; //Zero 

endmodule

/*
STATES

sR: RESET/waiting state 
sDecode: Decoder state, checks opcode 
sMOV1: loads rB = Rm
sMOV2: loads rC = output value of ALU 
sDecode: state of decoding
sA: puts Rn in Ra
sB: performs the operation described by op 
sC: puts result R into Rd
sD: directly puts Rm into rB
sM: directly puts Rn into rA

*/
module stateMachine(reset, opcode, op, nsel, vsel, asel, bsel, loada, loadb, loadc, loads, reset_pc, write, clk, load_ir, load_pc, addr_sel, mem_cmd, load_addr);

    input reset, clk;
    input[2:0] opcode; //110 = MOV, 101 = ALU instructions 
    input[1:0] op; //ALUop if opcode is 101, MOV operations if not

    output [2:0] nsel;
    output [1:0] vsel, mem_cmd;
    output loada, loadb, loadc, loads, write, asel, bsel, reset_pc, load_ir, load_pc, addr_sel, load_addr; 

    reg [2:0] nsel;
    reg [1:0] vsel, mem_cmd;
    reg loada, loadb, loadc, loads, write, asel, bsel, load_ir, load_pc, addr_sel; 
    reg reset_pc, load_addr; //w is same as reset_pc
    reg[4:0] next_state, present_state; 
    reg[23:0] update;

    //updates all inputs, outputs, states onces clk is high
    always @(posedge clk) begin
        {next_state, write, loada, loadb, loadc, loads, asel, bsel, vsel, nsel, load_ir, load_pc, reset_pc, addr_sel, mem_cmd, load_addr} = update;
        present_state = next_state; 
    end

    //gets every input ready for the next state and handles instructions based on the present state 
    always @(*) begin 
        /*case(present_state)
            `sR: reset_pc = 1'b1;
            default: reset_pc = 1'b0; 
        endcase */
        casex({present_state, reset, opcode, op}) 

            {`sR,1'b1,3'bxxx,2'bxx}: update = {`sR,12'b000000000000,1'b0,1'b1, 1'b1, 1'b0, 2'b00, 1'b0}; //stay in reset state if reset is 1
            {`sR,1'b0,3'bxxx,2'bxx}: update = {`sIF1,12'b000000000000,1'b0,1'b1, 1'b1, 1'b1, 2'b01, 1'b0}; // go to IF1 state 

            {`sIF1,6'b0xxxxx}: update = {`sIF2,12'b000000000000,7'b1001010}; // mem_cmd is read mode (01) and addr_sel =0
            {`sIF2,6'b0xxxxx}: update =  {`sUpdatePC,19'b0000000000000100000};  //loads pc

            {`sUpdatePC,6'b0xxxxx}: update = {`sDecode, 19'b0000000000000000000}; //updates pc then go decode state

            {`sDecode,1'b0, 3'b110, 2'b10}: update = {`sMOV1,1'b1,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,2'b10,3'b001,7'b0000000}; //go to MOV Rn,#<im8> state
            {`sDecode,1'b0, 3'b110, 2'b00}: update = {`sMOV2,1'b0,1'b0,1'b1,1'b0,1'b0,1'b0,1'b0,2'b00,3'b100,7'b0000000}; //go to MOV Rd,Rm{,<sh_op>} state, move Rm into rB first, loadb = 1
            
            {`sDecode,1'b0, 3'b101, 2'b00}: update = {`sA,1'b0,1'b1,1'b0,1'b0,1'b0,1'b0,1'b0,2'b00,3'b001,7'b0000000}; //go to state getA, puts Rn in rA
            {`sDecode,1'b0, 3'b101, 2'b01}: update = {`sA,1'b0,1'b1,1'b0,1'b0,1'b0,1'b0,1'b0,2'b00,3'b001,7'b0000000}; //go to state getA, puts Rn in rA
            {`sDecode,1'b0, 3'b101, 2'b10}: update = {`sA,1'b0,1'b1,1'b0,1'b0,1'b0,1'b0,1'b0,2'b00,3'b001,7'b0000000}; //go to state getA, puts Rn in rA
            {`sDecode,1'b0, 3'b101, 2'b11}: update = {`sB,1'b0,1'b0,1'b1,1'b0,1'b0,1'b0,1'b0,2'b00,3'b100,7'b0000000}; //go to state getB, puts Rm in rB

            //lab 7 new decodes 011, 100, 111
            {`sDecode,1'b0, 3'b011, 2'b00}: update = {`sLDRa, 12'b010000000001,7'b0000000}; // nsel=001, loada=1
            {`sDecode,1'b0, 3'b100, 2'b00}: update = {`sSTRa, 12'b010000000001,7'b0000000}; // nsel=001, loada=1
            {`sDecode,1'b0, 3'b111, 2'bxx}: update = {`sHALT, 12'b000000000000,7'b0000000}; // mem_cmd = 10 (write) and addr_sel =0
            
            {`sA,1'b0,3'bxxx,2'bxx}: update = {`sB,1'b0,1'b0,1'b1,1'b0,1'b0,1'b0,1'b0,2'b00,3'b100,7'b0000000}; //go to state getB, puts Rm in rB 
            {`sB,1'b0,3'bxxx,2'bxx}: update = {`sC,1'b0,1'b0,1'b0,1'b1,1'b0,1'b0,1'b0,2'b00,3'b100,7'b0000000}; //go to state operation, loadc = 1
            {`sC,1'b0,3'bxxx,2'b00}: update = {`sD,1'b1,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,2'b00,3'b010,7'b0000000}; //puts R into Rd 
            {`sC,1'b0,3'bxxx,2'b10}: update = {`sD,1'b1,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,2'b00,3'b010,7'b0000000}; //puts R into Rd 
            {`sC,1'b0,3'bxxx,2'b11}: update = {`sD,1'b1,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,2'b00,3'b010,7'b0000000}; //puts R into Rd 
            {`sC,1'b0,3'b101,2'b01}: update = {`sM,1'b0,1'b0,1'b0,1'b0,1'b1,1'b0,1'b0,2'b00,3'b010,7'b0000000}; //Changes status, loads = 1 

            {`sM, 6'b0xxxxx}: update = {`sIF1, 12'b000000000000,7'b0001010}; //Loads value Rn into rA
            {`sD, 6'b0xxxxx}: update = {`sIF1, 12'b000000000000,7'b0001010};  //Loads value Rm into rB
            {`sMOV2, 6'b0xxx00}: update = {`sW,1'b0,1'b0,1'b0,1'b1,1'b0,1'b1,1'b0,2'b00,3'b100,7'b0000000}; //Sends value at rB through the ALU with Ain=0 and adds these values together, loads this value to register C, loadc = 1;
            {`sW, 6'b0xxx00}: update = {`sD,1'b1,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,2'b00,3'b010,7'b0000000}; //loada = 0, asel=0 , bsel=0, loadc=1 in previous step
            {`sMOV1, 6'b0xxxxx}: update = {5'b01010, 12'b000000000000,7'b0001010}; //Loads value Rm into rB

            //LDR
            {`sLDRa, 6'b0xxxxx}: update = {`sLDRb, 12'b000100100001,7'b0000000}; //loada=0, asel=0,bsel=1,loadc=1
            {`sLDRb, 6'b0xxxxx}: update = {`sLDRc, 12'b000000000001,7'b0000001}; // load_addr = 1
            {`sLDRc, 6'b0xxxxx}: update = {`sLDRd, 12'b000000000001,7'b0000010}; // addr_sel=0, mem_cmd = read
            {`sLDRd, 6'b0xxxxx}: update = {`sLDRe, 12'b100000011010,7'b0000010}; // vsel = 11, nsel = 010, write = 1
            {`sLDRe, 6'b0xxxxx}: update = {`sIF1, 12'b000000000010,7'b0001010}; // mem_cmd = write, addr_sel = 0

            //STR
            {`sSTRa, 6'b0xxxxx}: update = {`sSTRb, 12'b000100100001,7'b0000000}; // loada=0, asel=0, bsel=1, loadc=1
            {`sSTRb, 6'b0xxxxx}: update = {`sSTRc, 12'b000000000001,7'b0000001}; // load_addr = 1 
            {`sSTRc, 6'b0xxxxx}: update = {`sSTRd, 12'b001000000010,7'b0000000}; // nsel=010, loadb=1 
            {`sSTRd, 6'b0xxxxx}: update = {`sSTRe, 12'b000101000010,7'b0000000}; // asel = 1, loadc = 1
            {`sSTRe, 6'b0xxxxx}: update = {`sSTRf, 12'b000000000010,7'b0000100}; // output into Rd
            {`sSTRf, 6'b0xxxxx}: update = {`sIF1, 12'b000000000000,7'b0001010}; // Load Rm into rB

            //HALT
            {`sHALT, 6'b0111xx}: update = {`sHALT, 19'b0000000000000000000}; // HALT everything not activated

            {5'bxxxxx,1'b1,3'bxxx,2'bxx}: update = {`sR,12'b000000000000,1'b0,1'b1, 1'b1, 1'b0, 2'b00, 1'b0}; //reset = 1 -> go to reset state
            default: update = {`sIF1,12'b000000000000,1'b0,1'b1, 1'b1, 1'b1, 2'b01, 1'b0}; 
        endcase
    end

endmodule

//instruction register block
module instructionRegister(in,load,clk,out);

    input[15:0] in;
    input load, clk;
    output[15:0] out;
    reg[15:0] out;

    //if clk is high and load is 1, out gets updated to in 
    always @(posedge clk) begin
        case(load)
            1'b1: out = in;
            1'b0: out = out;
            default: out = 16'd0; 
        endcase
    end

endmodule

//takes input from instruction register block, and decodes the tasks to be done
//op is used interchangedably with ALUop as described by Table 1 
//can be used to MOV or perform ALU actions as described by Table 1
module instructionDecoder(in, nsel, op, opcode, sximm8, sximm5, shift, readnum, writenum, Rn, Rm, Rd);

    input[15:0] in;
    input[2:0] nsel;

    output[1:0] op, shift;
    output[2:0] opcode, writenum, readnum, Rn, Rm, Rd;
    output[15:0] sximm5, sximm8;

    wire[4:0] imm5;
    wire[7:0] imm8; 
    wire[2:0] R; //multiplexer output between Rn/Rm/Rd

    //chooses between Rn Rm Rd: 100 = Rm, 010 = Rd, 001 = Rn  
    threeInputMultiplexer decider(nsel, Rm, Rd, Rn, R); 

    //sign extension of imm5
    assign sximm5 = {{11{imm5[4]}},imm5[4:0]};

    //sigh extension of imm8
    assign sximm8 = {{8{imm8[7]}},imm8[7:0]};

    //immediate values 
    assign imm5 = in[4:0];
    assign imm8 = in[7:0];

    //Rn Rd Rm assignments
    assign Rn = in[10:8];
    assign Rd = in[7:5];
    assign Rm = in[2:0];

    //op and opcode
    assign opcode = in[15:13];
    assign op = in[12:11];

    //shift
    assign shift = in[4:3];

    //readnum and writenum
    assign readnum = R;
    assign writenum = R;

endmodule 

//three input multiplexer used in instruction decoder for Rd Rm Rn
module threeInputMultiplexer(sel, in2, in1, in0, out);

    input[2:0] in2, in1, in0, sel;
    output[2:0] out; 
    reg[2:0] out;

    always @(*) begin
        case(sel)
            3'b100: out = in2;
            3'b010: out = in1;
            3'b001: out = in0;
            default: out = 3'bxxx; 
        endcase
    end

endmodule 

//for 9 bits bus
module twoInputMultiplexer(sel,in1,in0,out);

    input sel;
    input[8:0] in1, in0;

    output reg[8:0] out;

    always @(*) begin
        case(sel)
            1'b1: out = in1;
            1'b0: out = in0;
            default: out = 9'bxxxxxxxxx;
        endcase
    end
endmodule

//program counter
module ProgramCounter(next_pc,load_pc,clk,PC);

    input clk, load_pc;
    input[8:0] next_pc;

    output reg[8:0] PC;

    always @(posedge clk) begin
        if(load_pc == 1'b1) begin
            PC = next_pc;
        end
        else begin 
            PC = PC;
        end
    end
endmodule 

//plus one block
module plusOne(PC,plusOneOut);
    
    input[8:0] PC;
    output reg[8:0] plusOneOut;

    always_comb begin 
        plusOneOut = PC + 9'b000000001;
    end
endmodule 

module dataAddress(load_addr,clk,out,dataAddressOut);
    
    input load_addr, clk;
    input[8:0] out;
    output reg[8:0] dataAddressOut;

    always @(posedge clk) begin
        if(load_addr == 1'b1) begin
            dataAddressOut = out;
        end
        else begin
            dataAddressOut = dataAddressOut; 
        end
    end
endmodule 
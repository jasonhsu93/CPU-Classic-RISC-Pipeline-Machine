module ALU(Ain,Bin,ALUop,out,Z_out);

input[15:0] Ain,Bin; 
input[1:0] ALUop;
output[15:0] out;
output[2:0] Z_out; 
reg[2:0] Z_out; //{zero flag, negative flag, overflow flag}
reg[15:0] out;
//fill out the rest

always @(*) begin
    case(ALUop)
        2'b00: out = Ain + Bin;
        2'b01: out = Ain - Bin;
        2'b10: out = Ain & Bin;
        2'b11: out = ~Bin ;
        default: out = 16'bxxxxxxxxxxxxxxxx; 
    endcase 
    
    //zero flag on 0 bit
    if(out == 16'b0000000000000000)
        Z_out[0] = 1'b1;
    else   
        Z_out[0] = 1'b0;

    //negative flag on 1 bit
    if(out[15] == 1'b1)
        Z_out[1] = 1'b1;
    else 
        Z_out[1] = 1'b0;

    //overflow flag on 2 bit
    if(((Ain[15] & Bin[15]) && (ALUop == 2'b00) && (~out[15]))||
       ((Ain[15] & ~Bin[15]) && (ALUop == 2'b01) && (~out[15]))||
       ((~Ain[15] & ~Bin[15]) && (ALUop == 2'b00) && (out[15]))||
       ((~Ain[15] & Bin[15]) && (ALUop == 2'b01) && (out[15]))) 
        Z_out[2] = 1'b1;    
    else 
        Z_out[2] = 1'b0;
end

endmodule
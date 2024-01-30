module cpu_tb();

    // simulation I/O
    reg clk, reset, s, load;
    reg [15:0] in;
    wire [15:0] out;
    wire N, V, Z, w;

    //err wave to indicate any errors 
    reg err;

    //instantiate our cpu DUT
    cpu DUT(.clk(clk), .reset(reset), .s(s), .load(load), .in(in), .out(out), .N(N), .V(V), .Z(Z), .w(w));

    initial begin
    clk = 0; #5;
    forever begin
      clk = 1; #5;
      clk = 0; #5;
    end
    end

    //provides instruction to instruction register
    task instruction; 
		input [15:0] instruction;
	begin
		@(negedge clk);
		
		in = instruction;
		load = 1;
		#10;
		
		load = 0;
		s = 1;
		#10;
		
		s = 0;
		
		@(posedge w);
		#10;		
	end
	endtask

  initial begin

    //TEST1: Checking if w is being updated correctly everytime state is in wait/reset
    err = 0;
    reset = 1; s = 0; load = 0; in = 16'b0;
    #10;
    reset = 0; 
    #10;

    in = 16'b1101000000000111;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    if (cpu_tb.DUT.DP.REGFILE.R0 !== 16'h7) begin
      err = 1;
      $display("FAILED: MOV R0, #7");
      $stop;
    end

    @(negedge clk); // wait for falling edge of clock before changing inputs
    in = 16'b1101000100000010;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    if (cpu_tb.DUT.DP.REGFILE.R1 !== 16'h2) begin
      err = 1;
      $display("FAILED: MOV R1, #2");
      $stop;
    end

    @(negedge clk); // wait for falling edge of clock before changing inputs
    in = 16'b1010000101001000;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    if (cpu_tb.DUT.DP.REGFILE.R2 !== 16'h10) begin
      err = 1;
      $display("FAILED: ADD R2, R1, R0, LSL#1");
      $stop;
    end
    if (~err) $display("INTERFACE OK");

    //TEST 2: Checking ALL Functions
		err = 0;
		reset = 1; 
        s = 0; 
        load = 0; 
        in = 16'b0;

		#10;
		
		reset = 0;

		#10;
		
        //INSTRUCTION FOR MOV1: 3-opcode 2-op 3-Rn 8-imm8 
        //INSTRUCTION FOR MOV2: 3-opcode 2-op 000 3-Rd 2-sh 3-Rm
        //INSTRUCTION FOR ALU: 3-opcode 2-ALUop/op 3-Rn 3-Rd 2-Shift 3-Rm 

		//MOV1 Rn, #<im8> intruction tests

        //MOV R0, #1 
		instruction(16'b1101000000000010); 
		if (DUT.DP.REGFILE.R0 !== 16'h2) begin
			err = 1;
			$display("INSTRUCTION FAILED MOV R0, #2");
		end
		
        //MOV R1, #-1 
		instruction(16'b1101000111111111); 
		if (DUT.DP.REGFILE.R1 !== 16'b1111111111111111) begin
			err = 1;
			$display("INSTRUCTION FAILED MOV R1, #-1");
		end
		
        //MOV R2, #8 
		instruction(16'b1101001000001000); 
		if (DUT.DP.REGFILE.R2 !== 16'h8) begin
			err = 1;
			$display("INSTRUCTION FAILED MOV R2, #8");
		end
		
		//MOV2 Rd, Rm{,<sh_op>} tests

        //MOV R3, R0, LSR #1 
		instruction(16'b1100000001110000); 
		if (out !== 16'h1) begin
			err = 1;
			$display("INSTRUCTION FAILED MOV R3, R0, LSR #1");
		end
		
        //MOV R4, R0, LSL #1 
		instruction(16'b1100000010001000);  
		if (out !== 16'h4) begin
			err = 1;
			$display("INSTRUCTION FAILED MOV R4, R0, LSL #1");
		end
		
        //MOV R5, R1, ARS #1 
		instruction(16'b1100000010111001); 
		if (out !== 16'b1111111111111111) begin
			err = 1;
			$display("INSTRUCTION FAILED MOV R5, R1, ARS #1");
		end
		
		//ADD Rd, Rn, Rm{,<sh_op>} tests

        //ADD R6, R2, R0
		instruction(16'b1010000011000010); 
		if (out !== 16'hA) begin
			err = 1;
			$display("INSTRUCTION FAILED ADD R6, R2, R0");
		end
		
        //ADD R7, R2, R4, LSL #1
		instruction(16'b1010001011101100); 
		if (out !== 16'h10) begin
			err = 1;
			$display("INSTRUCTION FAILED ADD R7, R2, R4, LSL #1");
		end
		
        //ADD R7, R7, R7, LSR #1
		instruction(16'b1010011111110111); 
		if (out !== 16'h18) begin
			err = 1;
			$display("INSTRUCTION FAILED ADD R7, R7, R7, LSR #1");
		end
		
		//CMP Rn, Rm{,<sh_op>} tests

        //CMP R7, R6, LSL #1 
		instruction(16'b1010111100001110); 
		if (Z !== 0 || N !== 0 || V !== 0) begin
			err = 1;
			$display("INSTRUCTION FAILED CMP R7, R6, LSL #1");
		end
		
		//AND Rd, Rn, Rm{,<sh_op>} tests

        //AND R4, R5, R4
		instruction(16'b1011010110000100); 
		if (out !== 16'h4) begin
			err = 1;
			$display("INSTRUCTION FAILED AND R4, R5, R4");
		end
		
        //AND R5, R5, R1
		instruction(16'b1011010110100001); 
		if (out !== 16'b1111111111111111) begin
			err = 1;
			$display("INSTRUCTION FAILED AND R5, R5, R1");
		end
		
        //AND R5, R2, R4, LSL #1
		instruction(16'b1011001010101100); 
		if (out !== 16'b0000000000001000) begin
			err = 1;
			$display("INSTRUCTION FAILED AND R5, R2, R4, LSL #1");
		end
		
		//MVN Rd, Rm{,<sh_op>} tests

        //MVN R5, R3
		instruction(16'b1011100010100011); 
		if (out !== 16'b1111111111111110) begin
			err = 1;
			$display("INSTRUCTION FAILED MVN R5, R3");
		end
		
        //MVN R7, R7, LSR #1 
		instruction(16'b1011100011110111); 
		if (out !== 16'b1111111111110011) begin
			err = 1;
			$display("INSTRUCTION FAILED MVN R7, R7, LSR #1");
		end
		
        //MVN R6, R1, LSL #1 
		instruction(16'b1011100011001001); 
		if (out !== 16'h1) begin
			err = 1;
			$display("INSTRUCTION FAILED MVN R6, R1, LSL #1");
		end
		
		if(~err)
			$display("PASSED ALL THE TESTS!");
		else	
			$display("FAILED AT LEAST ONE TEST!");
		
		$stop;
  end

endmodule

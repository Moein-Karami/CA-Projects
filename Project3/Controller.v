module Controller(input clk, zero, input [5 : 0]op, input [5 : 0]func, output reg J_type, Branch, PCsrc, RegWrite, ALUsrc,
		MemRead , MemWrite, output reg[2 : 0]ALUop, output reg[1 : 0]RegDest, output reg[1 : 0] WriteReg);

	parameter[5:0] RT = 6'b000000, lw = 6'b100011, sw = 6'b101011, beq = 6'b000100, bne = 6'b000101,
			J = 6'b000010, Jal = 6'b000011, Jr = 6'b001000, add = 6'b100001, sub = 6'b100011, addi = 6'b001100,
			slt = 6'b101010, slti = 6'b001010;
	parameter ADD = 0;
	parameter SUB = 1;

	always @(func, op)
	begin
		RegDest = 0;
		RegWrite = 0;
		J_type = 0;
		ALUsrc = 0;
		ALUop = 0;
		WriteReg = 0;
		MemWrite = 0;
		MemRead = 0;
		PCsrc = 0;
		Branch = 0;

		if (op == lw) 
			RegDest = 0;
		else if (op == Jal)
			RegDest = 2;
		else if (op == RT | func == addi | func == slti)
		begin
			if (func == add | func == sub | func == slt)
				RegDest = 1;
			else
				RegDest = 0;
		end

		if (op == RT | op == lw | op == Jal | op == slti | op == addi)
			RegWrite = 1;

		if (op == RT & func == Jr)
			J_type = 1;

		if (op == lw | op == sw)
			ALUsrc = 1;
		else if (op == addi | op == slti)
		begin
				ALUsrc = 1;
		end

		if (op == lw | op == sw)
			ALUop = ADD;
		else if (op == beq | op == bne)
			ALUop = SUB;
		else if (op == RT | op == addi | op == slti)
		begin
			if (func == add | op == addi)
				ALUop = ADD;
			else
				ALUop = SUB;
		end

		if (op == Jal)
			WriteReg = 3;
		else if (op == lw)
			WriteReg = 2;
		else if ((op == RT & func == slt) | op == slti)
			WriteReg = 1;

		if (op == sw)
			MemWrite = 1;

		if (op == lw)
			MemRead = 1;

		if (op == beq)
			PCsrc = zero;
		else if (op == bne)
			PCsrc = ~zero;
		else if (op == J | op == Jal | op == Jr)
			PCsrc = 1;

		if(op == beq | op == bne)
			Branch = 1;
	end
endmodule

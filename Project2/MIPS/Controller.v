module Controller(input clk, zero, input [5 : 0]op, input [5 : 0]func, output reg J_type, Branch, PCsrc, RegWrite, ALUsrc,
		MemRead , MemWrite, output reg[2 : 0]ALUop, output reg[1 : 0]RegDest, output reg[1 : 0] WriteReg);

	parameter RT = 1;
	parameter lw = 1;
	parameter sw = 1;
	parameter beq = 1;
	parameter bne = 1;
	parameter J = 1;
	parameter Jal = 1;
	parameter Jr = 1;
	parameter add = 1;
	parameter sub = 1;
	parameter addi = 1;
	parameter slt = 1;
	parameter slti = 1;
	parameter ADD = 0;
	parameter SUB = 1;

	always @(posedge clk)
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
		else if (op == RT)
		begin
			if (func == add | func == sub | func == slt)
				RegDest = 1;
			else
				RegDest = 0;
		end

		if (op == RT | op == lw | op == Jal)
			RegWrite = 1;

		if (op == Jr)
			J_type = 1;

		if (op == lw | op == sw)
			ALUsrc = 1;
		else if (op == RT)
		begin
			if (func == addi | func == slti)
				ALUsrc = 1;
		end

		if (op == lw | op == sw)
			ALUop = ADD;
		else if (op == beq | op == bne)
			ALUop = SUB;
		else if (op == RT)
		begin
			if (func == add | func == addi)
				ALUop = ADD;
			else
				ALUop = SUB;
		end

		if (op == Jal)
			WriteReg = 3;
		else if (op == lw)
			WriteReg = 2;
		else if ((op == RT) & (func == slt | func == slti))
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

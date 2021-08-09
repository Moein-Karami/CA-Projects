module Controller(input clk, input rst,zero, input [5 : 0]op, input [5 : 0]func, output reg J_type3, output PCsrc, output reg RegWrite4, RegWrite3,
		ALUsrc2, MemRead3 , MemWrite3, beq3, bneq3, output reg[2 : 0]ALUop2, output reg[1 : 0]RegDest2,
		output reg [1:0] WriteReg3, output reg[1 : 0] WriteReg4);

	reg PCsrc3, PCsrc2;
	reg beq, beq2;
	reg bneq, bneq2;
	reg ALUsrc;
	reg ALUop;
	reg first;
	reg second;
	reg third;
	reg Jm1, Jm2, Jm3;
	reg MemWrite, MemWrite2;
	reg[1 : 0] WriteReg, WriteReg2;
	reg RegWrite, RegWrite2;
	reg J_type, J_type2;
	reg [1 : 0]RegDest;
	reg MemRead, MemRead2;

	parameter[5:0] RT = 6'b000000, lw = 6'b100011, sw = 6'b101011, BEQ = 6'b000100, BNEQ = 6'b000101,
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
		first = 1;
		second = 0;
		third = 0;
		Jm1 = 0;
		if (op == BEQ)
			beq = 1;
		else
			beq = 0;

		if (op == BNEQ)
			bneq = 1;
		else
			bneq = 0;

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
		else if (op == BEQ | op == BNEQ)
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
		if(op == J) 
			Jm1 = 1;

	end
	
	assign PCsrc = rst ? 1'b1: beq3 ? zero: bneq3 ? ~zero: Jm3 ? 1'b1:1'b0;
	
	always @(posedge clk)
	begin
		beq3 = beq2;
		beq2 = beq;

		bneq3 = bneq2;
		bneq2 = bneq;
		
		third = second;
		second = first;

		ALUsrc2 = ALUsrc;

		ALUop2 = ALUop;

		MemWrite3 = MemWrite2;
		MemWrite2 = MemWrite;

		MemRead3 = MemRead2;
		MemRead2 = MemRead;

		WriteReg4 = WriteReg3;
		WriteReg3 = WriteReg2;
		WriteReg2 = WriteReg;
		
		RegWrite4 = RegWrite3;
		RegWrite3 = RegWrite2;
		RegWrite2 = RegWrite;

		J_type3 = J_type2;
		J_type2 = J_type;
		
		Jm3 = Jm2;
		Jm2 = Jm1;

		RegDest2 = RegDest;
	end
endmodule

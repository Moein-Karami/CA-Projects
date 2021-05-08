module TopDesign(input clk, rst);
	wire Branch, PCsrc, RegWrite, ALUsrc, MemRead, MemWrite, zero, J_type;
	wire [2 : 0]ALUop;
	wire [1 : 0]RegDest;
	wire [1 : 0]WriteReg;
	wire [31 : 0]Adrout;
	wire [5 : 0]op;
	wire [5 : 0] func;

	Controller controller(clk, zero, op, func, J_type, Branch, PCsrc, RegWrite, ALUsrc, MemRead, MemWrite, ALUop,
			RegDest, WriteReg);
	DataPath data_path(clk, rst, J_type, Branch, PCsrc, RegWrite, ALUsrc, MemRead, MemWrite, ALUop, RegDest, WriteReg,
			zer, Adrout, op, func);
endmodule
module TopDesign(input clk, rst);
	wire Branch, PCsrc3, RegWrite4, RegWrite3, ALUsrc2, MemRead3, MemWrite3, zero, J_type3, beq3, bneq3;
	wire [2 : 0]ALUop2;
	wire [1 : 0]RegDest2;
	wire [1 : 0]WriteReg4;
	wire [31 : 0]Adrout;
	wire [5 : 0]op;
	wire [5 : 0] func;
	wire [1 : 0]forward1;
	wire [1 : 0]forward2;
	wire [4 : 0]write_adress_reg_out;
	wire [4 : 0] write_adress_reg2_out;
	wire [31 : 0] data1_reg_out;
	wire [31 : 0] data2_reg_out;

	Controller controller(clk, zero, op, func, J_type3,, PCsrc3, RegWrite4, RegWrite3, ALUsrc2, MemRead3, MemWrite3,
			beq3, bneq3, ALUop2, RegDest2, WriteReg4);
	DataPath data_path(clk, rst, J_type3, PCsrc3, RegWrite4, ALUsrc2, MemRead3, MemWrite3, beq3, bneq3, forward1, forward2,
			ALUop2, RegDest2, WriteReg4, zero, Adrout, op, func, write_adress_reg_out, write_adress_reg2_out, data1_reg_out,
			data2_reg_out);
	ForwardUnit forward_unit(RegWrite3, RegWrite4, write_adress_reg_out, write_adress_reg2_out, data1_reg_out, data2_reg_out,
			forward1, forward2);
endmodule
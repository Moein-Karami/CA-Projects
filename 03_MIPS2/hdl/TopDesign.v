module TopDesign(input clk, input rst);
	wire J_type3, PCsrc3, RegWrite4, ALUsrc2, MemRead3, MemWrite3, beq3, bneq3, RegWrite3;
	wire [4:0] write_adress_reg2_out;
	wire [4:0] write_adress_reg_out;
	wire [1:0] forward1, forward2;
	wire [2:0] ALUop2;
	wire [1:0] RegDest2;
	wire [1:0] WriteReg4, WriteReg3;
	wire zero;
	wire [31:0] Adrout;
	wire [5:0] op;
	wire [5:0] func;
	wire [31:0] data2_reg_out;
	wire [31:0] data1_reg_out;
	wire [4:0] write_adress_reg_in;
	wire [25:0] inst_ins;
	wire [25:0] inst2_ins;
	DataPath data_path(clk, rst, J_type3, PCsrc3, RegWrite4, ALUsrc2, MemRead3, MemWrite3, beq3, bneq3, forward1,
	forward2, ALUop2, RegDest2, WriteReg3, WriteReg4, zero,
	Adrout, op, func, write_adress_reg_out,
	write_adress_reg2_out, data1_reg_out, data2_reg_out, write_adress_reg_in, inst_ins);
	Controller controller(clk, rst,  zero, op, func, J_type3, PCsrc3, RegWrite4, RegWrite3,
		ALUsrc2, MemRead3 , MemWrite3, beq3, bneq3, ALUop2, RegDest2,
		WriteReg3, WriteReg4);
	ForwardUnit forward(RegWrite3, RegWrite4, write_adress_reg_out, write_adress_reg2_out,
		inst_ins[25:21], inst_ins[20:16], MemRead3, write_adress_reg_in, forward1, forward2);
endmodule

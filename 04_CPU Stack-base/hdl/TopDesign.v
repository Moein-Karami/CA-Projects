module TopDesign(input clk, rst);
	wire ld_pc, adrr, write, pc_dst, cn_pc_ds, ld_inst, st_data, push, pop, tos, ld_a, ALUsrcA;
	wire [1 : 0]ALUsrcB;
	wire [1 : 0]ALU_Control;
	wire [2 : 0]inst;
	wire pc_temp;

	Controller controller(pc_temp, clk, rst, inst, adrr, ld_inst, ALUsrcA, ALUsrcB, ld_pc, tos, pc_dst, ld_a, pop, write,
			ALU_Control, cn_pc_ds, push, st_data, ld_mem);
	DataPath data_path(clk, rst, ld_pc, pc_dst, cn_pc_ds, adrr, write, ld_inst, push, pop, tos, st_data, ld_a, ALUsrcA,
			ALUsrcB, ALU_Control, ld_mem, inst, pc_temp);
endmodule	
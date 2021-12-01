module DataPath(input clk, rst, J_type3, PCsrc3, RegWrite4, ALUsrc2, MemRead3, MemWrite3, beq3, bneq3, input [1:0] forward1,
		forward2, input [2 : 0]ALUop2, input [1 : 0]RegDest2, input [1:0] WriteReg3, input [1 : 0] WriteReg4, output zero_reg_out,
		output[31 : 0]Adrout, output [5 : 0]op, output [5 : 0]func, output [4 : 0]write_adress_reg_out,
		output [4 : 0] write_adress_reg2_out, output [31 : 0]data1_reg_out, output [31 : 0]data2_reg_out, output [4:0] write_adress_reg_in, wire [25:0] inst250_reg_out);

	wire [31 : 0] pc_in;
	wire [31 : 0] pc_out;
	wire [31 : 0] adder_out;
	wire [31 : 0] inst_mem_out;
	wire [31 : 0] pc_4_reg_out;
	wire [31 : 0] instructions_reg_out;
	wire [25 : 0] inst250_reg_2_out;
	wire [31 : 0] left_mux_0_in;
	wire zero;
	wire [31 : 0] left_mux_out;
	wire [31 : 0] pc_4_reg2_out;
	wire [31 : 0] read_data1;
	wire [31 : 0] read_data2;
	wire [31 : 0] pc_4_reg3_out;
	wire [31 : 0] sign_extend_out;
	wire [31 : 0] shift_left2_out;
	wire zero2;
	wire [31 : 0] inst150_reg_out;
	wire [31 : 0] inst150_reg2_out;
	wire [4 : 0] inst2016_reg_out;
	wire [4 : 0] inst1511_reg_out;
	wire [31 : 0] data2_out_reg_out;
	wire [31 : 0] ALU_A_in;
	wire [31 : 0] ALU_B_in;
	wire [31 : 0] ALU_out;
	wire [31 : 0] middle_mux_out;
	wire [31 : 0] ALU_res_reg_out;
	wire [31 : 0] pc_4_reg4_out;
	wire [31 : 0] data1_out_reg_out;
	wire [31 : 0] read_data;
	wire [31 : 0] ALU_res_reg2_out;
	wire [31 : 0] data_mem_reg_out;
	wire [31 : 0] right_mux_out;
	wire [31 : 0] right_mux_out2;
	
	assign op = instructions_reg_out[31 : 26];
	assign func = instructions_reg_out[5 : 0];

	Register #(32) pc_reg(clk, rst, pc_in, pc_out);
	InstMem inst_mem(clk, pc_out, inst_mem_out);
	Adder #(32)adder(4, pc_out, adder_out);
	Mux2 #(32)mux2_1(PCsrc3, adder_out, left_mux_out, pc_in);
	Register #(32) pc_4_reg(clk, rst, adder_out, pc_4_reg_out);
	Register #(32) instructions_reg(clk, rst, inst_mem_out, instructions_reg_out);
	Register #(26) inst250_reg(clk, rst, instructions_reg_out[25 : 0], inst250_reg_out);
	Mux2 #(32) mux2_2((zero_reg_out & beq3) | (~zero_reg_out & bneq3), left_mux_0_in, shift_left2_out, left_mux_out);
	Register #(26)inst250_reg2(clk, rst, inst250_reg_out, inst250_reg_2_out);
	Register #(32)pc_4_reg2(clk, rst, pc_4_reg_out, pc_4_reg2_out);
	Mux2 #(32) mux2_3(J_type3, {pc_4_reg3_out[31 : 28], inst250_reg_2_out, 2'b00}, ALU_A_in, left_mux_0_in);
	RegisterFile register_file(clk, rst, RegWrite4, instructions_reg_out[25 : 21], instructions_reg_out[20 : 16],
			write_adress_reg2_out, right_mux_out, read_data1, read_data2);
	Register #(32) data1_reg(clk, rst, read_data1, data1_reg_out);
	Register #(32) data2_reg(clk, rst, read_data2, data2_reg_out);
	Register #(32) pc_4_reg3(clk, rst, pc_4_reg2_out, pc_4_reg3_out);
	SignEx sign_ex(instructions_reg_out[15 : 0], sign_extend_out);
	assign shift_left2_out = {inst150_reg_out[29 : 0], 2'b00};
	Mux4 #(32) mux4_1(forward1, data1_reg_out, right_mux_out2, right_mux_out, read_data, ALU_A_in);
	Mux4 #(32) mux4_2(forward2, data2_reg_out, right_mux_out2, right_mux_out, read_data, middle_mux_out);
	Register #(32) inst150_reg(clk, rst, sign_extend_out, inst150_reg_out);
	Register #(5) inst2016_reg(clk, rst, instructions_reg_out[20 : 16], inst2016_reg_out);
	Register #(5) inst1511_Reg(clk, rst, instructions_reg_out[15 : 11], inst1511_reg_out);
	assign zero2 = (ALU_A_in == middle_mux_out);
	Mux2 #(32) mux2_4(ALUsrc2, middle_mux_out, inst150_reg_out, ALU_B_in);
	Register #(32) inst150_reg2(clk, rst, inst150_reg_out, inst150_reg2_out);
	Mux4 #(5) mux4_3(RegDest2, inst2016_reg_out, inst1511_reg_out, 5'b11111, 5'b0, write_adress_reg_in);
	ALU ALU(ALUop2, ALU_A_in, ALU_B_in, zero, ALU_out);
	Register #(32) data2_out_reg(clk, rst, middle_mux_out, data2_out_reg_out);
	Register #(32) data1_out_reg(clk, rst, ALU_A_in, data1_out_reg_out);
	Register #(1) zero_reg(clk, rst, zero, zero_reg_out);
	Register #(32) ALU_res_reg(clk, rst, ALU_out, ALU_res_reg_out);
	Register #(5) write_adress_reg(clk, rst, write_adress_reg_in, write_adress_reg_out);
	Register #(32) pc_4_reg4(clk, rst, pc_4_reg3_out, pc_4_reg4_out);
	DataMem data_mem(clk, MemRead3, MemWrite3, ALU_res_reg_out, data2_out_reg_out, read_data);
	assign Adrout = read_data;
	Register #(32) ALU_res_reg2(clk, rst, ALU_res_reg_out, ALU_res_reg2_out);
	Register #(5) write_adress_reg2(clk, rst, write_adress_reg_out, write_adress_reg2_out);
	Register #(32) data_mem_reg(clk, rst, read_data, data_mem_reg_out);
	Mux4 #(32) mux4_4(WriteReg4, ALU_res_reg2_out, {31'b0, ALU_res_reg2_out[31]}, data_mem_reg_out, pc_4_reg4_out - 4, right_mux_out);
	Mux4 #(32) mux4_5(WriteReg3, ALU_res_reg_out, {31'b0, ALU_res_reg_out[31]}, read_data, pc_4_reg3_out - 4, right_mux_out2);
endmodule

module Adder #(parameter N )(input [N - 1 : 0]a, input [N - 1 : 0]b, output [N - 1 : 0]out);
	assign {carry, out} = a + b;
endmodule

module Mux2 #(parameter N)(input s, input [N - 1 : 0]a, input [N - 1 : 0]b, output [N - 1 : 0]out);
	assign out = s ? b : a;
endmodule

module Mux4 #(parameter N)(input [1 : 0]s, input [N - 1 : 0]a, input [N - 1 : 0]b, input [N - 1 : 0]c,
		input [N - 1 : 0]d, output reg[N - 1 : 0]out);
	always @(s, a, b, c, d)
	begin
		case (s)
			2'b00 : out = a;
			2'b01 : out = b;
			2'b10 : out = c;
			2'b11 : out = d;
			default : out = 0;
		endcase
	end
endmodule

module Register #(parameter N)(input clk, rst, input [N - 1 : 0]in, output reg [N - 1 : 0]out);
	always @(posedge clk, posedge rst)
	begin
		if (rst)
			out = 0;
		else
			out = in;
	end
endmodule

module RegisterFile(input clk, rst, reg_write, input [4 : 0]reg1, input [4 : 0]reg2, input [4 : 0]write_reg,
		input [31 : 0]write_data, output reg[31 : 0]data1, output reg [31 : 0]data2);
	reg [31 : 0]registers[31 : 0];
	integer i;
	always@ (reg1, reg2) begin
			data1 = registers[reg1];
			data2 = registers[reg2];
	end
	always @(posedge clk, posedge rst)
	begin
		if (rst)
			for (i = 0; i < 32; i = i + 1)
				registers[i] = 0;
		else
		begin
			if (reg_write & write_reg != 0)
				registers[write_reg] = write_data;
		end
	end
endmodule

module DataMem(input clk, mem_read, mem_write, input [31 : 0]adress, input [31 : 0]data, output reg[31 : 0]out);
	reg [31 : 0]mem[65531 : 0];
	initial begin
		mem[1000] = {29'b0, 3'b011};
		mem[1004] = {29'b0, 3'b111};
		mem[1008] = {29'b0, 3'b100};
		mem[1012] = {29'b0, 3'b001};
		mem[1016] = {29'b0, 3'b101};
		mem[1020] = {29'b0, 3'b110};
		mem[1024] = {29'b0, 3'b011};
		mem[1028] = {29'b0, 3'b101};
		mem[1032] = {29'b0, 3'b011};
		mem[1036] = {29'b0, 3'b011};
	end
	always@ (adress or data) begin
		if(mem_read)
			out = mem[adress];
		if(mem_write)
			mem[adress] = data;
	end
endmodule

module ALU(input [2 : 0]ALUop, input [31 : 0]a, input [31 : 0]b, output zero, output reg [31 : 0]out);
	always @(ALUop, a, b)
	begin
		case(ALUop)
			3'b000 : out = a + b;
			3'b001 : out = a - b;
			default : out = 0;
		endcase
	end
	assign zero = (out == 32'b0);
endmodule

module SignEx(input [15 : 0]in, output [31 : 0]out);
	genvar i;
	generate
		for (i = 0; i < 32; i = i + 1)
			assign out[i] = (i > 15) ? in[15] : in[i];
	endgenerate
endmodule

module InstMem(input clk, input [31 : 0]in, output reg [31 : 0]out);
	reg [31 : 0]ins[65531 : 0];

	initial begin
		$display("Loading Instructions");
		$readmemb("instructions.mem", ins);
	end

	always @(posedge clk)
	begin
		out = ins[in];
	end
endmodule
module DataPath(input clk, rst, J_type, Branch, PCsrc, RegWrite, ALUsrc, MemRead, MemWrite, input [2 : 0]ALUop,
		input [1 : 0]RegDest, input [1 : 0] WriteReg, output zero, output[31 : 0]Adrout, output [5 : 0]op,
		output [5 : 0]func);
	wire [31 : 0] new_pc;
	wire [31 : 0] pc_4;
	wire [31 : 0] J_type_output;
	wire [31 : 0] Branch_output;
	wire [4 : 0] RegDest_output;
	wire [31 : 0] ALUsrc_output;
	wire [31 : 0] old_pc;
	wire [31 : 0] inst;
	wire [31 : 0] data1;
	wire [31 : 0] data2;
	wire [31 : 0] ALU_B_in;
	wire [31 : 0] ALU_out;
	wire [31 : 0] write_out;
	wire [31 : 0] Adr_inp;
	assign op = inst[31 : 26];
	assign func = inst[5 : 0];

	PC_Reg pc_reg(clk, rst, new_pc, old_pc);
	Adder #(32)adder(old_pc, 32'd4, pc_4);
	Mux2 #(32) J_type_mux(J_type, {old_pc[31 : 28], inst[25 : 0], 1'b0, 1'b0}, data1, J_type_output);
	Mux2 #(32) Branch_mux(Branch, J_type_output, {Adr_inp[29:0], 1'b0, 1'b0}, Branch_output);
	Mux2 #(32) PCsrc_mux(PCsrc, pc_4, Branch_output, new_pc);
	InstMem inst_mem(clk, old_pc, inst);
	Mux4 #(5) RegDest_mux(RegDest, inst[20 : 16], inst[15 : 11], 5'b11111, 5'b0, RegDest_output);
	RegisterFile register_file(clk, rst, RegWrite, inst[25 : 21], inst[20 : 16], RegDest_output, write_out, data1, data2);
	Mux2 #(32) ALUsrc_mux(ALUsrc, data2, Adr_inp, ALUsrc_output);
	ALU alu(ALUop, data1, ALUsrc_output, zero, ALU_out);
	SignEx signex(inst[15 : 0], Adr_inp);
	Mux4 #(32) WriteRegmux(WriteReg, ALU_out, {31'b0, ALU_out[31]}, Adrout, pc_4, write_out);
	DataMem data_mem(clk, MemRead, MemWrite, ALU_out, data1, Adrout);
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

module PC_Reg(input clk, rst, input [31 : 0]new_pc, output reg [31 : 0]pc);
	reg first_time;
	always @(posedge clk, posedge rst)
	begin
		if (rst) begin 
			pc = 0;
			first_time = 1'b0;
		end 
		else if(!first_time) 
		begin 
			pc <= pc;
			first_time = 1'b1;
		end
		else
			pc <= new_pc;
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

module DataMem(input clk, mem_read, mem_write, input [31 : 0]adress, input [31 : 0]data, output [31 : 0]out);
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

	assign out = mem[adress];
	always @(posedge clk) begin
		if (mem_write)
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
		$display("Loading instructions");
		$readmemb("instructions.mem", ins);
	end

	always @(posedge clk)
	begin
		out = ins[in];
	end
endmodule
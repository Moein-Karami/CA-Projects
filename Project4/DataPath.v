module DataPath(input clk, rst, ld_pc, pc_dst, cn_ps_ds, adrr, write, ld_inst, push, pop, tos, st_data, ld_a, ALUsrcA,
		input [1 : 0]ALUsrcB, input[1 : 0]ALU_Control, output [2 : 0]inst);
	wire [4 : 0]pc_input;
	wire [4 : 0]pc_output;
	wire [4 : 0]mem_adr;
	wire [7 : 0]mem_out;
	wire [7 : 0]inst_reg_out;
	wire [7 : 0]sign_ex_out;
	wire [7 : 0]d_in;
	wire [7 : 0]d_out;
	wire [7 : 0]A_reg_out;
	wire [7 : 0]A_in;
	wire [7 : 0]B_in;
	wire zero;
	wire [7 : 0]ALU_out;

	Register #(5)pc_reg(clk, rst, ld_pc, pc_input, pc_output);
	Mux2 #(5)adrr_mux(adrr, pc_output, inst_reg_out[4 : 0], mem_adr);
	Memory memory(clk, write, mem_adr, d_out, mem_out);
	Register #(8)inst_reg(clk, rst, ld_inst, mem_out, inst_reg_out);
	SignExtend sign_extend(pc_output, sign_ex_out);
	Mux2 #(8) stack_mux(st_data, mem_out, Alu, d_in);
	Stack stack(clk, rst, push, pop, tos, d_in, d_out);
	Register #(8)A_reg(clk, rst, ld_a, d_out, A_reg_out);
	Mux2 #(8)A_mux(ALUsrcA, sign_ex_out, A_reg_out, A_in);
	Mux4 #(8)B_mux(ALUsrcB, 0, 8'b11111111, 8'b00000001, d_out, B_in);
	ALU alu(ALU_Control, A_in, B_in, zero, ALU_out);
	Mux2 #(5)pc_mux((zero & cn_pc_ds) | pc_dst, inst_reg_out, ALU_out, pc_input);
endmodule

module Register #(parameter N)(input clk, rst, ld, input [N - 1 : 0]in, output reg [N - 1 : 0]out);
	always @(posedge clk, posedge rst)
	begin
		if (rst)
			out = 0;
		else
			out = in;
	end
endmodule

module Mux2 #(parameter N)(input s, input [N - 1 : 0]in1, input [N - 1 : 0]in0, output [N - 1 : 0]out);
	assign out = s ? in1 : in0;
endmodule

module Mux4 #(parameter N)(input [1 : 0]s, input [N - 1 : 0]in3, input [N - 1 : 0]in2, input [N - 1 : 0]in1,
		input [N - 1 : 0]in0, output [N - 1 : 0]out);
	assign out = (s == 2'b11) ? in3 :
			(s == 2'b10) ? in2:
			(s == 2'b01) ? in1 : in0;
endmodule

module Stack(input clk, rst, push, pop, tos, input[7 : 0]d_in, output reg[7 : 0]d_out);
	reg [7 : 0]data[1023 : 0];
	reg [9 : 0]pointer;
	always @(posedge clk, posedge rst)
	begin
		if (rst)
		begin
			pointer = 0;
			d_out = 0;
		end
		else if (push)
		begin
			data[pointer] = d_in;
			pointer = pointer + 1;
		end
		else if (tos)
			d_out = data[pointer - 1];
		else if (pop)
		begin
			pointer = pointer - 1;
			d_out = data[pointer];
		end
	end
endmodule

module Memory(input clk, write, input [4 : 0]adr, input [7 : 0]write_data, output [7 : 0]read_data);
	reg [7 : 0]data[31 : 0];
	assign read_data = data[adr];
	initial begin
		///
	end

	always @(posedge clk) begin
		if (write)
			data[adr] = write_data;
	end
endmodule

module ALU(input [1 : 0]cntrl, input [7 : 0]A, input [7 : 0]B, output zero, output reg[7 : 0]out);
	assign zero = (out == 0);

	always @(A, B, cntrl) begin
		case (cntrl)
			2'b00 : out = A + B;
			2'b01 : out = B - A;
			default : out = A & B;
		endcase
	end
endmodule

module SignExtend(input [4 : 0]in, output [7 : 0]out);
	assign out = {in[4], in[4], in[4], in};
endmodule
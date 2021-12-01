module DataPath(input clk, select_A, output_sel, ldA, ldQ, shf, Q_sel, ldQ0, ld_Div, input [5 : 0]inBus,
		output [5 : 0]outBus);
	wire [6 : 0] A_reg_input;
	wire [6 : 0] A_reg_output;
	wire [5 : 0] Q_reg_output;
	wire [5 : 0] select_A_output;
	wire selected_q0;
	wire q0_reg_input, q0_reg_output;
	wire [6 : 0] minus_output;
	wire [6 : 0] adder_output;
	wire [5 : 0] divisor_reg_output;
	wire [5 : 0] incrementer_input;
	wire [5 : 0] incrementer_output;
	wire [5 : 0] new_A;

	Mux #(6) select_output(A_reg_output[5 : 0], Q_reg_output, output_sel, outBus);
	ShiftRegister #(7) A_register(.clk(clk), .ld(ldA), .shift(shf), .sh_in(Q_reg_output[5]), .ld_in(A_reg_input),
			.out(A_reg_output));
	ShiftRegister #(6) Q_register(.clk(clk), .ld(ldQ), .shift(shf), .sh_in(selected_q0), .ld_in(inBus),
			.out(Q_reg_output));
	Mux #(6) select_A_reg_input(inBus, new_A, select_A, A_reg_input);
	Mux #(1) select_Q0(1'b0, q0_reg_output, selected_q0);
	Inverter #(1) invert_sign(minus_output[6], q0_reg_input);
	Register #(1) Q0_register(.clk(clk), .ld(ldQ0), .in(q0_reg_input), .out(q0_reg_output));
	Register #(6) Divisor_register(.clk(clk), .ld(ld_Div), .in(inBus), .out(divisor_reg_output));
	Inverter #(6) invert_divisor(divisor_reg_output, incrementer_input);
	Incrementer increment(incrementer_input, incrementer_output);
	Adder minus(A_reg_output, {incrementer_output[5], incrementer_output}, minus_output);
	Adder add({divisor_reg_output[5], divisor_reg_output}, minus_output, adder_output);
	Mux #(7) select_new_A(minus_output, adder_output, new_A);
endmodule

module ShiftRegister #(parameter N) (input clk, ld, shift, sh_in, input [N - 1 : 0] ld_in, output reg [N - 1 : 0] out);
	always @(posedge clk) begin
		out <= ld ? ld_in :
				shift ? {out[N - 2 : 0], sh_in} :
				out;
	end
endmodule

module Register #(parameter N) (input clk, ld, input [N - 1 : 0]in, output reg [N - 1 : 0]out);
	always @(posedge clk) begin
		out <= ld ? in : out;
	end
endmodule

module Mux #(parameter N) (input [N - 1 : 0]in0, input [N - 1 : 0]in1, input sel, output [N - 1 : 0]out);
	assign out = sel ? in1 : in0;
endmodule

module Inverter #(parameter N)(input [N - 1 : 0]in, output [N - 1 : 0]out);
	assign out = ~{in};
endmodule

module Incrementer(input [5 : 0]in, output [5 : 0]out);
	assign out = in + 1;
endmodule

module Adder(input [6 : 0]in1, input [6 : 0]in2, output [6 : 0]out);
	assign out = in1 + in2;
endmodule;
module Divisor(input clk, start, input [5 : 0]inBus, output [5 : 0]outBus);
	wire output_sel, ldA, shf, select_A, ldQ, Q_sel, ldQ0, ld_Div;
	Controller controller(.clk(clk), .start(start), .output_sel(output_sel), .ldA(ldA), .shf(shf), .select_A(select_A),
			.ldQ(ldQ), .Q_sel(Q_sel), .ldQ0(ldQ0), .ld_Div(ld_Div));
	DataPath data_path(.clk(clk), .output_sel(output_sel), .ldA(ldA), .shf(shf), .select_A(select_A),
			.ldQ(ldQ), .Q_sel(Q_sel), .ldQ0(ldQ0), .ld_Div(ld_Div), .inBus(inBus), .outBus(outBus));
endmodule;

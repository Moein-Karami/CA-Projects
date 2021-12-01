`timescale 1ns/1ns
module TB_MIPS();
	reg clk = 1'b0;
	reg rst = 1'b1;
	initial begin 
		repeat(1000) #30 clk = ~clk; 
	end 
	TopDesign INS_TOP(clk, rst);
	initial begin 
		#10 rst = 1'b0;
		#10000 $stop;
	end
endmodule
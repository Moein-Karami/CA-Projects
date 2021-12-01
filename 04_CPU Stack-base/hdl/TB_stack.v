`timescale 1ns/1ns
module TB_stack();	
	reg clk = 1'b0;
	reg rst = 1'b1;
	TopDesign Stack_INS(clk, rst);
	initial begin 
		 repeat(70) #30 clk = ~clk;
	end
	initial begin 
		#80 rst = 1'b0;
		#600000 $stop;
	end
endmodule
`timescale 1ns/1ns
module TB_Pipe();
	reg clk = 1'b0;
	reg rst = 1'b1;
	TopDesign TOP(clk, rst);
	initial begin 
		repeat(1000) #30 clk = ~clk;
	end	
	initial begin 
		#100 rst = 1'b0;
		#400000 $stop;
	end
endmodule
module ForwardUnit(input RegWrite3, RegWrite4, input [4 : 0]write_adress_reg, input [4 : 0]write_adress_reg2,
		input [31 : 0]data1_reg, input [31 : 0]data2_reg, output reg [1 : 0]forward1, output reg [1 : 0]forward2);
	always @(RegWrite3, RegWrite4, write_adress_reg, write_adress_reg2, data1_reg, data2_reg)
	begin
		if (RegWrite3 && (write_adress_reg == data1_reg))
			forward1 = 2'b01;
		else if (RegWrite4 && (write_adress_reg == data1_reg))
			forward1 = 2'b10;
		else
			forward1 = 2'b00;

		if (RegWrite3 && (write_adress_reg == data2_reg))
			forward2 = 2'b01;
		else if (RegWrite4 && (write_adress_reg2 == data2_reg))
			forward2 = 2'b10;
		else
			forward2 = 0;
	end
endmodule
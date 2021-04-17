module Controller(input start, clk, output ldA, select_A, ldQ, ld_Div, shf, Q_sel, ldQ0, output_sel, Done);
	reg [4 : 0]ps;
	reg [4 : 0]ns;

	parameter Idle = 5'b00000;
	parameter init1 = 5'b00001;
	parameter init2 = 5'b00010;
	parameter init3 = 5'b00011;
	parameter shift1 = 5'b00100;
	parameter cal1 = 5'b00101;
	parameter ldA1 = 5'b00110;
	parameter shift2 = 5'b00111;
	parameter cal2 = 5'b01000;
	parameter ldA2 = 5'b01001;
	parameter shift3 = 5'b01010;
	parameter cal3 = 5'b01011;
	parameter ldA3 = 5'b01100;
	parameter shift4 = 5'b01101;
	parameter cal4 = 5'b01110;
	parameter ldA4 = 5'b01111;
	parameter shift5 = 5'b10000;
	parameter cal5 = 5'b10001;
	parameter ldA5 = 5'b10010;
	parameter shift6 = 5'b10011;
	parameter cal6 = 5'b10100;
	parameter ldA6 = 5'b10101;
	parameter out1 = 5'b10110;
	parameter out2 = 5'b10111;

	always @(ps, start)
	begin
		if (ps == Idle)
			ns = start ? init1 : Idle;
		else if (ps == init1)
			ns = start ? init1 : init2;
		else if (ps > init1 && ps < out2)
			ns = ns + 1;
		else
			ns = Idle;
	end

	assign ldA = (ps == init1) || (ps == ldA1) || (ps == ldA2) || (ps == ldA3) || (ps == ldA4) || (ps == ldA5) ||
			(ps == ldA6);
	assign select_A = (ps == ldA1) || (ps == ldA2) || (ps == ldA3) || (ps == ldA4) || (ps == ldA5) ||
			(ps == ldA6);
	assign ldQ = (ps == init2);
	assign ld_Div = (ps == init3);
	assign shf = (ps == shift1) || (ps == shift2) || (ps == shift3) || (ps == shift4) || (ps == shift5) || (ps == shift6);
	assign Q_sel = (ps == shift2) || (ps == shift3) || (ps == shift4) || (ps == shift5) || (ps == shift6);
	assign ldQ0 = (ps == ldA1) || (ps == ldA2) || (ps == ldA3) || (ps == ldA4) || (ps == ldA5) ||
			(ps == ldA6);
	assign output_sel = (ps == out1) || (ps == out2);
	assign Done = (ps == out1) || (ps == out2);

	always @(posedge clk)
	begin
		ps <= ns;
	end
endmodule
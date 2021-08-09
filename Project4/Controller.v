module Controller(input pc_temp, clk, rst, [2 : 0]inst, output adrr, ld_inst, ALUsrcA, output [1 : 0]ALUsrcB, output ld_pc, tos,
		pc_dst, ld_a, pop, write, output [1 : 0]ALU_Control, output cn_pc_ds, push, st_data, ld_mem);

	parameter[3 : 0] IF = 4'b0000;
	parameter[3 : 0] JMP = 4'b0001;
	parameter[3 : 0] TOS = 4'b0010;
	parameter[3 : 0] POP = 4'b0011;
	parameter[3 : 0] MW = 4'b0100;
	parameter[3 : 0] PUSH = 4'b0101;
	parameter[3 : 0] JZ = 4'b0110;
	parameter[3 : 0] RT = 4'b0111;
	parameter[3 : 0] POP2 = 4'b1000;
	parameter[3 : 0] NOT = 4'b1001;
	parameter[3 : 0] ALU = 4'b1010;

	reg [3 : 0]ps;
	reg [3 : 0]ns;

	always @(ps, inst) begin
		case(ps)
			IF: ns = TOS;
			TOS: ns = (inst == 3'b110) ? JMP :
					(inst == 3'b101) ? POP :
					(inst == 3'b100) ? PUSH :
					((inst == 3'b000) | (inst == 3'b001) | (inst == 3'b010) | (inst == 3'b011)) ? RT : JZ;
			JMP: ns = IF;
			POP: ns = MW;
			MW: ns = IF;
			PUSH: ns = IF;
			JZ: ns = IF;
			RT: ns = (inst == 3'b011) ? NOT : POP2;
			NOT: ns = IF;
			POP2: ns = ALU;
			ALU: ns = IF;
		endcase
	end

	assign adrr = (ps == IF);
	assign ld_inst = (ps == IF);
	assign ld_mem = (ps == TOS);
	assign ALUsrcA = (ps == IF);
	assign ALUsrcB = (ps == IF) ? 2'b01 :
			(ps == NOT) ? 2'b10 : 2'b00;
	assign ld_pc = (ps == IF) || (ps == JMP) || (ps == JZ && (pc_temp));
	assign tos = (ps == IF) || (ps == TOS);
	assign pc_dst = (ps == JMP);
	assign ld_a = (ps == TOS) || (ps == RT);
	assign pop = (ps == POP) || (ps == RT) || (ps == POP2);
	assign write = (ps == MW);
	assign st_data = (ps == PUSH);
	assign ALU_Control = (ps == NOT) ? 2'b01 :
			(ps == ALU) ? inst[1 : 0] : 2'b00;
	assign cn_pc_ds = (ps == JZ);
	assign push = (ps == NOT) || (ps == ALU) || (ps == PUSH);

	always @(posedge clk, posedge rst)
	begin
		if (rst)
			ps = IF;
		else
			ps = ns;
	end
endmodule

`define BAD_MUX_SEL $fatal("%0t %s %0d: Illegal mux select", $time, `__FILE__, `__LINE__)

import rv32i_types::*;

module datapath
(
	input clk,
	input rst,
	
	// MDR input
	input rv32i_word mem_rdata,
	
	// register loads
	input load_mdr,
	input load_pc,
	input load_ir, 
	input load_regfile, 
	input load_mar, 
	input load_data_out,

	// MUX selects
	input pcmux::pcmux_sel_t pcmux_sel,
	input alumux::alumux1_sel_t alumux1_sel,
	input alumux::alumux2_sel_t alumux2_sel,
	input regfilemux::regfilemux_sel_t regfilemux_sel,
	input marmux::marmux_sel_t marmux_sel,
	input cmpmux::cmpmux_sel_t cmpmux_sel,
	
	// operation inputs
	input alu_ops aluop,
	input branch_funct3_t cmpop,
	
	// IR outputs
	output rv32i_opcode opcode,
	output logic [2:0] funct3,
	output logic [6:0] funct7,
	
	// MAR output
	output rv32i_word mem_address,
	
	// CMP output 
	output logic br_en,
	
	output logic [3:0] mask,
		
	// mem_data_out output
	output rv32i_word mem_wdata // signal used by RVFI Monitor

);

/******************* Signals Needed for RVFI Monitor *************************/
rv32i_word pcmux_out;
rv32i_word mdrreg_out;
/*****************************************************************************/

/****** INTERMEDIATE LOGIC ******/

rv32i_word mem_addr;

assign mem_addr = mem_address;

	// MUX outputs
	rv32i_word alumux1_out;
	rv32i_word alumux2_out;
	rv32i_word regfilemux_out;
	rv32i_word marmux_out;
	rv32i_word cmpmux_out;
	
	logic[1:0] mask_select;
	assign alignment = marmux_out[1:0];

	// Regfile
	rv32i_word rs1_out;
	rv32i_word rs2_out;
	
	// IR
	rv32i_word i_imm;
	rv32i_word u_imm; 
	rv32i_word b_imm; 
	rv32i_word s_imm; 
	rv32i_word j_imm;
	rv32i_reg rs1;
	rv32i_reg rs2;
	rv32i_reg rd;
	
	// ALU
	rv32i_word alu_out;

	// PC
	rv32i_word pc_out;

	// MAR
	rv32i_word mar_out;
	
	// mem_data_out output
	rv32i_word mem_wdata_orig;
	
/**********************************/

/***************************** Registers *************************************/

// Keep Instruction register named `IR` for RVFI Monitor
ir IR(
	.clk(clk),
   .rst(rst),
	.load (load_ir),
	.in (mdrreg_out),
   .funct3(funct3),
   .funct7(funct7),
   .opcode(opcode),
   .i_imm(i_imm),
   .s_imm(s_imm),
   .b_imm(b_imm),
   .u_imm(u_imm),
   .j_imm(j_imm), 
   .rs1(rs1),
   .rs2(rs2),
   .rd(rd)	
);

pc_register PC (
	.clk(clk),
	.rst(rst),
	.load(load_pc),
	.in(pcmux_out),
	.out(pc_out)
);

regfile regfile (
	.clk(clk),
	.rst(rst),
	.load (load_regfile),
	.in (regfilemux_out),
	.src_a (rs1),
	.src_b (rs2),
	.dest (rd),
	.reg_a (rs1_out),
	.reg_b (rs2_out)
);
	
register MDR(
	.clk (clk),
	.rst (rst),
	.load (load_mdr),
	.in (mem_rdata),
	.out (mdrreg_out)
);

assign mem_address = {mar_out[31:2], 2'b0};
register MAR(
	.clk (clk),
	.rst (rst),
	.load (load_mar),
	.in (marmux_out),
	.out (mar_out)
);

register mem_data_out (
	.clk (clk),
	.rst (rst),
	.load (load_data_out),
	.in (rs2_out),
	.out (mem_wdata_orig)
);

/******************************* ALU and CMP *********************************/

alu ALU (
	.aluop(aluop),
	.a(alumux1_out),
	.b(alumux2_out),
	.f(alu_out)
);
	
cmp CMP (
	.cmpop(cmpop),
	.a(rs1_out),
	.b(cmpmux_out),
	.f(br_en)
);

/*****************************************************************************/

/******************************** Muxes **************************************/
always_comb begin : MUXES
	// We provide one (incomplete) example of a mux instantiated using
	// a case statement. Using enumerated types rather than bit vectors
	// provides compile time type safety. Defensive programming is extremely
	// useful in SystemVerilog. In this case, we actually use
	// Offensive programming --- making simulation halt with a fatal message
	// warning when an unexpected mux select value occurs

	unique case (pcmux_sel)
		pcmux::pc_plus4: pcmux_out = pc_out + 4;
		pcmux::alu_out: pcmux_out = alu_out;
		pcmux::alu_mod2: pcmux_out = {alu_out[31:1], 1'b0};
		default: `BAD_MUX_SEL;
	endcase
	
	unique case (regfilemux_sel)
		regfilemux::alu_out: regfilemux_out = alu_out;
		regfilemux::br_en: regfilemux_out = {31'b0, br_en}; //zext br_en
		regfilemux::u_imm: regfilemux_out = u_imm;
		regfilemux::lw: regfilemux_out = mdrreg_out;
		regfilemux::pc_plus4: regfilemux_out = pc_out + 4;

		/**** HALF WORD AND BYTE HANDLING LOAD ****/
		regfilemux::lb: begin // sign extend byte
		
			case (mask_select)
				2'b00: begin 
					regfilemux_out = 32'(signed'(mdrreg_out[7:0])); 
				end 
				2'b01: begin 
					regfilemux_out = 32'(signed'(mdrreg_out[15:8])); 
				end 
				2'b10: begin 
					regfilemux_out = 32'(signed'(mdrreg_out[23:16])); 
				end 
				2'b11: begin 
					regfilemux_out = 32'(signed'(mdrreg_out[31:24])); 
				end
				default: ;
			endcase
			
		end
		
		regfilemux::lbu: begin // zero pad byte
			case (mask_select)
				2'b00: begin 
					regfilemux_out = {24'b0, mdrreg_out[7:0]}; 
				end
				2'b01: begin 
					regfilemux_out = {24'b0, mdrreg_out[15:8]}; 
				end
				2'b10: begin 
					regfilemux_out = {24'b0, mdrreg_out[23:16]}; 
				end
				2'b11: begin 
					regfilemux_out = {24'b0, mdrreg_out[31:24]}; 
				end
				default: ;
			endcase
		end
		
		regfilemux::lh: begin // sign extend half word
			case (mask_select)
				2'b00: begin 
					regfilemux_out = 32'(signed'(mdrreg_out[7:0])); 
				end 
				2'b01: begin 
					regfilemux_out = 32'(signed'(mdrreg_out[15:8])); 
				end 
				2'b10: begin 
					regfilemux_out = 32'(signed'(mdrreg_out[23:16])); 
				end
				2'b11: begin 
					regfilemux_out = 32'd0; 
				end
				default: ;
			endcase
		end 
		
		regfilemux::lhu: begin // zero pad half word
			case (mask_select)
				2'b00: begin 
					regfilemux_out = {16'b0, mdrreg_out[15:0]}; 
				end
				2'b01: begin
					regfilemux_out = {16'b0, mdrreg_out[23:8]}; 
				end
				2'b10: begin 
					regfilemux_out = {16'b0, mdrreg_out[31:16]}; 
				end
				2'b11: begin
					regfilemux_out = 32'd0; 
				end
				default: ;
			endcase
		end 
		
		default: `BAD_MUX_SEL;
	endcase
	
	unique case(marmux_sel)
		marmux::pc_out: marmux_out = pc_out;
		marmux::alu_out: marmux_out = alu_out;
		default: `BAD_MUX_SEL;
	endcase

	unique case(alumux1_sel)
		alumux::rs1_out: alumux1_out = rs1_out;
		alumux::pc_out: alumux1_out = pc_out;
		default: `BAD_MUX_SEL;
	endcase

	unique case(alumux2_sel)
		alumux::i_imm: alumux2_out = i_imm;
		alumux::u_imm: alumux2_out = u_imm;
		alumux::b_imm: alumux2_out = b_imm;
		alumux::s_imm: alumux2_out = s_imm;
		alumux::j_imm: alumux2_out = j_imm;
		alumux::rs2_out: alumux2_out = rs2_out;
		default: `BAD_MUX_SEL;
	endcase

	unique case(cmpmux_sel)
		cmpmux::rs2_out: cmpmux_out = rs2_out;
		cmpmux::i_imm: cmpmux_out = i_imm;
		default: `BAD_MUX_SEL;
	endcase

end
/*****************************************************************************/

/**** HALF WORD AND BYTE HANDLING STORE ****/
always_comb begin: STORE_MASK
	 mask = 4'b1111;
	 mem_wdata = mem_wdata_orig;
	 case (store_funct3_t ' (funct3)) 
		sb: begin 
			case (mask)
				2'b00: begin 
					mem_wdata = {24'b0, mem_wdata_orig[7:0]}; 
					mask = 4'b0001;
				end
				
				2'b01: begin 
					mem_wdata = {16'b0, mem_wdata_orig[7:0], 8'b0}; 
					mask = 4'b0010;
				end
				
				2'b10: begin 
					mem_wdata = {8'b0, mem_wdata_orig[7:0], 16'b0}; 
					mask = 4'b0100;
				end
				
				2'b11: begin 
					mem_wdata = {mem_wdata_orig[7:0], 24'b0}; 
					mask = 4'b1000;
				end
				
				default: begin 
					mem_wdata = mem_wdata_orig;
					mask = 4'hF;
				end 
			endcase 
		end 
		
		sh: begin
			case(mask)
				2'b00: begin
					mem_wdata = {16'b0, mem_wdata_orig[15:0]}; 
					mask = 4'b0011;
				end
				
				2'b01: begin
					mem_wdata = {8'b0, mem_wdata_orig[15:0], 8'b0}; 
					mask = 4'b0110;
				end
				
				2'b10: begin 
					mem_wdata = {mem_wdata_orig[15:0], 16'b0}; 
					mask = 4'b1100;
				end
				
				2'b11: begin
					mem_wdata = mem_wdata_orig;
					mask = 4'hF;
				end

				default: begin 
					mem_wdata = mem_wdata_orig;
					mask = 4'hF;
				end
			endcase
		end
		
	  endcase
end
/********************************************/

endmodule : datapath

import rv32i_types::*;

module cpu
(
    input clk,
    input rst,
    input mem_resp,
    input rv32i_word mem_rdata,
    output logic mem_read,
    output logic mem_write,
    output logic [3:0] mem_byte_enable,
    output rv32i_word mem_address,
    output rv32i_word mem_wdata
);

/******************* Signals Needed for RVFI Monitor *************************/
logic load_pc;
logic load_regfile;
/*****************************************************************************/

logic load_ir;
logic load_mar;
logic load_mdr;
logic load_data_out;

alu_ops aluop;
branch_funct3_t cmpop;

rv32i_opcode opcode;

logic [2:0] funct3;
logic [6:0] funct7;

logic br_en;

//logic [3:0] b_enable;

// control module logic only 
logic [4:0] rs1;
logic [4:0] rs2;

logic [3:0] mask;

/**************************** Control Signals ********************************/
pcmux::pcmux_sel_t pcmux_sel;
alumux::alumux1_sel_t alumux1_sel;
alumux::alumux2_sel_t alumux2_sel;
regfilemux::regfilemux_sel_t regfilemux_sel;
marmux::marmux_sel_t marmux_sel;
cmpmux::cmpmux_sel_t cmpmux_sel;
/*****************************************************************************/

/* Instantiate MP 1 top level blocks here */

// Keep datapath named `datapath` for RVFI Monitor
datapath datapath(

	// INPUTS
	.clk(clk),
	.rst(rst),
	.mem_rdata(mem_rdata),
	
	// register loads
	.load_pc(load_pc),
	.load_ir(load_ir),	
	.load_regfile(load_regfile),
	.load_mar(load_mar),
	.load_mdr(load_mdr),
	.load_data_out(load_data_out),
	
	// mux selects
	.pcmux_sel(pcmux_sel),
	.alumux1_sel(alumux1_sel),
	.alumux2_sel(alumux2_sel),
	.regfilemux_sel(regfilemux_sel),
	.marmux_sel(marmux_sel),
	.cmpmux_sel(cmpmux_sel),
	
	// operation inputs
	.aluop(aluop),
	.cmpop(cmpop),
	
	
	// OUTPUTS
	// IR outputs
	.opcode(opcode),
	.funct3(funct3),
	.funct7(funct7), 
	
	// MAR output 
	.mem_address(mem_address),
	
	// mem-data_out output
	.mem_wdata(mem_wdata),
	
	// lb, lbu, lh, lhu mask
	.mask(mask),
	
	// CMP output 
	.br_en(br_en)

);

// Keep control named `control` for RVFI Monitor
control control(

	// INPUTS
	.clk(clk),
	.rst(rst),
	.mem_resp(mem_resp),
	
	// IR outputs
	.opcode(opcode), 
	.funct3(funct3),
	.funct7(funct7),
	.rs1(rs1),
	.rs2(rs2), 
	
	// CMP output
	.br_en(br_en),
	
	// lb, lbu, lh, lhu mask
	.mask(mask),
	
	
	// OUTPUTS
	// mux selects
	.pcmux_sel(pcmux_sel),
	.alumux1_sel(alumux1_sel),
	.alumux2_sel(alumux2_sel),
	.regfilemux_sel(regfilemux_sel),
	.marmux_sel(marmux_sel),
	.cmpmux_sel(cmpmux_sel),
	
	// operation inputs 
	.aluop(aluop),
	.cmpop(cmpop),
	
	// register loads
	.load_pc(load_pc),
	.load_ir(load_ir),	
	.load_regfile(load_regfile),
	.load_mar(load_mar),
	.load_mdr(load_mdr),
	.load_data_out(load_data_out),
	
	.mem_read(mem_read),
	.mem_write(mem_write),
	.mem_byte_enable(mem_byte_enable)
	
);

endmodule : cpu

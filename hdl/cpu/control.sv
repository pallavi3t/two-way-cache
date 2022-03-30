import rv32i_types::*; /* Import types defined in rv32i_types.sv */

module control
(
    input clk,
    input rst,
	 
    input rv32i_opcode opcode,
	 input logic mem_resp, // ADDED

    input logic [2:0] funct3,
    input logic [6:0] funct7,
    input logic br_en,
    input logic [4:0] rs1,
    input logic [4:0] rs2,
	 
	 input logic [3:0] mask, // ADDED
	 
    output pcmux::pcmux_sel_t pcmux_sel,
    output alumux::alumux1_sel_t alumux1_sel,
    output alumux::alumux2_sel_t alumux2_sel,
    output regfilemux::regfilemux_sel_t regfilemux_sel,
    output marmux::marmux_sel_t marmux_sel,
    output cmpmux::cmpmux_sel_t cmpmux_sel,

    output alu_ops aluop,
	 output branch_funct3_t cmpop,
	 
    output logic load_pc,
    output logic load_ir,
    output logic load_regfile,
    output logic load_mar,
    output logic load_mdr,
    output logic load_data_out,
	 
	 output logic mem_read, // ADDED
    output logic mem_write,// ADDED
    output logic [3:0] mem_byte_enable// ADDED
);

/***************** USED BY RVFIMON --- ONLY MODIFY WHEN TOLD *****************/
logic trap;
logic [4:0] rs1_addr, rs2_addr;
logic [3:0] rmask, wmask;

branch_funct3_t branch_funct3;
store_funct3_t store_funct3;
load_funct3_t load_funct3;
arith_funct3_t arith_funct3;

assign arith_funct3 = arith_funct3_t'(funct3);
assign branch_funct3 = branch_funct3_t'(funct3);
assign load_funct3 = load_funct3_t'(funct3);
assign store_funct3 = store_funct3_t'(funct3);
assign rs1_addr = rs1;
assign rs2_addr = rs2;

always_comb
begin : trap_check
    trap = 0;
    rmask = '0;
    wmask = '0;

    case (opcode)
        op_lui, op_auipc, op_imm, op_reg, op_jal, op_jalr:;

        op_br: begin
            case (branch_funct3)
                beq, bne, blt, bge, bltu, bgeu:;
                default: trap = 1;
            endcase
        end

        op_load: begin
            case (load_funct3)
                lw: rmask = 4'b1111;
					 lh, lhu: rmask = mask /* Modify for MP1 Final */ ;
					 lb, lbu: rmask = mask /* Modify for MP1 Final */ ;
                default: trap = 1;
            endcase
        end

        op_store: begin
            case (store_funct3)
                sw: wmask = 4'b1111;
                sh: wmask = mask /* Modify for MP1 Final */ ;
                sb: wmask = mask /* Modify for MP1 Final */ ;
                default: trap = 1;
            endcase
        end

        default: trap = 1;
    endcase
end
/*****************************************************************************/

enum int unsigned {
    /* List of states */
	 fetch1 = 0,
	 fetch2 = 1,
	 fetch3 = 2,
	 decode = 3, 
	 imm = 4, 
	 lui = 5,
	 auipc = 6, 
	 br = 7,
	 calc_addr = 8, 
	 ld1 = 9, 
	 ld2 = 10,
	 st1 = 11, 
	 st2 = 12,
	 reg_instr = 13,
	 jal = 14,
	 jalr = 15
} state, next_state;

/************************* Function Definitions *******************************/
/**
 *  You do not need to use these functions, but it can be nice to encapsulate
 *  behavior in such a way.  For example, if you use the `loadRegfile`
 *  function, then you only need to ensure that you set the load_regfile bit
 *  to 1'b1 in one place, rather than in many.
 *
 *  SystemVerilog functions must take zero "simulation time" (as opposed to 
 *  tasks).  Thus, they are generally synthesizable, and appropraite
 *  for design code.  Arguments to functions are, by default, input.  But
 *  may be passed as outputs, inouts, or by reference using the `ref` keyword.
**/

/**
 *  Rather than filling up an always_block with a whole bunch of default values,
 *  set the default values for controller output signals in this function,
 *   and then call it at the beginning of your always_comb block.
**/
function void set_defaults();
    load_pc = 1'b0;
    load_ir = 1'b0;
    load_regfile = 1'b0;
    load_mar = 1'b0;
    load_mdr = 1'b0;
    load_data_out = 1'b0;
    pcmux_sel = pcmux::pc_plus4;
    cmpop = branch_funct3;
    alumux1_sel = alumux::rs1_out;
    alumux2_sel = alumux::i_imm;
    regfilemux_sel = regfilemux::alu_out;
    marmux_sel = marmux::pc_out;
    cmpmux_sel = cmpmux::rs2_out;
    aluop = alu_ops'(funct3);
    mem_read =  1'b0;
    mem_write = 1'b0;
endfunction

/**
 *  Use the next several functions to set the signals needed to
 *  load various registers
**/
function void loadPC(pcmux::pcmux_sel_t sel);
    load_pc = 1'b1;
    pcmux_sel = sel;
endfunction

function void loadRegfile(regfilemux::regfilemux_sel_t sel);
    load_regfile = 1'b1;
    regfilemux_sel = sel;
endfunction

function void loadMAR(marmux::marmux_sel_t sel);
    load_mar = 1'b1;
    marmux_sel = sel;
endfunction

function void loadMDR();
    load_mdr = 1'b1;
endfunction

/**
 * SystemVerilog allows for default argument values in a way similar to
 *   C++.
**/
function void setALU(alumux::alumux1_sel_t sel1,
                               alumux::alumux2_sel_t sel2,
                               logic setop = 1'b0, alu_ops op = alu_add);
    /* Student code here */
    alumux1_sel = sel1;
    alumux2_sel = sel2;
    if (setop)
        aluop = op; // else default value
endfunction

function automatic void setCMP(cmpmux::cmpmux_sel_t sel, branch_funct3_t op);
    cmpmux_sel = sel;
    cmpop = op;
endfunction

/*****************************************************************************/

    /* Remember to deal with rst signal */

always_comb
begin : state_actions
    /* Default output assignments */
    set_defaults();
	 
    /* Actions for each state */
    case (state)
	 
        fetch1: begin   
				loadMAR(marmux::pc_out);
		  end
		  
        fetch2: begin
				loadMDR();
				mem_read = 1'b1;
		  end
		  
        fetch3: begin  
				load_ir = 1'b1;
		  end
		  
        decode:; // nothing to do 
		  
        imm: begin      
		  
				case(arith_funct3)
				slt: begin 
					loadRegfile(regfilemux::br_en);
					loadPC(pcmux::pc_plus4);
					setCMP(cmpmux::i_imm, rv32i_types::blt);
				end 
				
				sltu: begin 
					loadRegfile(regfilemux::br_en);
					loadPC(pcmux::pc_plus4);
					setCMP(cmpmux::i_imm, rv32i_types::bltu);
				end
				
				sr: begin 
					loadRegfile(regfilemux::alu_out);
					loadPC(pcmux::pc_plus4);
					if (funct7 == 0)
						aluop = alu_srl;
					else 
						aluop = alu_sra;
				end 
				
				default: begin 
					loadRegfile(regfilemux::alu_out);
					loadPC(pcmux::pc_plus4);
					aluop = alu_ops ' (funct3);
				end 
				endcase
		  end 
		  
		  lui: begin
				loadRegfile(regfilemux::u_imm);
				loadPC(pcmux::pc_plus4);
		  end
		  
		  auipc: begin 
			 setALU(alumux::pc_out, alumux::u_imm, 1'b1);
			 load_regfile = 1'b1;
			 loadPC(pcmux::pc_plus4);
		  end
		  
		  br: begin 
				loadPC(pcmux::pcmux_sel_t ' (br_en));
				setALU(alumux::pc_out, alumux::b_imm, 1'b1);
		  end 

		  calc_addr: begin
				if (opcode == op_store) begin 
					aluop = alu_add;
					alumux2_sel = alumux::s_imm;
					loadMAR(marmux::alu_out);
					load_data_out = 1'b1;
				end
				else begin 
					aluop = alu_add;
					loadMAR(marmux::alu_out);
				end
		  end
		  
        ld1: begin 
				loadMDR();
				mem_read = 1'b1;
		  end
		  
		  ld2: begin 
				case (load_funct3_t ' (funct3))
					lb: loadRegfile(regfilemux::lb);
					lh: loadRegfile(regfilemux::lh);
					lw: loadRegfile(regfilemux::lw);
					lbu: loadRegfile(regfilemux::lbu);
					lhu: loadRegfile(regfilemux::lhu);
					default: loadRegfile(regfilemux::lw);
				endcase
				loadPC(pcmux::pc_plus4);
		  end 
		  
        st1: begin 
				mem_write = 1'b1;
		  end 
		  
        st2: begin 
				loadPC(pcmux::pc_plus4);
		  end 
		  
		  reg_instr: begin 
				case(arith_funct3)
				add: begin //check bit30 for sub if op_reg opcode
					load_regfile = 1'b1;
					loadPC(pcmux::pc_plus4);
					alumux2_sel = alumux::rs2_out;
					if (funct7 == 0)
						aluop = alu_add;
					else
						aluop = alu_sub;
				end 
				
				slt: begin 
					loadRegfile(regfilemux::br_en);
					loadPC(pcmux::pc_plus4);
					cmpop = blt;
					cmpmux_sel = cmpmux::rs2_out;
					alumux2_sel = alumux::rs2_out;
				end 
				
				sltu: begin 
					loadRegfile(regfilemux::br_en);
					loadPC(pcmux::pc_plus4);
					cmpop = bltu;
					cmpmux_sel = cmpmux::rs2_out;
					alumux2_sel = alumux::rs2_out;
				end
				
				sr: begin //check bit30 for logical/arithmetic
					loadPC(pcmux::pc_plus4);
					load_regfile = 1'b1;
					if (funct7 == 0)
						aluop = alu_srl;
					else 
						aluop = alu_sra;
					alumux2_sel = alumux::rs2_out;
				end 
				
				default: begin 
					load_regfile = 1'b1;
					alumux2_sel = alumux::rs2_out;				
					loadPC(pcmux::pc_plus4);
					aluop = alu_ops ' (funct3);
				end 
				endcase
		  end

		  jal: begin
			  load_regfile = 1'b1;
			  regfilemux_sel = regfilemux::pc_plus4;
			  load_pc = 1'b1; 
			  pcmux_sel = pcmux::alu_out;
			  alumux1_sel = alumux::pc_out;
			  alumux2_sel = alumux::j_imm;
			  aluop = alu_add;        
		  end 
  
		  jalr: begin 
			  load_regfile = 1'b1;
			  regfilemux_sel = regfilemux::pc_plus4;
			  load_pc = 1'b1; 
			  pcmux_sel = pcmux::alu_mod2;
			  alumux1_sel = alumux::rs1_out;
			  alumux2_sel = alumux::i_imm;
			  aluop = alu_add;
		  end		  

    endcase
end

always_comb
begin : next_state_logic
    /* Next state information and conditions (if any)
     * for transitioning between states */
	  
	  case (state)
	  
	  	fetch1: next_state = fetch2;
			
		fetch2: begin
			if (mem_resp == 0) 
				next_state = fetch2; 
			else 
				next_state = fetch3;
		end
				
		fetch3: next_state = decode;
			
		decode: begin
			case (opcode)
				op_imm: next_state = imm;
				op_lui: next_state = lui;
				op_load: next_state = calc_addr;
				op_store: next_state = calc_addr;
				op_auipc: next_state = auipc;
				op_br: next_state = br;
				op_jal: next_state = jal;
				op_jalr: next_state = jalr;
				op_reg: next_state = reg_instr;
				op_csr: next_state = fetch1; // idk if i need to do this
			endcase
		end
			
		imm: next_state = fetch1;
			
		lui: next_state = fetch1;
			
		auipc: next_state = fetch1;
			
		br: next_state = fetch1;
			
		calc_addr: begin
			case(opcode)
				op_load:
					next_state = ld1;
				op_store:
					next_state = st1;
				default:
					next_state = fetch1;
			endcase
		end
			
		ld1: begin
			if (mem_resp == 0)
				next_state = ld1;
			else 
				next_state = ld2;
		end
				
		st1: begin
			if (mem_resp == 0)
				next_state = st1;
			else 
				next_state = st2;	
		end
				
		ld2:
			next_state = fetch1;
			
		st2:
			next_state = fetch1;
			
		reg_instr:
			next_state = fetch1;
			
		jal: 
			next_state = fetch1;
			
		jalr:
			next_state = fetch1;			
		
	  endcase
end

always_ff @(posedge clk)
begin: next_state_assignment
    /* Assignment of next state on clock edge */
    if (rst)
        state <= fetch1;
    else
        state <= next_state;
end

endmodule : control

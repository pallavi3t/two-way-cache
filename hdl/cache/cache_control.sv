/* MODIFY. The cache controller. It is a state machine
that controls the behavior of the cache. */

module cache_control #(
    parameter s_offset = 5,
    parameter s_index  = 3,
    parameter s_tag    = 32 - s_offset - s_index,
    parameter s_mask   = 2**s_offset,
    parameter s_line   = 8*s_mask,
    parameter num_sets = 2**s_index
)
(
    input clk, 
	 input rst,

	 input logic [31:0] mem_address,

    // from cpu
    input logic mem_read, mem_write,
    input logic [31:0] write_mask,
	 
	 // to cpu
	 output logic mem_resp,
	
	 // from memory
	 input logic pmem_resp,
	 
	 // to memory
    output logic pmem_read, pmem_write,
	 output logic [31:0] phys_mem_addr,

	 // from datapath
	 input logic lru_out, hit,
	 input logic hit0, hit1,
	 input logic [23:0] Tag0, Tag1, 
	 input logic D0, D1,
	 
    // to datapath
    output logic ldLRU, lru_in, 
	 output logic ldTag0, ldTag1, ldV0, ldV1, ldD0, ldD1,
	 output logic dirty_in, valid_in,	 
    output logic [31:0] write_en,
    output logic way
);

logic [2:0] idx;
assign idx = mem_address[7:5];

function void set_defaults();
	 mem_resp = 1'b0;
    pmem_read = 1'b0;
	 pmem_write = 1'b0;
	 phys_mem_addr = 32'b0;
	 ldLRU = 1'b0;
    lru_in = 1'b0;	 
	 ldTag0 = 1'b0;
	 ldTag1 = 1'b0;
	 ldV0 = 1'b0;
	 ldV1 = 1'b0;
	 ldD0 = 1'b0;
	 ldD1 = 1'b0;
    dirty_in = 1'b0;	 
    valid_in = 1'b0; 	 
    write_en = 31'h0;
	 way = 1'b0;
endfunction
 
enum logic [2:0] { IDLE, CHECK_TAG, WRITE_BACK, ALLOCATE_CACHE1 , ALLOCATE_CACHE2} state, next_state;

always_comb begin : state_actions

	set_defaults();

	case (state)
	
		IDLE: begin 		
		end 

		CHECK_TAG: begin 
			if (hit && mem_read) begin 
				mem_resp = 1'b1;
				ldLRU = 1'b1;
				valid_in = 1'b1;
				
				if (hit0) begin
					way = 1'b0;
					lru_in = 1'b1;
				end 
				
				else if (hit1) begin 
					way = 1'b1;
					lru_in = 1'b0;					
				end
			
			end 
			
			else if (hit && mem_write) begin 
				mem_resp = 1'b1;
				ldLRU = 1'b1;
				valid_in = 1'b1;
				write_en = write_mask;

				if (hit0) begin
					way = 1'b0;
					lru_in = 1'b1;
				end 
				
				else if (hit1) begin 
					way = 1'b1;
					lru_in = 1'b0;
				end
				
				dirty_in = 1'b1;
				ldD0 = 1'b1;
				ldD1 = 1'b1;
			end		

		end 

		  WRITE_BACK: begin
            pmem_write = 1'b1;
				if (!lru_out)
					phys_mem_addr = {Tag0, idx, 5'h0};
				else
					phys_mem_addr = {Tag1, idx, 5'h0};
        end
		  
		  ALLOCATE_CACHE1: begin
            pmem_read = 1'b1;
            write_en = 32'hFFFFFFFF;
        end
		  
		  ALLOCATE_CACHE2: begin
				 way = lru_out;
				 valid_in = 1'b1;
				 case (way)
					1'b0: begin 
						ldTag0 = 1'b1;
						ldV0 = 1'b1;
						ldD0 = 1'b1;
					end
					1'b1: begin 
						ldTag1 = 1'b1;
						ldV1 = 1'b1;
						ldD1 = 1'b1;
					end
				 endcase 
		 end

    endcase
end

always_comb begin : next_state_logic  
		  
	case (state)
 
	  IDLE: begin 
		if (mem_read || mem_write)
			next_state = CHECK_TAG;
		else 
			next_state = IDLE;
	  end 

	  CHECK_TAG: begin
			if (hit) // found data we need, go back to IDLE
				next_state = IDLE;

			else if (!D0 && !D1) // cache miss & old block clean, we allocate space in cache
				next_state = ALLOCATE_CACHE1;				
				
			else if (D0 || D1) // cache miss & old block dirty we write cache to mem
				next_state = WRITE_BACK;
				
			else
				next_state = CHECK_TAG;
	  end

	  WRITE_BACK: begin
			if (!pmem_resp) 
				next_state = WRITE_BACK;
			else 
				next_state = ALLOCATE_CACHE1;
	  end
	  
	  ALLOCATE_CACHE1: begin
			if (!pmem_resp) 
				next_state = ALLOCATE_CACHE1;
			else 
				next_state = ALLOCATE_CACHE2;
	  end
	  
	  ALLOCATE_CACHE2: begin
			next_state = CHECK_TAG;
	  end
	  
 endcase
end


always_ff @(posedge clk)
begin: next_state_assignment
    /* Assignment of next state on clock edge */
    if (rst) begin 
        state <= IDLE;
	 end
	 
	 else begin 
		state <= next_state;
	 end
end


endmodule : cache_control

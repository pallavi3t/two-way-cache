/* MODIFY. Your cache design. It contains the cache
controller, cache datapath, and bus adapter. */

module cache #(
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

	 // from cpu
    input [31:0] mem_address,
	 input mem_read, mem_write,
    input [31:0] mem_wdata,
	 input [3:0] mem_byte_enable,
	 
	 // to cpu
    output [31:0] mem_rdata,
    output mem_resp,

	 // from memory / cacheline adaptor
	 input [255:0] pmem_rdata,       
    input pmem_resp,
	 
    // to memory / cacheline adaptor
    output [31:0] pmem_address,
	 output pmem_read, pmem_write,
    output [255:0] pmem_wdata        
);

/**** between cache control and cache datapath ****/
logic [31:0] write_en;
logic ldLRU, lru_in;
logic ldTag0, ldTag1, ldV0, ldV1, ldD0, ldD1; 
logic dirty_in, valid_in;
logic way;
logic lru_out, hit;
logic hit0, hit1;
logic [23:0] Tag0, Tag1;
logic V0, V1, D0, D1;
/*************************************************/

/**** cache control ****/ 
logic [31:0] write_mask;
/***********************/
cache_control control
(
    .*
);

/**** cache datapath ****/
logic [255:0] datain,  data_mask;
logic [255:0] dataout;

assign pmem_wdata = dataout;

logic [31:0] phys_mem_addr;
/*** PMEM MUX ***/
mem_addr_mux pmem_addr_mux (
	.datain_zero({mem_address[31:5], 5'b0}),
	.datain_one(phys_mem_addr),
	.select(pmem_write),
	.out(pmem_address)
);
/****************/

/*** DATA IN MUX ***/
data_mux choice_datain(
	.datain_zero(pmem_rdata),
	.datain_one(dataout),
	.select(mem_write & hit),
	.out(datain)
);
/********************/

/***********************/
cache_datapath datapath
(
    .*
);

/**** bus adapter ****/
logic [255:0] mem_wdata256;

logic [255:0] mem_rdata256;
assign mem_rdata256 = dataout;

logic [31:0] address;
assign address = mem_address;
/*********************/
bus_adapter bus_adapter
(
    .*, .mem_byte_enable256(write_mask)
);

endmodule : cache

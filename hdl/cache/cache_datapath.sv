/* MODIFY. The cache datapath. It contains the data,
valid, dirty, tag, and LRU arrays, comparators, muxes,
logic gates and other supporting logic. */

module cache_datapath #(
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
	 input logic [31:0] mem_address,
    
    input [31:0] write_en,
    input [255:0] datain,
    output logic [255:0] dataout,

    // from cache control 
    input logic ldLRU, lru_in,	 
	 input logic ldTag0, ldTag1, ldV0, ldV1, ldD0, ldD1,
	 input logic dirty_in, valid_in,
	 input logic way,
	 
	 // to cache control
    output logic lru_out, hit,
	 output logic hit0, hit1,
	 output logic [23:0] Tag0, Tag1, 
	 output logic V0, V1, D0, D1
);

logic [23:0] tag_in;
assign tag_in = mem_address[31:8];

logic [2:0] idx;
assign idx = mem_address[7:5];

// output from data arrays: WAY 0 & WAY 1
logic [255:0] Data0, Data1; 

/**** WAY ARRAYS ****/
// tag array 0 : WAY 0
array #(.s_index(3), .width(24))
tag0_array(
    .clk(clk),
    .rst(rst),
    .read(1'b1),
    .load(ldTag0),
    .rindex(idx),
    .windex(idx),
    .datain(tag_in),
    .dataout(Tag0)
);

// tag array 1 : WAY 1
array #(.s_index(3), .width(24))
tag1_array(
    .clk(clk),
    .rst(rst),
    .read(1'b1),
    .load(ldTag1),
    .rindex(idx),
    .windex(idx),
    .datain(tag_in),
    .dataout(Tag1)
);

// valid array 0 : WAY 0
array #(.s_index(3), .width(1))
valid0_array(
    .clk(clk),
    .rst(rst),
    .read(1'b1),
    .load(ldV0),
    .rindex(idx),
    .windex(idx),
    .datain(valid_in),
    .dataout(V0)  
);

// valid array 1 : WAY 1
array #(.s_index(3), .width(1))
valid1_array(
    .clk(clk),
    .rst(rst),
    .read(1'b1),
    .load(ldV1),
    .rindex(idx),
    .windex(idx),
    .datain(valid_in),
    .dataout(V1)
);

// dirty array 0: WAY 0
array #(.s_index(3), .width(1))
dirty0_array(
    .clk(clk),
    .rst(rst),
    .read(1'b1),
    .load(ldD0),
    .rindex(idx),
    .windex(idx),
    .datain(dirty_in),
    .dataout(D0)
);

// dirty array 1 : WAY 1
array #(.s_index(3), .width(1))
dirty1_array(
    .clk(clk),
    .rst(rst),
    .read(1'b1),
    .load(ldD1),
    .rindex(idx),
    .windex(idx),
    .datain(dirty_in),
    .dataout(D1)
);

// data array 0 : WAY 0
data_array data0_array (
    .clk(clk),
    .rst(rst),
    .read(1'b1),
    .write_en(write_en),
    .rindex(idx),
    .windex(idx),
    .datain(datain),
    .dataout(Data0)
);
// data array 1: WAY 1
data_array data1_array(
    .clk(clk),
    .rst(rst),
    .read(1'b1),
    .write_en(write_en),
    .rindex(idx),
    .windex(idx),
    .datain(datain),
    .dataout(Data1)
);
/*********************/

/*** DATA OUT MUX ***/
data_mux way_dataout(
	.datain_zero(Data0),
	.datain_one(Data1),
	.select(way),
	.out(dataout)
);
/********************/

/**** LRU ****/
array #(.s_index(3), .width(1))
lru_array(
    .clk(clk),
    .rst(rst),
    .read(1'b1),
    .load(ldLRU),
    .rindex(idx),
    .windex(idx),
    .datain(lru_in),
    .dataout(lru_out)
);
/*************/

/**** HIT LOGIC ****/
always_comb begin: hit_logic
	hit0 = (tag_in == Tag0) & V0;
	hit1 = (tag_in == Tag1) & V1;
	hit = hit0 | hit1;
end
/*******************/

endmodule : cache_datapath

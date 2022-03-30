module data_mux (

	input wire [255:0] datain_zero, datain_one,
	input wire select,
	output reg [255:0] out
);

always_comb begin: dataout_logic
	case (select)
		1'b0: out = datain_zero;
		1'b1: out = datain_one;
	endcase
end

endmodule: data_mux

module mem_addr_mux (

	input wire [31:0] datain_zero, datain_one,
	input wire select,
	output reg [31:0] out
);

always_comb begin: dataout_logic
	case (select)
		1'b0: out = datain_zero;
		1'b1: out = datain_one;
	endcase
end

endmodule: mem_addr_mux
module eco32f_true_dpram_sclk #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32
)(
	input 			clk,
	input [ADDR_WIDTH-1:0] 	addr_a,
	input 			we_a,
	input [DATA_WIDTH-1:0] 	din_a,
	output [DATA_WIDTH-1:0] dout_a,
	input [ADDR_WIDTH-1:0] 	addr_b,
	input 			we_b,
	input [DATA_WIDTH-1:0] 	din_b,
	output [DATA_WIDTH-1:0] dout_b
);

reg [DATA_WIDTH-1:0]     mem[(1<<ADDR_WIDTH)-1:0];

reg [DATA_WIDTH-1:0]     rdata_a;
reg [DATA_WIDTH-1:0]     rdata_b;

assign dout_a = rdata_a;
assign dout_b = rdata_b;

always @(posedge clk) begin
	if (we_a) begin
		mem[addr_a] <= din_a;
		rdata_a <= din_a;
	end else begin
		rdata_a <= mem[addr_a];
	end
end

always @(posedge clk) begin
	if (we_b) begin
		mem[addr_b] <= din_b;
		rdata_b <= din_b;
	end else begin
		rdata_b <= mem[addr_b];
	end
end

endmodule

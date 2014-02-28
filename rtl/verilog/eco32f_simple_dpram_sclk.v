/*
 * Copyright (c) 2014, Stefan Kristiansson <stefan.kristiansson@saunalahti.fi>
 * All rights reserved.
 *
 * Redistribution and use in source and non-source forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in non-source form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS WORK IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * WORK, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

module eco32f_simple_dpram_sclk #(
	parameter ADDR_WIDTH = 32,
	parameter DATA_WIDTH = 32,
	parameter ENABLE_BYPASS = 1
)(
	input 			clk,
	input [ADDR_WIDTH-1:0] 	raddr,
	input 			re,
	input [ADDR_WIDTH-1:0] 	waddr,
	input 			we,
	input [DATA_WIDTH-1:0] 	din,
	output [DATA_WIDTH-1:0] dout
);

reg [DATA_WIDTH-1:0]     mem[(1<<ADDR_WIDTH)-1:0];
reg [DATA_WIDTH-1:0]     rdata;

generate
if (ENABLE_BYPASS) begin : bypass_gen
reg [DATA_WIDTH-1:0]     din_r;
reg 			 bypass;

assign dout = bypass ? din_r : rdata;

always @(posedge clk)
	if (re)
		din_r <= din;

always @(posedge clk)
	if (waddr == raddr && we && re)
		bypass <= 1;
	else if (re)
		bypass <= 0;
end else begin
assign dout = rdata;
end
endgenerate

always @(posedge clk) begin
	if (we)
		mem[waddr] <= din;
	if (re)
		rdata <= mem[raddr];
end

endmodule

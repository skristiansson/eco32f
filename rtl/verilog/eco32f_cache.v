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

/*
 * Common module for icache and dcache.
 */

module eco32f_cache (
	input 	      rst,
	input 	      clk,

	input [31:0]  read_addr,
	input [31:0]  match_addr,
	output [31:0] read_data,

	output 	      miss,

	input [31:0]  write_addr,
	input [31:0]  write_data,
	input 	      write_en,

	input 	      invalidate
);

wire [20:0]	tag_mem_dout;
wire [20:0]	tag_mem_din;
wire [19:0]	tag;
wire		valid;

assign tag = tag_mem_dout[19:0];
assign valid = tag_mem_dout[20];
assign miss = tag != match_addr[31:12] | !valid;

//
// Each way is 4 kbyte.
// Cachelines are 32 byte.
// Currently, number of ways supported are one only.
//
assign tag_mem_din = {!invalidate, write_addr[31:12]};

eco32f_simple_dpram_sclk #(
	.ADDR_WIDTH			(7),
	.DATA_WIDTH			(20+1), // tag + valid bit
	.ENABLE_BYPASS			(0)
)
tag_mem
       (
	.clk				(clk),
	.raddr				(read_addr[11:5]),
	.re				(1'b1),
	.waddr				(write_addr[11:5]),
	.we				(write_en),
	.din				(tag_mem_din),
	.dout				(tag_mem_dout)
);

eco32f_simple_dpram_sclk #(
	.ADDR_WIDTH			(10),
	.DATA_WIDTH			(32),
	.ENABLE_BYPASS			(0)
)
data_mem
       (
	.clk				(clk),
	.raddr				(read_addr[11:2]),
	.re				(1'b1),
	.waddr				(write_addr[11:2]),
	.we				(write_en),
	.din				(write_data),
	.dout				(read_data)
);

endmodule

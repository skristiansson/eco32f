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

//
// Instruction Fetch unit
//

`include "eco32f.vh"

module eco32f_fetch #(
	parameter RESET_PC = 32'he0000000
)(
	input 		  rst,
	input 		  clk,

	input 		  if_stall,
	input 		  if_flush,

	output reg [31:0] id_pc,
	output [31:0] 	  id_insn,

	output 		  id_exc_ibus_fault,

	output [31:0] 	  itlb_va,
	input [31:0] 	  itlb_pa,
	input 		  itlb_kmiss,
	input 		  itlb_umiss,
	input 		  itlb_invalid,

	input 		  do_exception,
	input [31:0] 	  exception_pc,
	input 		  do_branch,
	input [31:0] 	  branch_pc,

	// Bus interface (wishbone)
	output reg [31:0] iwbm_adr_o,
	output reg 	  iwbm_stb_o,
	output reg 	  iwbm_cyc_o,
	output [3:0] 	  iwbm_sel_o,
	output 		  iwbm_we_o,
	output [2:0] 	  iwbm_cti_o,
	output [1:0] 	  iwbm_bte_o,
	output [31:0] 	  iwbm_dat_o,
	input 		  iwbm_err_i,
	input 		  iwbm_ack_i,
	input [31:0] 	  iwbm_dat_i,
	input 		  iwbm_rty_i
);

localparam [2:0]
	IF_START_CACHE_HIT_CHECK	= 0,
	IF_CACHE_HIT_CHECK		= 1,
	IF_CACHE_REFILL			= 2;

reg [2:0]	if_state;
reg [31:0]	if_pc;

wire		itlb_miss;

reg [2:0]	refill_cnt;
reg [7:0]	refill_valid;
reg [7:0]	refill_valid_r;

wire		cache_miss;
wire		cache_hit;
wire [31:0]	cache_rd_data;
reg [31:0]	cache_wr_addr;
reg [31:0]	cache_wr_data;
reg		cache_wr_en;

// Cache writes take two cycles to propagate, this is accounted for by
// using the refill_valid_r signals.
assign cache_hit = !cache_miss & refill_valid_r[itlb_pa[4:2]];

assign itlb_miss = itlb_kmiss | itlb_umiss;
assign itlb_va = if_pc;
assign id_insn = cache_hit & !itlb_miss & !itlb_invalid & !do_branch ?
		 cache_rd_data : `ECO32F_INSN_NOP;

/* PC generation */
always @(*)
	if (rst)
		if_pc = RESET_PC;
	else if (do_exception)
		if_pc = exception_pc;
	else if (do_branch)
		if_pc = branch_pc;
	else if (cache_hit & !if_stall)
		if_pc = id_pc + 4;
	else
		if_pc = id_pc;

always @(posedge clk)
	if (rst)
		id_pc <= RESET_PC;
	else if (!if_stall)
		id_pc <= if_pc;

// Wrapping burst with a length of 8.
assign iwbm_bte_o = 2'b10;
assign iwbm_sel_o = 4'b1111;
assign iwbm_cti_o = (refill_cnt == 0) ? 3'b111 : 3'b010;

always @(posedge clk) begin
	if (rst) begin
		refill_valid <= 0;
		refill_valid_r <= 0;
		if_state <= IF_START_CACHE_HIT_CHECK;
	end else begin
		if (cache_wr_en)
			refill_valid[cache_wr_addr[4:2]] <= 1;
		refill_valid_r <= refill_valid;

		iwbm_stb_o <= 0;
		iwbm_cyc_o <= 0;
		cache_wr_en <= 0;

		case (if_state)
		IF_START_CACHE_HIT_CHECK: begin
			if_state <= IF_CACHE_HIT_CHECK;
		end

		IF_CACHE_HIT_CHECK: begin
			iwbm_adr_o <= itlb_pa;
			if (cache_miss & !if_stall & !if_flush) begin
				refill_valid <= 0;
				refill_valid_r <= 0;
				refill_cnt <= 7;
				if_state <= IF_CACHE_REFILL;
			end
		end

		IF_CACHE_REFILL: begin
			iwbm_stb_o <= 1;
			iwbm_cyc_o <= 1;

			if (iwbm_ack_i) begin
				iwbm_adr_o <= {iwbm_adr_o[31:5],
					       iwbm_adr_o[4:0] + 5'd4};
				cache_wr_addr <= iwbm_adr_o;
				cache_wr_data <= iwbm_dat_i;
				cache_wr_en <= 1;
				if (refill_cnt == 0) begin
					if_state <= IF_START_CACHE_HIT_CHECK;
					iwbm_stb_o <= 0;
					iwbm_cyc_o <= 0;
				end
				refill_cnt <= refill_cnt - 1;
			end
		end
		endcase
	end
end

eco32f_cache #(
) eco32f_icache (
		.rst			(rst),
		.clk			(clk),

		.miss			(cache_miss),

		.read_addr		(itlb_va),
		.match_addr		(itlb_pa),
		.read_data		(cache_rd_data),

		.write_addr		(cache_wr_addr),
		.write_data		(cache_wr_data),
		.write_en		(cache_wr_en),
		.invalidate		(0)

);

endmodule

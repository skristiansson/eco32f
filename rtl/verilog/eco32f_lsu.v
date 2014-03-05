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
// Load Store Unit
//

module eco32f_lsu #(
)(
	input 		  rst,
	input 		  clk,

	input 		  ex_stall,

	input 		  ex_lsu_sext,
	input [1:0] 	  ex_lsu_len,

	input 		  ex_op_load,
	input 		  ex_op_store,

	output reg 	  mem_op_load,
	output reg 	  mem_op_store,

	input [31:0] 	  ex_add_result,

	input [31:0] 	  ex_rf_y,

	output [31:0] 	  dtlb_va,
	input [31:0] 	  dtlb_pa,
	input 		  dtlb_umiss,
	input 		  dtlb_kmiss,
	input 		  dtlb_invalid,
	input 		  dtlb_write,

	output 		  lsu_stall,
	output reg [31:0] mem_lsu_result,

	// Exceptions generated in memory stage
	output 		  mem_exc_dtlb_umiss,
	output 		  mem_exc_dtlb_kmiss,
	output 		  mem_exc_dtlb_invalid,

	// Bus interface (wishbone)
	output reg [31:0] dwbm_adr_o,
	output reg 	  dwbm_stb_o,
	output reg 	  dwbm_cyc_o,
	output reg [3:0]  dwbm_sel_o,
	output 		  dwbm_we_o,
	output [2:0] 	  dwbm_cti_o,
	output [1:0] 	  dwbm_bte_o,
	output [31:0] 	  dwbm_dat_o,
	input 		  dwbm_err_i,
	input 		  dwbm_ack_i,
	input [31:0] 	  dwbm_dat_i,
	input 		  dwbm_rty_i
);

localparam [2:0]
	LSU_START_CACHE_HIT_CHECK	= 0,
	LSU_CACHE_HIT_CHECK		= 1,
	LSU_CACHE_REFILL		= 2,
	LSU_START_WRITE			= 3,
	LSU_WRITE			= 4,
	LSU_NONCACHE_READ		= 5,
	LSU_NONCACHE_READ_DONE		= 6;

reg [2:0]	lsu_state;

reg [2:0]	refill_cnt;

reg [3:0]	bsel;

wire		cache_miss;
wire [31:0]	cache_rd_data;
reg [31:0]	cache_wr_addr;
reg [31:0]	cache_wr_data;
reg		cache_wr_en;

wire [31:0]	rd_data;

reg 		mem_lsu_sext;
reg [1:0]	mem_lsu_len;

reg [31:0]	mem_lsu_addr;


// Register signals from execute stage to memory stage
always @(posedge clk)
	if (!ex_stall) begin
		mem_op_load <= ex_op_load;
		mem_op_store <= ex_op_store;
		mem_lsu_sext <= ex_lsu_sext;
		mem_lsu_len <= ex_lsu_len;
		mem_lsu_addr <= ex_add_result;
	end

assign mem_exc_dtlb_umiss = (mem_op_load | mem_op_store) & dtlb_umiss;
assign mem_exc_dtlb_kmiss = (mem_op_load | mem_op_store) & dtlb_kmiss;
assign mem_exc_dtlb_invalid = (mem_op_load | mem_op_store) & dtlb_invalid;
assign mem_exc_dtlb_write = (mem_op_load | mem_op_store) & dtlb_write;

// SJK DEBUG
always @(posedge clk)
	if (mem_exc_dtlb_umiss | mem_exc_dtlb_kmiss | mem_exc_dtlb_write |
	    mem_exc_dtlb_invalid) begin
		$display("dtlb miss/fault!");
		$finish();
	end

// SJK DEBUG END

assign lsu_stall = (mem_op_load & cache_miss ||
		   lsu_state != LSU_CACHE_HIT_CHECK) &&
		   lsu_state != LSU_NONCACHE_READ_DONE;

//
// Mux non-cached read data with cached.
// The cache_wr_data might seem malplaced, but we re-use that to register
// the incoming bus data.
//
assign rd_data = lsu_state == LSU_NONCACHE_READ_DONE ?
		 cache_wr_data : cache_rd_data;

always @(*)
	casez ({mem_lsu_len, mem_lsu_addr[1:0]})
	// byte accesses
	4'b0000:
		mem_lsu_result = {{24{mem_lsu_sext & rd_data[31]}}, rd_data[31:24]};
	4'b0001:
		mem_lsu_result = {{24{mem_lsu_sext & rd_data[23]}}, rd_data[23:16]};
	4'b0010:
		mem_lsu_result = {{24{mem_lsu_sext & rd_data[15]}}, rd_data[15:8]};
	4'b0011:
		mem_lsu_result = {{24{mem_lsu_sext & rd_data[7]}}, rd_data[7:0]};
	// half word accesses
	4'b010?:
		mem_lsu_result = {{16{mem_lsu_sext & rd_data[31]}}, rd_data[31:16]};
	4'b011?:
		mem_lsu_result = {{16{mem_lsu_sext & rd_data[15]}}, rd_data[15:0]};
	// word accesses
	default:
		mem_lsu_result = rd_data;
	endcase

assign dtlb_va = !ex_stall ? ex_add_result : mem_lsu_addr;

assign dwbm_dat_o = cache_wr_data;
assign dwbm_bte_o = 2'b10;
assign dwbm_cti_o = (refill_cnt == 0) ? 3'b111 : 3'b010;

// Byte select
always @(*)
	casez ({ex_lsu_len, dtlb_va[1:0]})
	// byte accesses
	4'b0000:
		bsel = 4'b1000;
	4'b0001:
		bsel = 4'b0100;
	4'b0010:
		bsel = 4'b0010;
	4'b0011:
		bsel = 4'b0001;
	// half word accesses
	4'b010?:
		bsel = 4'b1100;
	4'b011?:
		bsel = 4'b0011;
	// word accesses
	default:
		bsel = 4'b1111;
	endcase

always @(posedge clk)
	if (rst) begin
		lsu_state <= LSU_START_CACHE_HIT_CHECK;
	end else begin
		dwbm_stb_o <= 0;
		dwbm_cyc_o <= 0;
		cache_wr_en <= 0;

		case (lsu_state)
		LSU_START_CACHE_HIT_CHECK: begin
			refill_cnt <= 0;
			lsu_state <= LSU_CACHE_HIT_CHECK;
		end

		LSU_CACHE_HIT_CHECK: begin
			dwbm_adr_o <= dtlb_pa;
			dwbm_sel_o <= bsel;
			//
			// Prioritize loads, the (potential) store will
			// still be waiting for us when we come back here from
			// the refill.
			//
			if (mem_op_load & (mem_lsu_addr[31:28] == 4'hf)) begin
				dwbm_sel_o <= 4'b1111;
				lsu_state <= LSU_NONCACHE_READ;
			end else if (mem_op_load & cache_miss) begin
				refill_cnt <= 7;
				dwbm_sel_o <= 4'b1111;
				lsu_state <= LSU_CACHE_REFILL;
			end else if (ex_op_store) begin
				lsu_state <= LSU_START_WRITE;

				case (ex_lsu_len)
				2'b00: // byte access
					cache_wr_data <= {4{ex_rf_y[7:0]}};
				2'b01: // half word access
					cache_wr_data <= {2{ex_rf_y[15:0]}};
				2'b10: // word access
					cache_wr_data <= ex_rf_y;
				default:
					cache_wr_data <= ex_rf_y;
				endcase
			end
		end

		LSU_CACHE_REFILL: begin
			dwbm_stb_o <= 1;
			dwbm_cyc_o <= 1;

			if (dwbm_ack_i) begin
				dwbm_adr_o <= {dwbm_adr_o[31:5],
					       dwbm_adr_o[4:0] + 5'd4};
				cache_wr_addr <= dwbm_adr_o;
				cache_wr_data <= dwbm_dat_i;
				cache_wr_en <= 1;
				if (refill_cnt == 0) begin
					lsu_state <= LSU_START_CACHE_HIT_CHECK;
					dwbm_stb_o <= 0;
					dwbm_cyc_o <= 0;
				end
				refill_cnt <= refill_cnt - 1;
			end
		end

		LSU_START_WRITE: begin
			dwbm_stb_o <= 1;
			dwbm_cyc_o <= 1;
			dwbm_we_o <= 1;
			dwbm_adr_o <= dtlb_pa;
			cache_wr_addr <= dtlb_pa;

			//
			// Mux in data from cache in case of a cache hit.
			//
			if (!dwbm_sel_o[3])
				cache_wr_data[31:24] <= cache_rd_data[31:24];
			if (!dwbm_sel_o[2])
				cache_wr_data[23:16] <= cache_rd_data[23:16];
			if (!dwbm_sel_o[1])
				cache_wr_data[15:8] <= cache_rd_data[15:8];
			if (!dwbm_sel_o[0])
				cache_wr_data[7:0] <= cache_rd_data[7:0];

			if (!cache_miss)
				cache_wr_en <= 1;

			lsu_state <= LSU_WRITE;
		end

		LSU_WRITE: begin
			dwbm_stb_o <= 1;
			dwbm_cyc_o <= 1;
			dwbm_we_o <= 1;
			if (dwbm_ack_i) begin
				dwbm_stb_o <= 0;
				dwbm_cyc_o <= 0;
				dwbm_we_o <= 0;
				lsu_state <= LSU_CACHE_HIT_CHECK;
			end
		end

		LSU_NONCACHE_READ: begin
			dwbm_stb_o <= 1;
			dwbm_cyc_o <= 1;
			if (dwbm_ack_i) begin
				dwbm_stb_o <= 0;
				dwbm_cyc_o <= 0;
				// Small hack to save some resources:
				// Reuse cache_wr_data to hold the read data.
				cache_wr_data <= dwbm_dat_i;
				lsu_state <= LSU_NONCACHE_READ_DONE;
			end
		end

		LSU_NONCACHE_READ_DONE: begin
			lsu_state <= LSU_CACHE_HIT_CHECK;
		end
		endcase
	end

eco32f_cache #(
) eco32f_dcache (
		.rst			(rst),
		.clk			(clk),

		.miss			(cache_miss),

		.read_addr		(dtlb_va),
		.match_addr		(dtlb_pa),
		.read_data		(cache_rd_data),

		.write_addr		(cache_wr_addr),
		.write_data		(cache_wr_data),
		.write_en		(cache_wr_en),
		.invalidate		(0) // TODO

);

endmodule

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
// MMU
//
// There are no seperate instruction and data tlbs, but seperate
// interfaces are still presented so the tlbs can be read at the different
// stages in the pipeline.
//
// 0x00000000 - 0x7fffffff is user access area
// 0x80000000 - 0xbfffffff is kernel access area
// 0xc0000000 - 0xffffffff is direct-mapped to 0x00000000 - 0x3fffffff
//

`include "eco32f.vh"

module eco32f_mmu #(
)(
	input 		  rst,
	input 		  clk,

	input [31:0] 	  psw,

	// Read/write access to tlbs
	input [4:0] 	  tlb_index,
	input [31:0] 	  tlb_entry_hi_wr_data,
	input 		  tlb_entry_hi_we,
	input [31:0] 	  tlb_entry_lo_wr_data,
	input 		  tlb_entry_lo_we,
	output [31:0] 	  tlb_entry_hi_rd_data,
	output [31:0] 	  tlb_entry_lo_rd_data,

	input [31:0] 	  itlb_va,
	output reg [31:0] itlb_pa,
	output reg 	  itlb_umiss,
	output reg 	  itlb_kmiss,
	output reg 	  itlb_invalid,
	output reg 	  itlb_priv,

	input [31:0] 	  dtlb_va,
	output reg [31:0] dtlb_pa,
	output reg 	  dtlb_umiss,
	output reg 	  dtlb_kmiss,
	output reg 	  dtlb_invalid,
	output reg 	  dtlb_write,
	output reg 	  dtlb_priv,
	input 		  dtlb_write_access
);

// Virtual page frame
reg [19:0] 	    tlb_vpf [31:0];
// Physical page frame
reg [19:0] 	    tlb_ppf [31:0];
// Write enable flag
reg [31:0] 	    tlb_we;
// Valid flag
reg [31:0] 	    tlb_valid;

wire 		    itlb_miss;
wire 		    itlb_direct_map;
wire 		    itlb_kaccess;
wire [4:0] 	    itlb_index;

wire 		    dtlb_miss;
wire 		    dtlb_direct_map;
wire 		    dtlb_kaccess;
wire [4:0] 	    dtlb_index;

//
// Read/write access into tlb entries
//
always @(posedge clk)
	if (tlb_entry_hi_we)
		tlb_vpf[tlb_index] <= tlb_entry_hi_wr_data[31:12];

always @(posedge clk)
	if (tlb_entry_lo_we) begin
		tlb_ppf[tlb_index] <= tlb_entry_lo_wr_data[31:12];
		tlb_we[tlb_index] <= tlb_entry_lo_wr_data[1];
		tlb_valid[tlb_index] <= tlb_entry_lo_wr_data[0];
	end

assign tlb_entry_hi_rd_data = {tlb_vpf[tlb_index], 12'h0};
assign tlb_entry_lo_rd_data = {tlb_ppf[tlb_index], 10'h0, tlb_we[tlb_index],
			       tlb_valid[tlb_index]};

//
// Searches the tlb_vpf for an entry that matches va, returns 0b100000 (32)
// when no entry is found.
//
function [5:0] tlb_search;
input [19:0] va;
integer i;
begin
	tlb_search = 6'b100000;
	for (i = 0; i < 32; i = i + 1) begin
		if (va == tlb_vpf[i])
			tlb_search = i;
	end
end
endfunction

assign itlb_direct_map = &itlb_va[31:30];
assign itlb_kaccess = itlb_va[19];
assign {itlb_miss, itlb_index} = tlb_search(itlb_va[31:12]);

assign dtlb_direct_map = &dtlb_va[31:30];
assign dtlb_kaccess = dtlb_va[19];
assign {dtlb_miss, dtlb_index} = tlb_search(dtlb_va[31:12]);

// Instruction tlb request handling
always @(posedge clk)
	// Direct-mapped
	if (itlb_direct_map)
		itlb_pa <= {2'b00, itlb_va[29:0]};
	else
		itlb_pa <= {tlb_ppf[itlb_index], itlb_va[11:0]};

// Faults and misses
always @(posedge clk) begin
	itlb_kmiss <= itlb_miss & !itlb_direct_map & itlb_kaccess;
	itlb_umiss <= itlb_miss & !itlb_direct_map & !itlb_kaccess;
	itlb_invalid <= !tlb_valid[itlb_index] & !itlb_direct_map;
	itlb_priv <= itlb_kaccess & psw[`ECO32F_SPR_PSW_UC];
end

// Data tlb request handling
always @(posedge clk)
	// Direct-mapped
	if (dtlb_direct_map)
		dtlb_pa <= {2'b00, dtlb_va[29:0]};
	else
		dtlb_pa <= {tlb_ppf[dtlb_index], dtlb_va[11:0]};

// Faults and misses
always @(posedge clk) begin
	dtlb_kmiss <= dtlb_miss & !dtlb_direct_map & dtlb_kaccess;
	dtlb_umiss <= dtlb_miss & !dtlb_direct_map & !dtlb_kaccess;
	dtlb_invalid <= !tlb_valid[dtlb_index] & !dtlb_direct_map;
	dtlb_write <= dtlb_write_access & !tlb_we[dtlb_index] &
		      !dtlb_direct_map;
	dtlb_priv <= dtlb_kaccess & psw[`ECO32F_SPR_PSW_UC];
end

endmodule

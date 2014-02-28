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
// Pipeline control.
// Handles stall logic, exceptions, spr accesses etc.
//
module eco32f_ctrl #(
      )(
	input 		  rst,
	input 		  clk,

	output 		  if_stall,
	output 		  id_stall,
	output 		  ex_stall,
	output 		  mem_stall,

	output 		  if_flush,
	output 		  id_flush,
	output 		  ex_flush,
	output 		  mem_flush,

	input 		  id_bubble,
	input 		  alu_stall,
	input 		  lsu_stall,

	input [31:0] 	  ex_rf_x,
	input [31:0] 	  ex_branch_imm,

	input 		  ex_op_rrb,
	input 		  ex_op_j,
	input 		  ex_op_jr,

	input 		  ex_cond_true,

	output 		  do_branch,
	output [31:0] 	  branch_pc,

	// Special Purpose Registers
	output reg [31:0] psw,
	output reg [31:0] tlb_index,
	output [31:0] 	  tlb_entry_hi_wr_data,
	output 		  tlb_entry_hi_we,
	output [31:0] 	  tlb_entry_lo_wr_data,
	output 		  tlb_entry_lo_we,
	input [31:0] 	  tlb_entry_hi_rd_data,
	input [31:0] 	  tlb_entry_lo_rd_data,
	output reg [31:0] tlb_bad_address,

	// IRQs and exceptions
	input [15:0] 	  irq,
	input 		  ex_exc_ibus_fault,

	output 		  do_exception,
	output [31:0] 	  exception_pc
);

reg		mem_exc_ibus_fault;

// Stall logic - a stall in a later stage stalls the earlier stages
assign mem_stall = lsu_stall;
assign ex_stall = mem_stall | alu_stall;
assign id_stall = ex_stall;
assign if_stall = id_stall | id_bubble;

// Flush logic
assign mem_flush = 0;
assign ex_flush = 0;
assign id_flush = 0;
assign if_flush = 0;

// Branch/jump logic
assign do_branch = ex_op_j | ex_op_jr | ex_op_rrb & ex_cond_true;
assign branch_pc = ex_op_jr ? ex_rf_x : ex_branch_imm;

// Register signals from execute to memory stage
always @(posedge clk)
	if (!mem_stall) begin
		mem_exc_ibus_fault <= ex_exc_ibus_fault;
	end


endmodule
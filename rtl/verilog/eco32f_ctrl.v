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

`include "eco32f.vh"

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

	input [31:0] 	  ex_pc,
	input [31:0] 	  ex_alu_result,

	input [31:0] 	  ex_rf_x,
	input [31:0] 	  ex_rf_y,
	input [31:0] 	  ex_imm,
	input [31:0] 	  ex_branch_imm,

	input 		  ex_op_mvfs,
	input 		  ex_op_mvts,

	input 		  ex_op_rfx,

	input 		  ex_op_rrb,
	input 		  ex_op_j,
	input 		  ex_op_jr,

	input 		  ex_cond_true,

	output 		  do_branch,
	output [31:0] 	  branch_pc,

	// Registered output from execute stage to memory stage
	output reg [31:0] mem_pc,
	output reg [31:0] mem_alu_result,

	// Special Purpose Registers
	output reg [31:0] psw,
	output reg [31:0] tlb_index,
	output reg [31:0] tlb_entry_hi,
	output 		  tlb_entry_hi_we,
	output reg [31:0] tlb_entry_lo,
	output 		  tlb_entry_lo_we,
	input [31:0] 	  tlb_entry_hi_rd_data,
	input [31:0] 	  tlb_entry_lo_rd_data,
	output reg [31:0] tlb_bad_address,

	// IRQs and exceptions
	input [15:0] 	  irq,
	input 		  ex_exc_ibus_fault,
	input 		  ex_exc_illegal_insn,
	input 		  ex_exc_itlb_kmiss,
	input 		  ex_exc_itlb_umiss,
	input 		  ex_exc_itlb_invalid,
	input 		  ex_exc_itlb_priv,
	input 		  ex_exc_div_by_zero,
	input 		  ex_exc_trap,
	input 		  mem_exc_dtlb_kmiss,
	input 		  mem_exc_dtlb_umiss,
	input 		  mem_exc_dtlb_write,
	input 		  mem_exc_dtlb_invalid,
	input 		  mem_exc_dtlb_priv,

	output 		  do_exception,
	output [31:0] 	  exception_pc
);

reg [31:0]	ex_spr_result;

reg		mem_exc_ibus_fault;
reg		mem_exc_illegal_insn;
reg		mem_exc_itlb_kmiss;
reg		mem_exc_itlb_umiss;
reg		mem_exc_itlb_invalid;
reg		mem_exc_itlb_priv;
reg 		mem_exc_div_by_zero;
reg 		mem_exc_trap;
reg		mem_exc_irq;
wire		mem_exc_itlb;
wire		mem_exc_dtlb;

reg [15:0]	mem_masked_irq;

// Stall logic - a stall in a later stage stalls the earlier stages
assign mem_stall = lsu_stall;
assign ex_stall = mem_stall | alu_stall;
assign id_stall = ex_stall;
assign if_stall = id_stall | id_bubble;

// Flush logic
assign mem_flush = 0;
assign ex_flush = do_exception;
assign id_flush = do_exception;
assign if_flush = do_exception | do_branch;

// Branch/jump logic
assign do_branch = ex_op_j | ex_op_jr | ex_op_rfx | ex_op_rrb & ex_cond_true;
assign branch_pc = (ex_op_jr | ex_op_rfx) ? ex_rf_x : ex_branch_imm;

// Register signals from execute to memory stage
always @(posedge clk)
	if (!ex_stall | ex_flush) begin
		mem_exc_ibus_fault <= ex_exc_ibus_fault;
		mem_exc_illegal_insn <= ex_exc_illegal_insn;
		mem_exc_itlb_kmiss <= ex_exc_itlb_kmiss;
		mem_exc_itlb_umiss <= ex_exc_itlb_umiss;
		mem_exc_itlb_invalid <= ex_exc_itlb_invalid;
		mem_exc_itlb_priv <= ex_exc_itlb_priv;
 		mem_exc_div_by_zero <= ex_exc_div_by_zero;
 		mem_exc_trap <= ex_exc_trap;
		mem_exc_irq <= |(psw[`ECO32F_SPR_PSW_IEN] & irq) &
			       psw[`ECO32F_SPR_PSW_IC];
		mem_masked_irq <= psw[`ECO32F_SPR_PSW_IEN] & irq;

		mem_pc <= ex_pc;
		if (ex_op_mvfs)
			mem_alu_result <= ex_spr_result;
		else
			mem_alu_result <= ex_alu_result;

		if (ex_flush) begin
			mem_exc_ibus_fault <= 0;
			mem_exc_illegal_insn <= 0;
			mem_exc_itlb_kmiss <= 0;
			mem_exc_itlb_umiss <= 0;
			mem_exc_itlb_invalid <= 0;
			mem_exc_itlb_priv <= 0;
 			mem_exc_div_by_zero <= 0;
 			mem_exc_trap <= 0;
			mem_exc_irq <= 0;
		end
	end

assign mem_exc_itlb = mem_exc_itlb_kmiss |
		      mem_exc_itlb_umiss |
		      mem_exc_itlb_invalid |
		      mem_exc_itlb_priv;

assign mem_exc_dtlb = mem_exc_dtlb_kmiss |
		      mem_exc_dtlb_umiss |
		      mem_exc_dtlb_write |
		      mem_exc_dtlb_invalid |
		      mem_exc_dtlb_priv;

assign do_exception = (mem_exc_ibus_fault |
		       mem_exc_illegal_insn |
		       mem_exc_itlb |
		       mem_exc_div_by_zero |
		       mem_exc_trap |
		       mem_exc_dtlb |
		       mem_exc_irq) & !mem_stall;

assign exception_pc = ((mem_exc_itlb_umiss | mem_exc_dtlb_umiss) ?
		       32'h00000008 : 32'h00000004) |
		      (psw[`ECO32F_SPR_PSW_V] ? 32'hc0000000 : 32'he0000000);

// SJK DEBUG
/*
always @(posedge clk)
	if (do_exception)
		$display("EXCEPTION!");
	else if (ex_op_rfx & !ex_stall)
		$display("RFX!");
 */
// SJK DEBUG END

//
// Special Purpose Register (SPR) handling
//

// SPR read mux
always @*
	case (ex_imm[15:0])
	`ECO32F_SPR_PSW:
		ex_spr_result = psw;
	`ECO32F_SPR_TLB_INDEX:
		ex_spr_result = tlb_index;
	`ECO32F_SPR_TLB_ENTRY_HI:
		ex_spr_result = tlb_entry_hi;
	`ECO32F_SPR_TLB_ENTRY_LO:
		ex_spr_result = tlb_entry_lo;
	`ECO32F_SPR_TLB_BAD_ADDR:
		ex_spr_result = tlb_bad_address;
	default:
		ex_spr_result = 0;
	endcase

// Find first one
function [4:0] ff1;
input [15:0] bits;
reg [4:0] i;
begin
	for (i = 15; i >= 0; i = i - 1)
		if (bits[i])
			ff1 = i;
end
endfunction

//
// -Verilator- chokes on the function above, so to decode the masked irqs
// to the EID number, just do it open handed here for the time being.
//
reg [5:0] irq_eid;
always @*
	if (mem_masked_irq[0])
		irq_eid = 5'd0;
	else if (mem_masked_irq[1])
		irq_eid = 5'd1;
	else if (mem_masked_irq[2])
		irq_eid = 5'd2;
	else if (mem_masked_irq[3])
		irq_eid = 5'd3;
	else if (mem_masked_irq[4])
		irq_eid = 5'd4;
	else if (mem_masked_irq[5])
		irq_eid = 5'd5;
	else if (mem_masked_irq[6])
		irq_eid = 5'd6;
	else if (mem_masked_irq[7])
		irq_eid = 5'd7;
	else if (mem_masked_irq[8])
		irq_eid = 5'd8;
	else if (mem_masked_irq[9])
		irq_eid = 5'd9;
	else if (mem_masked_irq[10])
		irq_eid = 5'd10;
	else if (mem_masked_irq[11])
		irq_eid = 5'd11;
	else if (mem_masked_irq[12])
		irq_eid = 5'd12;
	else if (mem_masked_irq[13])
		irq_eid = 5'd13;
	else if (mem_masked_irq[14])
		irq_eid = 5'd14;
	else if (mem_masked_irq[15])
		irq_eid = 5'd15;
	else
		irq_eid = 5'd0;

// PSW - Processor Status Word
always @(posedge clk)
	if (rst) begin
		psw <= 0;
	end else if (!mem_stall & do_exception) begin
		// TODO: order by priority
		if (mem_exc_irq)
			psw[`ECO32F_SPR_PSW_EID] <= irq_eid;
		if (mem_exc_ibus_fault)
			psw[`ECO32F_SPR_PSW_EID] <= `ECO32F_EID_BUS_TIMEOUT;
		if (mem_exc_illegal_insn)
			psw[`ECO32F_SPR_PSW_EID] <= `ECO32F_EID_ILLEGAL_INSN;
		if (mem_exc_itlb_priv | mem_exc_dtlb_priv)
			psw[`ECO32F_SPR_PSW_EID] <= `ECO32F_EID_PRIVILEGED_INSN;
		if (mem_exc_div_by_zero)
			psw[`ECO32F_SPR_PSW_EID] <= `ECO32F_EID_DIV_BY_ZERO;
		if (mem_exc_trap)
			psw[`ECO32F_SPR_PSW_EID] <= `ECO32F_EID_TRAP;
		if (mem_exc_itlb_umiss | mem_exc_itlb_kmiss |
			 mem_exc_dtlb_umiss | mem_exc_dtlb_kmiss)
			psw[`ECO32F_SPR_PSW_EID] <= `ECO32F_EID_TLB_MISS;
		if (mem_exc_dtlb_write)
			psw[`ECO32F_SPR_PSW_EID] <= `ECO32F_EID_TLB_WRITE;
		if (mem_exc_itlb_invalid | mem_exc_dtlb_invalid)
			psw[`ECO32F_SPR_PSW_EID] <= `ECO32F_EID_TLB_INVALID;

		psw[`ECO32F_SPR_PSW_IC] <= 0;
		psw[`ECO32F_SPR_PSW_IP] <= psw[`ECO32F_SPR_PSW_IC];
		psw[`ECO32F_SPR_PSW_IO] <= psw[`ECO32F_SPR_PSW_IP];

		psw[`ECO32F_SPR_PSW_UC] <= 0;
		psw[`ECO32F_SPR_PSW_UP] <= psw[`ECO32F_SPR_PSW_UC];
		psw[`ECO32F_SPR_PSW_UO] <= psw[`ECO32F_SPR_PSW_UP];
	end else if (!ex_stall) begin
		if (ex_op_rfx) begin
			psw[`ECO32F_SPR_PSW_IC] <= psw[`ECO32F_SPR_PSW_IP];
			psw[`ECO32F_SPR_PSW_IP] <= psw[`ECO32F_SPR_PSW_IO];
			psw[`ECO32F_SPR_PSW_IO] <= 0;

			psw[`ECO32F_SPR_PSW_UC] <= psw[`ECO32F_SPR_PSW_UP];
			psw[`ECO32F_SPR_PSW_UP] <= psw[`ECO32F_SPR_PSW_UO];
			psw[`ECO32F_SPR_PSW_UO] <= 0;
		end
		if (ex_op_mvts & (ex_imm[15:0] == `ECO32F_SPR_PSW))
			psw <= ex_rf_y;
	end


endmodule
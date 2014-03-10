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
// Top file for eco32f.
// Contains module instantiation and interconnect, no logic.
//

module eco32f #(
	parameter RESET_PC = 32'he0000000
)(
	input 	      rst,
	input 	      clk,

	input [15:0]  irq,

	// Wishbone interface
	output [31:0] iwbm_adr_o,
	output 	      iwbm_stb_o,
	output 	      iwbm_cyc_o,
	output [3:0]  iwbm_sel_o,
	output 	      iwbm_we_o,
	output [2:0]  iwbm_cti_o,
	output [1:0]  iwbm_bte_o,
	output [31:0] iwbm_dat_o,
	input 	      iwbm_err_i,
	input 	      iwbm_ack_i,
	input [31:0]  iwbm_dat_i,
	input 	      iwbm_rty_i,

	output [31:0] dwbm_adr_o,
	output 	      dwbm_stb_o,
	output 	      dwbm_cyc_o,
	output [3:0]  dwbm_sel_o,
	output 	      dwbm_we_o,
	output [2:0]  dwbm_cti_o,
	output [1:0]  dwbm_bte_o,
	output [31:0] dwbm_dat_o,
	input 	      dwbm_err_i,
	input 	      dwbm_ack_i,
	input [31:0]  dwbm_dat_i,
	input 	      dwbm_rty_i
);

/*AUTOWIRE*/
// Beginning of automatic wires (for undeclared instantiated-module outputs)
wire			alu_stall;		// From eco32f_alu of eco32f_alu.v
wire [31:0]		branch_pc;		// From eco32f_ctrl of eco32f_ctrl.v
wire			do_branch;		// From eco32f_ctrl of eco32f_ctrl.v
wire			do_exception;		// From eco32f_ctrl of eco32f_ctrl.v
wire			dtlb_invalid;		// From eco32f_mmu of eco32f_mmu.v
wire			dtlb_kmiss;		// From eco32f_mmu of eco32f_mmu.v
wire [31:0]		dtlb_pa;		// From eco32f_mmu of eco32f_mmu.v
wire			dtlb_priv;		// From eco32f_mmu of eco32f_mmu.v
wire			dtlb_umiss;		// From eco32f_mmu of eco32f_mmu.v
wire [31:0]		dtlb_va;		// From eco32f_lsu of eco32f_lsu.v
wire			dtlb_write;		// From eco32f_mmu of eco32f_mmu.v
wire [31:0]		ex_add_result;		// From eco32f_alu of eco32f_alu.v
wire [31:0]		ex_alu_result;		// From eco32f_alu of eco32f_alu.v
wire [31:0]		ex_branch_imm;		// From eco32f_decode of eco32f_decode.v
wire			ex_bubble;		// From eco32f_decode of eco32f_decode.v
wire			ex_cond_true;		// From eco32f_alu of eco32f_alu.v
wire			ex_exc_ibus_fault;	// From eco32f_decode of eco32f_decode.v
wire			ex_exc_itlb_invalid;	// From eco32f_decode of eco32f_decode.v
wire			ex_exc_itlb_kmiss;	// From eco32f_decode of eco32f_decode.v
wire			ex_exc_itlb_priv;	// From eco32f_decode of eco32f_decode.v
wire			ex_exc_itlb_umiss;	// From eco32f_decode of eco32f_decode.v
wire			ex_flush;		// From eco32f_ctrl of eco32f_ctrl.v
wire [31:0]		ex_imm;			// From eco32f_decode of eco32f_decode.v
wire			ex_imm_sel;		// From eco32f_decode of eco32f_decode.v
wire [1:0]		ex_lsu_len;		// From eco32f_decode of eco32f_decode.v
wire			ex_lsu_sext;		// From eco32f_decode of eco32f_decode.v
wire			ex_op_add;		// From eco32f_decode of eco32f_decode.v
wire			ex_op_and;		// From eco32f_decode of eco32f_decode.v
wire			ex_op_beq;		// From eco32f_decode of eco32f_decode.v
wire			ex_op_bge;		// From eco32f_decode of eco32f_decode.v
wire			ex_op_bgeu;		// From eco32f_decode of eco32f_decode.v
wire			ex_op_bgt;		// From eco32f_decode of eco32f_decode.v
wire			ex_op_bgtu;		// From eco32f_decode of eco32f_decode.v
wire			ex_op_ble;		// From eco32f_decode of eco32f_decode.v
wire			ex_op_bleu;		// From eco32f_decode of eco32f_decode.v
wire			ex_op_blt;		// From eco32f_decode of eco32f_decode.v
wire			ex_op_bltu;		// From eco32f_decode of eco32f_decode.v
wire			ex_op_bne;		// From eco32f_decode of eco32f_decode.v
wire			ex_op_div;		// From eco32f_decode of eco32f_decode.v
wire			ex_op_j;		// From eco32f_decode of eco32f_decode.v
wire			ex_op_jal;		// From eco32f_decode of eco32f_decode.v
wire			ex_op_jr;		// From eco32f_decode of eco32f_decode.v
wire			ex_op_ldhi;		// From eco32f_decode of eco32f_decode.v
wire			ex_op_load;		// From eco32f_decode of eco32f_decode.v
wire			ex_op_mul;		// From eco32f_decode of eco32f_decode.v
wire			ex_op_mvfs;		// From eco32f_decode of eco32f_decode.v
wire			ex_op_mvts;		// From eco32f_decode of eco32f_decode.v
wire			ex_op_or;		// From eco32f_decode of eco32f_decode.v
wire			ex_op_rem;		// From eco32f_decode of eco32f_decode.v
wire			ex_op_rfx;		// From eco32f_decode of eco32f_decode.v
wire			ex_op_rrb;		// From eco32f_decode of eco32f_decode.v
wire			ex_op_sar;		// From eco32f_decode of eco32f_decode.v
wire			ex_op_sll;		// From eco32f_decode of eco32f_decode.v
wire			ex_op_slr;		// From eco32f_decode of eco32f_decode.v
wire			ex_op_store;		// From eco32f_decode of eco32f_decode.v
wire			ex_op_sub;		// From eco32f_decode of eco32f_decode.v
wire			ex_op_tbri;		// From eco32f_decode of eco32f_decode.v
wire			ex_op_tbs;		// From eco32f_decode of eco32f_decode.v
wire			ex_op_tbwi;		// From eco32f_decode of eco32f_decode.v
wire			ex_op_tbwr;		// From eco32f_decode of eco32f_decode.v
wire			ex_op_trap;		// From eco32f_decode of eco32f_decode.v
wire			ex_op_xnor;		// From eco32f_decode of eco32f_decode.v
wire			ex_op_xor;		// From eco32f_decode of eco32f_decode.v
wire [31:0]		ex_pc;			// From eco32f_decode of eco32f_decode.v
wire [4:0]		ex_rf_r_addr;		// From eco32f_decode of eco32f_decode.v
wire			ex_rf_r_we;		// From eco32f_decode of eco32f_decode.v
wire [31:0]		ex_rf_x;		// From eco32f_registerfile of eco32f_registerfile.v
wire [4:0]		ex_rf_x_addr;		// From eco32f_decode of eco32f_decode.v
wire [31:0]		ex_rf_y;		// From eco32f_registerfile of eco32f_registerfile.v
wire [4:0]		ex_rf_y_addr;		// From eco32f_decode of eco32f_decode.v
wire			ex_signed_div;		// From eco32f_decode of eco32f_decode.v
wire			ex_stall;		// From eco32f_ctrl of eco32f_ctrl.v
wire [31:0]		exception_pc;		// From eco32f_ctrl of eco32f_ctrl.v
wire			id_bubble;		// From eco32f_decode of eco32f_decode.v
wire			id_exc_ibus_fault;	// From eco32f_fetch of eco32f_fetch.v
wire			id_exc_itlb_invalid;	// From eco32f_fetch of eco32f_fetch.v
wire			id_exc_itlb_kmiss;	// From eco32f_fetch of eco32f_fetch.v
wire			id_exc_itlb_priv;	// From eco32f_fetch of eco32f_fetch.v
wire			id_exc_itlb_umiss;	// From eco32f_fetch of eco32f_fetch.v
wire			id_flush;		// From eco32f_ctrl of eco32f_ctrl.v
wire [31:0]		id_insn;		// From eco32f_fetch of eco32f_fetch.v
wire [31:0]		id_pc;			// From eco32f_fetch of eco32f_fetch.v
wire [4:0]		id_rf_r_addr;		// From eco32f_decode of eco32f_decode.v
wire			id_rf_r_we;		// From eco32f_decode of eco32f_decode.v
wire [4:0]		id_rf_x_addr;		// From eco32f_decode of eco32f_decode.v
wire [4:0]		id_rf_y_addr;		// From eco32f_decode of eco32f_decode.v
wire			id_stall;		// From eco32f_ctrl of eco32f_ctrl.v
wire			if_flush;		// From eco32f_ctrl of eco32f_ctrl.v
wire			if_stall;		// From eco32f_ctrl of eco32f_ctrl.v
wire			itlb_invalid;		// From eco32f_mmu of eco32f_mmu.v
wire			itlb_kmiss;		// From eco32f_mmu of eco32f_mmu.v
wire [31:0]		itlb_pa;		// From eco32f_mmu of eco32f_mmu.v
wire			itlb_priv;		// From eco32f_mmu of eco32f_mmu.v
wire			itlb_umiss;		// From eco32f_mmu of eco32f_mmu.v
wire [31:0]		itlb_va;		// From eco32f_fetch of eco32f_fetch.v
wire			lsu_stall;		// From eco32f_lsu of eco32f_lsu.v
wire [31:0]		mem_alu_result;		// From eco32f_ctrl of eco32f_ctrl.v
wire			mem_exc_dtlb_invalid;	// From eco32f_lsu of eco32f_lsu.v
wire			mem_exc_dtlb_kmiss;	// From eco32f_lsu of eco32f_lsu.v
wire			mem_exc_dtlb_priv;	// From eco32f_lsu of eco32f_lsu.v
wire			mem_exc_dtlb_umiss;	// From eco32f_lsu of eco32f_lsu.v
wire			mem_exc_dtlb_write;	// From eco32f_lsu of eco32f_lsu.v
wire			mem_flush;		// From eco32f_ctrl of eco32f_ctrl.v
wire [31:0]		mem_lsu_addr;		// From eco32f_lsu of eco32f_lsu.v
wire [31:0]		mem_lsu_result;		// From eco32f_lsu of eco32f_lsu.v
wire			mem_op_load;		// From eco32f_lsu of eco32f_lsu.v
wire			mem_op_mul;		// From eco32f_alu of eco32f_alu.v
wire			mem_op_store;		// From eco32f_lsu of eco32f_lsu.v
wire [31:0]		mem_pc;			// From eco32f_ctrl of eco32f_ctrl.v
wire [4:0]		mem_rf_r_addr;		// From eco32f_registerfile of eco32f_registerfile.v
wire			mem_rf_r_we;		// From eco32f_registerfile of eco32f_registerfile.v
wire			mem_stall;		// From eco32f_ctrl of eco32f_ctrl.v
wire [31:0]		psw;			// From eco32f_ctrl of eco32f_ctrl.v
wire [5:0]		tbs_result;		// From eco32f_mmu of eco32f_mmu.v
wire [31:0]		tlb_bad_address;	// From eco32f_ctrl of eco32f_ctrl.v
wire [31:0]		tlb_entry_hi;		// From eco32f_ctrl of eco32f_ctrl.v
wire [31:0]		tlb_entry_hi_rd_data;	// From eco32f_mmu of eco32f_mmu.v
wire			tlb_entry_hi_we;	// From eco32f_ctrl of eco32f_ctrl.v
wire [31:0]		tlb_entry_lo;		// From eco32f_ctrl of eco32f_ctrl.v
wire [31:0]		tlb_entry_lo_rd_data;	// From eco32f_mmu of eco32f_mmu.v
wire			tlb_entry_lo_we;	// From eco32f_ctrl of eco32f_ctrl.v
wire [4:0]		tlb_index;		// From eco32f_ctrl of eco32f_ctrl.v
wire [31:0]		wb_mul_result;		// From eco32f_alu of eco32f_alu.v
wire			wb_op_mul;		// From eco32f_alu of eco32f_alu.v
wire [31:0]		wb_rf_r;		// From eco32f_writeback of eco32f_writeback.v
wire [4:0]		wb_rf_r_addr;		// From eco32f_writeback of eco32f_writeback.v
wire			wb_rf_r_we;		// From eco32f_writeback of eco32f_writeback.v
// End of automatics

eco32f_fetch
      #(
	.RESET_PC			(RESET_PC)
)
eco32f_fetch
       (/*AUTOINST*/
	// Outputs
	.id_pc				(id_pc[31:0]),
	.id_insn			(id_insn[31:0]),
	.id_exc_ibus_fault		(id_exc_ibus_fault),
	.id_exc_itlb_kmiss		(id_exc_itlb_kmiss),
	.id_exc_itlb_umiss		(id_exc_itlb_umiss),
	.id_exc_itlb_invalid		(id_exc_itlb_invalid),
	.id_exc_itlb_priv		(id_exc_itlb_priv),
	.itlb_va			(itlb_va[31:0]),
	.iwbm_adr_o			(iwbm_adr_o[31:0]),
	.iwbm_stb_o			(iwbm_stb_o),
	.iwbm_cyc_o			(iwbm_cyc_o),
	.iwbm_sel_o			(iwbm_sel_o[3:0]),
	.iwbm_we_o			(iwbm_we_o),
	.iwbm_cti_o			(iwbm_cti_o[2:0]),
	.iwbm_bte_o			(iwbm_bte_o[1:0]),
	.iwbm_dat_o			(iwbm_dat_o[31:0]),
	// Inputs
	.rst				(rst),
	.clk				(clk),
	.if_stall			(if_stall),
	.if_flush			(if_flush),
	.itlb_pa			(itlb_pa[31:0]),
	.itlb_kmiss			(itlb_kmiss),
	.itlb_umiss			(itlb_umiss),
	.itlb_invalid			(itlb_invalid),
	.itlb_priv			(itlb_priv),
	.do_exception			(do_exception),
	.exception_pc			(exception_pc[31:0]),
	.do_branch			(do_branch),
	.branch_pc			(branch_pc[31:0]),
	.iwbm_err_i			(iwbm_err_i),
	.iwbm_ack_i			(iwbm_ack_i),
	.iwbm_dat_i			(iwbm_dat_i[31:0]),
	.iwbm_rty_i			(iwbm_rty_i));

eco32f_decode #(
)
eco32f_decode
       (/*AUTOINST*/
	// Outputs
	.id_bubble			(id_bubble),
	.id_rf_x_addr			(id_rf_x_addr[4:0]),
	.id_rf_y_addr			(id_rf_y_addr[4:0]),
	.id_rf_r_addr			(id_rf_r_addr[4:0]),
	.id_rf_r_we			(id_rf_r_we),
	.ex_op_add			(ex_op_add),
	.ex_op_sub			(ex_op_sub),
	.ex_op_mul			(ex_op_mul),
	.ex_op_div			(ex_op_div),
	.ex_op_rem			(ex_op_rem),
	.ex_op_and			(ex_op_and),
	.ex_op_or			(ex_op_or),
	.ex_op_xor			(ex_op_xor),
	.ex_op_xnor			(ex_op_xnor),
	.ex_op_sll			(ex_op_sll),
	.ex_op_slr			(ex_op_slr),
	.ex_op_sar			(ex_op_sar),
	.ex_op_ldhi			(ex_op_ldhi),
	.ex_op_beq			(ex_op_beq),
	.ex_op_bne			(ex_op_bne),
	.ex_op_ble			(ex_op_ble),
	.ex_op_bleu			(ex_op_bleu),
	.ex_op_blt			(ex_op_blt),
	.ex_op_bltu			(ex_op_bltu),
	.ex_op_bge			(ex_op_bge),
	.ex_op_bgeu			(ex_op_bgeu),
	.ex_op_bgt			(ex_op_bgt),
	.ex_op_bgtu			(ex_op_bgtu),
	.ex_op_load			(ex_op_load),
	.ex_op_store			(ex_op_store),
	.ex_op_mvfs			(ex_op_mvfs),
	.ex_op_mvts			(ex_op_mvts),
	.ex_op_tbs			(ex_op_tbs),
	.ex_op_tbwr			(ex_op_tbwr),
	.ex_op_tbri			(ex_op_tbri),
	.ex_op_tbwi			(ex_op_tbwi),
	.ex_op_trap			(ex_op_trap),
	.ex_op_rfx			(ex_op_rfx),
	.ex_op_rrb			(ex_op_rrb),
	.ex_op_jal			(ex_op_jal),
	.ex_op_j			(ex_op_j),
	.ex_op_jr			(ex_op_jr),
	.ex_signed_div			(ex_signed_div),
	.ex_lsu_sext			(ex_lsu_sext),
	.ex_lsu_len			(ex_lsu_len[1:0]),
	.ex_branch_imm			(ex_branch_imm[31:0]),
	.ex_pc				(ex_pc[31:0]),
	.ex_exc_ibus_fault		(ex_exc_ibus_fault),
	.ex_exc_itlb_kmiss		(ex_exc_itlb_kmiss),
	.ex_exc_itlb_umiss		(ex_exc_itlb_umiss),
	.ex_exc_itlb_invalid		(ex_exc_itlb_invalid),
	.ex_exc_itlb_priv		(ex_exc_itlb_priv),
	.ex_rf_x_addr			(ex_rf_x_addr[4:0]),
	.ex_rf_y_addr			(ex_rf_y_addr[4:0]),
	.ex_rf_r_addr			(ex_rf_r_addr[4:0]),
	.ex_rf_r_we			(ex_rf_r_we),
	.ex_bubble			(ex_bubble),
	.ex_imm_sel			(ex_imm_sel),
	.ex_imm				(ex_imm[31:0]),
	// Inputs
	.rst				(rst),
	.clk				(clk),
	.id_stall			(id_stall),
	.id_flush			(id_flush),
	.id_pc				(id_pc[31:0]),
	.id_insn			(id_insn[31:0]),
	.id_exc_ibus_fault		(id_exc_ibus_fault),
	.id_exc_itlb_kmiss		(id_exc_itlb_kmiss),
	.id_exc_itlb_umiss		(id_exc_itlb_umiss),
	.id_exc_itlb_invalid		(id_exc_itlb_invalid),
	.id_exc_itlb_priv		(id_exc_itlb_priv),
	.mem_rf_r_addr			(mem_rf_r_addr[4:0]),
	.mem_op_mul			(mem_op_mul));

eco32f_registerfile #(
)
eco32f_registerfile
       (/*AUTOINST*/
	// Outputs
	.mem_rf_r_addr			(mem_rf_r_addr[4:0]),
	.mem_rf_r_we			(mem_rf_r_we),
	.ex_rf_x			(ex_rf_x[31:0]),
	.ex_rf_y			(ex_rf_y[31:0]),
	// Inputs
	.rst				(rst),
	.clk				(clk),
	.id_stall			(id_stall),
	.ex_stall			(ex_stall),
	.ex_flush			(ex_flush),
	.id_rf_x_addr			(id_rf_x_addr[4:0]),
	.id_rf_y_addr			(id_rf_y_addr[4:0]),
	.ex_rf_x_addr			(ex_rf_x_addr[4:0]),
	.ex_rf_y_addr			(ex_rf_y_addr[4:0]),
	.ex_rf_r_addr			(ex_rf_r_addr[4:0]),
	.ex_rf_r_we			(ex_rf_r_we),
	.mem_alu_result			(mem_alu_result[31:0]),
	.wb_rf_r_addr			(wb_rf_r_addr[4:0]),
	.wb_rf_r_we			(wb_rf_r_we),
	.wb_rf_r			(wb_rf_r[31:0]));

eco32f_alu #(
)
eco32f_alu
       (/*AUTOINST*/
	// Outputs
	.alu_stall			(alu_stall),
	.ex_add_result			(ex_add_result[31:0]),
	.ex_cond_true			(ex_cond_true),
	.ex_alu_result			(ex_alu_result[31:0]),
	.mem_op_mul			(mem_op_mul),
	.wb_op_mul			(wb_op_mul),
	.wb_mul_result			(wb_mul_result[31:0]),
	// Inputs
	.rst				(rst),
	.clk				(clk),
	.id_stall			(id_stall),
	.ex_stall			(ex_stall),
	.mem_stall			(mem_stall),
	.ex_flush			(ex_flush),
	.mem_flush			(mem_flush),
	.id_pc				(id_pc[31:0]),
	.ex_op_add			(ex_op_add),
	.ex_op_sub			(ex_op_sub),
	.ex_op_mul			(ex_op_mul),
	.ex_op_div			(ex_op_div),
	.ex_op_rem			(ex_op_rem),
	.ex_op_or			(ex_op_or),
	.ex_op_and			(ex_op_and),
	.ex_op_xor			(ex_op_xor),
	.ex_op_xnor			(ex_op_xnor),
	.ex_op_sll			(ex_op_sll),
	.ex_op_slr			(ex_op_slr),
	.ex_op_sar			(ex_op_sar),
	.ex_op_beq			(ex_op_beq),
	.ex_op_bne			(ex_op_bne),
	.ex_op_ble			(ex_op_ble),
	.ex_op_bleu			(ex_op_bleu),
	.ex_op_blt			(ex_op_blt),
	.ex_op_bltu			(ex_op_bltu),
	.ex_op_bge			(ex_op_bge),
	.ex_op_bgeu			(ex_op_bgeu),
	.ex_op_bgt			(ex_op_bgt),
	.ex_op_bgtu			(ex_op_bgtu),
	.ex_op_jal			(ex_op_jal),
	.ex_op_rrb			(ex_op_rrb),
	.ex_signed_div			(ex_signed_div),
	.ex_rf_x			(ex_rf_x[31:0]),
	.ex_rf_y			(ex_rf_y[31:0]),
	.ex_imm				(ex_imm[31:0]),
	.ex_imm_sel			(ex_imm_sel));

eco32f_lsu #(
)
eco32f_lsu
       (/*AUTOINST*/
	// Outputs
	.mem_op_load			(mem_op_load),
	.mem_op_store			(mem_op_store),
	.mem_lsu_addr			(mem_lsu_addr[31:0]),
	.dtlb_va			(dtlb_va[31:0]),
	.lsu_stall			(lsu_stall),
	.mem_lsu_result			(mem_lsu_result[31:0]),
	.mem_exc_dtlb_umiss		(mem_exc_dtlb_umiss),
	.mem_exc_dtlb_kmiss		(mem_exc_dtlb_kmiss),
	.mem_exc_dtlb_invalid		(mem_exc_dtlb_invalid),
	.mem_exc_dtlb_priv		(mem_exc_dtlb_priv),
	.mem_exc_dtlb_write		(mem_exc_dtlb_write),
	.dwbm_adr_o			(dwbm_adr_o[31:0]),
	.dwbm_stb_o			(dwbm_stb_o),
	.dwbm_cyc_o			(dwbm_cyc_o),
	.dwbm_sel_o			(dwbm_sel_o[3:0]),
	.dwbm_we_o			(dwbm_we_o),
	.dwbm_cti_o			(dwbm_cti_o[2:0]),
	.dwbm_bte_o			(dwbm_bte_o[1:0]),
	.dwbm_dat_o			(dwbm_dat_o[31:0]),
	// Inputs
	.rst				(rst),
	.clk				(clk),
	.ex_stall			(ex_stall),
	.ex_flush			(ex_flush),
	.mem_flush			(mem_flush),
	.ex_lsu_sext			(ex_lsu_sext),
	.ex_lsu_len			(ex_lsu_len[1:0]),
	.ex_op_load			(ex_op_load),
	.ex_op_store			(ex_op_store),
	.ex_add_result			(ex_add_result[31:0]),
	.ex_rf_y			(ex_rf_y[31:0]),
	.dtlb_pa			(dtlb_pa[31:0]),
	.dtlb_umiss			(dtlb_umiss),
	.dtlb_kmiss			(dtlb_kmiss),
	.dtlb_invalid			(dtlb_invalid),
	.dtlb_priv			(dtlb_priv),
	.dtlb_write			(dtlb_write),
	.dwbm_err_i			(dwbm_err_i),
	.dwbm_ack_i			(dwbm_ack_i),
	.dwbm_dat_i			(dwbm_dat_i[31:0]),
	.dwbm_rty_i			(dwbm_rty_i));

eco32f_ctrl #(
)
eco32f_ctrl
       (/*AUTOINST*/
	// Outputs
	.if_stall			(if_stall),
	.id_stall			(id_stall),
	.ex_stall			(ex_stall),
	.mem_stall			(mem_stall),
	.if_flush			(if_flush),
	.id_flush			(id_flush),
	.ex_flush			(ex_flush),
	.mem_flush			(mem_flush),
	.do_branch			(do_branch),
	.branch_pc			(branch_pc[31:0]),
	.mem_pc				(mem_pc[31:0]),
	.mem_alu_result			(mem_alu_result[31:0]),
	.psw				(psw[31:0]),
	.tlb_index			(tlb_index[4:0]),
	.tlb_entry_hi			(tlb_entry_hi[31:0]),
	.tlb_entry_hi_we		(tlb_entry_hi_we),
	.tlb_entry_lo			(tlb_entry_lo[31:0]),
	.tlb_entry_lo_we		(tlb_entry_lo_we),
	.tlb_bad_address		(tlb_bad_address[31:0]),
	.do_exception			(do_exception),
	.exception_pc			(exception_pc[31:0]),
	// Inputs
	.rst				(rst),
	.clk				(clk),
	.id_bubble			(id_bubble),
	.alu_stall			(alu_stall),
	.lsu_stall			(lsu_stall),
	.ex_pc				(ex_pc[31:0]),
	.ex_alu_result			(ex_alu_result[31:0]),
	.ex_rf_x			(ex_rf_x[31:0]),
	.ex_rf_y			(ex_rf_y[31:0]),
	.ex_imm				(ex_imm[31:0]),
	.ex_branch_imm			(ex_branch_imm[31:0]),
	.ex_op_mvfs			(ex_op_mvfs),
	.ex_op_mvts			(ex_op_mvts),
	.ex_op_tbs			(ex_op_tbs),
	.ex_op_tbwr			(ex_op_tbwr),
	.ex_op_tbri			(ex_op_tbri),
	.ex_op_tbwi			(ex_op_tbwi),
	.ex_op_trap			(ex_op_trap),
	.ex_op_rfx			(ex_op_rfx),
	.ex_op_rrb			(ex_op_rrb),
	.ex_op_j			(ex_op_j),
	.ex_op_jr			(ex_op_jr),
	.ex_cond_true			(ex_cond_true),
	.mem_lsu_addr			(mem_lsu_addr[31:0]),
	.tlb_entry_hi_rd_data		(tlb_entry_hi_rd_data[31:0]),
	.tlb_entry_lo_rd_data		(tlb_entry_lo_rd_data[31:0]),
	.tbs_result			(tbs_result[5:0]),
	.irq				(irq[15:0]),
	.ex_exc_ibus_fault		(ex_exc_ibus_fault),
	.ex_exc_illegal_insn		(ex_exc_illegal_insn),
	.ex_exc_itlb_kmiss		(ex_exc_itlb_kmiss),
	.ex_exc_itlb_umiss		(ex_exc_itlb_umiss),
	.ex_exc_itlb_invalid		(ex_exc_itlb_invalid),
	.ex_exc_itlb_priv		(ex_exc_itlb_priv),
	.ex_exc_div_by_zero		(ex_exc_div_by_zero),
	.mem_exc_dtlb_kmiss		(mem_exc_dtlb_kmiss),
	.mem_exc_dtlb_umiss		(mem_exc_dtlb_umiss),
	.mem_exc_dtlb_write		(mem_exc_dtlb_write),
	.mem_exc_dtlb_invalid		(mem_exc_dtlb_invalid),
	.mem_exc_dtlb_priv		(mem_exc_dtlb_priv));

eco32f_writeback #(
)
eco32f_writeback
       (/*AUTOINST*/
	// Outputs
	.wb_rf_r			(wb_rf_r[31:0]),
	.wb_rf_r_we			(wb_rf_r_we),
	.wb_rf_r_addr			(wb_rf_r_addr[4:0]),
	// Inputs
	.rst				(rst),
	.clk				(clk),
	.do_exception			(do_exception),
	.mem_stall			(mem_stall),
	.mem_pc				(mem_pc[31:0]),
	.mem_alu_result			(mem_alu_result[31:0]),
	.mem_lsu_result			(mem_lsu_result[31:0]),
	.mem_rf_r_we			(mem_rf_r_we),
	.mem_rf_r_addr			(mem_rf_r_addr[4:0]),
	.mem_op_load			(mem_op_load),
	.wb_op_mul			(wb_op_mul),
	.wb_mul_result			(wb_mul_result[31:0]));

eco32f_mmu #(
)
eco32f_mmu
       (/*AUTOINST*/
	// Outputs
	.tlb_entry_hi_rd_data		(tlb_entry_hi_rd_data[31:0]),
	.tlb_entry_lo_rd_data		(tlb_entry_lo_rd_data[31:0]),
	.tbs_result			(tbs_result[5:0]),
	.itlb_pa			(itlb_pa[31:0]),
	.itlb_umiss			(itlb_umiss),
	.itlb_kmiss			(itlb_kmiss),
	.itlb_invalid			(itlb_invalid),
	.itlb_priv			(itlb_priv),
	.dtlb_pa			(dtlb_pa[31:0]),
	.dtlb_umiss			(dtlb_umiss),
	.dtlb_kmiss			(dtlb_kmiss),
	.dtlb_invalid			(dtlb_invalid),
	.dtlb_write			(dtlb_write),
	.dtlb_priv			(dtlb_priv),
	// Inputs
	.rst				(rst),
	.clk				(clk),
	.psw				(psw[31:0]),
	.tlb_index			(tlb_index[4:0]),
	.tlb_entry_hi			(tlb_entry_hi[31:0]),
	.tlb_entry_hi_we		(tlb_entry_hi_we),
	.tlb_entry_lo			(tlb_entry_lo[31:0]),
	.tlb_entry_lo_we		(tlb_entry_lo_we),
	.itlb_va			(itlb_va[31:0]),
	.dtlb_va			(dtlb_va[31:0]),
	.ex_op_store			(ex_op_store));

endmodule

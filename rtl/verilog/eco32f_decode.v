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

`include "eco32f.vh"

module eco32f_decode #(
)(
	input 		  rst,
	input 		  clk,

	input 		  id_stall,
	input 		  id_flush,
	input [31:0] 	  id_pc,
	input [31:0] 	  id_insn,

	output 		  id_bubble,

	// Exceptions from fetch stage
	input 		  id_exc_ibus_fault,

	// Register file address and control signals
	output [4:0] 	  id_rf_x_addr,
	output [4:0] 	  id_rf_y_addr,
	output [4:0] 	  id_rf_r_addr,
	output 		  id_rf_r_we,
	input [4:0] 	  mem_rf_r_addr,

	// mul instruction in memory stage
	input 		  mem_op_mul,

	// Registered output to execute stage
	output reg 	  ex_op_add,
	output reg 	  ex_op_sub,
	output reg 	  ex_op_mul,
	output reg 	  ex_op_div,
	output reg 	  ex_op_rem,
	output reg 	  ex_op_and,
	output reg 	  ex_op_or,
	output reg 	  ex_op_xor,
	output reg 	  ex_op_xnor,
	output reg 	  ex_op_sll,
	output reg 	  ex_op_slr,
	output reg 	  ex_op_sar,
	output reg 	  ex_op_ldhi,
	output reg 	  ex_op_beq,
	output reg 	  ex_op_bne,
	output reg 	  ex_op_ble,
	output reg 	  ex_op_bleu,
	output reg 	  ex_op_blt,
	output reg 	  ex_op_bltu,
	output reg 	  ex_op_bge,
	output reg 	  ex_op_bgeu,
	output reg 	  ex_op_bgt,
	output reg 	  ex_op_bgtu,
	output reg 	  ex_op_load,
	output reg 	  ex_op_store,
	output reg 	  ex_op_mvfs,
	output reg 	  ex_op_mvts,
	output reg 	  ex_op_tbs,
	output reg 	  ex_op_tbwr,
	output reg 	  ex_op_tbri,
	output reg 	  ex_op_tbwi,

	output reg 	  ex_op_rfx,

	output reg 	  ex_op_rrb,
	output reg 	  ex_op_jal,
	output reg 	  ex_op_j,
	output reg 	  ex_op_jr,

	output reg 	  ex_signed_div,

	output reg 	  ex_lsu_sext,
	output reg [1:0]  ex_lsu_len,

	output reg [31:0] ex_branch_imm,

	output reg [31:0] ex_pc /* verilator public */,

	output reg 	  ex_exc_ibus_fault,

	output reg [4:0]  ex_rf_x_addr,
	output reg [4:0]  ex_rf_y_addr,
	output reg [4:0]  ex_rf_r_addr,
	output 		  ex_rf_r_we,

	output reg 	  ex_bubble,
	output reg 	  ex_imm_sel,
	output reg [31:0] ex_imm
);

wire [5:0]	op_code;

// op code types
wire		op_rrr; // three registers
wire		op_rrs; // two registers and a simm16
wire		op_rri; // two registers and an imm16
wire		op_rrb; // two registers and a pc-relative simm16 (branch)
wire		op_j;   // jump to pc-relative simm26
wire		op_jr;  // jump to register
wire		op_jal; // jump and link

wire 		op_add;
wire 		op_sub;
wire 		op_mul;
wire 		op_div;
wire 		op_rem;
wire 		op_and;
wire 		op_or;
wire 		op_xor;
wire 		op_xnor;
wire 		op_sll;
wire 		op_slr;
wire 		op_sar;
wire 		op_ldhi;
wire 		op_beq;
wire 		op_bne;
wire 		op_ble;
wire 		op_bleu;
wire	 	op_blt;
wire 		op_bltu;
wire 		op_bge;
wire 		op_bgeu;
wire 		op_bgt;
wire 		op_bgtu;
wire 		op_load;
wire 		op_store;
wire		op_mvfs;
wire		op_mvts;
wire		op_tbs;
wire		op_tbwr;
wire		op_tbri;
wire		op_tbwi;

wire		op_rfx;

wire		lsu_zext;
wire		lsu_sext;
reg [1:0]	lsu_len;
wire [31:0]	imm;
wire [31:0]	br_imm;
wire		signed_div;
wire		id_bubble;

assign op_code = id_insn[31:26];

assign op_rrr = (op_code == `ECO32F_OP_ADD)	|
		(op_code == `ECO32F_OP_SUB)	|
		(op_code == `ECO32F_OP_MUL)	|
		(op_code == `ECO32F_OP_MULU)	|
		(op_code == `ECO32F_OP_DIV)	|
		(op_code == `ECO32F_OP_DIVU)	|
		(op_code == `ECO32F_OP_REM)	|
		(op_code == `ECO32F_OP_REMU)	|
		(op_code == `ECO32F_OP_AND)	|
		(op_code == `ECO32F_OP_OR)	|
		(op_code == `ECO32F_OP_XOR)	|
		(op_code == `ECO32F_OP_XNOR)	|
		(op_code == `ECO32F_OP_SLL)	|
		(op_code == `ECO32F_OP_SLR)	|
		(op_code == `ECO32F_OP_SAR);

assign op_rrs = (op_code == `ECO32F_OP_ADDI)	|
		(op_code == `ECO32F_OP_SUBI)	|
		(op_code == `ECO32F_OP_MULI)	|
		(op_code == `ECO32F_OP_DIVI)	|
		(op_code == `ECO32F_OP_REMI);

assign op_rri = (op_code == `ECO32F_OP_MULUI)	|
		(op_code == `ECO32F_OP_DIVUI)	|
		(op_code == `ECO32F_OP_REMUI)	|
		(op_code == `ECO32F_OP_ANDI)	|
		(op_code == `ECO32F_OP_ORI)	|
		(op_code == `ECO32F_OP_XORI)	|
		(op_code == `ECO32F_OP_XNORI)	|
		(op_code == `ECO32F_OP_SLLI)	|
		(op_code == `ECO32F_OP_SLRI)	|
		(op_code == `ECO32F_OP_SARI);

assign op_rrb = (op_code == `ECO32F_OP_BEQ)	|
		(op_code == `ECO32F_OP_BNE)	|
		(op_code == `ECO32F_OP_BLE)	|
		(op_code == `ECO32F_OP_BLEU)	|
		(op_code == `ECO32F_OP_BLT)	|
		(op_code == `ECO32F_OP_BLTU)	|
		(op_code == `ECO32F_OP_BGE)	|
		(op_code == `ECO32F_OP_BGEU)	|
		(op_code == `ECO32F_OP_BGT)	|
		(op_code == `ECO32F_OP_BGTU);

assign op_beq  = (op_code == `ECO32F_OP_BEQ);
assign op_bne  = (op_code == `ECO32F_OP_BNE);
assign op_ble  = (op_code == `ECO32F_OP_BLE);
assign op_bleu = (op_code == `ECO32F_OP_BLEU);
assign op_blt  = (op_code == `ECO32F_OP_BLT);
assign op_bltu = (op_code == `ECO32F_OP_BLTU);
assign op_bge  = (op_code == `ECO32F_OP_BGE);
assign op_bgeu = (op_code == `ECO32F_OP_BGEU);
assign op_bgt  = (op_code == `ECO32F_OP_BGT);
assign op_bgtu = (op_code == `ECO32F_OP_BGTU);

assign op_j   = (op_code == `ECO32F_OP_J)	|
		(op_code == `ECO32F_OP_JAL);

assign op_jr  = (op_code == `ECO32F_OP_JR)	|
		(op_code == `ECO32F_OP_JALR);

assign op_jal = (op_code == `ECO32F_OP_JAL)	|
		(op_code == `ECO32F_OP_JALR);

assign op_add = (op_code == `ECO32F_OP_ADD)	|
		(op_code == `ECO32F_OP_ADDI);

assign op_sub = (op_code == `ECO32F_OP_SUB)	|
		(op_code == `ECO32F_OP_SUBI);

assign op_mul = (op_code == `ECO32F_OP_MUL)	|
		(op_code == `ECO32F_OP_MULI)	|
		(op_code == `ECO32F_OP_MULU)	|
		(op_code == `ECO32F_OP_MULUI);

assign op_div = (op_code == `ECO32F_OP_DIV)	|
		(op_code == `ECO32F_OP_DIVI)	|
		(op_code == `ECO32F_OP_DIVU)	|
		(op_code == `ECO32F_OP_DIVUI);

assign op_rem = (op_code == `ECO32F_OP_REM)	|
		(op_code == `ECO32F_OP_REMI)	|
		(op_code == `ECO32F_OP_REMU)	|
		(op_code == `ECO32F_OP_REMUI);

assign op_and = (op_code == `ECO32F_OP_AND)	|
		(op_code == `ECO32F_OP_ANDI);

assign op_or = (op_code == `ECO32F_OP_OR)	|
	       (op_code == `ECO32F_OP_ORI);

assign op_xor = (op_code == `ECO32F_OP_XOR)	|
		(op_code == `ECO32F_OP_XORI);

assign op_xnor = (op_code == `ECO32F_OP_XNOR)	|
		 (op_code == `ECO32F_OP_XNORI);

assign op_sll = (op_code == `ECO32F_OP_SLL)	|
		(op_code == `ECO32F_OP_SLLI);
assign op_slr = (op_code == `ECO32F_OP_SLR)	|
		(op_code == `ECO32F_OP_SLRI);
assign op_sar = (op_code == `ECO32F_OP_SAR)	|
		(op_code == `ECO32F_OP_SARI);

assign op_ldhi = (op_code == `ECO32F_OP_LDHI);

assign op_load = (op_code == `ECO32F_OP_LDW)	|
		 (op_code == `ECO32F_OP_LDH)	|
		 (op_code == `ECO32F_OP_LDHU)	|
		 (op_code == `ECO32F_OP_LDB)	|
		 (op_code == `ECO32F_OP_LDBU);

assign op_store = (op_code == `ECO32F_OP_STW)	|
		  (op_code == `ECO32F_OP_STH)	|
		  (op_code == `ECO32F_OP_STB);

assign op_mvfs = (op_code == `ECO32F_OP_MVFS);

assign op_mvts = (op_code == `ECO32F_OP_MVTS);

assign op_tbs = (op_code == `ECO32F_OP_TBS);
assign op_tbwr = (op_code == `ECO32F_OP_TBWR);
assign op_tbri = (op_code == `ECO32F_OP_TBRI);
assign op_tbwi = (op_code == `ECO32F_OP_TBWI);

assign op_rfx = (op_code == `ECO32F_OP_RFX);

assign lsu_zext = (op_code == `ECO32F_OP_LDHU)	|
		  (op_code == `ECO32F_OP_LDBU);

assign lsu_sext = !lsu_zext;

assign signed_div = (op_code == `ECO32F_OP_DIV)		|
		    (op_code == `ECO32F_OP_DIVI)	|
		    (op_code == `ECO32F_OP_REM)		|
		    (op_code == `ECO32F_OP_REM);

always @(*)
	case (op_code)
	`ECO32F_OP_STB,
	`ECO32F_OP_LDB,
	`ECO32F_OP_LDBU:
		lsu_len = 2'b00;

	`ECO32F_OP_STH,
	`ECO32F_OP_LDH,
	`ECO32F_OP_LDHU:
		lsu_len = 2'b01;

	`ECO32F_OP_STW,
	`ECO32F_OP_LDW:
		lsu_len = 2'b10;

	default:
		lsu_len = 2'b10;
	endcase

// Register file operands
assign id_rf_x_addr = op_rfx ? 5'd30 : id_insn[25:21];
assign id_rf_y_addr = id_insn[20:16];
assign id_rf_r_addr = op_rrr ? id_insn[15:11] :
		      op_jal ? 5'd31 :
		      id_insn[20:16];
assign id_rf_r_we = op_rrr | op_rrs | op_rri | op_ldhi | op_load | op_jal |
		    op_mvfs;

// Pick out immediate from insn
assign imm = op_rri ? {16'h0, id_insn[15:0]} : // zero extend
	     op_ldhi ? {id_insn[15:0], 16'h0} : // load hi
	     {{16{id_insn[15]}}, id_insn[15:0]}; // sign extend

// Pick out branch immediate from insn
assign br_imm = op_rrb ? {{14{id_insn[15]}}, id_insn[15:0], 2'h0} :
	     /* op_j ? */ {{4{id_insn[25]}}, id_insn[25:0], 2'h0};

//
// Handle the situation when a result would be needed to be 'forwarded'
// backwards in the pipeline, i.e. when execute stage need a result from memory
// stage. This is handled by inserting a 'nop bubble' in the pipeline.
//
assign id_bubble = (ex_op_load | ex_op_mul) & (id_rf_x_addr == ex_rf_r_addr ||
					       id_rf_y_addr == ex_rf_r_addr) |
		   mem_op_mul & (id_rf_x_addr == mem_rf_r_addr ||
				 id_rf_y_addr == mem_rf_r_addr);

// Registered output to execute stage
always @(posedge clk)
	if (!id_stall | id_flush) begin
		ex_op_add <= op_add;
		ex_op_sub <= op_sub;
		ex_op_mul <= op_mul;
		ex_op_div <= op_div;
		ex_op_rem <= op_rem;
		ex_op_and <= op_and;
		ex_op_or <= op_or;
		ex_op_xor <= op_xor;
		ex_op_xnor <= op_xnor;
		ex_op_sll <= op_sll;
		ex_op_slr <= op_slr;
		ex_op_sar <= op_sar;
		ex_op_ldhi <= op_ldhi;
		ex_op_beq <= op_beq;
		ex_op_bne <= op_bne;
		ex_op_ble <= op_ble;
		ex_op_bleu <= op_bleu;
		ex_op_blt <= op_blt;
		ex_op_bltu <= op_bltu;
		ex_op_bge <= op_bge;
		ex_op_bgeu <= op_bgeu;
		ex_op_bgt <= op_bgt;
		ex_op_bgtu <= op_bgtu;
		ex_op_load <= op_load;
		ex_op_store <= op_store;
		ex_op_mvfs <= op_mvfs;
		ex_op_mvts <= op_mvts;
		ex_op_tbs <= op_tbs;
		ex_op_tbwr <= op_tbwr;
		ex_op_tbri <= op_tbri;
		ex_op_tbwi <= op_tbwi;

		ex_op_rfx <= op_rfx;

		ex_op_rrb <= op_rrb;
		ex_op_jal <= op_jal;
		ex_op_j <= op_j;
		ex_op_jr <= op_jr;

		ex_lsu_sext <= lsu_sext;
		ex_lsu_len <= lsu_len;

		ex_signed_div <= signed_div;

		ex_exc_ibus_fault <= id_exc_ibus_fault;
		ex_imm_sel <= !op_rrr & !op_rrb;
		ex_imm <= imm;
		ex_branch_imm <= br_imm + id_pc + 4;

		ex_pc <= id_pc;

		ex_rf_x_addr <= id_rf_x_addr;
		ex_rf_y_addr <= id_rf_y_addr;
		ex_rf_r_addr <= id_rf_r_addr;
		ex_rf_r_we <= id_rf_r_we;

		ex_bubble <= id_bubble;

		// Push out a no-op
		if (id_flush | id_bubble) begin
			ex_op_add <= 0;
			ex_op_sub <= 0;
			ex_op_mul <= 0;
			ex_op_div <= 0;
			ex_op_rem <= 0;
			ex_op_and <= 0;
			ex_op_or <= 0;
			ex_op_xor <= 0;
			ex_op_xnor <= 0;
			ex_op_sll <= 0;
			ex_op_slr <= 0;
			ex_op_sar <= 0;
			ex_op_ldhi <= 0;
			ex_op_beq <= 0;
			ex_op_bne <= 0;
			ex_op_ble <= 0;
			ex_op_bleu <= 0;
			ex_op_blt <= 0;
			ex_op_bltu <= 0;
			ex_op_bge <= 0;
			ex_op_bgeu <= 0;
			ex_op_bgt <= 0;
			ex_op_bgtu <= 0;
			ex_op_load <= 0;
			ex_op_store <= 0;
			ex_op_mvfs <= 0;
			ex_op_mvts <= 0;
 			ex_op_tbs <= 0;
			ex_op_tbwr <= 0;
			ex_op_tbri <= 0;
 			ex_op_tbwi <= 0;

			ex_op_rfx <= 0;

			ex_op_rrb <= 0;
			ex_op_jal <= 0;
			ex_op_j <= 0;
			ex_op_jr <= 0;

			ex_rf_r_addr <= 0;
			ex_rf_r_we <= 0;
		end
	end

endmodule
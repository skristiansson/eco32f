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

module eco32f_alu #(
)(
	input 		  rst,
	input 		  clk,

	input 		  id_stall,
	input 		  ex_stall,
	input 		  mem_stall,

	input 		  ex_flush,
	input 		  mem_flush,

	output 		  alu_stall,

	input [31:0] 	  id_pc,

	input 		  ex_op_add,
	input 		  ex_op_sub,
	input 		  ex_op_mul,
	input 		  ex_op_div,
	input 		  ex_op_rem,
	input 		  ex_op_or,
	input 		  ex_op_and,
	input 		  ex_op_xor,
	input 		  ex_op_xnor,
	input 		  ex_op_sll,
	input 		  ex_op_slr,
	input 		  ex_op_sar,
	input 		  ex_op_beq,
	input 		  ex_op_bne,
	input 		  ex_op_ble,
	input 		  ex_op_bleu,
	input 		  ex_op_blt,
	input 		  ex_op_bltu,
	input 		  ex_op_bge,
	input 		  ex_op_bgeu,
	input 		  ex_op_bgt,
	input 		  ex_op_bgtu,
	input 		  ex_op_jal,

	input 		  ex_op_rrb,

	input 		  ex_signed_div,

	input [31:0] 	  ex_rf_x,
	input [31:0] 	  ex_rf_y,
	input [31:0] 	  ex_imm,
	input 		  ex_imm_sel,

	output [31:0] 	  ex_add_result,

	output 		  ex_cond_true,
	output [31:0] 	  ex_alu_result,

	output reg 	  mem_op_mul,
	output reg 	  wb_op_mul,
	output reg [31:0] wb_mul_result
);

wire [31:0]	x;
wire [31:0]	y;
wire 		add_carry;
wire 		add_overflow;
wire 		sub_overflow;
wire [31:0]	add_result;
wire [31:0]	or_result;
wire [31:0]	xor_result;
wire [31:0]	and_result;
wire [31:0]	slr_result;
wire [31:0]	sll_result;
wire [31:0]	sar_result;
wire [31:0]	div_result;
wire [31:0]	rem_result;
wire		x_eq_y;
wire		x_lts_y;
wire		x_ltu_y;

wire		div_stall;

assign x = ex_rf_x;
assign y = ex_imm_sel ? ex_imm : ex_rf_y;
assign {add_carry, add_result} = (ex_op_sub | ex_op_rrb) ? x - y : x + y;
assign add_overflow = (x[31] == y[31]) & (x[31] ^ add_result[31]);
assign sub_overflow = (x[31] != y[31]) & (x[31] ^ add_result[31]);
assign or_result = x | y;
assign xor_result = x ^ y;
assign and_result = x & y;

assign sll_result = x << y[4:0];
assign slr_result = x >> y[4:0];
assign sar_result = (x >> y[4:0]) | ({32{x[31]}} << (32 - y[4:0]));

// Condition compare
assign x_eq_y = !(|xor_result);
assign x_ltu_y = add_carry;
assign x_lts_y = add_result[31] != sub_overflow;

assign ex_cond_true = ex_op_beq & x_eq_y |
		      ex_op_bne & !x_eq_y |
		      ex_op_ble & (x_lts_y | x_eq_y) |
		      ex_op_bleu & (x_ltu_y | x_eq_y) |
		      ex_op_blt & x_lts_y |
		      ex_op_bltu & x_ltu_y |
		      ex_op_bge & !x_lts_y |
		      ex_op_bgeu & !x_ltu_y |
	 	      ex_op_bgt & !x_lts_y & !x_eq_y |
		      ex_op_bgtu & !x_ltu_y & !x_eq_y;

assign ex_alu_result = ex_op_or ? or_result :
		       ex_op_and ? and_result :
		       ex_op_xor ? xor_result :
		       ex_op_xnor ? ~xor_result :
		       ex_op_sll ? sll_result :
		       ex_op_slr ? slr_result :
		       ex_op_sar ? sar_result :
		       ex_op_div ? div_result :
		       ex_op_rem ? rem_result :
		       ex_op_jal ? id_pc :
		       add_result;

assign ex_add_result = add_result;

assign alu_stall = div_stall;

//
// Serial divider
// Handles div* and rem* instructions
//
reg [5:0]	div_cnt;
reg [31:0]	div_n;
reg [31:0]	div_d;
reg [31:0]	div_r;
reg [32:0]	div_sub;
reg		div_neg;
reg		div_load;
reg		div_in_progress;
reg		div_by_zero;

assign div_stall = div_in_progress | (ex_op_div | ex_op_rem) & div_load;

assign div_result = div_neg ? ~div_n + 1 : div_n;
assign rem_result = div_neg ? ~div_r + 1 : div_r;

always @(posedge clk)
	div_load <= !id_stall;

// Serial division is performed in 32 clock cycles (1 per bit).
always @(posedge clk)
	if (div_load)
		div_cnt <= 32;
	else if (div_cnt != 0)
		div_cnt <= div_cnt - 1;

assign div_sub = {div_r[30:0], div_n[31]} - div_d;

always @(posedge clk) begin
	if (div_load) begin
		div_in_progress <= ex_op_div | ex_op_rem;
		div_n <= x;
		div_d <= y;
		div_r <= 0;
		div_neg <= 0;
		div_by_zero <= ex_op_div & (y == 0);

		// Perform unsigned division on converted operands.
		if (ex_signed_div) begin
			if (ex_op_div)
				div_neg <= x[31] ^ y[31];
			else
				div_neg <= x[31];

			if (x[31])
				div_n <= ~x + 1;
			if (y[31])
				div_d <= ~y + 1;
		end
	end else if (div_in_progress) begin
		if (!div_sub[32]) begin // div_sub >= 0
			div_r <= div_sub[31:0];
			div_n <= {div_n[30:0], 1'b1};
		end else begin // div_sub < 0
		        div_r <= {div_r[30:0], div_n[31]};
			div_n <= {div_n[30:0], 1'b0};
		end
		if (div_cnt == 1)
			div_in_progress <= 0;
	end
end

//
// Pipelined multiplier.
// Result is ready in wb stage.
//
reg [31:0] mul_x;
reg [31:0] mul_y;

always @(posedge clk) begin
	if (!ex_stall) begin
		mul_x <= x;
		mul_y <= y;
		mem_op_mul <= ex_op_mul;
	end

	if (ex_flush)
		mem_op_mul <= 0;

	if (!mem_stall) begin
		wb_mul_result <= mul_x * mul_y;
		wb_op_mul <= mem_op_mul;
	end

	if (mem_flush)
		wb_op_mul <= 0;
end

endmodule

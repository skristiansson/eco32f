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

module eco32f_writeback (
	input 		 rst,
	input 		 clk,

	input 		 do_exception,

	input 		 mem_stall,
	input [31:0] 	 mem_pc,
	input [31:0] 	 mem_alu_result,
	input [31:0] 	 mem_lsu_result,
	input 		 mem_rf_r_we,
	input [4:0] 	 mem_rf_r_addr,

	input 		 mem_op_load,

	input 		 wb_op_mul,
	input [31:0] 	 wb_mul_result,

	output [31:0] 	 wb_rf_r,
	output reg 	 wb_rf_r_we,
	output reg [4:0] wb_rf_r_addr
);

reg [31:0]	wb_result;

always @(posedge clk)
	if (!mem_stall | do_exception) begin
		if (do_exception)
			wb_result <= mem_pc;
		else if (mem_op_load)
			wb_result <= mem_lsu_result;
		else
			wb_result <= mem_alu_result;

		if (do_exception)
			wb_rf_r_addr <= 5'd30;
		else
			wb_rf_r_addr <= mem_rf_r_addr;

		wb_rf_r_we <= mem_rf_r_we | do_exception;
	end

assign wb_rf_r = wb_op_mul ? wb_mul_result : wb_result;

endmodule

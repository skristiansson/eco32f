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

module eco32f_registerfile #(
)(
	input 		 rst,
	input 		 clk,

	input 		 id_stall,
	input 		 ex_stall,

	input [4:0] 	 id_rf_x_addr,
	input [4:0] 	 id_rf_y_addr,
	input [4:0] 	 ex_rf_x_addr,
	input [4:0] 	 ex_rf_y_addr,
	input [4:0] 	 ex_rf_r_addr,
	input 		 ex_rf_r_we,

	output reg [4:0] mem_rf_r_addr,
	output reg 	 mem_rf_r_we,

	input [31:0] 	 mem_alu_result,

	output [31:0] 	 ex_rf_x,
	output [31:0] 	 ex_rf_y,

	input [4:0] 	 wb_rf_r_addr,
	input 		 wb_rf_r_we,
	input [31:0] 	 wb_rf_r
);

wire [31:0]	rf_x;
wire [31:0] 	rf_y;

wire	rf_re;

assign rf_re = !id_stall;

always @(posedge clk)
	if (!ex_stall) begin
		mem_rf_r_addr <= ex_rf_r_addr;
		mem_rf_r_we <= ex_rf_r_we;
	end

//
// RAW (Read After Write) Hazard handling
//
// Execute stage register bypass.
// I.e. bypass previous execute stage result to execute stage input.
reg ex_bypass_x;
reg ex_bypass_y;
always @(posedge clk)
	if (!id_stall) begin
		ex_bypass_x <= ex_rf_r_we & (ex_rf_r_addr == id_rf_x_addr);
		ex_bypass_y <= ex_rf_r_we & (ex_rf_r_addr == id_rf_y_addr);
	end

// Memory stage register bypass
// I.e. bypass previous memory stage result to execute stage input.
reg mem_bypass_x;
reg mem_bypass_y;
always @(posedge clk)
	if (!id_stall) begin
		mem_bypass_x <= mem_rf_r_we & (mem_rf_r_addr == id_rf_x_addr);
		mem_bypass_y <= mem_rf_r_we & (mem_rf_r_addr == id_rf_y_addr);
	end

// Register file output generation
assign ex_rf_x = (ex_rf_x_addr == 0) ? 0 : // Register $0 access
		 ex_bypass_x ? mem_alu_result :
		 mem_bypass_x ? wb_rf_r :
		 rf_x;
assign ex_rf_y = (ex_rf_y_addr == 0) ? 0 : // Register $0 access
		 ex_bypass_y ? mem_alu_result :
		 mem_bypass_y ? wb_rf_r :
		 rf_y;

eco32f_simple_dpram_sclk #(
	.ADDR_WIDTH	(5),
	.DATA_WIDTH	(32),
	.ENABLE_BYPASS	(1)
) rf_ram_x (
	.clk		(clk),
	.raddr		(id_rf_x_addr),
	.re		(rf_re),
	.waddr		(wb_rf_r_addr),
	.we		(wb_rf_r_we),
	.din		(wb_rf_r),
	.dout		(rf_x)
);

eco32f_simple_dpram_sclk #(
	.ADDR_WIDTH	(5),
	.DATA_WIDTH	(32),
	.ENABLE_BYPASS	(1)
) rf_ram_y (
	.clk		(clk),
	.raddr		(id_rf_y_addr),
	.re		(rf_re),
	.waddr		(wb_rf_r_addr),
	.we		(wb_rf_r_we),
	.din		(wb_rf_r),
	.dout		(rf_y)
);

endmodule
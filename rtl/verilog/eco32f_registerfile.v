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

module eco32f_registerfile (
	input 		 rst,
	input 		 clk,

	input 		 if_stall,
	input 		 id_stall,
	input 		 ex_stall,
	input 		 ex_flush,

	input [4:0] 	 if_rf_x_addr,
	input [4:0] 	 if_rf_y_addr,
	input [4:0] 	 id_rf_x_addr,
	input [4:0] 	 id_rf_y_addr,
	input [4:0] 	 ex_rf_x_addr,
	input [4:0] 	 ex_rf_y_addr,
	input [4:0] 	 ex_rf_r_addr,
	input 		 ex_rf_r_we,

	output reg [4:0] mem_rf_r_addr,
	output reg 	 mem_rf_r_we,

	input [31:0] 	 mem_alu_result,

	output [31:0] 	 id_rf_x,
	output [31:0] 	 id_rf_y,
	output [31:0] 	 ex_rf_x,
	output [31:0] 	 ex_rf_y,

	input [4:0] 	 wb_rf_r_addr,
	input 		 wb_rf_r_we,
	input [31:0] 	 wb_rf_r
);

wire [31:0]	rf_dout_x;
wire [31:0] 	rf_dout_y;

reg [31:0]	ex_rf_dout_x;
reg [31:0] 	ex_rf_dout_y;

wire	rf_re;

assign rf_re = !id_stall;

always @(posedge clk) begin
	if (!ex_stall) begin
		mem_rf_r_addr <= ex_rf_r_addr;
		mem_rf_r_we <= ex_rf_r_we;
	end

	if (ex_flush)
		mem_rf_r_we <= 0;
end

//
// RAW (Read After Write) Hazard handling
//
// Execute to decode stage register bypass.
// I.e. bypass previous execute stage result to decode stage input.
reg ex2id_bypass_x;
reg ex2id_bypass_y;
always @(posedge clk) begin
	ex2id_bypass_x <= ex_rf_r_we & (ex_rf_r_addr == if_rf_x_addr);
	ex2id_bypass_y <= ex_rf_r_we & (ex_rf_r_addr == if_rf_y_addr);
end

// Memory to decode stage register bypass
// I.e. bypass previous memory stage result to decode stage input.
reg mem2id_bypass_x;
reg mem2id_bypass_y;
always @(posedge clk) begin
	mem2id_bypass_x <= mem_rf_r_we & (mem_rf_r_addr == if_rf_x_addr);
	mem2id_bypass_y <= mem_rf_r_we & (mem_rf_r_addr == if_rf_y_addr);
end
// Execute to execute stage register bypass.
// I.e. bypass previous execute stage result to execute stage input.
reg ex2ex_bypass_x;
reg ex2ex_bypass_y;
always @(posedge clk)
	if (!id_stall) begin
		ex2ex_bypass_x <= ex_rf_r_we & (ex_rf_r_addr == id_rf_x_addr);
		ex2ex_bypass_y <= ex_rf_r_we & (ex_rf_r_addr == id_rf_y_addr);
	end

// Memory to execute stage register bypass
// I.e. bypass previous memory stage result to execute stage input.
reg mem2ex_bypass_x;
reg mem2ex_bypass_y;
always @(posedge clk)
	if (!id_stall) begin
		mem2ex_bypass_x <= mem_rf_r_we & (mem_rf_r_addr == id_rf_x_addr);
		mem2ex_bypass_y <= mem_rf_r_we & (mem_rf_r_addr == id_rf_y_addr);
	end

// Writeback to execute stage register bypass
// I.e. bypass previous memory stage result to execute stage input.
reg wb2ex_bypass_x;
reg wb2ex_bypass_y;
always @(posedge clk) begin
	if (!id_stall) begin
		wb2ex_bypass_x <= wb_rf_r_we & (wb_rf_r_addr == id_rf_x_addr);
		wb2ex_bypass_y <= wb_rf_r_we & (wb_rf_r_addr == id_rf_y_addr);
	end else begin
		wb2ex_bypass_x <= 0;
		wb2ex_bypass_y <= 0;
	end
end

reg [31:0] wb2ex_bypass_result;
always @(posedge clk)
	if (wb_rf_r_we)
		wb2ex_bypass_result <= wb_rf_r;

assign id_rf_x = (id_rf_x_addr == 0) ? 0 : // Register $0 access
		 ex2id_bypass_x ? mem_alu_result :
		 mem2id_bypass_x ? wb_rf_r :
		 !rf_dout_valid ? id_last_rf_x :
		 rf_dout_x;

assign id_rf_y = (id_rf_y_addr == 0) ? 0 : // Register $0 access
		 ex2id_bypass_y ? mem_alu_result :
		 mem2id_bypass_y ? wb_rf_r :
		 !rf_dout_valid ? id_last_rf_y :
		 rf_dout_y;

// Save the last latched value
reg		rf_dout_valid;
reg [31:0]	id_last_rf_x;
reg [31:0]	id_last_rf_y;
always @(posedge clk) begin
	rf_dout_valid <= rf_re;
	if (rf_dout_valid) begin
		id_last_rf_x <= id_rf_x;
		id_last_rf_y <= id_rf_y;
	end
end

always @(posedge clk)
	if (!id_stall) begin
		ex_rf_dout_x <= id_rf_x;
		ex_rf_dout_y <= id_rf_y;
	end

// Register file output generation
assign ex_rf_x = (ex_rf_x_addr == 0) ? 0 : // Register $0 access
		 ex2ex_bypass_x ? mem_alu_result :
		 mem2ex_bypass_x ? wb_rf_r :
		 wb2ex_bypass_x ? wb2ex_bypass_result :
		 ex_rf_dout_x;
assign ex_rf_y = (ex_rf_y_addr == 0) ? 0 : // Register $0 access
		 ex2ex_bypass_y ? mem_alu_result :
		 mem2ex_bypass_y ? wb_rf_r :
		 wb2ex_bypass_y ? wb2ex_bypass_result :
		 ex_rf_dout_y;

eco32f_simple_dpram_sclk #(
	.ADDR_WIDTH	(5),
	.DATA_WIDTH	(32),
	.ENABLE_BYPASS	(1)
) rf_ram_x (
	.clk		(clk),
	.raddr		(if_rf_x_addr),
	.re		(rf_re),
	.waddr		(wb_rf_r_addr),
	.we		(wb_rf_r_we),
	.din		(wb_rf_r),
	.dout		(rf_dout_x)
);

eco32f_simple_dpram_sclk #(
	.ADDR_WIDTH	(5),
	.DATA_WIDTH	(32),
	.ENABLE_BYPASS	(1)
) rf_ram_y (
	.clk		(clk),
	.raddr		(if_rf_y_addr),
	.re		(rf_re),
	.waddr		(wb_rf_r_addr),
	.we		(wb_rf_r_we),
	.din		(wb_rf_r),
	.dout		(rf_dout_y)
);

endmodule
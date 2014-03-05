//
// op code defines
//

// add/sub ops
`define ECO32F_OP_ADD		6'h00
`define ECO32F_OP_ADDI		6'h01
`define ECO32F_OP_SUB		6'h02
`define ECO32F_OP_SUBI		6'h03

// mul/div/rem ops
`define ECO32F_OP_MUL		6'h04
`define ECO32F_OP_MULI		6'h05
`define ECO32F_OP_MULU		6'h06
`define ECO32F_OP_MULUI		6'h07
`define ECO32F_OP_DIV		6'h08
`define ECO32F_OP_DIVI		6'h09
`define ECO32F_OP_DIVU		6'h0a
`define ECO32F_OP_DIVUI		6'h0b
`define ECO32F_OP_REM		6'h0c
`define ECO32F_OP_REMI		6'h0d
`define ECO32F_OP_REMU		6'h0e
`define ECO32F_OP_REMUI		6'h0f

// logic ops
`define ECO32F_OP_AND		6'h10
`define ECO32F_OP_ANDI		6'h11
`define ECO32F_OP_OR		6'h12
`define ECO32F_OP_ORI		6'h13
`define ECO32F_OP_XOR		6'h14
`define ECO32F_OP_XORI		6'h15
`define ECO32F_OP_XNOR		6'h16
`define ECO32F_OP_XNORI		6'h17

// shift ops
`define ECO32F_OP_SLL		6'h18
`define ECO32F_OP_SLLI		6'h19
`define ECO32F_OP_SLR		6'h1a
`define ECO32F_OP_SLRI		6'h1b
`define ECO32F_OP_SAR		6'h1c
`define ECO32F_OP_SARI		6'h1d

// move op
`define ECO32F_OP_LDHI		6'h1f

// conditional branches
`define ECO32F_OP_BEQ		6'h20
`define ECO32F_OP_BNE		6'h21
`define ECO32F_OP_BLE		6'h22
`define ECO32F_OP_BLEU		6'h23
`define ECO32F_OP_BLT		6'h24
`define ECO32F_OP_BLTU		6'h25
`define ECO32F_OP_BGE		6'h26
`define ECO32F_OP_BGEU		6'h27
`define ECO32F_OP_BGT		6'h28
`define ECO32F_OP_BGTU		6'h29

// unconditional jumps
`define ECO32F_OP_J		6'h2a
`define ECO32F_OP_JR		6'h2b
`define ECO32F_OP_JAL		6'h2c
`define ECO32F_OP_JALR		6'h2d

// trap and return from exception
`define ECO32F_OP_TRAP		6'h2e
`define ECO32F_OP_RFX		6'h2f

// load/store ops
`define ECO32F_OP_LDW		6'h30
`define ECO32F_OP_LDH		6'h31
`define ECO32F_OP_LDHU		6'h32
`define ECO32F_OP_LDB		6'h33
`define ECO32F_OP_LDBU		6'h34

`define ECO32F_OP_STW		6'h35
`define ECO32F_OP_STH		6'h36
`define ECO32F_OP_STB		6'h37

// special register handling instructions
`define ECO32F_OP_MVFS		6'h38
`define ECO32F_OP_MVTS		6'h39

// TLB handling instructions
`define ECO32F_OP_TBS		6'h3a
`define ECO32F_OP_TBWR		6'h3b
`define ECO32F_OP_TBRI		6'h3c
`define ECO32F_OP_TBWI		6'h3d

// NOP instruction (add $r0,$r0,$r0)
`define ECO32F_INSN_NOP		32'h00000000

//
// SPR defines
//
`define ECO32F_SPR_PSW		16'h0
`define ECO32F_SPR_TLB_INDEX	16'h1
`define ECO32F_SPR_TLB_ENTRY_HI	16'h2
`define ECO32F_SPR_TLB_ENTRY_LO	16'h3
`define ECO32F_SPR_TLB_BAD_ADDR	16'h4

`define ECO32F_SPR_PSW_V	27
`define ECO32F_SPR_PSW_UC	26
`define ECO32F_SPR_PSW_UP	25
`define ECO32F_SPR_PSW_UO	24
`define ECO32F_SPR_PSW_IC	23
`define ECO32F_SPR_PSW_IP	22
`define ECO32F_SPR_PSW_IO	21
`define ECO32F_SPR_PSW_EID	20:16
`define ECO32F_SPR_PSW_IEN	15:0

// EID defines
// 0..15 Device Interrupts
`define ECO32F_EID_BUS_TIMEOUT		5'd16
`define ECO32F_EID_ILLEGAL_INSN		5'd17
`define ECO32F_EID_PRIVILEGED_INSN	5'd18
`define ECO32F_EID_DIV_BY_ZERO		5'd19
`define ECO32F_EID_TRAP			5'd20
`define ECO32F_EID_TLB_MISS		5'd21
`define ECO32F_EID_TLB_WRITE		5'd22
`define ECO32F_EID_TLB_INVALID		5'd23
`define ECO32F_EID_ILLEGAL_ADDR		5'd24
`define ECO32F_EID_PRIVILEGED_ADDR	5'd25
// 26..31 Reserved

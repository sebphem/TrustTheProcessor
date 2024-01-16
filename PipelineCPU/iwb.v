`define OPCODE_COMPUTE    7'b0110011
`define OPCODE_COMPUTE_IMM 7'b0010011
`define OPCODE_BRANCH     7'b1100011
`define OPCODE_LOAD       7'b0000011
`define OPCODE_JMP        7'b1101111
`define OPCODE_JMP_LINK   7'b1100111
`define OPCODE_STORE      7'b0100011 
`define OPCODE_LUI       7'b0110111
`define OPCODE_AUIPC     7'b0010111
`define FUNC_ADD      3'b000
`define AUX_FUNC_ADD  7'b0000000
`define AUX_FUNC_SUB  7'b0100000
`define SIZE_BYTE  2'b00
`define SIZE_HWORD 2'b01
`define SIZE_WORD  2'b10
`define PCSel_4 1'b0
`define PCSel_ALU 1'b1
`define ImmSel_I 3'b000
`define ImmSel_B 3'b001
`define ImmSel_J 3'b010
`define ImmSel_U 3'b011
`define ImmSel_S 3'b100
`define ASel_Reg 1'b0
`define ASel_PC 1'b1
`define BSel_Reg 1'b0
`define BSel_IMM 1'b1
`define MemRW_Write 1'b0
`define MemRW_Read 1'b1
`define RWrEn_Enable 1'b0
`define RWrEn_Disable 1'b1
`define WBSel_ALU 2'b00
`define WBSel_PC4 2'b01
`define WBSel_Mem 2'b10
`define WBSel_Imm 2'b11
`define BEQ 3'b000
`define BNE 3'b001
`define BLT 3'b100
`define BGE 3'b101
`define BLTU 3'b110
`define BGEU 3'b111

module IWB(
    input [31:0] PC_in_wb,
    input [31:0] Inst_in_wb,
    input [1:0] WBSel_in_wb,
    input [31:0] LoadExtended_in_wb,
    input [31:0] Immediate_in_wb,
    input [31:0] ALUOutput_in_wb,
    input [4:0] Rdst_in_wb,
    input halt_in_wb,
    output [4:0] Rw_out_wb,
    output [31:0] Di_out_wb,
    input WEN,
    input CLK,
    input RST);

assign Di_out_wb = (WBSel_in_wb == `WBSel_ALU) ? ALUOutput_in_wb : (WBSel_in_wb == `WBSel_PC4) ? PC_in_wb + 4 : (WBSel_in_wb == `WBSel_Mem) ? LoadExtended_in_wb : Immediate_in_wb;
assign Rw_out_wb = (halt_in_wb)? 0 : Rdst_in_wb;
endmodule
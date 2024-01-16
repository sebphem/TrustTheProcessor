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

module IFetch(
    input halt_in_if,
    input PCSel_in_if,
    input [31:0] pc_br_jmp_target_in_if,
    output halt_out_if,
    output [31:0] pc_out_if,
    output [31:0] pc4_out_if,
    output [31:0] instr_out_if,
    input clk,
    input stall,
    input rst);
wire [31:0] NPC;
wire [31:0] PC;
Reg PC_REG(.Din(NPC), .Qout(PC), .WEN(stall), .CLK(clk), .RST(rst));
assign pc_out_if = PC;
assign pc4_out_if = PC + 4;
InstMem IMEM(.Addr(PC), .Size(`SIZE_WORD), .DataOut(instr_out_if), .CLK(clk));

// Check PC Alignment
wire unalignedPC;
assign unalignedPC = (~(PC[0] == 1'b0)) | (~(PC[1] == 1'b0));
assign halt_out_if = halt_in_if | unalignedPC;

assign NPC = (rst == 1'b0) ? 32'b0 :
             (halt_out_if == 1'b1) ? PC :
             (PCSel_in_if == `PCSel_ALU) ? pc_br_jmp_target_in_if :
             pc4_out_if;
endmodule // IFetch
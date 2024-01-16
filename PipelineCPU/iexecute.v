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

module IExecute(
    input halt_in_ex,
    input [31:0] Instr_in_ex,
    input [31:0] PC_in_ex,
    input [1:0] ALUSrc_in_ex,
    input [31:0] Rdata1_in_ex,
    input [31:0] Rdata2_in_ex,
    input ASel_in_ex,
    input BSel_in_ex,
    input BR_in_ex,
    input JMP_in_ex,
    input [31:0] Immediate_in_ex,
    output [31:0] Instr_out_ex,
    output [31:0] PC_out_ex,
    output [31:0] ALUOutput_out_ex,
    output [31:0] Rdata2_out_ex,
    output PCSel_out_ex,
    output halt_out_ex,
    output send_nops,
    input clk,
    input rst);
assign send_nops = (JMP_in_ex || ((BR_in_ex==1'b1) && (BranchTaken==1'b1))) ? 1 : 0;
wire BranchTaken;
wire invalidBranch;
wire invalidBranchOp;
BranchComparison BC(
    .Rdata1(Rdata1_in_ex),
    .Rdata2(Rdata2_in_ex),
    .funct3(funct3),
    .BR(BranchTaken),
    .halt(invalidBranchOp)
    );
assign invalidBranch = (BR_in_ex & invalidBranchOp) ? 1 : 0;

wire [2:0]  funct3;
assign funct3 = Instr_in_ex[14:12];
wire [6:0]  funct7;
assign funct7 = Instr_in_ex[31:25];
wire [6:0]  opcode;
assign opcode = Instr_in_ex[6:0];

assign PCSel_out_ex = ((BR_in_ex == 1'b1 && BranchTaken == 1'b1) || JMP_in_ex) ? `PCSel_ALU : `PCSel_4;
wire [31:0] ALU_A;
wire [31:0] ALU_B;
wire [31:0] ALUOutput;
wire invalidALUOp;
assign ALU_A = (ASel_in_ex == 1'b0) ? Rdata1_in_ex : PC_in_ex; 
assign ALU_B = (BSel_in_ex == 1'b0) ? Rdata2_in_ex : Immediate_in_ex;
ExecutionUnit EU(.out(ALUOutput_out_ex), .opA(ALU_A), .opB(ALU_B), .func(funct3), .auxFunc(funct7), .opcode(opcode), .halt(invalidALUOp));

assign Rdata2_out_ex = Rdata2_in_ex;
assign PC_out_ex = PC_in_ex;
assign Instr_out_ex = Instr_in_ex;
assign halt_out_ex = halt_in_ex | invalidALUOp | invalidBranch;
endmodule // IExecute

// Module which determines whether a branch should be taken
module BranchComparison(
    input [31:0] Rdata1,
    input [31:0] Rdata2,
    input [2:0] funct3,
    output reg BR,
    output reg halt
);
wire signed [31:0]  Rdata1_s;
wire signed [31:0] Rdata2_s;
assign Rdata1_s = Rdata1;
assign Rdata2_s = Rdata2;
always @(*) begin
    halt <= 1'b0;
    case(funct3)
        `BEQ: BR = (Rdata1_s == Rdata2_s)? 1 : 0;
        `BNE: BR = (Rdata1_s != Rdata2_s)? 1 : 0;
        `BLT: BR = (Rdata1_s < Rdata2_s)? 1 : 0;
        `BGE: BR = (Rdata1_s >= Rdata2_s)? 1 : 0;
        `BLTU: BR = (Rdata1 < Rdata2)? 1 : 0;
        `BGEU: BR = (Rdata1 >= Rdata2)? 1 : 0;
        default: begin BR = 1'bx;
            halt <= 1'b1;
        end
    endcase
end
endmodule

// ExecutionUnit
module ExecutionUnit(out, opA, opB, func, auxFunc, opcode, halt);
output reg [31:0] out;
output reg [63:0] mulout;
input [31:0] opA, opB;
input [2:0] func;
input [6:0] auxFunc;
input [6:0] opcode;
output reg halt;
wire signed [31:0] s_opA, s_opB;
assign s_opA = opA;
assign s_opB = opB;
always @(*) begin
    halt <= 1'b0;
    if(opcode == `OPCODE_COMPUTE || opcode == `OPCODE_COMPUTE_IMM) begin
        if (auxFunc == 7'b0000001) begin
            case (func)
                3'b000: begin 
                        out = {s_opA * s_opB};
                        out[31] = s_opA[31] ^ s_opB[31];
                end
                3'b001: begin 
                        mulout = s_opA * s_opB;
                        out = mulout[63:32];
                end
                3'b010: begin 
                    
                        mulout = s_opA * $signed({32'b0, opB});
                        out = mulout[63:32];
                end
                3'b011: begin 
                        mulout = opA * opB;
                        out = mulout[63:32];
                end
                3'b100: out = s_opA / s_opB;
                3'b101: out = opA / opB;
                3'b110: out = s_opA % s_opB;
                3'b111: out = opA % opB;
                default: begin 
                    halt <= 1'b1;
                    out = 32'b0;
                end
            endcase
        end else begin
            case (func)
            3'b000: 
                if(auxFunc == 7'b0000000 || opcode == `OPCODE_COMPUTE_IMM)
                    out = opA + opB;
                else
                    out = opA - opB;
            3'b001: out = opA << opB[4:0];
            3'b010: out = (s_opA < s_opB)? 1 : 0;
            3'b011: out = (opA < opB)? 1 : 0;
            3'b100: out = opA ^ opB;
            3'b101: 
                if(auxFunc == 7'b0000000)
                    out = opA >> opB[4:0];
                else
                    out = s_opA >>> opB[4:0];
            3'b110: out = opA | opB;
            3'b111: out = opA & opB;
            default: begin 
                halt <= 1'b1;
                out = 32'b0;
            end
        endcase
        end
    end
    else out = opA + opB;
end
endmodule // ExecutionUnit


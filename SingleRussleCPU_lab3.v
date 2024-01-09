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
// Template for Northwestern - CompEng 361 - Lab3
// Groupname: SingleRussleCPU
// NetIDs: ljp0624, swp8132
// Datapath used from https://inst.eecs.berkeley.edu/~cs61c/su21/pdfs/lectures/fa20-trimmed/lec20_lec21.pdf
module SingleCycleCPU(halt, clk, rst);
   output halt;
   input clk, rst;

wire stall_in_ex;
wire stall_in_mem;
HazardDetect hazardDetect(
   .Rsrc1_out_id(Rsrc1_out_id),
   .Rsrc2_out_id(Rsrc2_out_id),
   .Rsrc1_in_ex(Rsrc1_in_ex),
   .Rsrc2_in_ex(Rsrc2_in_ex),
   .Rdst_in_ex(Rdst_in_ex),
   .Rdst_in_mem(Rdst_in_mem),
   .Rdst_in_wb(Rdst_in_wb),
   .RWrEn_in_ex(RWrEn_in_ex),
   .RWrEn_in_mem(RWrEn_in_mem),
   .stall_in_ex(stall_in_ex),
   .stall_in_mem(stall_in_mem));
wire stall;
assign stall = stall_in_ex | stall_in_mem;



wire halt_out_if;
wire PCSel;
wire [31:0] PC_out_if;
wire [31:0] PC4_out_if;
wire [31:0] instr_out_if;
wire [31:0] DONE;
wire squash_with_nops;
IFetch fetchStage(
   .halt_in_if(1'b0),
   .PCSel_in_if(PCSel),
   .pc_br_jmp_target_in_if(ALUOutput_out_ex),
   .halt_out_if(halt_out_if),
   .pc_out_if(PC_out_if),
   .pc4_out_if(PC4_out_if),
   .instr_out_if(instr_out_if),
   .clk(clk),
   .stall(stall),
   .rst(rst)
);
wire halt_in_id;
wire [31:0] PC_in_id;
wire [31:0] instr_in_id;
wire valid_id;
IF_ID_Register firstStage(
   .PC_if(PC_out_if),
   .Inst_if(instr_out_if),
   .halt_if(halt_out_if),
   .valid_if(1'b1),
   .halt_id(halt_in_id),
   .PC_id(PC_in_id),
   .Inst_id(instr_in_id),
   .valid_id(valid_id),
   .squash(squash_with_nops),
   .stall(stall),
   .WEN(1'b0),
   .CLK(clk),
   .RST(rst)
);
wire [31:0] RWrData;
wire [4:0] RW_in_id;
wire [31:0] PC_out_id;
wire [31:0] Immediate_out_id;
wire halt_out_id, MemRW_Out_id;
wire RWrEn_out_id;
wire [1:0] ALUOp_out_id;
wire [1:0] ALUSrc_out_id;
wire [2:0] ImmSel_out_id;
wire ASel_out_id, BSel_out_id;
wire [2:0] BranchType_out_id;
wire JMP_out_id, BR_out_id;
wire [1:0] MemSize_out_id;
wire [31:0] Rdata1_out_id;
wire [31:0] Rdata2_out_id;
wire [4:0] Rdst_out_id;
wire [1:0] WBSel_out_id;
wire [4:0] Rsrc1_out_id;
wire [4:0] Rsrc2_out_id;
IDecode decoder(
   .halt_in_id(halt_in_id),
   .instr_in_id(instr_in_id),
   .pc_in_id(PC_in_id),
   .RWrData_in_id(RWrData),
   .RWrEn_in_wb(RWrEn_in_wb),
   .RW_in_wb(Rdst_in_wb),
   .halt_out_id(halt_out_id),
   .pc_out_id(PC_out_id),
   .MemRW_out_id(MemRW_Out_id),
   .RWrEn_out_id(RWrEn_out_id),
   .ALUOp_out_id(ALUOp_out_id),
   .ALUSrc_out_id(ALUSrc_out_id),
   .ImmSel_out_id(ImmSel_out_id),
   .ASel_out_id(ASel_out_id),
   .BSel_out_id(BSel_out_id),
   .JMP_out_id(JMP_out_id),
   .BR_out_id(BR_out_id),
   .MemSize_out_id(MemSize_out_id),
   .Immediate_out_id(Immediate_out_id),
   .Rdata1_out_id(Rdata1_out_id),
   .Rdata2_out_id(Rdata2_out_id),
   .Rdst_out_id(Rdst_out_id),
   .Rsrc1_out_id(Rsrc1_out_id),  
   .Rsrc2_out_id(Rsrc2_out_id),
   .WBSel_out_id(WBSel_out_id),
   .clk(clk),
   .rst(rst)
);

wire [31:0] PC_in_ex;
wire [31:0] Inst_in_ex;
wire MemRW_in_ex, RWrEn_in_ex;
wire [1:0] WBSel_in_ex;
wire [1:0] ALUOp_in_ex;
wire [1:0] ALUSrc_in_ex;
wire [4:0] Rdst_in_ex;
wire [2:0] ImmSel_in_ex;
wire ASel_in_ex, BSel_in_ex;
wire [2:0] BranchType_in_ex;
wire JMP_in_ex, BR_in_ex;
wire [1:0] MemSize_in_ex;
wire [31:0] Immediate_in_ex;
wire [31:0] Rdata1_in_ex;
wire [31:0] Rdata2_in_ex;
wire [4:0] Rsrc1_in_ex;
wire [4:0] Rsrc2_in_ex;
wire valid_ex;
ID_EX_Register secondStage(
   .PC_id(PC_in_id),
   .Inst_id(instr_in_id),
   .MemRW_id(MemRW_Out_id),
   .RWrEn_id(RWrEn_out_id),
   .ALUOp_id(ALUOp_out_id),
   .ALUSrc_id(ALUSrc_out_id),
   .RegDst_id(Rdst_out_id),
   .ImmSel_id(ImmSel_out_id),
   .ASel_id(ASel_out_id),
   .BSel_id(BSel_out_id),
   .JMP_id(JMP_out_id),
   .BR_id(BR_out_id),
   .WBSel_id(WBSel_out_id),
   .MemSize_id(MemSize_out_id),
   .Immediate_id(Immediate_out_id),
   .Rdata1_id(Rdata1_out_id),
   .Rdata2_id(Rdata2_out_id),
   .Rsrc1_id(Rsrc1_out_id),
   .Rsrc2_id(Rsrc2_out_id),
   .halt_id(halt_out_id),
   .valid_id(valid_id),
   .PC_ex(PC_in_ex),
   .Inst_ex(Inst_in_ex),
   .MemRW_ex(MemRW_in_ex),
   .RWrEn_ex(RWrEn_in_ex),
   .ALUOp_ex(ALUOp_in_ex),
   .ALUSrc_ex(ALUSrc_in_ex),
   .RegDst_ex(Rdst_in_ex),
   .ImmSel_ex(ImmSel_in_ex),
   .Rdata1_ex(Rdata1_in_ex),
   .Rdata2_ex(Rdata2_in_ex),
   .ASel_ex(ASel_in_ex),
   .BSel_ex(BSel_in_ex),
   .JMP_ex(JMP_in_ex),
   .BR_ex(BR_in_ex),
   .WBSel_ex(WBSel_in_ex),
   .Immediate_ex(Immediate_in_ex),
   .MemSize_ex(MemSize_in_ex),
   .Rsrc1_ex(Rsrc1_in_ex),
   .Rsrc2_ex(Rsrc2_in_ex),
   .halt_ex(halt_in_ex),
   .valid_ex(valid_ex),
   .squash(squash_with_nops | stall_in_ex),
   .stall(stall_in_mem & !stall_in_ex),
   .WEN(1'b0),
   .CLK(clk),
   .RST(rst)
);

wire [31:0] Immediate_out_ex;
wire [31:0] Instr_out_ex;
wire [31:0] PC_out_ex;
wire [31:0] ALUOutput_out_ex;
wire [31:0] Rdata2_out_ex;
wire halt_out_ex;
IExecute executeModule(
   .halt_in_ex(halt_in_ex),
   .Instr_in_ex(Inst_in_ex),
   .PC_in_ex(PC_in_ex),
   .ALUSrc_in_ex(ALUSrc_in_ex),
   .Rdata1_in_ex(Rdata1_in_ex),
   .Rdata2_in_ex(Rdata2_in_ex),
   .ASel_in_ex(ASel_in_ex),
   .BSel_in_ex(BSel_in_ex),
   .BR_in_ex(BR_in_ex),
   .JMP_in_ex(JMP_in_ex),
   .Immediate_in_ex(Immediate_in_ex),
   .Instr_out_ex(Instr_out_ex),
   .PC_out_ex(PC_out_ex),
   .ALUOutput_out_ex(ALUOutput_out_ex),
   .Rdata2_out_ex(Rdata2_out_ex),
   .PCSel_out_ex(PCSel),
   .halt_out_ex(halt_out_ex),
   .send_nops(squash_with_nops),
   .clk(clk),
   .rst(rst)
);

wire [31:0] PC_in_mem;
wire [31:0] Inst_in_mem;
wire MemRW_in_mem, RWrEn_in_mem;
wire JMP_in_mem;
wire BR_in_mem;
wire BranchCondTrue_in_mem;
wire [1:0] WBSel_in_mem;
wire [1:0] MemSize_in_mem;
wire [31:0] ALUOutput_in_mem;
wire [31:0] Immediate_in_mem;
wire halt_in_mem;
wire [4:0] Rdst_in_mem;
wire [31:0] Rdata2_in_mem;
wire valid_mem;
EX_MEM_Register thirdStage(
   .PC_ex(PC_out_ex),
   .Inst_ex(Instr_out_ex),
   .MemRW_ex(MemRW_in_ex),
   .RWrEn_ex(RWrEn_in_ex),
   .BranchCondTrue_ex(BranchCondTrue_in_ex),
   .WBSel_ex(WBSel_in_ex),
   .MemSize_ex(MemSize_in_ex),
   .ALUOutput_ex(ALUOutput_out_ex),
   .Immediate_ex(Immediate_in_ex),
   .Rdst_ex(Rdst_in_ex),
   .Rdata2_ex(Rdata2_out_ex),
   .halt_ex(halt_out_ex),
   .valid_ex(valid_ex),
   .PC_mem(PC_in_mem),
   .Inst_mem(Inst_in_mem),
   .MemRW_mem(MemRW_in_mem),
   .RWrEn_mem(RWrEn_in_mem),
   .BranchCondTrue_mem(BranchCondTrue_in_mem),
   .WBSel_mem(WBSel_in_mem),
   .MemSize_mem(MemSize_in_mem),
   .ALUoutput_mem(ALUOutput_in_mem),
   .Immediate_mem(Immediate_in_mem),
   .Rdst_mem(Rdst_in_mem),
   .Rdata2_mem(Rdata2_in_mem),
   .halt_mem(halt_in_mem),
   .valid_mem(valid_mem),
   .squash(stall_in_mem & !stall_in_ex),
   .WEN(1'b0),
   .CLK(clk),
   .RST(rst)
);

wire [31:0] LoadExtended_out_mem;
wire halt_out_mem;
IMem memoryUnit(
   .halt_in_mem(halt_in_mem),
   .PC_in_mem(PC_in_mem),
   .Instr_in_mem(Inst_in_mem),
   .ALUOutput_in_mem(ALUOutput_in_mem),
   .MemSize_in_mem(MemSize_in_mem),
   .MemWrEn_in_mem(MemRW_in_mem),
   .Rdata2_in_mem(Rdata2_in_mem),
   .LoadExtended_out_mem(LoadExtended_out_mem),
   .halt_out_mem(halt_out_mem),
   .clk(clk),
   .rst(rst)
);

wire [31:0] PC_in_wb;
wire [31:0] Inst_in_wb;
wire [31:0] ALUOutput_in_wb;
wire [31:0] Immediate_in_wb;
wire [31:0] LoadExtended_in_wb;

wire RegWrite_mem;
wire MemToReg_mem;
wire [1:0] WBSel_in_wb;
wire [4:0] Rdst_in_wb;
wire valid_wb;
MEM_WB_Register fourthStage(
   .PC_mem(PC_in_mem),
   .Inst_mem(Inst_in_mem),
   .MemRW_mem(MemRW_in_mem),
   .RWrEn_mem(RWrEn_in_mem),
   .WBSel_mem(WBSel_in_mem),
   .LoadExtended_mem(LoadExtended_out_mem),
   .Immediate_mem(Immediate_in_mem),
   .ALUOutput_mem(ALUOutput_in_mem),
   .Rdst_mem(Rdst_in_mem),
   .halt_mem(halt_out_mem),
   .valid_mem(valid_mem),
   .PC_wb(PC_in_wb),
   .Inst_wb(Inst_in_wb),
   .MemRW_wb(MemRW_in_wb),
   .RWrEn_wb(RWrEn_in_wb),
   .WBSel_wb(WBSel_in_wb),
   .LoadExtended_wb(LoadExtended_in_wb),
   .Immediate_wb(Immediate_in_wb),
   .ALUOutput_wb(ALUOutput_in_wb),
   .Rdst_wb(Rdst_in_wb),
   .halt_wb(halt_in_wb),
   .valid_wb(valid_wb),
   .WEN(1'b0),
   .CLK(clk),
   .RST(rst)
);

IWB writebackModule(
   .PC_in_wb(PC_in_wb),
   .Inst_in_wb(Inst_in_wb),
   .WBSel_in_wb(WBSel_in_wb),
   .LoadExtended_in_wb(LoadExtended_in_wb),
   .Immediate_in_wb(Immediate_in_wb),
   .ALUOutput_in_wb(ALUOutput_in_wb),
   .Rdst_in_wb(Rdst_in_wb),
   .halt_in_wb(halt_in_wb),
   .Rw_out_wb(RW_in_id),
   .Di_out_wb(RWrData),
   .WEN(1'b0),
   .CLK(clk),
   .RST(rst)
);

assign halt = halt_in_wb & valid_wb;

endmodule // SingleCycleCPU

module HazardDetect(
   input [4:0] Rsrc1_out_id,
   input [4:0] Rsrc2_out_id,
   input [4:0] Rsrc1_in_ex,
   input [4:0] Rsrc2_in_ex,
   input [4:0] Rdst_in_ex,
   input [4:0] Rdst_in_mem,
   input [4:0] Rdst_in_wb,
   input RWrEn_in_ex,
   input RWrEn_in_mem,
   output reg stall_in_ex,
   output reg stall_in_mem);
   always @(*) begin
      if ((Rsrc1_out_id == Rdst_in_ex) & (!RWrEn_in_ex) & (Rdst_in_ex != 0))
         stall_in_ex <= 1;
      else if ((Rsrc2_out_id == Rdst_in_ex) & (!RWrEn_in_ex) & (Rdst_in_ex != 0))
         stall_in_ex <= 1;
      else
         stall_in_ex <= 0;
      if ((Rsrc1_out_id == Rdst_in_mem) & (!RWrEn_in_mem) & (Rdst_in_mem != 0))
         stall_in_mem <= 1;
      else if ((Rsrc2_out_id == Rdst_in_mem) & (!RWrEn_in_mem) & (Rdst_in_mem != 0))
         stall_in_mem <= 1;
      else
         stall_in_mem <= 0;
	end
endmodule

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
input [31:0] opA, opB;
input [2:0] func;
input [6:0] auxFunc;
input [6:0] opcode;
output reg halt;
// Place your code here
wire signed [31:0] s_opA, s_opB;
assign s_opA = opA;
assign s_opB = opB;
always @(*) begin
    halt <= 1'b0;
    if(opcode == `OPCODE_COMPUTE || opcode == `OPCODE_COMPUTE_IMM) begin
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
    else
        out = opA + opB;
end
endmodule // ExecutionUnit

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

module IMem(
    input halt_in_mem,
    input [31:0] PC_in_mem,
    input [31:0] Instr_in_mem,
    input [31:0] ALUOutput_in_mem,
    input [1:0] MemSize_in_mem,
    input MemWrEn_in_mem,
    input [31:0] Rdata2_in_mem,
    input clk,
    input rst,
    output [31:0] LoadExtended_out_mem,
    output halt_out_mem);

    wire [6:0] opcode;
    wire unalignedAccess;
    assign opcode = Instr_in_mem[6:0];
    assign unalignedAccess = (opcode == `OPCODE_LOAD || opcode == `OPCODE_STORE)? 
                    (((MemSize_in_mem == `SIZE_HWORD && ALUOutput_in_mem[0] != 1'b0) || (MemSize_in_mem == `SIZE_WORD && (ALUOutput_in_mem[0] != 1'b0 || ALUOutput_in_mem[0] != 1'b0)))?
                        1: 0) :0;
    wire MemRW_Halt_Gated;
    wire [31:0] DataWord;
    wire [2:0] funct3;
    assign funct3 = Instr_in_mem[14:12];
    assign halt_out_mem = halt_in_mem | unalignedAccess | (opcode == `OPCODE_LOAD && invalidLoadExtend);
    assign MemRW_Halt_Gated = (halt_out_mem | MemWrEn_in_mem);
    DataMem DMEM(.Addr(ALUOutput_in_mem), .Size(MemSize_in_mem), .DataIn(Rdata2_in_mem), .DataOut(DataWord), .WEN(MemRW_Halt_Gated), .CLK(clk));
    wire invalidLoadExtend;
    LoadExtend LoadExtender(.DataWord(DataWord), .funct3(funct3), .LoadExtended(LoadExtended_out_mem), .halt(invalidLoadExtend));
endmodule

module LoadExtend(
    input [31:0] DataWord,
    input [2:0] funct3,
    output reg [31:0] LoadExtended,
    output reg halt
);
always @(*) begin
    halt <= 1'b0;
    case (funct3)
        3'b000: LoadExtended = {{24{DataWord[7]}}, DataWord[7:0]};
        3'b001: LoadExtended = {{16{DataWord[15]}}, {DataWord[15:0]} };
        3'b010: LoadExtended = DataWord;
        3'b100: LoadExtended = {{24{1'b0}}, {DataWord[7:0]} };
        3'b101: LoadExtended = {{16{1'b0}}, {DataWord[15:0]} };
        default: begin
            halt <= 1'b1;
            LoadExtended = {32{1'b0}};
        end
    endcase
   end
endmodule

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

module IF_ID_Register(
    input [31:0] PC_if,
    input [31:0] Inst_if,
    input halt_if,
    input valid_if,
    output reg valid_id,
    output reg halt_id,
    output reg [31:0] PC_id,
    output reg [31:0] Inst_id,
    input stall,
    input squash,
    input WEN, 
    input CLK, 
    input RST);
    always @ (negedge CLK or negedge RST)
        if(squash) begin
            halt_id <= 0;
            PC_id <= 0;
            Inst_id <= 32'h00000013;
            valid_id <= 1'b1;
        end else if (stall) begin
            // Latch values
        end else if (!RST || (valid_if == 1'b0)) begin
            halt_id <= 0;
            PC_id <= 0;
            Inst_id <= 0;
            valid_id <= 0;
        end else if (!WEN) begin
            valid_id <= valid_if;
            halt_id <= halt_if & valid_if;
            PC_id <= PC_if;
            Inst_id <= Inst_if;
        end
endmodule // IF_ID_Register

module ID_EX_Register(
    input [31:0] PC_id,
    input [31:0] Inst_id,
    input MemRW_id, 
    input RWrEn_id,
    input [1:0] ALUOp_id,
    input [1:0] ALUSrc_id,
    input [4:0] RegDst_id,
    input [2:0] ImmSel_id,
    input ASel_id,
    input BSel_id,
    input JMP_id,
    input BR_id,
    input [1:0] WBSel_id,
    input [31:0] Immediate_id,
    input [1:0] MemSize_id,
    input [31:0] Rdata1_id,
    input [31:0] Rdata2_id,
    input [4:0] Rsrc1_id,
    input [4:0] Rsrc2_id,
    input halt_id,
    input stall,
    input valid_id,
    output reg valid_ex,
    output reg [31:0] PC_ex,
    output reg [31:0] Inst_ex,
    output reg MemRW_ex,
    output reg RWrEn_ex,
    output reg [1:0] ALUOp_ex,
    output reg [1:0] ALUSrc_ex,
    output reg [4:0] RegDst_ex,
    output reg [2:0] ImmSel_ex,
    output reg [31:0] Rdata1_ex,
    output reg [31:0] Rdata2_ex,
    output reg ASel_ex,
    output reg BSel_ex,
    output reg JMP_ex,
    output reg BR_ex,
    output reg [1:0] WBSel_ex,
    output reg [31:0] Immediate_ex,
    output reg [1:0] MemSize_ex,
    output reg [4:0] Rsrc1_ex,
    output reg [4:0] Rsrc2_ex,
    output reg halt_ex,
    input squash,
    input WEN, 
    input CLK, 
    input RST);
    wire [2:0] test;
    always @ (negedge CLK or negedge RST) begin
        if(squash) begin
            valid_ex <= 0;
            PC_ex <= 0;
            Inst_ex <= 32'h00000013;
            MemRW_ex <= 1;
            RWrEn_ex <= 1;
            ALUOp_ex <= 0;
            ALUSrc_ex <= 0;
            RegDst_ex <= 0;
            ImmSel_ex <= 0;
            ASel_ex <= 0;
            BSel_ex <= 0;
            JMP_ex <= 0;
            BR_ex <= 0;
            WBSel_ex <= 0;
            Immediate_ex <= 0;
            MemSize_ex <= 0;
            Rdata1_ex <= 0;
            Rdata2_ex <= 0;
            Rsrc1_ex <= 0;
            Rsrc2_ex <= 0;
            halt_ex <= 0;
        end else if (stall) begin
            // latch values
        end else if (!RST || (valid_id == 1'b0)) begin
            valid_ex <= 0;
            PC_ex <= 0;
            Inst_ex <= 0;
            MemRW_ex <= 0;
            RWrEn_ex <= 0;
            ALUOp_ex <= 0;
            ALUSrc_ex <= 0;
            RegDst_ex <= 0;
            ImmSel_ex <= 0;
            ASel_ex <= 0;
            BSel_ex <= 0;
            JMP_ex <= 0;
            BR_ex <= 0;
            WBSel_ex <= 0;
            Immediate_ex <= 0;
            MemSize_ex <= 0;
            Rdata1_ex <= 0;
            Rdata2_ex <= 0;
            Rsrc1_ex <= 0;
            Rsrc2_ex <= 0;
            halt_ex <= 0;
        end else if (!WEN) begin
            valid_ex <= valid_id;
            PC_ex <= PC_id;
            Inst_ex <= Inst_id;
            MemRW_ex <= MemRW_id;
            RWrEn_ex <= RWrEn_id;
            ALUOp_ex <= ALUOp_id;
            ALUSrc_ex <= ALUSrc_id;
            RegDst_ex <= RegDst_id;
            ImmSel_ex <= ImmSel_id;
            ASel_ex <= ASel_id;
            BSel_ex <= BSel_id;
            JMP_ex <= JMP_id;
            BR_ex <= BR_id;
            WBSel_ex <= WBSel_id;
            Immediate_ex <= Immediate_id;
            MemSize_ex <= MemSize_id;
            Rdata1_ex <= Rdata1_id;
            Rdata2_ex <= Rdata2_id;
            Rsrc1_ex <= Rsrc1_id;
            Rsrc2_ex <= Rsrc2_id;
            halt_ex <= halt_id && valid_id;
        end
    end
endmodule // ID_EX_Register

module EX_MEM_Register(
    input [31:0] PC_ex,
    input [31:0] Inst_ex,
    input MemRW_ex,
    input RWrEn_ex,
    input MemToReg_ex,
    input BranchCondTrue_ex,  
    input [1:0] WBSel_ex,
    input [1:0] MemSize_ex,
    input [31:0] ALUOutput_ex,
    input [31:0] Immediate_ex,
    input [4:0] Rdst_ex,
    input [31:0] Rdata2_ex,
    input halt_ex,
    input valid_ex,
    output reg valid_mem,
    output reg [31:0] PC_mem,
    output reg [31:0] Inst_mem,
    output reg MemRW_mem,
    output reg RWrEn_mem,
    output reg BranchCondTrue_mem,
    output reg [1:0] WBSel_mem,
    output reg [1:0] MemSize_mem,
    output reg [31:0] ALUoutput_mem,
    output reg [31:0] Immediate_mem,
    output reg [4:0] Rdst_mem,
    output reg [31:0] Rdata2_mem,
    output reg halt_mem,
    input squash,
    input WEN, 
    input CLK, 
    input RST);
    always @ (negedge CLK or negedge RST)
        if (!RST || !valid_ex) begin
            valid_mem <= 0;
            PC_mem <= 0;
            Inst_mem <= 0;
            MemRW_mem <= 1;
            RWrEn_mem <= 0;
            BranchCondTrue_mem <= 0;
            WBSel_mem <= 0;
            MemSize_mem <= 0;
            ALUoutput_mem <= 0;
            Rdst_mem <= 0;
            Rdata2_mem <= 0;
            Immediate_mem <= 0;
            halt_mem <= 0;
        end 
        else if (squash) begin
            valid_mem <= 1;
            PC_mem <= PC_ex;
            Inst_mem <= 32'h00000013;
            MemRW_mem <= `MemRW_Read;
            RWrEn_mem <= `RWrEn_Disable;
            BranchCondTrue_mem <= 1'b0;
            WBSel_mem <= `WBSel_PC4;
            MemSize_mem <= `SIZE_WORD;
            ALUoutput_mem <= 32'h00000000;
            Immediate_mem <= 32'h00000000;
            Rdst_mem <= 5'b00000;
            Rdata2_mem <= 32'h00000000;
            halt_mem <= 1'b0;
        end else if (!WEN) begin
            valid_mem <= valid_ex;
            PC_mem <= PC_ex;
            Inst_mem <= Inst_ex;
            MemRW_mem <= MemRW_ex;
            RWrEn_mem <= RWrEn_ex;
            BranchCondTrue_mem <= BranchCondTrue_ex;
            WBSel_mem <= WBSel_ex;
            MemSize_mem <= MemSize_ex;
            ALUoutput_mem <= ALUOutput_ex;
            Immediate_mem <= Immediate_ex;
            Rdst_mem <= Rdst_ex;
            Rdata2_mem <= Rdata2_ex;
            halt_mem <= halt_ex && valid_ex;
        end
endmodule // EX_MEM_Register

module MEM_WB_Register(
    input [31:0] PC_mem,
    input [31:0] Inst_mem,
    input MemRW_mem,
    input RWrEn_mem,
    input [1:0] WBSel_mem,
    input [31:0] LoadExtended_mem,
    input [31:0] Immediate_mem,
    input [31:0] ALUOutput_mem,
    input [4:0] Rdst_mem,
    input halt_mem,
    input valid_mem,
    output reg valid_wb,
    output reg [31:0] PC_wb,
    output reg [31:0] Inst_wb,
    output reg MemRW_wb,
    output reg RWrEn_wb,
    output reg [1:0] WBSel_wb,
    output reg [31:0] LoadExtended_wb,
    output reg [31:0] Immediate_wb,
    output reg [31:0] ALUOutput_wb,
    output reg [4:0] Rdst_wb,
    output reg halt_wb,
    input WEN, 
    input CLK, 
    input RST);
    always @ (negedge CLK or negedge RST)
        if (!RST || !valid_mem) begin
            valid_wb <= 0;
            PC_wb <= 0;
            Inst_wb <= 0;
            WBSel_wb <= 0;
            LoadExtended_wb <= 0;
            Immediate_wb <= 0;
            ALUOutput_wb <= 0;
            halt_wb <= 0;
            Rdst_wb <= 0;
            RWrEn_wb <= 1;
        end else if (!WEN) begin
            valid_wb <= valid_mem;
            PC_wb <= PC_mem;
            Inst_wb <= Inst_mem;
            WBSel_wb <= WBSel_mem;
            LoadExtended_wb <= LoadExtended_mem;
            Immediate_wb <= Immediate_mem;  
            ALUOutput_wb <= ALUOutput_mem;
            Rdst_wb <= Rdst_mem;
            RWrEn_wb <= RWrEn_mem;
            halt_wb <= halt_mem && valid_mem;
        end
endmodule // MEM_WB_Register

module IDecode(
    input halt_in_id,
    input [31:0] instr_in_id,
    input [31:0] pc_in_id,
    input [31:0] RWrData_in_id,
    input RWrEn_in_wb,
    input [4:0] RW_in_wb,
    output halt_out_id,
    output [31:0] pc_out_id,
    output MemRW_out_id,
    output RWrEn_out_id,
    output [1:0] ALUOp_out_id,
    output [1:0] ALUSrc_out_id,
    output [2:0] ImmSel_out_id,
    output ASel_out_id,
    output BSel_out_id,
    output JMP_out_id,
    output BR_out_id,
    output [1:0] MemSize_out_id,
    output [31:0] Immediate_out_id,
    output [31:0] Rdata1_out_id,
    output [31:0] Rdata2_out_id,
    output [4:0] Rdst_out_id,
    output [1:0] WBSel_out_id,
    output [4:0] Rsrc1_out_id,
    output [4:0] Rsrc2_out_id,
    input clk,
    input rst);

// Decode instruction
wire [6:0]  opcode;
wire [6:0]  funct7;
wire [2:0]  funct3;
wire [4:0]  Rsrc1, Rsrc2, Rdst;
assign opcode = instr_in_id[6:0];   
assign Rdst = instr_in_id[11:7]; 
assign Rsrc1 = instr_in_id[19:15]; 
assign Rsrc2 = instr_in_id[24:20];
assign funct3 = instr_in_id[14:12];
assign funct7 = instr_in_id[31:25]; 
 
// Generate signals
wire invalidOpcode;
OpDecoder decoder(
    .op(opcode),
    .funct3(funct3),
    .funct7(funct7),
    .ImmSel(ImmSel_out_id),
    .ASel(ASel_out_id),
    .BSel(BSel_out_id),
    .MemRW(MemRW_out_id),
    .RWrEn(RWrEn_out_id),
    .WBSel(WBSel_out_id),
    .BR_out_id(BR_out_id),
    .JMP_out_id(JMP_out_id),
    .halt(invalidOpcode)
);

// Get size of instr
wire invalidOpSize;
SizeModule size(
    .funct3(funct3),
    .MemSize(MemSize_out_id),
    .halt(invalidOpSize)
);

// Generate immediate
ImmGen immGenerator(
    .InstWord(instr_in_id),
    .ImmSel(ImmSel_out_id),
    .Imm(Immediate_out_id)
);

// Get registers
wire RWrEn_Halt_Gated;
wire halt;
assign halt = invalidOpcode | (((opcode == `OPCODE_STORE || opcode == `OPCODE_LOAD) && invalidOpSize)? 1 : 0);
assign RWrEn_Halt_Gated = (RWrEn_in_wb); // TODO: Halt Gate
RegFile RF(.AddrA(Rsrc1), .DataOutA(Rdata1_out_id), 
        .AddrB(Rsrc2), .DataOutB(Rdata2_out_id), 
        .AddrW(RW_in_wb), .DataInW(RWrData_in_id), .WenW(RWrEn_Halt_Gated), .CLK(~clk));

// Propogate and assign halt
assign halt_out_id = halt_in_id | invalidOpcode | (((opcode == `OPCODE_STORE || opcode == `OPCODE_LOAD) && invalidOpSize)? 1 : 0);
assign Rdst_out_id = Rdst;
assign pc_out_id = pc_in_id;
assign Rsrc1_out_id = Rsrc1;
assign Rsrc2_out_id = Rsrc2;
endmodule // IDecode

module OpDecoder(
   input [6:0] op,
   input [2:0] funct3,
   input [6:0] funct7,
   output reg [2:0] ImmSel, 
   output reg ASel, 
   output reg BSel, 
   output reg MemRW, 
   output reg RWrEn, 
   output reg [1:0] WBSel,
   output reg BR_out_id,
   output reg JMP_out_id,
   output reg halt
);
   always @(*) begin
      case (op)
            `OPCODE_COMPUTE: // R-Type
                begin
                halt <= 1'b0;
                BR_out_id <= 1'b0;
                JMP_out_id <= 1'b0;
                ImmSel <= 3'bx;
                ASel <= `ASel_Reg;
                BSel <= `BSel_Reg;
                MemRW <= `MemRW_Read;
                RWrEn <= `RWrEn_Enable;
                WBSel <= `WBSel_ALU;
                end
            `OPCODE_COMPUTE_IMM: // I-Type
                begin
                halt <= 1'b0;
                BR_out_id <= 1'b0;
                JMP_out_id <= 1'b0;
                ImmSel <= `ImmSel_I;
                ASel <= `ASel_Reg;
                BSel <= `BSel_IMM;
                MemRW <= `MemRW_Read;
                RWrEn <= `RWrEn_Enable;
                WBSel <= `WBSel_ALU;
                end
            `OPCODE_BRANCH: 
               begin
                halt <= 1'b0;
                BR_out_id <= 1'b1;
                JMP_out_id <= 1'b0;
                ImmSel <= `ImmSel_B;
                ASel <= `ASel_PC;
                BSel <= `BSel_IMM;
                MemRW <= `MemRW_Read;
                RWrEn <= `RWrEn_Disable;
                WBSel <= 2'bxx;
               end
            `OPCODE_LOAD:
               begin
                halt <= 1'b0;
               BR_out_id <= 1'b0;
               JMP_out_id <= 1'b0;
               ImmSel <= `ImmSel_I;
               ASel <= `ASel_Reg;
               BSel <= `BSel_IMM;
               MemRW <= `MemRW_Read;
               RWrEn <= `RWrEn_Enable;
               WBSel <= `WBSel_Mem;
               end
            `OPCODE_STORE:
               begin
                halt <= 1'b0;
               BR_out_id <= 1'b0;
               JMP_out_id <= 1'b0;
               ImmSel <= `ImmSel_S;
               ASel <= `ASel_Reg;
               BSel <= `BSel_IMM;
               MemRW <= `MemRW_Write;
               RWrEn <= `RWrEn_Disable;
               WBSel <= 2'bxx;
               end
            `OPCODE_JMP:
                begin
                    halt <= 1'b0;
                    BR_out_id <= 1'b0;
                    JMP_out_id <= 1'b1;
                    ImmSel <= `ImmSel_J;
                    ASel <= `ASel_PC;
                    BSel <= `BSel_IMM;
                    MemRW <= `MemRW_Read;
                    RWrEn <= `RWrEn_Enable;
                    WBSel <= `WBSel_PC4;
                end
            `OPCODE_JMP_LINK:
                begin
                    halt <= 1'b0;
                    BR_out_id <= 1'b0;
                    JMP_out_id <= 1'b1;
                    ImmSel <= `ImmSel_I;
                    ASel <= `ASel_Reg;
                    BSel <= `BSel_IMM;
                    MemRW <= `MemRW_Read;
                    RWrEn <= `RWrEn_Enable;
                    WBSel <= `WBSel_PC4;
                end
            `OPCODE_AUIPC:
                begin
                    halt <= 1'b0;
                    BR_out_id <= 1'b0;
                    JMP_out_id <= 1'b0;
                    ImmSel <= `ImmSel_U;
                    ASel <= `ASel_PC;
                    BSel <= `BSel_IMM;
                    MemRW <= `MemRW_Read;
                    RWrEn <= `RWrEn_Enable;
                    WBSel <= `WBSel_ALU;
                end
            `OPCODE_LUI:
                begin
                    halt <= 1'b0;
                    BR_out_id <= 1'b0;
                    JMP_out_id <= 1'b0;
                    ImmSel <= `ImmSel_U;
                    ASel <= `ASel_Reg;
                    BSel <= `BSel_IMM;
                    MemRW <= `MemRW_Read;
                    RWrEn <= `RWrEn_Enable;
                    WBSel <= `WBSel_Imm;
                end
            default:
            begin
               halt <= 1'b1;
            end
        endcase
   end
endmodule

module SizeModule(input [2:0] funct3,
                output reg [1:0] MemSize,
                output reg halt
                );
always @(*) begin
    halt <= 1'b0;
    case (funct3)
        3'b000: MemSize = `SIZE_BYTE;
        3'b001: MemSize = `SIZE_HWORD;
        3'b010: MemSize = `SIZE_WORD;
        default: begin 
            halt <= 1'b1;
            MemSize = 3'bxxx;
        end
    endcase
   end
endmodule

module ImmGen(
   input [31:0] InstWord,
   input [2:0]  ImmSel, 
   output reg [31:0] Imm);
   always @(*) 
   case(ImmSel)
        `ImmSel_I: Imm = { {21{InstWord[31]}}, InstWord[30:25], InstWord[24:21], InstWord[20]};
        `ImmSel_S: Imm = { {21{InstWord[31]}}, InstWord[30:25], InstWord[11:8], InstWord[7]};
        `ImmSel_U: Imm = { InstWord[31], InstWord[30:20], InstWord[19:12], {12{1'b0}} };
        `ImmSel_J: Imm = { {12{InstWord[31]}}, InstWord[19:12], InstWord[20], InstWord[30:25], InstWord[24:21], {1{1'b0}}};
        `ImmSel_B: Imm = { {20{InstWord[31]}}, InstWord[7], InstWord[30:25], InstWord[11:8], {1{1'b0}}};
        default: Imm = 32'bx;
    endcase
endmodule
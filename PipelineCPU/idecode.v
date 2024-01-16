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
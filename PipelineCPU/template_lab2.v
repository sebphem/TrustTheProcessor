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
// Template for Northwestern - CompEng 361 - Lab2
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

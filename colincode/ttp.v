// Northwestern - CompEng 362
// Company Name: Trust the Processor
// Daniel Shaver, Sebastian Phemister, (Ran)

//
// defines
//

// size codes
`define SIZE_BYTE  2'b00
`define SIZE_HWORD 2'b01
`define SIZE_WORD  2'b10

// opcode types
`define OPCODE_COMPUTE    7'b0110011
`define OPCODE_ICOMPUTE   7'b0010011
`define OPCODE_BRANCH     7'b1100011
`define OPCODE_LOAD       7'b0000011
`define OPCODE_STORE      7'b0100011 
`define OPCODE_JAL        7'b1101111
`define OPCODE_JALR       7'b1100111
`define OPCODE_LUI        7'b0110111
`define OPCODE_AUIPC      7'b0010111

// immediate types0
`define R_IMM 3'b000
`define I_IMM 3'b001
`define S_IMM 3'b010
`define B_IMM 3'b011
`define U_IMM 3'b100
`define J_IMM 3'b101
`define SI_IMM 3'b110

// branch Y/N
`define BR_FALSE 2'b10
`define BR_TRUE 2'b11
`define BR_NOTABRANCH 2'b00


// writeback select codes
`define WB_ALU 2'b00
`define WB_MEM 2'b01
`define WB_PC4 2'b10
`define WB_IMM 2'b11
`define WB_UNDEF 2'bxx

// ALU select codes
`define ALU_A_REG 1'b0
`define ALU_A_PC  1'b1
`define ALU_B_REG 1'b0
`define ALU_B_IMM 1'b1

// PC source codes
`define PC_PCPLUS4 1'b0
`define PC_ALUOUT 1'b1

// Load size/type codes
`define LOAD_BYTE 3'b000
`define LOAD_HALF 3'b001
`define LOAD_WORD 3'b010
`define LOAD_BYTE_U 3'b100
`define LOAD_HALF_U 3'b101

// writeback codes
`define WB_ALU 2'b00
`define WB_MEM 2'b01
`define WB_PC4 2'b10
`define WB_IMM 2'b11

module PipelinedCPU(halt, CLK, rst);
    output halt;
    input CLK, rst;

    // initialize wiring
    // IF --> ID
    wire [31:0] InstWord_out_F,     InstWord_in_D;
    wire [31:0] PC_out_F,           PC_in_D;
    wire [31:0] PC_Plus4_out_F,     PC_Plus4_in_D;

    // ID --> Forwarding Data
    wire [31:0] RegAData_out_D_into_FU, RegBData_out_D_into_FU;
    // EX --> Forwarding Data
    wire [31:0] Rdst_Data_out_E;

    // ID --> EX Data
    wire [31:0] InstWord_out_D,     InstWord_in_E;
    wire [31:0] PC_out_D,           PC_in_E;
    wire [31:0] PC_Plus4_out_D,     PC_Plus4_in_E;
    wire [31:0] RegAData_out_D,     RegAData_in_E;
    wire [31:0] RegBData_out_D,     RegBData_in_E;
    wire [4:0] Rdst_out_D,          Rdst_in_E;
    wire [31:0] Immediate_out_D,    Immediate_in_E;
    wire [31:0] targetAddr_in_E;
    // ID --> EX Ctrl
    wire ALUsrcA_out_D,             ALUsrcA_in_E;
    wire ALUsrcB_out_D,             ALUsrcB_in_E;
    wire [1:0] WBSel_out_D,         WBSel_in_E;
    wire [2:0] ImmSel_out_D,        ImmSel_in_E;
    wire MemWrEn_out_D,             MemWrEn_in_E;
    wire RegWrEn_out_D,             RegWrEn_in_E;
    wire [2:0] LoadType_out_D,      LoadType_in_E;
    wire [1:0] MemSize_out_D,       MemSize_in_E;
    wire didBranch_out_D,           didBranch_in_E;
    // ID --> hazard wires
    wire [4:0] RegA_out_ID, RegB_out_ID;
    
    // ID --> IF for branch/jump
    wire PCSel_ID_IF, PCSel_EX_IF;
    wire [31:0] targetAddr_ID_IF, targetAddr_EX_IF;

    // EX --> ID for branch/jump
    wire [1:0] taken_EX_ID;
    
    wire ID_EX_nop, IF_ID_nop;
    
    // EX --> MEM data
    wire [31:0] ALUresult_out_E,    ALUresult_in_M;
    wire [31:0] RegBData_out_E,     RegBData_in_M;
    wire [31:0] Immediate_out_E,    Immediate_in_M;
    wire [31:0] PC_Plus4_out_E,     PC_Plus4_in_M;
    wire [4:0] Rdst_out_E,          Rdst_in_M;
    wire [31:0] InstWord_out_E,     InstWord_in_M;
  
    // EX --> MEM ctrl
    wire MemWrEn_out_E,             MemWrEn_in_M;
    wire RegWrEn_out_E,             RegWrEn_in_M;
    wire [1:0] WBSel_out_E,         WBSel_in_M;
    wire [2:0] LoadType_out_E,      LoadType_in_M;
    wire [1:0] MemSize_out_E,       MemSize_in_M;

    // MEM --> WB data
    wire [31:0] Rdst_Data_out_M;
    wire [31:0] Rdst_Data_in_W;
    wire [4:0] Rdst_out_M,          Rdst_in_W; // goes straight back to regfile

    // MEM --> WB ctrl
    wire RegWrEn_out_M,             RegWrEn_in_W; // goes straight back to regfile
    wire [1:0] WBSel_out_M,         WBSel_in_W;

    // MEM --> EX full forwarding
    wire [31:0] FF_out_M;


    // WB --> ID data to writeback to regfile
    wire [31:0] RegWrData_out_W;

    // halt wires
    wire halt_IF_in;
    wire halt_ID_out;
    wire halt_EX_in, halt_EX_out;
    wire halt_MEM_in, halt_MEM_out;
    wire halt_WB_in, halt_WB_out;
    // if any halt is in progress, stop the PC and don't continue to execute
    wire halt_in_progress;
    assign halt_in_progress = halt_ID_out || halt_EX_out || halt_MEM_out || halt_WB_in;

    // initialization stage wires
    wire IF_ID_NEW;
    wire ID_EX_NEW;
    wire EX_MEM_NEW;
    wire MEM_WB_NEW;

    wire IF_ID_stall_out;
    wire hazard_MEM_only;
    // if ID / EX hazard detected, freeze PC & IF/ID reg, make ID/EX nop
    // if ID / MEM hazard detected & no EX hazard, freeze PC, IF/ID, 
    // and ID/EX, and make EX/MEM nop

    // pipeline stages
    // stage 1 - IF
    //assign halt_IF_in = (halt_ID_out && !(IF_ID_NEW)) ||
    //                    (halt_EX_out && !(ID_EX_NEW)) ||
    //                    (halt_MEM_out && !(EX_MEM_NEW)) ||
    //                    (halt_WB_out && !(MEM_WB_NEW));

    InstructionFetch IF(.CLK(CLK), .RST(rst),
                        .stall(IF_ID_stall_out),
                        .PCSel_ID(PCSel_ID_IF & (!ID_EX_nop)), .PCSel_EX(PCSel_EX_IF), .targetAddr_EX(targetAddr_EX_IF), .targetAddr_ID(targetAddr_ID_IF),
                        .InstWord(InstWord_out_F), .PC(PC_out_F), .PC_Plus4(PC_Plus4_out_F));

    // transfer to ID
    IF_ID_data_reg IF_ID(.WEN(1'b0), .CLK(CLK), .RST(rst),
                            .InstWord_F(InstWord_out_F),    .InstWord_D(InstWord_in_D),
                            .PC_F(PC_out_F),                .PC_D(PC_in_D),
                            .PC_Plus4_F(PC_Plus4_out_F),    .PC_Plus4_D(PC_Plus4_in_D),
                            .NEW(IF_ID_NEW),
                            .nop(IF_ID_nop),
                            //TODO Fix this
                            .stall(IF_ID_stall_out));

    // stage 2 - ID
    InstructionDecode ID(.CLK(CLK), .RST(rst), .InstWord_in(InstWord_in_D), .PC_in(PC_in_D), .PC_Plus4_in(PC_Plus4_in_D),
                        .InstWord_out(InstWord_out_D), .PC_out(PC_out_D), .PC_Plus4_out(PC_Plus4_out_D),
                        .RegWrData_WB(Rdst_Data_in_W), .RegWrEn_WB(RegWrEn_in_W), .Rdst_WB(Rdst_in_W),
                        .RegAData(RegAData_out_D_into_FU), .RegBData(RegBData_out_D_into_FU),
                        .Rdst(Rdst_out_D),
                        .ALUsrcA(ALUsrcA_out_D), .ALUsrcB(ALUsrcB_out_D), .WBSel(WBSel_out_D), .ImmSel(ImmSel_out_D),
                        .MemWrEn(MemWrEn_out_D), .RegWrEn(RegWrEn_out_D), .LoadType(LoadType_out_D), .MemSize(MemSize_out_D),
                        //reg names
                        .RegA(RegA_out_ID), .RegB(RegB_out_ID),
                        .halt_ID_out(halt_ID_out),
                        .targetAddr(targetAddr_ID_IF), .PCSel(PCSel_ID_IF), .taken((IF_ID_NEW) ? 2'b00 : taken_EX_ID),
                        .isNew(IF_ID_NEW),
                        .immediate(Immediate_out_D), .doBranch(didBranch_out_D));

    StallUnit StallDeterminer(
        //input
        .RegA_ID(RegA_out_ID), .RegB_ID(RegB_out_ID),
        .opcode_EX(InstWord_in_E[6:0]),
        .Rdst_EX(Rdst_in_E),
        //outputs
        .IF_ID_stall(IF_ID_stall_out)
    );

    ForwardingUnit Forwarding(
        //input
        .RegA_ID(RegA_out_ID), .RegB_ID(RegB_out_ID),
        .RegA_ID_cur(RegAData_out_D_into_FU), .RegB_ID_cur(RegBData_out_D_into_FU),
        // .opcode_EX(opcode_EX_out),
        .opcode_ID(InstWord_in_D[6:0]),
        .opcode_EX(InstWord_in_E[6:0]),
        .opcode_MEM(InstWord_in_M[6:0]),
        .Rdst_EX_Data(Rdst_Data_out_E),
        .Rdst_MEM_Data(Rdst_Data_out_M),
        .Rdst_EX_Name(Rdst_in_E), .Rdst_MEM_Name(Rdst_in_M),
        .RegWREN_EX(RegWrEn_in_E), .RegWREN_MEM(RegWrEn_in_M),
        //outputs
        .RegA_ID_Data(RegAData_out_D), .RegB_ID_Data(RegBData_out_D)
    );

    // transfer to EX
    ID_EX_data_reg ID_EX_data(.WEN(1'b0), .CLK(CLK), .RST(rst),
                            .InstWord_D(InstWord_out_D),    .InstWord_E(InstWord_in_E),
                            .PC_D(PC_out_D),                .PC_E(PC_in_E),
                            .PC_Plus4_D(PC_Plus4_out_D),    .PC_Plus4_E(PC_Plus4_in_E),
                            .RegAData_D(RegAData_out_D),    .RegAData_E(RegAData_in_E),
                            .RegBData_D(RegBData_out_D),    .RegBData_E(RegBData_in_E),
                            .targetAddr_D(targetAddr_ID_IF),.targetAddr_E(targetAddr_in_E),
                            .Immediate_D(Immediate_out_D),  .Immediate_E(Immediate_in_E),

                            .Rdst_D(Rdst_out_D),            .Rdst_E(Rdst_in_E),
                            .nop((ID_EX_nop || IF_ID_stall_out)),
                            .stall(1'b0));

    ID_EX_ctrl_reg ID_EX_ctrl(.WEN(1'b0), .CLK(CLK), .RST(rst),
                            .ALUsrcA_D(ALUsrcA_out_D),      .ALUsrcA_E(ALUsrcA_in_E),
                            .ALUsrcB_D(ALUsrcB_out_D),      .ALUsrcB_E(ALUsrcB_in_E),
                            .WBSel_D(WBSel_out_D),          .WBSel_E(WBSel_in_E),
                            .ImmSel_D(ImmSel_out_D),        .ImmSel_E(ImmSel_in_E),
                            .MemWrEn_D(MemWrEn_out_D),      .MemWrEn_E(MemWrEn_in_E),
                            .RegWrEn_D(RegWrEn_out_D),      .RegWrEn_E(RegWrEn_in_E),
                            .LoadType_D(LoadType_out_D),    .LoadType_E(LoadType_in_E),
                            .MemSize_D(MemSize_out_D),      .MemSize_E(MemSize_in_E),
                            .halt_D(halt_ID_out),           .halt_E(halt_EX_in),
                            .didBranch_D(didBranch_out_D),  .didBranch_E(didBranch_in_E),
                            .NEW_IN(IF_ID_NEW),             .NEW_OUT(ID_EX_NEW),
                            .nop((ID_EX_nop || IF_ID_stall_out)),
                            .stall(1'b0));

    // stage 3 - EX
    Execute EX(     // inputs
                    .InstWord(InstWord_in_E), .PC(PC_in_E), .PC_Plus4(PC_Plus4_in_E),
                    .RegAData(RegAData_in_E), .RegBData(RegBData_in_E), .Rdst(Rdst_in_E),
                    .ALUsrcA(ALUsrcA_in_E), .ALUsrcB(ALUsrcB_in_E), .WBSel(WBSel_in_E),
                    .ImmSel(ImmSel_in_E), .MemWrEn(MemWrEn_in_E), .RegWrEn(RegWrEn_in_E),
                    .LoadType(LoadType_in_E), .MemSize(MemSize_in_E), .immediate_in(Immediate_in_E),
                    .isNew(ID_EX_NEW), .didBranch(didBranch_in_E), .targetAddr_in(targetAddr_in_E),
                    // outputs
                    .ALUout(ALUresult_out_E), .RegBData_out(RegBData_out_E), .PC_Plus4_out(PC_Plus4_out_E),
                    .Rdst_out(Rdst_out_E), .immediate_out(Immediate_out_E),
                    //NOT ALWATS APPLICABLE, ONLY FOR CERTAIN CASES OF FULL FORWARDING
                    .Rdst_Data_out(Rdst_Data_out_E),
                    .MemWrEn_out(MemWrEn_out_E),
                    .RegWrEn_out(RegWrEn_out_E), .WBSel_out(WBSel_out_E), .LoadType_out(LoadType_out_E),
                    .MemSize_out(MemSize_out_E),
                    .take(taken_EX_ID), .targetAddr_out(targetAddr_EX_IF), .PCSel(PCSel_EX_IF),
                    .EX_nop(ID_EX_nop), .ID_nop(IF_ID_nop),
                    .InstWord_out(InstWord_out_E),
                    // halt signal
                    .halt_EX_in(halt_EX_in), .halt_EX_out(halt_EX_out));

    // transfer to MEM
    EX_MEM_data_reg EX_MEM_data(.WEN(1'b0), .CLK(CLK), .RST(rst),
                            .ALUresult_E(ALUresult_out_E),  .ALUresult_M(ALUresult_in_M),
                            .RegBData_E(RegBData_out_E),    .RegBData_M(RegBData_in_M),
                            .Immediate_E(Immediate_out_E),  .Immediate_M(Immediate_in_M),
                            .PC_Plus4_E(PC_Plus4_out_E),    .PC_Plus4_M(PC_Plus4_in_M),
                            .Rdst_E(Rdst_out_E),            .Rdst_M(Rdst_in_M),
                            .InstWord_E(InstWord_out_E),    .InstWord_M(InstWord_in_M)
                            );

    EX_MEM_ctrl_reg EX_MEM_ctrl(.WEN(1'b0), .CLK(CLK), .RST(rst),
                            .MemWrEn_E(MemWrEn_out_E),      .MemWrEn_M(MemWrEn_in_M),
                            .RegWrEn_E(RegWrEn_out_E),      .RegWrEn_M(RegWrEn_in_M),
                            .WBSel_E(WBSel_out_E),          .WBSel_M(WBSel_in_M),
                            .LoadType_E(LoadType_out_E),    .LoadType_M(LoadType_in_M),
                            .MemSize_E(MemSize_out_E),      .MemSize_M(MemSize_in_M),
                            .halt_E(halt_EX_out),           .halt_M(halt_MEM_in),
                            .NEW_IN(ID_EX_NEW),             .NEW_OUT(EX_MEM_NEW)
                            );

    // stage 4 - MEM
    MemoryAccess MEM( // inputs
                    .CLK(CLK), .ALUresult_in(ALUresult_in_M), .RegBData(RegBData_in_M),
                    .Immediate_in(Immediate_in_M), .PC_Plus4_in(PC_Plus4_in_M), .Rdst_in(Rdst_in_M),
                    .MemWrEn(MemWrEn_in_M), .RegWrEn_in(RegWrEn_in_M), .WBSel_in(WBSel_in_M),
                    .MemSize(MemSize_in_M), .LoadType(LoadType_in_M),
                    .isNew(EX_MEM_NEW), .InstWord(InstWord_in_M),
                    // outputs
                    .Rdst_out(Rdst_out_M),
                    .Rdst_Data_out(Rdst_Data_out_M),
                    .RegWrEn_out(RegWrEn_out_M),
                    // halt signal
                    .halt_MEM_in(halt_MEM_in), .halt_MEM_out(halt_MEM_out));

    // transfer to WB
    MEM_WB_data_reg MEM_WB_data(.WEN(1'b0), .CLK(CLK), .RST(rst),
                            .Rdst_Data_M(Rdst_Data_out_M),          .Rdst_Data_W(Rdst_Data_in_W),
                            .Rdst_M(Rdst_out_M),                    .Rdst_W(Rdst_in_W));

    MEM_WB_ctrl_reg MEM_WB_ctrl(.WEN(1'b0), .CLK(CLK), .RST(rst),
                            .RegWrEn_M(RegWrEn_out_M),          .RegWrEn_W(RegWrEn_in_W),
                            .halt_M(halt_MEM_out),              .halt_W(halt_WB_in),
                            .NEW_IN(EX_MEM_NEW),                .NEW_OUT(MEM_WB_NEW));

    // stage 5 - WB
    // WriteBack WB(.WB_data_in(Rdst_Data_in_W), .WB_data_out(RegWrData_out_W),
    //             .halt_WB_in(halt_WB_in), .halt_WB_out(halt_WB_out));

    // if halt reached the end of the pipeline, then halt!
    assign halt = halt_WB_in;
endmodule

// IF stage
// IN: current PC
// OUT: InstWord, PC+4
module InstructionFetch(CLK, RST, stall,
                        PCSel_ID, PCSel_EX, targetAddr_ID, targetAddr_EX,
                        InstWord, PC, PC_Plus4);
    input CLK, RST;
    input PCSel_ID, PCSel_EX;
    input stall;
    input [31:0] targetAddr_ID, targetAddr_EX;
    output [31:0] PC, PC_Plus4;
    output [31:0] InstWord;

    // PC mux to choose next PC
    wire [31:0] Next_PC;
    assign PC_Plus4 = PC + 4;

    // if stall is 1, then PC stops updating, otherwise it updates
    // if PCSel is 1, then target address from ALU, otherwise PC+4

    assign Next_PC = (stall) ? PC : ((PCSel_EX == 1'b1) ? targetAddr_EX : (PCSel_ID == 1'b1) ? targetAddr_ID : PC_Plus4);

    // PC reg
    Reg #(32) PC_REG(.Din(Next_PC), .Qout(PC), .WEN(1'b0), .CLK(CLK), .RST(RST));

    // instruction memory
    InstMem IMEM(.Addr(PC), .Size(`SIZE_WORD), .DataOut(InstWord), .CLK(CLK));
endmodule

// ID stage
// IN: InstWord, CLK, PC, PC+4, and RegWrData, RegWrEn, Rdst for WB data
// OUT: RegA & RegB data, various control signals
module InstructionDecode(InstWord_in, CLK, RST, PC_in, PC_Plus4_in,
                        RegWrData_WB, RegWrEn_WB, Rdst_WB,
                        RegAData, RegBData, Rdst, InstWord_out, PC_out, PC_Plus4_out,
                        ALUsrcA, ALUsrcB, WBSel, ImmSel, MemWrEn, RegWrEn, LoadType, MemSize,
                        RegA, RegB, immediate, doBranch,
                        halt_ID_out, targetAddr, taken, PCSel,
                        isNew);
    input [31:0] InstWord_in, PC_in, PC_Plus4_in;
    input CLK, RST;
    input [31:0] RegWrData_WB;
    input RegWrEn_WB;
    input [4:0] Rdst_WB;
    input isNew;

    // out data
    output [31:0] RegAData, RegBData;
    output [4:0] Rdst;
    output [31:0] InstWord_out, PC_out, PC_Plus4_out;
    output [4:0] RegA, RegB;

    // control signals
    output ALUsrcA, ALUsrcB, MemWrEn, RegWrEn;
    output [1:0] WBSel, MemSize;
    output [2:0] ImmSel, LoadType;

    output [31:0] targetAddr, immediate;
    input [1:0] taken;
    output PCSel;

    // halt signal
    output halt_ID_out;

    // instruction separate
    wire [6:0] opcode;
    wire [2:0] funct3;
    wire [6:0] funct7;

    assign opcode = InstWord_in[6:0];
    assign Rdst = InstWord_in[11:7];
    assign RegA = InstWord_in[19:15];
    assign RegB = InstWord_in[24:20];
    assign funct3 = InstWord_in[14:12];
    assign funct7 = InstWord_in[31:25];

    // register file read / write
    RegFile RF( // inputs
                .CLK(!CLK), .AddrA(RegA), .AddrB(RegB),
                .AddrW(Rdst_WB), .DataInW(RegWrData_WB), .WenW(RegWrEn_WB),
                // outputs
                .DataOutA(RegAData), .DataOutB(RegBData));

    // assign control signals
    wire controlHalt;
    ControlUnit CU( // inputs
                    .opcode(opcode), .funct3(funct3),
                    // outputs
                    .ImmSel(ImmSel), .RWrEn(RegWrEn), .ALUsrcA(ALUsrcA), 
                    .ALUsrcB(ALUsrcB), .MemWrEn(MemWrEn), .WBSel(WBSel), 
                    .LoadType(LoadType), .MemSize(MemSize), .halt(controlHalt));

    // immediate generation
    ImmediateGenerator IG(.InstWord(InstWord_in), .ImmSel(ImmSel), .immediate(immediate));
    
    // target address generation
    assign targetAddr = (opcode == `OPCODE_JALR) ? RegAData + immediate : PC_in + immediate;

    
    output doBranch;
    wire [1:0] curstate;
    wire [1:0] newstate;
    BranchPredictor BP(.opcode(opcode), .taken(taken), .prediction(doBranch), .curstate(isNew ? 2'b10 : curstate), .newstate(newstate));

    Reg #(2) BP_State_Reg(.Din(newstate), .Qout(curstate), .WEN(1'b0), .CLK(CLK), .RST(RST));

    // if we do branch, check if valid address
    wire alignHalt;
    PCAlign PCA(.PC(targetAddr), .doBranch(doBranch), .halt(alignHalt));

    // PCSel only comes from ALU if doBranch true
    assign PCSel = isNew ? `PC_PCPLUS4 : (doBranch ? `PC_ALUOUT : `PC_PCPLUS4);


    // assign halt based on control output & PC alignment if jump/br
    assign halt_ID_out = ((controlHalt | alignHalt) && !(isNew));

    // any values that we need to pass directly thru
    assign InstWord_out = InstWord_in;
    assign PC_out = PC_in;
    assign PC_Plus4_out = PC_Plus4_in;
endmodule

// EX stage
// IN: InstWord, PC, PC+4, RegAData, RegBData, Rdst, various control signals
// OUT: ALUout, immediate, various control signals
module Execute(     // inputs
                    InstWord, PC, PC_Plus4, RegAData, RegBData, Rdst,
                    ALUsrcA, ALUsrcB, WBSel, ImmSel, MemWrEn, RegWrEn,
                    LoadType, MemSize, immediate_in,
                    halt_EX_in, didBranch, targetAddr_in,
                    isNew,
                    // outputs
                    ALUout, RegBData_out, PC_Plus4_out, Rdst_out, PCSel, immediate_out,
                    Rdst_Data_out, MemWrEn_out, RegWrEn_out, WBSel_out, LoadType_out, MemSize_out,
                    InstWord_out, take, EX_nop, ID_nop, targetAddr_out,
                    halt_EX_out
                    // full forwarding
                    );
    // declare inputs
    input [31:0] InstWord, PC, PC_Plus4, RegAData, RegBData, immediate_in, targetAddr_in;
    input ALUsrcA, ALUsrcB, MemWrEn, RegWrEn;
    input [1:0] WBSel, MemSize;
    input [2:0] ImmSel, LoadType;
    input [4:0] Rdst;   
    input isNew;

    // declare outputs
    // data going to MEM
    output [31:0] ALUout, RegBData_out, PC_Plus4_out, Rdst_Data_out, targetAddr_out, immediate_out;
    output [4:0] Rdst_out;
    output [31:0] InstWord_out;

    // control signals going to MEM
    output MemWrEn_out, RegWrEn_out;
    output [1:0] WBSel_out, MemSize_out;
    output [2:0] LoadType_out;
    // halt signal
    input halt_EX_in;
    output halt_EX_out;

    // if the branch predictor branched
    input didBranch;
    output EX_nop, ID_nop, PCSel;

    // break up instruction parts that we need
    wire [6:0] opcode;
    wire [2:0] funct3;
    wire [6:0] funct7;
    assign opcode = InstWord[6:0];
    assign funct3 = InstWord[14:12];
    assign funct7 = InstWord[31:25];
    
        // detect if we branch or not
    output [1:0] take;
    wire branchHalt;
    BranchUnit BU(.opA(RegAData), .opB(RegBData), .funct3(funct3), .opcode(opcode), 
                .out(take), .halt(branchHalt), .didBranch(didBranch), .EX_nop(EX_nop), .ID_nop(ID_nop));

    assign targetAddr_out = (didBranch) ? PC_Plus4 : targetAddr_in;
    assign PCSel = (ID_nop & EX_nop) ? `PC_ALUOUT : `PC_PCPLUS4;

    // muxes determine ops for ALU
    wire [31:0] ALUopA, ALUopB;
    assign ALUopA = (ALUsrcA == `ALU_A_REG) ? RegAData : PC;
    assign ALUopB = (ALUsrcB == `ALU_B_REG) ? RegBData : immediate_in;

    // execute ALU
    wire ALUhalt;
    ArithmeticLogicUnit ALU(.opcode(opcode), .opA(ALUopA), .opB(ALUopB), .func(funct3), 
                            .auxFunc(funct7), .out(ALUout), .halt(ALUhalt));

    // assign halt based on branch or ALU or incoming halt signal
    assign halt_EX_out = (branchHalt | ALUhalt | halt_EX_in) && (!isNew);

    //TODO GET RID OF WRITE BACK STAGE
    // I still don't know what the purpose of immediate is
    assign Rdst_Data_out =  (WBSel == `WB_ALU) ? ALUout :
                                    ((WBSel == `WB_PC4) ? PC_Plus4 :
                                        ((WBSel == `WB_IMM) ? immediate_in: 32'b0)) ;
    // pass thru any signals that don't get changed by this stage
    // data signals
    assign RegBData_out = RegBData;
    assign PC_Plus4_out = PC_Plus4;
    assign Rdst_out = Rdst;
    assign InstWord_out = InstWord;
    assign immediate_out = immediate_in;
    // ctrl signals
    assign MemWrEn_out = MemWrEn;
    assign RegWrEn_out = RegWrEn;
    assign WBSel_out = WBSel;
    assign LoadType_out = LoadType;
    assign MemSize_out = MemSize;
endmodule

module MemoryAccess(CLK, ALUresult_in, RegBData, Immediate_in, PC_Plus4_in, Rdst_in,
                    MemWrEn, RegWrEn_in, WBSel_in, MemSize, LoadType, InstWord,
                    Rdst_out, Rdst_Data_out,
                    RegWrEn_out,
                    halt_MEM_in, halt_MEM_out,
                    isNew);
    // inputs
    input CLK;
    input [31:0] ALUresult_in, RegBData, Immediate_in, PC_Plus4_in, InstWord;
    input [4:0] Rdst_in;
    input MemWrEn, RegWrEn_in;
    input [1:0] WBSel_in, MemSize;
    input [2:0] LoadType;
    input isNew;

    // outputs
    output [4:0] Rdst_out;
    output RegWrEn_out;
    //choose data source
    output [31:0] Rdst_Data_out;

    // halt signal
    input halt_MEM_in;
    output halt_MEM_out;

    // ensure data address is properly aligned
    wire alignHalt;
    SizeCheck SC(.MemAddress(ALUresult_in), .opcode(InstWord[6:0]),
                .MemSize(MemSize), .halt(alignHalt));

    // access data memory
    // don't allow write if this instruction is halting
    wire [31:0] MemReadDataRaw;
    DataMem DMEM(.Addr(ALUresult_in), .Size(MemSize), .DataIn(RegBData),
                .DataOut(MemReadDataRaw), .WEN(MemWrEn || halt_MEM_out), .CLK(CLK));

    // sext MemReadDataRaw if necessary
    wire [31:0] MemReadData;
    MemSext MS(.MemReadIn(MemReadDataRaw), .LoadType(LoadType), .MemReadOut(MemReadData));

    // halt if MemReadData is not properly aligned
    assign halt_MEM_out = (halt_MEM_in || alignHalt) && (!isNew);

    // get the correct memory to put out - THIS IS LITERALLY JUST WRITE BACK STAGE
    //TODO GET RID OF WRITE BACK STAGE
    assign Rdst_Data_out =  (WBSel_in == `WB_ALU) ? ALUresult_in :
                                ((WBSel_in == `WB_MEM) ? MemReadData :
                                    ((WBSel_in == `WB_PC4) ? PC_Plus4_in :
                                        ((WBSel_in == `WB_IMM) ? Immediate_in: 32'b0))) ;
    // ctrl signals
    assign RegWrEn_out = RegWrEn_in;
    assign Rdst_out = Rdst_in;
endmodule

module WriteBack(WB_data_in, WB_data_out,
                halt_WB_in, halt_WB_out);
    // inputs
    input [31:0] WB_data_in;
    // output
    output [31:0] WB_data_out;
    // halt signal
    input halt_WB_in;
    output halt_WB_out;

    // halt can't get asserted here so just pass thru
    assign halt_WB_out = halt_WB_in;
    assign WB_data_out = WB_data_in;
endmodule

//
// ArithmeticLogicUnit
// in - opcode, opA, opB, funct3, funct7
// out - out, halt
module ArithmeticLogicUnit(opcode, opA, opB, func, auxFunc, out, halt);
    output reg [31:0] out;
    reg [63:0] mulout;
    output reg halt;
    input [6:0] opcode;
    input [31:0] opA, opB;
    input [2:0] func;
    input [6:0] auxFunc;

    wire signed [31:0] sopA, sopB;
    assign sopA = opA;
    assign sopB = opB;

    always @(*) begin
        halt = 1'b0; // default no halt
        if (opcode == `OPCODE_COMPUTE && auxFunc == 7'b0000001) begin 
            out = {sopA * sopB};
            out[31] = sopA[31] ^ sopB[31];
        end else if (opcode == `OPCODE_COMPUTE || opcode == `OPCODE_ICOMPUTE) begin
            case(func)
                3'b000: 
                    if (opcode == `OPCODE_ICOMPUTE)
                        out = opA + opB; // addi
                    else if (auxFunc == 7'h00)
                        out = opA + opB; // add
                    else if (auxFunc == 7'h20)
                        out = opA - opB; // sub 
                    else
                        halt = 1'b1; // invalid auxFunc
                3'b001: 
                    if(auxFunc == 7'h00)
                        out = opA << opB; // sll
                    else
                        halt = 1'b1; // invalid auxFunc
                3'b010: out = (sopA < sopB) ? 1 : 0; // slt
                3'b011: out = (opA < opB) ? 1 : 0; // sltu
                3'b100: out = opA ^ opB; // xor
                3'b101: // srl or sra
                    if(auxFunc == 7'h20) // sra
                        //out = sopA >>> opB; this works according to google but not with every compiler
                            out = (opA[31] == 1'b0) ? (opA >> opB) : (~(~opA >> opB));
                    else if (auxFunc == 7'h00) // srl
                        out = opA >> opB; 
                    else
                        halt = 1'b1; // invalid auxFunc
                3'b110: out = opA | opB; // or
                3'b111: out = opA & opB; // and
            endcase
        end
        else if (opcode == `OPCODE_COMPUTE && auxFunc != 7'h00)
                halt = 1'b1; // invalid auxFunc (doesn't apply to i-type)
        else 
            out = opA + opB; // default add for all other instruction types
    end
endmodule

// Branch Unit
// in - opA, opB, funct3, opcode
// out - out, halt
// outputs 1 if branch is taken, 0 if not
module BranchUnit(opA, opB, funct3, opcode, out, halt, didBranch, EX_nop, ID_nop);
  input [31:0] opA, opB;
  input [2:0] funct3;
  input [6:0] opcode;
  input didBranch;
  output reg EX_nop, ID_nop;
  output reg [1:0] out;
  output reg halt;

  wire signed [31:0] sopA, sopB;
  assign sopA = opA;
  assign sopB = opB;

  always @(*) begin
    halt = 1'b0;
    if (opcode == `OPCODE_JAL || opcode == `OPCODE_JALR)
        out = `BR_TRUE; // if jal or jalr, always branch (jump)
        else if (opcode == `OPCODE_BRANCH)
            begin
                case(funct3)
                3'b000: out = (opA == opB) ? `BR_TRUE : `BR_FALSE; // beq
                3'b001: out = (opA != opB) ? `BR_TRUE : `BR_FALSE; // bne
                3'b100: out = (sopA < sopB) ? `BR_TRUE : `BR_FALSE; // blt
                3'b101: out = (sopA >= sopB) ? `BR_TRUE : `BR_FALSE; // bge
                3'b110: out = (opA < opB) ? `BR_TRUE : `BR_FALSE; // bltu
                3'b111: out = (opA >= opB) ? `BR_TRUE : `BR_FALSE; // bgeu
                default: 
                    if (opcode == `OPCODE_BRANCH)
                    halt = 1'b1; // if branch opcode but not a valid br instruction, halt
                    else
                    out = `BR_NOTABRANCH; // default
                endcase
            end
    else 
        out = `BR_NOTABRANCH; // default

    if (opcode == `OPCODE_BRANCH || opcode == `OPCODE_JAL || opcode == `OPCODE_JALR) begin
        if (didBranch != out[0]) begin
            EX_nop = 1'b1;
            ID_nop = 1'b1;
        end
        else if (didBranch == 1'b1 & out == `BR_TRUE) begin
            EX_nop = 1'b1;
            ID_nop = 1'b0;
        end
        else begin
            EX_nop = 1'b0;
            ID_nop = 1'b0;
        end
    end else begin
        EX_nop = 1'b0;
        ID_nop = 1'b0;
    end

  end
endmodule // Branch Unit

// 2-bit Branch Predictor
module BranchPredictor(opcode, taken, prediction, curstate, newstate);
    input [1:0] taken;
    input [6:0] opcode;
    output reg prediction;
    input [1:0] curstate; 
    output reg [1:0] newstate;

    always @(*) begin
        if (opcode == `OPCODE_BRANCH) begin
            case (curstate)
            2'b00: prediction <= `BR_FALSE;
            2'b01: prediction <= `BR_FALSE;     
            2'b10: prediction <= `BR_TRUE;      
            2'b11: prediction <= `BR_TRUE; 
            endcase
        end
        else if (opcode == `OPCODE_JAL || opcode == `OPCODE_JALR) begin
            prediction <= `BR_TRUE;
        end
        else begin
            prediction <= `BR_FALSE;
        end
    
        if (taken != `BR_NOTABRANCH) begin
            case (curstate)
                2'b00: begin
                    if (taken == `BR_TRUE) newstate <= 2'b01;
                    else if (taken == `BR_FALSE) newstate <= 2'b00;
                end
                2'b01: begin
                    if (taken == `BR_TRUE) newstate <= 2'b10;
                    else if (taken == `BR_FALSE) newstate <= 2'b00;
                end        
                2'b10: begin
                    if (taken == `BR_TRUE) newstate <= 2'b11;  
                    else if (taken == `BR_FALSE) newstate <= 2'b01;
                end        
                2'b11: begin
                    if (taken == `BR_TRUE) newstate <= 2'b11;
                    else if (taken == `BR_FALSE) newstate <= 2'b10;
                end  
            endcase
        end else newstate <= curstate;
    end
endmodule


module StallUnit (
    //input
    RegA_ID, RegB_ID,
    opcode_EX,
    Rdst_EX,
    //outputs
    IF_ID_stall
);
    input [4:0] RegA_ID, RegB_ID;
    input [6:0] opcode_EX;

    input [4:0] Rdst_EX;

    output reg IF_ID_stall;
    always@(*) begin
        if(opcode_EX == `OPCODE_LOAD)begin
            if(RegA_ID == Rdst_EX || RegB_ID == Rdst_EX) begin
                IF_ID_stall = 1'b1;
            end
        end
        else begin
                IF_ID_stall = 1'b0;
        end
    end
endmodule


module ForwardingUnit(
    //input
    RegA_ID, RegB_ID,
    RegA_ID_cur, RegB_ID_cur,
    opcode_ID, opcode_EX, opcode_MEM,
    Rdst_EX_Data, Rdst_MEM_Data,
    Rdst_EX_Name, Rdst_MEM_Name,
    RegWREN_EX, RegWREN_MEM,
    //outputs
    RegA_ID_Data, RegB_ID_Data
);
    input [4:0] RegA_ID, RegB_ID;
    input [6:0] opcode_ID, opcode_EX, opcode_MEM;

    input [4:0] Rdst_EX_Name, Rdst_MEM_Name;
    input [31:0] Rdst_EX_Data, Rdst_MEM_Data;
    input [31:0] RegA_ID_cur, RegB_ID_cur;
    input RegWREN_EX, RegWREN_MEM;
    output reg [31:0] RegA_ID_Data, RegB_ID_Data;
    
    always@(*)begin
        if(opcode_EX != `OPCODE_LOAD && opcode_EX != `OPCODE_STORE && opcode_EX != `OPCODE_BRANCH && opcode_EX != `OPCODE_JAL && opcode_EX != `OPCODE_JALR && RegWREN_EX == 1'b0) begin
            RegA_ID_Data = (Rdst_EX_Name == RegA_ID && RegA_ID != 5'b0) ? Rdst_EX_Data : RegA_ID_cur;
            RegB_ID_Data = (Rdst_EX_Name == RegB_ID && RegB_ID != 5'b0) ? Rdst_EX_Data : RegB_ID_cur;
            if((opcode_MEM == `OPCODE_LOAD ||  opcode_MEM == `OPCODE_COMPUTE ||  opcode_MEM == `OPCODE_ICOMPUTE ||  opcode_MEM == `OPCODE_JAL ||  opcode_MEM == `OPCODE_JALR) && RegWREN_MEM == 1'b0) begin
                RegA_ID_Data = (Rdst_MEM_Name == RegA_ID && Rdst_EX_Name != RegA_ID && RegA_ID != 5'b0) ? Rdst_MEM_Data : RegA_ID_Data;
                RegB_ID_Data = (Rdst_MEM_Name == RegB_ID && Rdst_EX_Name != RegB_ID && RegB_ID != 5'b0) ? Rdst_MEM_Data : RegB_ID_Data;
            end
        end
        else begin
            if((opcode_MEM == `OPCODE_LOAD ||  opcode_MEM == `OPCODE_COMPUTE ||  opcode_MEM == `OPCODE_ICOMPUTE ||  opcode_MEM == `OPCODE_JAL ||  opcode_MEM == `OPCODE_JALR) && RegWREN_MEM == 1'b0) begin
                RegA_ID_Data = (Rdst_MEM_Name == RegA_ID && Rdst_EX_Name != RegA_ID && RegA_ID != 5'b0) ? Rdst_MEM_Data : RegA_ID_cur;
                RegB_ID_Data = (Rdst_MEM_Name == RegB_ID && Rdst_EX_Name != RegB_ID && RegB_ID != 5'b0) ? Rdst_MEM_Data : RegB_ID_cur;
            end
            else begin
                RegA_ID_Data = RegA_ID_cur;
                RegB_ID_Data = RegB_ID_cur;
            end
        end
    end
endmodule

//
// ControlUnit
// in - opcode, funct3
// out - PCSel, ImmSel, RWrEn, ALUsrcA, ALUsrcB, MemWrEn, WBSel, halt
module ControlUnit(opcode, funct3, ImmSel, WBSel, 
                    RWrEn, ALUsrcA, ALUsrcB, MemWrEn, LoadType, MemSize, halt);
    input [6:0] opcode;
    input [2:0] funct3;
    output reg [2:0] ImmSel, LoadType;
    output reg [1:0] WBSel, MemSize;
    output reg RWrEn, ALUsrcA, ALUsrcB, MemWrEn;
    output reg halt;

    always @(*) begin
        halt = 1'b0;
        case(opcode)
            `OPCODE_COMPUTE: // R-type instructions
                begin
                    ImmSel = `R_IMM; // no immediate, 0s
                    RWrEn = 1'b0; // register write enabled
                    ALUsrcA = `ALU_A_REG; // ALU source A is register
                    ALUsrcB = `ALU_B_REG; // ALU source B is register
                    MemWrEn = 1'b1; // mem write disabled
                    WBSel = `WB_ALU; // write back ALU to register
                end
            `OPCODE_ICOMPUTE: // I-type instructions
                begin
                    if (funct3 == 3'b001 || funct3 == 3'b101)
                        ImmSel = `SI_IMM; // I-type immediate for shifts
                    else
                        ImmSel = `I_IMM; // I-type immediate
                    RWrEn = 1'b0; // register write enabled
                    ALUsrcA = `ALU_A_REG; // ALU source A is register
                    ALUsrcB = `ALU_B_IMM; // ALU source B is immediate
                    MemWrEn = 1'b1; // mem write disabled
                    WBSel = `WB_ALU; // write back ALU to register
                end
            `OPCODE_LOAD: // Load instructions
                begin
                    ImmSel = `I_IMM; // I-type immediate
                    RWrEn = 1'b0; // register write enabled
                    ALUsrcA = `ALU_A_REG; // ALU source A is register
                    ALUsrcB = `ALU_B_IMM; // ALU source B is immediate
                    MemWrEn = 1'b1; // mem write disabled
                    WBSel = `WB_MEM; // write back memory to register
                    LoadType = funct3;
                    case(funct3)
                        `LOAD_BYTE: MemSize = `SIZE_BYTE; // lb
                        `LOAD_HALF: MemSize = `SIZE_HWORD; // lh
                        `LOAD_WORD: MemSize = `SIZE_WORD; // lw
                        `LOAD_BYTE_U: MemSize = `SIZE_BYTE; // lbu
                        `LOAD_HALF_U: MemSize = `SIZE_HWORD; // lhu
                        default: halt = 1'b1; // invalid func3
                    endcase
                end
            `OPCODE_STORE: // Store instructions
                begin
                    ImmSel = `S_IMM; // S-type immediate
                    RWrEn = 1'b1; // register write disabled
                    ALUsrcA = `ALU_A_REG; // ALU source A is register
                    ALUsrcB = `ALU_B_IMM; // ALU source B is immediate
                    MemWrEn = 1'b0; // mem write enabled
                    WBSel = `WB_UNDEF; // undefined writeback
                    case(funct3)
                        3'b000: MemSize = `SIZE_BYTE; // sb
                        3'b001: MemSize = `SIZE_HWORD; // sh
                        3'b010: MemSize = `SIZE_WORD; // sw
                        default: halt = 1'b1; // invalid func3
                    endcase
                end
            `OPCODE_BRANCH: // Branch instructions
                begin
                    ImmSel = `B_IMM; // B-type immediate
                    RWrEn = 1'b1; // register write disabled
                    ALUsrcA = `ALU_A_PC; // ALU source A is PC
                    ALUsrcB = `ALU_B_IMM; // ALU source B is immediate
                    MemWrEn = 1'b1; // mem write disabled
                    WBSel = `WB_UNDEF; // undefined writeback
                end
            `OPCODE_JAL: // JAL
                begin
                    ImmSel = `J_IMM; // J-type immediate 
                    RWrEn = 1'b0; // register write enabled
                    ALUsrcA = `ALU_A_PC; // ALU source A is PC
                    ALUsrcB = `ALU_B_IMM; // ALU source B is immediate
                    MemWrEn = 1'b1; // mem write disabled
                    WBSel = `WB_PC4; // write PC + 4 to register
                end
            `OPCODE_JALR: // JALR
                begin
                    ImmSel = `I_IMM; // I-type immediate 
                    RWrEn = 1'b0; // register write enabled
                    ALUsrcA = `ALU_A_REG; // ALU source A is register
                    ALUsrcB = `ALU_B_IMM; // ALU source B is immediate
                    MemWrEn = 1'b1; // mem write disabled
                    WBSel = `WB_PC4; // write PC + 4 to register
                end
            `OPCODE_LUI: // LUI
                begin
                    ImmSel = `U_IMM; // U-type immediate 
                    RWrEn = 1'b0; // register write enabled
                    ALUsrcA = `ALU_A_REG; // ALU source A doesn't matter
                    ALUsrcB = `ALU_B_REG; // ALU source B doesn't matter
                    MemWrEn = 1'b1; // mem write disabled
                    WBSel = `WB_IMM; // write immediate back
                end
            `OPCODE_AUIPC: // AUIPC
                begin
                    ImmSel = `U_IMM; // U-type immediate 
                    RWrEn = 1'b0; // register write enabled
                    ALUsrcA = `ALU_A_PC; // ALU source A is PC
                    ALUsrcB = `ALU_B_IMM; // ALU source B is immediate
                    MemWrEn = 1'b1; // mem write disabled
                    WBSel = `WB_ALU; // write ALU to register
                end
            default:
                begin
                RWrEn = 1'b1;
                halt = 1'b1; // if not a valid opcode, halt signal
                end
        endcase
    end
endmodule

// ImmediateGenerator
// in - InstWord, ImmSel
// out - immediate
module ImmediateGenerator(InstWord, ImmSel, immediate);
    input [31:0] InstWord;
    input [2:0] ImmSel;
    output reg [31:0] immediate;

    always @(*) begin
    case(ImmSel)
      `R_IMM: // R-type imm (nothing)
        immediate = 32'b0;
      `I_IMM: // I-type imm (upper 12 bits of instruction, sign extended)
        immediate = { {20{InstWord[31]}}, InstWord[31:20] };
      `S_IMM: // S-type imm (upper 7 bits of instruction + 4 more bits (11:7), sign extended)
        immediate = { {20{InstWord[31]}}, InstWord[31:25], InstWord[11:7] };
      `B_IMM: // B-type imm (bit 31, bit 7, 6 bits (30:25), 4 bits (11:8), 1'b0 for x2, sign extended)
        immediate = { {19{InstWord[31]}}, InstWord[31], InstWord[7], InstWord[30:25], InstWord[11:8], 1'b0 };
      `U_IMM: // U-type imm (upper 20 bits of instruction, shifted left 12 bits)
        immediate = { {12{InstWord[31]}}, InstWord[31:12], 12'b0 };
      `J_IMM: // J-type imm (bit 31, bits 19:12, bit 20, bits 30:21, 1'b0 for x2, sign extended)
        immediate = { {11{InstWord[31]}}, InstWord[31], InstWord[19:12], InstWord[20], InstWord[30:21], 1'b0 };
      `SI_IMM: // I-type imm for shifts (lower 5 bits of instruction)
        immediate = { {27{1'b0}}, InstWord[24:20] };
    endcase
  end
endmodule // END ImmediateGenerator

module MemSext(MemReadIn, LoadType, MemReadOut);
    input [31:0] MemReadIn;
    input [2:0] LoadType;
    output reg [31:0] MemReadOut;

    always@(*) begin
        case(LoadType)
            `LOAD_BYTE: MemReadOut = { {24{MemReadIn[7]}}, MemReadIn[7:0] }; // lb
            `LOAD_HALF: MemReadOut = { {16{MemReadIn[15]}}, MemReadIn[15:0] }; // lh
            `LOAD_BYTE_U: MemReadOut = { {24{1'b0}}, MemReadIn[7:0] }; // lbu
            `LOAD_HALF_U: MemReadOut = { {16{1'b0}}, MemReadIn[15:0] }; // lhu
            default: MemReadOut = MemReadIn; // default word size
        endcase
    end
endmodule

module SizeCheck(MemAddress, opcode, MemSize, halt);
    input [31:0] MemAddress;
    input [6:0] opcode;
    input [1:0] MemSize;
    output reg halt;

    always @(*) begin
        // only consider misalignment halts for ld/str
        if ((opcode == `OPCODE_LOAD) || opcode == `OPCODE_STORE) begin
            case(MemSize)
                // check if lowest bits are 0, if yes aligned, if no misaligned
                `SIZE_HWORD: halt = ((MemAddress[0] == 1'b0) ? 1'b0 : 1'b1);
                `SIZE_WORD: halt = ((MemAddress[1:0] == 2'b00) ? 1'b0 : 1'b1);
                default: halt = 1'b0; // bytes can't be misaligned
            endcase
        end else
            halt = 1'b0;
    end
endmodule

module PCAlign(PC, doBranch, halt);
    input [31:0] PC;
    input doBranch;
    output reg halt;

    always @(*) begin
        if (doBranch == 1'b1) 
            // check if lowest bits are 0, if yes aligned, if no misaligned
            halt = ((PC[1:0] == 2'b00) ? 1'b0 : 1'b1);
        else
            halt = 1'b0;
    end
endmodule

// library files for reg / mem from lab 2
// Library Modules for Northwestern - CompEng 361 - Lab2

module InstMem(Addr, Size, DataOut, CLK);
   input [31:0] Addr;
   input [1:0] 	Size;
   output [31:0] DataOut;
   reg [31:0] DataOut;   
   input 	CLK;
   reg [7:0] 	Mem[0:1024];
   wire [31:0] 	AddrW;

   // Addresses are word aligned
   assign AddrW = Addr & 32'hfffffffc;

   // Little endian 
   always @ *
     DataOut = {Mem[AddrW+3], Mem[AddrW+2], 
		Mem[AddrW+1], Mem[AddrW]};
      
endmodule // InstMem

module DataMem(Addr, Size, DataIn, DataOut, WEN, CLK);
   input [31:0] Addr;
   input [1:0] 	Size;   
   input [31:0] DataIn;   
   output [31:0] DataOut;
   reg [31:0] DataOut;   
   input      WEN, CLK;
   reg [7:0] 	Mem[0:1024];

   wire [31:0] 	AddrH, AddrW;

   assign AddrH = Addr & 32'hfffffffe;
   assign AddrW = Addr & 32'hfffffffc;

   always @ * 
     DataOut = (Size == 2'b00) ? {4{Mem[Addr]}} :
	       ((Size == 2'b01) ? {2{Mem[AddrH+1],Mem[AddrH]}} :
		{Mem[AddrW+3], Mem[AddrW+2], Mem[AddrW+1], Mem[AddrW]});
   
   always @ (negedge CLK)
     if (!WEN) begin
	case (Size)
	  2'b00: begin // Write byte
	     Mem[Addr] <= DataIn[7:0];
	  end
	  2'b01: begin  // Write halfword
	     Mem[AddrH] <= DataIn[7:0];
	     Mem[AddrH+1] <= DataIn[15:8];
	  end
	  2'b10, 2'b11: begin // Write word
	     Mem[AddrW] <= DataIn[7:0];
	     Mem[AddrW+1] <= DataIn[15:8];
	     Mem[AddrW+2] <= DataIn[23:16];
	     Mem[AddrW+3] <= DataIn[31:24];
	  end
	endcase // case (Size)
     end // if (!WEN)
      
endmodule // InstMem

module RegFile(AddrA, DataOutA,
	       AddrB, DataOutB,
	       AddrW, DataInW, WenW, CLK);
   input [4:0] AddrA, AddrB, AddrW;
   output [31:0] DataOutA, DataOutB;
   reg [31:0] DataOutA, DataOutB;   
   input [31:0]  DataInW;
   input 	 WenW, CLK;
   reg [31:0] 	 Mem[0:31];
   
   always @ * begin
      // Remember that x0 == 0
      DataOutA = (AddrA == 0) ? 32'h00000000 : Mem[AddrA];
      DataOutB = (AddrB == 0) ? 32'h00000000 : Mem[AddrB]; 
   end

   always @ (negedge CLK) begin
     if (!WenW) begin
       Mem[AddrW] <= DataInW;
     end
      Mem[0] <= 0; // Enforce the invariant that x0 = 0
   end
   
endmodule // RegFile

module Reg(Din, Qout, WEN, CLK, RST);
   parameter width = 32;
   input [width-1:0] Din;
   output [width-1:0] Qout;
   input 	      WEN, CLK, RST;

   reg [width-1:0]    Qout;
   
   always @ (negedge CLK or negedge RST)
     if (!RST)
       Qout <= 0;
     else
       if (!WEN)
	 Qout <= Din;
  
endmodule // Reg

// custom pipeline registers
// designed specifically to carry needed info between pipeline stages
module IF_ID_data_reg(WEN, CLK, RST, NEW, stall,
                        InstWord_F, InstWord_D, PC_F, PC_D,
                        PC_Plus4_F, PC_Plus4_D, nop);
    input WEN, CLK, RST;
    output reg NEW;
    input [31:0] InstWord_F;
    output reg [31:0] InstWord_D;
    input [31:0] PC_F;
    output reg [31:0] PC_D;
    input [31:0] PC_Plus4_F;
    output reg [31:0] PC_Plus4_D;

    input stall;
    input nop;

    always @ (negedge CLK or negedge RST)
        if (!RST) begin
            // set all out vals to 0
            InstWord_D <= 32'b0;
            PC_D <= 32'b0;
            PC_Plus4_D <= 32'b0;
            // if just reset, this program is NEW
            NEW <= 1'b1;
        end else if (stall) begin
            // maintain the same signal but with "stalled" output
            InstWord_D <= InstWord_D;
            PC_D <= PC_D;
            PC_Plus4_D <= PC_Plus4_D;
            NEW <= 1'b0;
        end else if (nop) begin
            // set value to nop
            InstWord_D <= 32'h13;
            PC_D <= PC_F;
            PC_Plus4_D <= PC_Plus4_F;
            // not reset, not new
            NEW <= 1'b0;
        end
        else if (!WEN) begin
            // set all out vals to in vals
            InstWord_D <= InstWord_F;
            PC_D <= PC_F;
            PC_Plus4_D <= PC_Plus4_F;
            // if not just reset, this program is not NEW
            NEW <= 1'b0;
        end
endmodule

module ID_EX_data_reg(WEN, CLK, RST, InstWord_D, InstWord_E, PC_D, PC_E, PC_Plus4_D, PC_Plus4_E, 
                        RegAData_D, RegAData_E, RegBData_D, RegBData_E, 
                        targetAddr_D, targetAddr_E,
                        Immediate_D, Immediate_E,
                        
                        Rdst_D, Rdst_E,
                        stall, nop
                        );
    input WEN, CLK, RST;
    input [31:0] InstWord_D, PC_D, PC_Plus4_D;
    output reg [31:0] InstWord_E, PC_E, PC_Plus4_E;
    input [31:0] Immediate_D;
    output reg [31:0] Immediate_E;
    input [31:0] RegAData_D, RegBData_D;
    output reg [31:0] RegAData_E, RegBData_E;
    input [4:0] Rdst_D;
    output reg [4:0] Rdst_E;
    input [31:0] targetAddr_D;
    output reg [31:0] targetAddr_E;

    input nop, stall;

    always @ (negedge CLK or negedge RST)
        if (!RST) begin
            // set all vals to 0
            InstWord_E <= 32'b0;
            PC_E <= 32'b0;
            PC_Plus4_E <= 32'b0;
            RegAData_E <= 32'b0;
            RegBData_E <= 32'b0;
            Rdst_E <= 5'b0;
            targetAddr_E <= 32'b0;
            Immediate_E <= 32'b0;

        end else if (stall) begin
            // maintain the same signal but with "stalled" output
            InstWord_E <= InstWord_E;
            PC_E <= PC_E;
            PC_Plus4_E <= PC_Plus4_E;
            RegAData_E <= RegAData_E;
            RegBData_E <= RegBData_E;
            Rdst_E <= Rdst_E;
            targetAddr_E <= targetAddr_D;
            Immediate_E <= Immediate_E;

        end else if (nop) begin
            // set value to nop
            InstWord_E <= 32'h13;
            PC_E <= PC_D;
            PC_Plus4_E <= PC_Plus4_D;
            RegAData_E <= 32'b0;
            RegBData_E <= 32'b0;
            Rdst_E <= 5'b0; 
            targetAddr_E <= 32'b0;
            Immediate_E <= 32'b0;

        end
        else if (!WEN) begin
            // write all in values to outs
            InstWord_E <= InstWord_D;
            PC_E <= PC_D;
            PC_Plus4_E <= PC_Plus4_D;
            Rdst_E <= Rdst_D;
            RegAData_E <= RegAData_D;
            RegBData_E <= RegBData_D;
            Rdst_E <= Rdst_D;
            targetAddr_E <= targetAddr_D;
            Immediate_E <= Immediate_D;
            
        end
endmodule

module ID_EX_ctrl_reg(WEN, CLK, RST, ALUsrcA_D, ALUsrcB_D, WBSel_D, ImmSel_D,
                        MemWrEn_D, RegWrEn_D, LoadType_D, MemSize_D,
                        ALUsrcA_E, ALUsrcB_E, WBSel_E, ImmSel_E,
                        MemWrEn_E, RegWrEn_E, LoadType_E, MemSize_E,
                        halt_D, halt_E, didBranch_D, didBranch_E,
                        NEW_IN, NEW_OUT,
                        nop, stall);
    input WEN, CLK, RST;
    input ALUsrcA_D, ALUsrcB_D;
    output reg ALUsrcA_E, ALUsrcB_E;
    input MemWrEn_D, RegWrEn_D;
    output reg MemWrEn_E, RegWrEn_E;
    input [1:0] WBSel_D, MemSize_D;
    output reg [1:0] WBSel_E, MemSize_E;
    input [2:0] ImmSel_D, LoadType_D;
    output reg [2:0] ImmSel_E, LoadType_E;
    input halt_D;
    output reg halt_E;
    input NEW_IN;
    output reg NEW_OUT;
    input didBranch_D;
    output reg didBranch_E;

    input nop, stall;

    always @ (negedge CLK or negedge RST)
        if(!RST) begin
            // set all vals to 0
            ALUsrcA_E <= 1'b0;
            ALUsrcB_E <= 1'b0;
            WBSel_E <= 2'b0;
            ImmSel_E <= 3'b0;
            MemWrEn_E <= 1'b1;
            RegWrEn_E <= 1'b1;
            LoadType_E <= 3'b0;
            MemSize_E <= 2'b0;
            halt_E <= 1'b0;
            didBranch_E <= 1'b0;
            // if just reset, this program is NEW
            NEW_OUT <= 1'b1;
        end
        else if (stall) begin
            // maintain the same signal but with "stalled" output
            ALUsrcA_E <= ALUsrcA_E;
            ALUsrcB_E <= ALUsrcB_E;
            WBSel_E <= WBSel_E;
            ImmSel_E <= ImmSel_E;
            MemWrEn_E <= MemWrEn_E;
            RegWrEn_E <= RegWrEn_E;
            LoadType_E <= LoadType_E;
            MemSize_E <= MemSize_E;
            halt_E <= halt_E;
            didBranch_E <= didBranch_E;
            NEW_OUT <= NEW_IN;
        end else if (nop) begin
            // set value to nop
            ALUsrcA_E <= `ALU_A_REG; // 0 cause add
            ALUsrcB_E <= `ALU_B_IMM; // imm cause addi
            WBSel_E <= `WB_ALU;
            ImmSel_E <= `I_IMM;
            MemWrEn_E <= 1'b1;
            RegWrEn_E <= 1'b1;
            LoadType_E <= LoadType_D;
            MemSize_E <= MemSize_D;
            didBranch_E <= 1'b0;
            halt_E <= 1'b0;
            // if not just reset, this program is not NEW
            NEW_OUT <= NEW_IN;
        end else if (!WEN) begin
            // write all in values to outs
            ALUsrcA_E <= ALUsrcA_D;
            ALUsrcB_E <= ALUsrcB_D;
            WBSel_E <= WBSel_D;
            ImmSel_E <= ImmSel_D;
            MemWrEn_E <= MemWrEn_D;
            RegWrEn_E <= RegWrEn_D;
            LoadType_E <= LoadType_D;
            MemSize_E <= MemSize_D;
            halt_E <= halt_D;
            didBranch_E <= didBranch_D;
            // if not just reset, this program is not NEW
            NEW_OUT <= NEW_IN;
        end
endmodule

module EX_MEM_data_reg(WEN, CLK, RST, NEW, ALUresult_E, RegBData_E, Immediate_E, PC_Plus4_E, Rdst_E, InstWord_E,
                        ALUresult_M, RegBData_M, Immediate_M, PC_Plus4_M, Rdst_M, InstWord_M);
    input WEN, CLK, RST;
    output reg NEW;
    input [31:0] ALUresult_E, RegBData_E, PC_Plus4_E, Immediate_E;
    output reg [31:0] ALUresult_M, RegBData_M, PC_Plus4_M, Immediate_M;
    input [4:0] Rdst_E;
    output reg [4:0] Rdst_M;
    input [31:0] InstWord_E;
    output reg [31:0] InstWord_M;

    always @ (negedge CLK or negedge RST)
        if (!RST) begin
            // set all vals to 0
            ALUresult_M <= 32'b0;
            RegBData_M <= 32'b0;
            PC_Plus4_M <= 32'b0;
            Rdst_M <= 5'b0;
            InstWord_M <= 32'b0;
            Immediate_M <= 32'b0;
            // if just reset, this program is NEW
            NEW <= 1'b1;
        end 
        // else if (nop) begin
        //     // set value to nop
        //     ALUresult_M <= 32'b0;
        //     RegBData_M <= 32'b0;
        //     PC_Plus4_M <= 32'b0;
        //     Rdst_M <= 5'b0;
        //     InstWord_M <= 32'h13;
        //     Immediate_M <= 32'b0;
        //     // if not just reset, this program is not NEW
        //     NEW <= 1'b0;
        // end 
        else if (!WEN) begin
            // write all in values to outs
            ALUresult_M <= ALUresult_E;
            RegBData_M <= RegBData_E;
            PC_Plus4_M <= PC_Plus4_E;
            Rdst_M <= Rdst_E;
            InstWord_M <= InstWord_E;
            Immediate_M <= Immediate_E;
            // if not just reset, this program is not NEW
            NEW <= 1'b0;
        end 
endmodule

module EX_MEM_ctrl_reg(WEN, CLK, RST, MemWrEn_E, RegWrEn_E, WBSel_E, LoadType_E, MemSize_E, 
                        MemWrEn_M, RegWrEn_M, WBSel_M, LoadType_M, MemSize_M,
                        halt_E, halt_M,
                        NEW_IN, NEW_OUT);
    input WEN, CLK, RST;
    input MemWrEn_E, RegWrEn_E;
    output reg MemWrEn_M, RegWrEn_M;
    input [1:0] WBSel_E, MemSize_E;
    output reg [1:0] WBSel_M, MemSize_M;
    input [2:0] LoadType_E;
    output reg [2:0] LoadType_M;
    input halt_E;
    output reg halt_M;
    input NEW_IN;
    output reg NEW_OUT;

    always @ (negedge CLK or negedge RST)
        if (!RST) begin
            // set all vals to 0
            MemWrEn_M <= 1'b1;
            RegWrEn_M <= 1'b1;
            WBSel_M <= 2'b0;
            LoadType_M <= 3'b0;
            MemSize_M <= 2'b0;
            halt_M <= 1'b0;
            // if just reset, this program is NEW
            NEW_OUT <= 1'b1;
        end 
        // else if (nop) begin
        //     // set value to nop
        //     MemWrEn_M <= 1'b1;
        //     RegWrEn_M <= 1'b1;
        //     WBSel_M <= 2'b0;
        //     LoadType_M <= 3'b0;
        //     MemSize_M <= 2'b0;
        //     halt_M <= 1'b0;
        //     // if not just reset, this program is not NEW
        //     NEW_OUT <= NEW_IN;
        // end
        else if (!WEN) begin
            // write all in values to outs
            MemWrEn_M <= MemWrEn_E;
            RegWrEn_M <= RegWrEn_E;
            WBSel_M <= WBSel_E;
            LoadType_M <= LoadType_E;
            MemSize_M <= MemSize_E;
            halt_M <= halt_E;
            // if not just reset, this program is not NEW
            NEW_OUT <= NEW_IN;
        end
endmodule

module MEM_WB_data_reg(WEN, CLK, RST, Rdst_Data_M, Rdst_Data_W, Rdst_M, Rdst_W);
    input WEN, CLK, RST;
    input [31:0] Rdst_Data_M;
    output reg [31:0] Rdst_Data_W;
    input [4:0] Rdst_M;
    output reg [4:0] Rdst_W;

    always @ (negedge CLK or negedge RST)
        if (!RST) begin
            // set all vals to 0
            Rdst_Data_W <= 32'b0;
            Rdst_W <= 5'b0;
        end else if (!WEN) begin
            // write all in values to outs
            Rdst_Data_W <= Rdst_Data_M;
            Rdst_W <= Rdst_M;
        end
endmodule

module MEM_WB_ctrl_reg(WEN, CLK, RST, RegWrEn_M,
                        RegWrEn_W,
                        halt_M, halt_W,
                        NEW_IN, NEW_OUT);
    input WEN, CLK, RST;
    input RegWrEn_M;
    output reg RegWrEn_W;
    input halt_M;
    output reg halt_W;
    input NEW_IN;
    output reg NEW_OUT;

    always @ (negedge CLK or negedge RST)
        if (!RST) begin
            // set all vals to 0
            RegWrEn_W <= 1'b1;
            halt_W <= 1'b0;
            // if just reset, this program is NEW
            NEW_OUT <= 1'b1;
        end else if (!WEN) begin
            // write all in values to outs
            RegWrEn_W <= RegWrEn_M;
            halt_W <= halt_M;
            // if not just reset, this program is not NEW
            NEW_OUT <= NEW_IN;
        end
endmodule
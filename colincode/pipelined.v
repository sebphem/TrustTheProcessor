// Northwestern - CompEng 361 - Lab3
// Groupname: Finite Russ Machine
// NetIDs: cjs7245, dzs0826

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
    InstructionDecode ID(.CLK(CLK), .InstWord_in(InstWord_in_D), .PC_in(PC_in_D), .PC_Plus4_in(PC_Plus4_in_D),
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
module InstructionDecode(InstWord_in, CLK, PC_in, PC_Plus4_in,
                        RegWrData_WB, RegWrEn_WB, Rdst_WB,
                        RegAData, RegBData, Rdst, InstWord_out, PC_out, PC_Plus4_out,
                        ALUsrcA, ALUsrcB, WBSel, ImmSel, MemWrEn, RegWrEn, LoadType, MemSize,
                        RegA, RegB, immediate, doBranch,
                        halt_ID_out, targetAddr, taken, PCSel,
                        isNew);
    input [31:0] InstWord_in, PC_in, PC_Plus4_in;
    input CLK;
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


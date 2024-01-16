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

// immediate types
`define R_IMM 3'b000
`define I_IMM 3'b001
`define S_IMM 3'b010
`define B_IMM 3'b011
`define U_IMM 3'b100
`define J_IMM 3'b101
`define SI_IMM 3'b110

// branch Y/N
`define BR_FALSE 1'b0
`define BR_TRUE 1'b1

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

    // ID --> EX Data
    wire [31:0] InstWord_out_D,     InstWord_in_E;
    wire [31:0] PC_out_D,           PC_in_E;
    wire [31:0] PC_Plus4_out_D,     PC_Plus4_in_E;
    wire [31:0] RegAData_out_D,     RegAData_in_E;
    wire [31:0] RegBData_out_D,     RegBData_in_E;
    wire [4:0] Rdst_out_D,          Rdst_in_E;

    // ID --> EX Ctrl
    wire ALUsrcA_out_D,             ALUsrcA_in_E;
    wire ALUsrcB_out_D,             ALUsrcB_in_E;
    wire [1:0] WBSel_out_D,         WBSel_in_E;
    wire [2:0] ImmSel_out_D,        ImmSel_in_E;
    wire MemWrEn_out_D,             MemWrEn_in_E;
    wire RegWrEn_out_D,             RegWrEn_in_E;
    wire [2:0] LoadType_out_D,      LoadType_in_E;
    wire [1:0] MemSize_out_D,       MemSize_in_E;

    // ID --> hazard wires
    wire [4:0] RegA_out_D, RegB_out_D;
    
    // EX --> IF for branch/jump
    wire PCsel_EX_IF;
    wire [31:0] targetAddr_EX_IF;
    
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
    wire [31:0] ALUresult_out_M,    ALUresult_in_W;
    wire [31:0] MemReadData_out_M,  MemReadData_in_W;
    wire [31:0] Immediate_out_M,    Immediate_in_W;
    wire [31:0] PC_Plus4_out_M,     PC_Plus4_in_W;
    wire [4:0] Rdst_out_M,          Rdst_in_W; // goes straight back to regfile

    // MEM --> WB ctrl
    wire RegWrEn_out_M,             RegWrEn_in_W; // goes straight back to regfile
    wire [1:0] WBSel_out_M,         WBSel_in_W;

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
    assign halt_in_progress = halt_ID_out || halt_EX_out || halt_MEM_out || halt_WB_out;

    // initialization stage wires
    wire IF_ID_NEW;
    wire ID_EX_NEW;
    wire EX_MEM_NEW;
    wire MEM_WB_NEW;

    // stall wires
    wire hazard_EX, hazard_MEM;
    wire hazard_MEM_only;
    assign hazard_MEM_only = (hazard_MEM && !hazard_EX);
    // if ID / EX hazard detected, freeze PC & IF/ID reg, make ID/EX nop
    // if ID / MEM hazard detected & no EX hazard, freeze PC, IF/ID, 
    // and ID/EX, and make EX/MEM nop

    // pipeline stages
    // stage 1 - IF
    //assign halt_IF_in = (halt_ID_out && !(IF_ID_NEW)) || 
    //                    (halt_EX_out && !(ID_EX_NEW)) || 
    //                    (halt_MEM_out && !(EX_MEM_NEW)) || 
    //                    (halt_WB_out && !(MEM_WB_NEW));

    InstructionFetch IF(.CLK(CLK), .RST(rst), .stall((hazard_EX || hazard_MEM)),
                        .PCSel(PCsel_EX_IF), .targetAddr(targetAddr_EX_IF),
                        .InstWord(InstWord_out_F), .PC(PC_out_F), .PC_Plus4(PC_Plus4_out_F));
    
    // transfer to ID
    IF_ID_data_reg IF_ID(.WEN(1'b0), .CLK(CLK), .RST(rst),
                            .InstWord_F(InstWord_out_F),    .InstWord_D(InstWord_in_D), 
                            .PC_F(PC_out_F),                .PC_D(PC_in_D), 
                            .PC_Plus4_F(PC_Plus4_out_F),    .PC_Plus4_D(PC_Plus4_in_D),
                            .NEW(IF_ID_NEW),               
                            .nop(PCsel_EX_IF), .stall((hazard_EX || hazard_MEM)));

    // stage 2 - ID
    InstructionDecode ID(.CLK(CLK), .InstWord_in(InstWord_in_D), .PC_in(PC_in_D), .PC_Plus4_in(PC_Plus4_in_D),
                        .InstWord_out(InstWord_out_D), .PC_out(PC_out_D), .PC_Plus4_out(PC_Plus4_out_D),
                        .RegWrData_WB(RegWrData_out_W), .RegWrEn_WB(RegWrEn_in_W), .Rdst_WB(Rdst_in_W),
                        .RegAData(RegAData_out_D), .RegBData(RegBData_out_D), .Rdst(Rdst_out_D),
                        .ALUsrcA(ALUsrcA_out_D), .ALUsrcB(ALUsrcB_out_D), .WBSel(WBSel_out_D), .ImmSel(ImmSel_out_D), 
                        .MemWrEn(MemWrEn_out_D), .RegWrEn(RegWrEn_out_D), .LoadType(LoadType_out_D), .MemSize(MemSize_out_D),    
                        .RegA(RegA_out_D), .RegB(RegB_out_D),
                        .halt_ID_out(halt_ID_out),
                        .isNew(IF_ID_NEW));

    // hazard detection
    HazardUnit HU(.RegA_ID(RegA_out_D), .RegB_ID(RegB_out_D), .opcode_ID(InstWord_in_D[6:0]), 
                .opcode_EX(InstWord_in_E[6:0]), .Rdst_EX(Rdst_in_E), 
                .opcode_MEM(InstWord_in_M[6:0]), .Rdst_MEM(Rdst_in_M),
                .EX_stall(hazard_EX), .MEM_stall(hazard_MEM));

    // transfer to EX
    ID_EX_data_reg ID_EX_data(.WEN(1'b0), .CLK(CLK), .RST(rst), 
                            .InstWord_D(InstWord_out_D),    .InstWord_E(InstWord_in_E), 
                            .PC_D(PC_out_D),                .PC_E(PC_in_E), 
                            .PC_Plus4_D(PC_Plus4_out_D),    .PC_Plus4_E(PC_Plus4_in_E), 
                            .RegAData_D(RegAData_out_D),    .RegAData_E(RegAData_in_E), 
                            .RegBData_D(RegBData_out_D),    .RegBData_E(RegBData_in_E), 
                            .Rdst_D(Rdst_out_D),            .Rdst_E(Rdst_in_E),
                            .nop((PCsel_EX_IF || hazard_EX)), 
                            .stall(hazard_MEM_only));

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
                            .NEW_IN(IF_ID_NEW),             .NEW_OUT(ID_EX_NEW),
                            .nop((PCsel_EX_IF || hazard_EX)), 
                            .stall(hazard_MEM_only));

    // stage 3 - EX
    Execute EX(     // inputs
                    .InstWord(InstWord_in_E), .PC(PC_in_E), .PC_Plus4(PC_Plus4_in_E), 
                    .RegAData(RegAData_in_E), .RegBData(RegBData_in_E), .Rdst(Rdst_in_E),
                    .ALUsrcA(ALUsrcA_in_E), .ALUsrcB(ALUsrcB_in_E), .WBSel(WBSel_in_E), 
                    .ImmSel(ImmSel_in_E), .MemWrEn(MemWrEn_in_E), .RegWrEn(RegWrEn_in_E),
                    .LoadType(LoadType_in_E), .MemSize(MemSize_in_E),
                    .isNew(ID_EX_NEW),
                    // outputs
                    .ALUout(ALUresult_out_E), .RegBData_out(RegBData_out_E), .PC_Plus4_out(PC_Plus4_out_E), 
                    .Rdst_out(Rdst_out_E), .immediate(Immediate_out_E),  .MemWrEn_out(MemWrEn_out_E), 
                    .RegWrEn_out(RegWrEn_out_E), .WBSel_out(WBSel_out_E), .LoadType_out(LoadType_out_E), 
                    .MemSize_out(MemSize_out_E), 
                    .PCsel(PCsel_EX_IF), .targetAddr(targetAddr_EX_IF), 
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
                            .InstWord_E(InstWord_out_E),    .InstWord_M(InstWord_in_M),
                            .nop(hazard_MEM_only));

    EX_MEM_ctrl_reg EX_MEM_ctrl(.WEN(1'b0), .CLK(CLK), .RST(rst),
                            .MemWrEn_E(MemWrEn_out_E),      .MemWrEn_M(MemWrEn_in_M),
                            .RegWrEn_E(RegWrEn_out_E),      .RegWrEn_M(RegWrEn_in_M),
                            .WBSel_E(WBSel_out_E),          .WBSel_M(WBSel_in_M),
                            .LoadType_E(LoadType_out_E),    .LoadType_M(LoadType_in_M),
                            .MemSize_E(MemSize_out_E),      .MemSize_M(MemSize_in_M),
                            .halt_E(halt_EX_out),           .halt_M(halt_MEM_in),
                            .NEW_IN(ID_EX_NEW),             .NEW_OUT(EX_MEM_NEW),
                            .nop(hazard_MEM_only));

    // stage 4 - MEM
    MemoryAccess MEM( // inputs
                    .CLK(CLK), .ALUresult_in(ALUresult_in_M), .RegBData(RegBData_in_M), 
                    .Immediate_in(Immediate_in_M), .PC_Plus4_in(PC_Plus4_in_M), .Rdst_in(Rdst_in_M),
                    .MemWrEn(MemWrEn_in_M), .RegWrEn_in(RegWrEn_in_M), .WBSel_in(WBSel_in_M), 
                    .MemSize(MemSize_in_M), .LoadType(LoadType_in_M),
                    .isNew(EX_MEM_NEW), .InstWord(InstWord_in_M),
                    // outputs
                    .ALUresult_out(ALUresult_out_M), .PC_Plus4_out(PC_Plus4_out_M), .Immediate_out(Immediate_out_M), 
                    .Rdst_out(Rdst_out_M), .MemReadData(MemReadData_out_M),
                    .RegWrEn_out(RegWrEn_out_M), .WBSel_out(WBSel_out_M),
                    // halt signal
                    .halt_MEM_in(halt_MEM_in), .halt_MEM_out(halt_MEM_out));

    // transfer to WB
    MEM_WB_data_reg MEM_WB_data(.WEN(1'b0), .CLK(CLK), .RST(rst), 
                            .ALUresult_M(ALUresult_out_M),          .ALUresult_W(ALUresult_in_W),
                            .MemReadData_M(MemReadData_out_M),      .MemReadData_W(MemReadData_in_W),
                            .Immediate_M(Immediate_out_M),          .Immediate_W(Immediate_in_W),
                            .PC_Plus4_M(PC_Plus4_out_M),            .PC_Plus4_W(PC_Plus4_in_W),
                            .Rdst_M(Rdst_out_M),                    .Rdst_W(Rdst_in_W));

    MEM_WB_ctrl_reg MEM_WB_ctrl(.WEN(1'b0), .CLK(CLK), .RST(rst),
                            .RegWrEn_M(RegWrEn_out_M),          .RegWrEn_W(RegWrEn_in_W),
                            .WBSel_M(WBSel_out_M),              .WBSel_W(WBSel_in_W),
                            .halt_M(halt_MEM_out),              .halt_W(halt_WB_in),          
                            .NEW_IN(EX_MEM_NEW),                .NEW_OUT(MEM_WB_NEW));

    // stage 5 - WB
    WriteBack WB(.ALUresult(ALUresult_in_W), .MemReadData(MemReadData_in_W), .Immediate(Immediate_in_W), 
                .PC_Plus4(PC_Plus4_in_W), .WBSel(WBSel_in_W), 
                .WBresult(RegWrData_out_W),
                .halt_WB_in(halt_WB_in), .halt_WB_out(halt_WB_out));

    // if halt reached the end of the pipeline, then halt!
    assign halt = halt_WB_out;
endmodule

// IF stage
// IN: current PC
// OUT: InstWord, PC+4
module InstructionFetch(CLK, RST, stall,
                        PCSel, targetAddr, 
                        InstWord, PC, PC_Plus4);
    input CLK, RST;
    input PCSel;
    input stall;
    input [31:0] targetAddr;
    output [31:0] PC, PC_Plus4;
    output [31:0] InstWord;
    
    // PC mux to choose next PC
    wire [31:0] Next_PC, Target_PC;
    assign PC_Plus4 = PC + 4;

    // if stall is 1, then PC stops updating, otherwise it updates
    // if PCSel is 1, then target address from ALU, otherwise PC+4
    assign Target_PC = (PCSel == `PC_ALUOUT) ? targetAddr : PC_Plus4;
    assign Next_PC = (stall == 1) ? PC : Target_PC;

    // PC reg
    Reg PC_REG(.Din(Next_PC), .Qout(PC), .WEN(1'b0), .CLK(CLK), .RST(RST));

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
                        RegA, RegB,
                        halt_ID_out,
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
                    .PCSel(PCSel), .ImmSel(ImmSel), .RWrEn(RegWrEn), .ALUsrcA(ALUsrcA), 
                    .ALUsrcB(ALUsrcB), .MemWrEn(MemWrEn), .WBSel(WBSel), 
                    .LoadType(LoadType), .MemSize(MemSize), .halt(controlHalt));

    
    
    // assign halt based on control output & PC alignment if jump/br
    assign halt_ID_out = (controlHalt && !(isNew));

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
                    LoadType, MemSize,
                    halt_EX_in,
                    isNew,
                    // outputs
                    ALUout, RegBData_out, PC_Plus4_out, Rdst_out, immediate,  
                    MemWrEn_out, RegWrEn_out, WBSel_out, LoadType_out, MemSize_out,
                    PCsel, targetAddr, InstWord_out, 
                    halt_EX_out);
    // declare inputs
    input [31:0] InstWord, PC, PC_Plus4, RegAData, RegBData;
    input ALUsrcA, ALUsrcB, MemWrEn, RegWrEn;
    input [1:0] WBSel, MemSize;
    input [2:0] ImmSel, LoadType;
    input [4:0] Rdst;
    input isNew;

    // declare outputs
    // data going to MEM
    output [31:0] ALUout, RegBData_out, PC_Plus4_out, immediate;
    output [4:0] Rdst_out;
    output [31:0] InstWord_out;
    // control signals going to MEM
    output MemWrEn_out, RegWrEn_out;
    output [1:0] WBSel_out, MemSize_out;
    output [2:0] LoadType_out;
    // control signals going to IF
    output [31:0] targetAddr;
    output PCsel;
    // halt signal
    input halt_EX_in;
    output halt_EX_out;

    // break up instruction parts that we need
    wire [6:0] opcode;
    wire [2:0] funct3;
    wire [6:0] funct7;
    assign opcode = InstWord[6:0];
    assign funct3 = InstWord[14:12];
    assign funct7 = InstWord[31:25];

    // immediate generation
    ImmediateGenerator IG(.InstWord(InstWord), .ImmSel(ImmSel), .immediate(immediate));

    // target address generation
    assign targetAddr = PC + immediate;

    // detect if we branch or not
    wire branchHalt, doBranch;
    BranchUnit BU(.opA(RegAData), .opB(RegBData), .funct3(funct3), .opcode(opcode), 
                .out(doBranch), .halt(branchHalt));

    // if we do branch, check if valid address
    wire alignHalt;
    PCAlign PCA(.PC(targetAddr), .doBranch(doBranch), .halt(alignHalt));

    // PCSel only comes from ALU if doBranch true
    assign PCsel = isNew ? `PC_PCPLUS4 : (doBranch ? `PC_ALUOUT : `PC_PCPLUS4); 

    // muxes determine ops for ALU
    wire [31:0] ALUopA, ALUopB;
    assign ALUopA = (ALUsrcA == `ALU_A_REG) ? RegAData : PC;
    assign ALUopB = (ALUsrcB == `ALU_B_REG) ? RegBData : immediate;

    // execute ALU
    wire ALUhalt;
    ArithmeticLogicUnit ALU(.opcode(opcode), .opA(ALUopA), .opB(ALUopB), .func(funct3), 
                            .auxFunc(funct7), .out(ALUout), .halt(ALUhalt));

    // assign halt based on branch or ALU or incoming halt signal
    assign halt_EX_out = (branchHalt | ALUhalt | alignHalt | halt_EX_in) && (!isNew);

    // pass thru any signals that don't get changed by this stage
    // data signals
    assign RegBData_out = RegBData;
    assign PC_Plus4_out = PC_Plus4;
    assign Rdst_out = Rdst;
    assign InstWord_out = InstWord;
    // ctrl signals
    assign MemWrEn_out = MemWrEn;
    assign RegWrEn_out = RegWrEn;
    assign WBSel_out = WBSel;
    assign LoadType_out = LoadType;
    assign MemSize_out = MemSize;
endmodule

module MemoryAccess(CLK, ALUresult_in, RegBData, Immediate_in, PC_Plus4_in, Rdst_in,
                    MemWrEn, RegWrEn_in, WBSel_in, MemSize, LoadType, InstWord,
                    ALUresult_out, PC_Plus4_out, Immediate_out, Rdst_out, MemReadData,
                    RegWrEn_out, WBSel_out,
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
    output [31:0] ALUresult_out, PC_Plus4_out, Immediate_out, MemReadData;
    output [4:0] Rdst_out;
    output RegWrEn_out;
    output [1:0] WBSel_out;

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
    MemSext MS(.MemReadIn(MemReadDataRaw), .LoadType(LoadType), .MemReadOut(MemReadData));

    // halt if MemReadData is not properly aligned 
    assign halt_MEM_out = (halt_MEM_in || alignHalt) && (!isNew);

    // pass thru any signals that don't get changed by this stage
    // data signals
    assign ALUresult_out = ALUresult_in;
    assign PC_Plus4_out = PC_Plus4_in;
    assign Immediate_out = Immediate_in;
    assign Rdst_out = Rdst_in;
    // ctrl signals
    assign RegWrEn_out = RegWrEn_in;
    assign WBSel_out = WBSel_in;
endmodule

module WriteBack(ALUresult, MemReadData, Immediate, 
                PC_Plus4, WBSel, WBresult,
                halt_WB_in, halt_WB_out);
    // inputs
    input [31:0] ALUresult, MemReadData, Immediate, PC_Plus4;
    input [1:0] WBSel;
    // output
    output reg [31:0] WBresult;
    // halt signal
    input halt_WB_in;
    output halt_WB_out;

    // halt can't get asserted here so just pass thru
    assign halt_WB_out = halt_WB_in;

    // mux for reg file write
    always @(*) begin
        case (WBSel)
            `WB_ALU: WBresult = ALUresult;
            `WB_MEM: WBresult = MemReadData;
            `WB_PC4: WBresult = PC_Plus4;
            `WB_IMM: WBresult = Immediate;
            default: WBresult = 32'b0;
        endcase
    end
endmodule

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
                        Rdst_D, Rdst_E, 
                        stall, nop);
    input WEN, CLK, RST;
    input [31:0] InstWord_D, PC_D, PC_Plus4_D;
    output reg [31:0] InstWord_E, PC_E, PC_Plus4_E;
    input [31:0] RegAData_D, RegBData_D;
    output reg [31:0] RegAData_E, RegBData_E;
    input [4:0] Rdst_D;
    output reg [4:0] Rdst_E;

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
        end else if (stall) begin
            // maintain the same signal but with "stalled" output
            InstWord_E <= InstWord_E;
            PC_E <= PC_E;
            PC_Plus4_E <= PC_Plus4_E;
            RegAData_E <= RegAData_E;
            RegBData_E <= RegBData_E;
            Rdst_E <= Rdst_E;
        end else if (nop) begin
            // set value to nop
            InstWord_E <= 32'h13;
            PC_E <= PC_D;
            PC_Plus4_E <= PC_Plus4_D;
            RegAData_E <= 32'b0;
            RegBData_E <= 32'b0;
            Rdst_E <= 5'b0; 
        end
        else if (!WEN) begin
            // write all in values to outs
            InstWord_E <= InstWord_D;
            PC_E <= PC_D;
            PC_Plus4_E <= PC_Plus4_D;
            RegAData_E <= RegAData_D;
            RegBData_E <= RegBData_D;
            Rdst_E <= Rdst_D;
        end
endmodule

module ID_EX_ctrl_reg(WEN, CLK, RST, ALUsrcA_D, ALUsrcB_D, WBSel_D, ImmSel_D, 
                        MemWrEn_D, RegWrEn_D, LoadType_D, MemSize_D, 
                        ALUsrcA_E, ALUsrcB_E, WBSel_E, ImmSel_E, 
                        MemWrEn_E, RegWrEn_E, LoadType_E, MemSize_E,
                        halt_D, halt_E,
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

    input nop, stall;

    always @ (negedge CLK or negedge RST)
        if(!RST) begin
            // set all vals to 0
            ALUsrcA_E <= 1'b0;
            ALUsrcB_E <= 1'b0;
            WBSel_E <= 2'b0;
            ImmSel_E <= 3'b0;
            MemWrEn_E <= 1'b0;
            RegWrEn_E <= 1'b0;
            LoadType_E <= 3'b0;
            MemSize_E <= 2'b0;
            halt_E <= 1'b0;
            // if just reset, this program is NEW
            NEW_OUT <= 1'b1;
        end else if (stall) begin
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
            // if not just reset, this program is not NEW
            NEW_OUT <= NEW_IN;
        end
endmodule

module EX_MEM_data_reg(WEN, CLK, RST, NEW, ALUresult_E, RegBData_E, Immediate_E, PC_Plus4_E, Rdst_E, InstWord_E,
                        ALUresult_M, RegBData_M, Immediate_M, PC_Plus4_M, Rdst_M, InstWord_M,
                        nop);
    input WEN, CLK, RST;
    output reg NEW;
    input [31:0] ALUresult_E, RegBData_E, PC_Plus4_E, Immediate_E;
    output reg [31:0] ALUresult_M, RegBData_M, PC_Plus4_M, Immediate_M;
    input [4:0] Rdst_E;
    output reg [4:0] Rdst_M;
    input [31:0] InstWord_E;
    output reg [31:0] InstWord_M;
    inout nop;

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
        end else if (nop) begin
            // set value to nop
            ALUresult_M <= 32'b0;
            RegBData_M <= 32'b0;
            PC_Plus4_M <= 32'b0;
            Rdst_M <= 5'b0;
            InstWord_M <= 32'h13;
            Immediate_M <= 32'b0;
            // if not just reset, this program is not NEW
            NEW <= 1'b0;
        end else if (!WEN) begin
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
                        halt_E, halt_M, nop,
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
    input nop;

    always @ (negedge CLK or negedge RST)
        if (!RST) begin
            // set all vals to 0
            MemWrEn_M <= 1'b0;
            RegWrEn_M <= 1'b0;
            WBSel_M <= 2'b0;
            LoadType_M <= 3'b0;
            MemSize_M <= 2'b0;
            halt_M <= 1'b0;
            // if just reset, this program is NEW
            NEW_OUT <= 1'b1;
        end else if (nop) begin
            // set value to nop
            MemWrEn_M <= 1'b1;
            RegWrEn_M <= 1'b1;
            WBSel_M <= 2'b0;
            LoadType_M <= 3'b0;
            MemSize_M <= 2'b0;
            halt_M <= 1'b0;
            // if not just reset, this program is not NEW
            NEW_OUT <= NEW_IN;
        end else if (!WEN) begin
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

module MEM_WB_data_reg(WEN, CLK, RST, ALUresult_M, MemReadData_M, Immediate_M, PC_Plus4_M, Rdst_M, 
                        ALUresult_W, MemReadData_W, Immediate_W, PC_Plus4_W, Rdst_W);
    input WEN, CLK, RST;
    input [31:0] ALUresult_M, MemReadData_M, Immediate_M, PC_Plus4_M;
    output reg [31:0] ALUresult_W, MemReadData_W, Immediate_W, PC_Plus4_W;
    input [4:0] Rdst_M;
    output reg [4:0] Rdst_W;

    always @ (negedge CLK or negedge RST)
        if (!RST) begin
            // set all vals to 0
            ALUresult_W <= 32'b0;
            MemReadData_W <= 32'b0;
            PC_Plus4_W <= 32'b0;
            Immediate_W <= 32'b0;
            Rdst_W <= 5'b0;
        end else if (!WEN) begin
            // write all in values to outs
            ALUresult_W <= ALUresult_M;
            MemReadData_W <= MemReadData_M;
            PC_Plus4_W <= PC_Plus4_M;
            Immediate_W <= Immediate_M;
            Rdst_W <= Rdst_M;
        end 
endmodule

module MEM_WB_ctrl_reg(WEN, CLK, RST, RegWrEn_M, WBSel_M, 
                        RegWrEn_W, WBSel_W,
                        halt_M, halt_W,  
                        NEW_IN, NEW_OUT);
    input WEN, CLK, RST;
    input RegWrEn_M;
    output reg RegWrEn_W;
    input [1:0] WBSel_M;
    output reg [1:0] WBSel_W;
    input halt_M;
    output reg halt_W;
    input NEW_IN;
    output reg NEW_OUT;

    always @ (negedge CLK or negedge RST)
        if (!RST) begin
            // set all vals to 0
            RegWrEn_W <= 1'b0;
            WBSel_W <= 2'b0;
            halt_W <= 1'b0;
            // if just reset, this program is NEW
            NEW_OUT <= 1'b1;
        end else if (!WEN) begin
            // write all in values to outs
            RegWrEn_W <= RegWrEn_M; 
            WBSel_W <= WBSel_M;
            halt_W <= halt_M;
            // if not just reset, this program is not NEW
            NEW_OUT <= NEW_IN;
        end
endmodule

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

// immediate types
`define R_IMM 3'b000
`define I_IMM 3'b001
`define S_IMM 3'b010
`define B_IMM 3'b011
`define U_IMM 3'b100
`define J_IMM 3'b101
`define SI_IMM 3'b110

// branch Y/N
`define BR_FALSE 1'b0
`define BR_TRUE 1'b1

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

// 
// ArithmeticLogicUnit
// in - opcode, opA, opB, funct3, funct7
// out - out, halt
module ArithmeticLogicUnit(opcode, opA, opB, func, auxFunc, out, halt);
    output reg [31:0] out;
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
        if (opcode == `OPCODE_COMPUTE || opcode == `OPCODE_ICOMPUTE) begin 
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
                default:
                    if (opcode == `OPCODE_COMPUTE && auxFunc != 7'h00)
                        halt = 1'b1; // invalid auxFunc (doesn't apply to i-type)
                    else
                        out = opA + opB; // default add
            endcase
        end
        else 
            out = opA + opB; // default add for all other instruction types
    end
endmodule

// Branch Unit
// in - opA, opB, funct3, opcode
// out - out, halt
// outputs 1 if branch is taken, 0 if not
module BranchUnit(opA, opB, funct3, opcode, out, halt);
  input [31:0] opA, opB;
  input [2:0] funct3;
  input [6:0] opcode;
  output reg out;
  output reg halt;

  wire signed [31:0] sopA, sopB;
  assign sopA = opA;
  assign sopB = opB;

  always @(*) begin
    halt = 1'b0;
    if (opcode == `OPCODE_BRANCH || opcode == `OPCODE_JAL || opcode == `OPCODE_JALR)
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
                else if (opcode == `OPCODE_JAL || opcode == `OPCODE_JALR)
                out = `BR_TRUE; // if jal or jalr, always branch (jump
                else
                out = `BR_FALSE; // default
            endcase
        end
    else 
        out = `BR_FALSE; // default
  end
endmodule // Branch Unit

//
// ControlUnit
// in - opcode, funct3
// out - PCSel, ImmSel, RWrEn, ALUsrcA, ALUsrcB, MemWrEn, WBSel, halt
module ControlUnit(opcode, funct3, ImmSel, WBSel, PCSel, 
                    RWrEn, ALUsrcA, ALUsrcB, MemWrEn, LoadType, MemSize, halt);
    input [6:0] opcode;
    input [2:0] funct3;
    output reg [2:0] ImmSel, LoadType;
    output reg [1:0] WBSel, MemSize;
    output reg PCSel, RWrEn, ALUsrcA, ALUsrcB, MemWrEn;
    output reg halt;

    always @(*) begin
        halt = 1'b0; 
        case(opcode)
            `OPCODE_COMPUTE: // R-type instructions
                begin
                    PCSel = `PC_PCPLUS4; // PC source is PC+4
                    ImmSel = `R_IMM; // no immediate, 0s
                    RWrEn = 1'b0; // register write enabled
                    ALUsrcA = `ALU_A_REG; // ALU source A is register
                    ALUsrcB = `ALU_B_REG; // ALU source B is register
                    MemWrEn = 1'b1; // mem write disabled
                    WBSel = `WB_ALU; // write back ALU to register
                end
            `OPCODE_ICOMPUTE: // I-type instructions
                begin
                    PCSel = `PC_PCPLUS4; // PC source is PC+4
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
                    PCSel = `PC_PCPLUS4; // PC source is PC+4
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
                    PCSel = `PC_PCPLUS4; // PC source is PC+4
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
                    PCSel = `PC_PCPLUS4; // assume never taken, will change if taken
                    ImmSel = `B_IMM; // B-type immediate
                    RWrEn = 1'b1; // register write disabled
                    ALUsrcA = `ALU_A_PC; // ALU source A is PC 
                    ALUsrcB = `ALU_B_IMM; // ALU source B is immediate
                    MemWrEn = 1'b1; // mem write disabled
                    WBSel = `WB_UNDEF; // undefined writeback
                end
            `OPCODE_JAL: // JAL
                begin
                    PCSel = `PC_ALUOUT; // PC source is ALU output 
                    ImmSel = `J_IMM; // J-type immediate 
                    RWrEn = 1'b0; // register write enabled
                    ALUsrcA = `ALU_A_PC; // ALU source A is PC 
                    ALUsrcB = `ALU_B_IMM; // ALU source B is immediate
                    MemWrEn = 1'b1; // mem write disabled
                    WBSel = `WB_PC4; // write PC + 4 to register
                end
            `OPCODE_JALR: // JALR
                begin
                    PCSel = `PC_ALUOUT; // PC source is ALU output 
                    ImmSel = `I_IMM; // I-type immediate 
                    RWrEn = 1'b0; // register write enabled
                    ALUsrcA = `ALU_A_REG; // ALU source A is register
                    ALUsrcB = `ALU_B_IMM; // ALU source B is immediate
                    MemWrEn = 1'b1; // mem write disabled
                    WBSel = `WB_PC4; // write PC + 4 to register
                end
            `OPCODE_LUI: // LUI
                begin
                    PCSel = `PC_PCPLUS4; // PC source is PC + 4 
                    ImmSel = `U_IMM; // U-type immediate 
                    RWrEn = 1'b0; // register write enabled
                    ALUsrcA = `ALU_A_REG; // ALU source A doesn't matter
                    ALUsrcB = `ALU_B_REG; // ALU source B doesn't matter
                    MemWrEn = 1'b1; // mem write disabled
                    WBSel = `WB_IMM; // write immediate back
                end
            `OPCODE_AUIPC: // AUIPC
                begin
                    PCSel = `PC_PCPLUS4; // PC source is PC + 4
                    ImmSel = `U_IMM; // U-type immediate 
                    RWrEn = 1'b0; // register write enabled
                    ALUsrcA = `ALU_A_PC; // ALU source A is PC
                    ALUsrcB = `ALU_B_IMM; // ALU source B is immediate
                    MemWrEn = 1'b1; // mem write disabled
                    WBSel = `WB_ALU; // write ALU to register
                end
            default:
                halt = 1'b1; // if not a valid opcode, halt signal
        endcase
    end
endmodule

module HazardUnit(RegA_ID, RegB_ID, opcode_ID, 
                    opcode_EX, Rdst_EX, opcode_MEM, Rdst_MEM, 
                    EX_stall, MEM_stall);
    input [4:0] RegA_ID, RegB_ID;
    input [6:0] opcode_ID;
    
    input [4:0] Rdst_EX, Rdst_MEM;
    input [6:0] opcode_EX, opcode_MEM;
    
    output reg EX_stall, MEM_stall;
    
    reg [1:0] ID_use;
    reg EX_affect, MEM_affect;

    always @(*) begin
        case(opcode_ID)
            `OPCODE_COMPUTE: ID_use = 2'b01;
            `OPCODE_ICOMPUTE: ID_use = 2'b10;
            `OPCODE_LOAD: ID_use = 2'b10;
            `OPCODE_STORE:  ID_use = 2'b01;
            `OPCODE_BRANCH: ID_use = 2'b01;
            default: ID_use = 2'b00;
        endcase

        case(opcode_EX)
            `OPCODE_COMPUTE: EX_affect = 1'b1;
            `OPCODE_ICOMPUTE: EX_affect = 1'b1;
            `OPCODE_LOAD: EX_affect = 1'b1;
            `OPCODE_AUIPC: EX_affect = 1'b1;
            `OPCODE_LUI: EX_affect = 1'b1;
            `OPCODE_JAL: EX_affect = 1'b1;
            `OPCODE_JALR: EX_affect = 1'b1;
            default: EX_affect = 1'b0;
        endcase

        case(opcode_MEM)
            `OPCODE_COMPUTE: MEM_affect = 1'b1;
            `OPCODE_ICOMPUTE: MEM_affect = 1'b1;
            `OPCODE_LOAD: MEM_affect = 1'b1;
            `OPCODE_AUIPC: MEM_affect = 1'b1;
            `OPCODE_LUI: MEM_affect = 1'b1;
            `OPCODE_JAL: MEM_affect = 1'b1;
            `OPCODE_JALR: MEM_affect = 1'b1;
            default: MEM_affect = 1'b0;
        endcase

        // hazard possible iff EX_affect and ID_use are NOT 0
        if ((ID_use != 2'b00) && (MEM_affect == 1'b1))
            begin
                case(ID_use)
                    2'b01: MEM_stall = (((RegA_ID == Rdst_MEM) || (RegB_ID == Rdst_MEM)) ? 1'b1 : 1'b0);
                    2'b10: MEM_stall = ((RegA_ID == Rdst_MEM) ? 1'b1 : 1'b0);
                    default: MEM_stall = 1'b0;
                endcase

                if(Rdst_MEM == 0)
                    MEM_stall = 1'b0;
            end
        else 
            MEM_stall = 1'b0;

        if ((ID_use != 2'b00) && (EX_affect == 1'b1))
            begin
                case(ID_use)
                    2'b01: EX_stall = (((RegA_ID == Rdst_EX) || (RegB_ID == Rdst_EX)) ? 1'b1 : 1'b0);
                    2'b10: EX_stall = ((RegA_ID == Rdst_EX) ? 1'b1 : 1'b0);
                    default: EX_stall = 1'b0;
                endcase

                if (Rdst_EX == 0)
                    EX_stall = 1'b0;
            end
        else 
            EX_stall = 1'b0;
    end
endmodule // HazardUnit

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
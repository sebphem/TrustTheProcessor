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


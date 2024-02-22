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
    output reg [63:0] mulout;
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
             if (auxFunc == 7'b0000001) begin
            case (func)
                3'b000: begin 
                        out = {sopA * sopB};
                        out[31] = sopA[31] ^ sopB[31];
                end
                3'b001: begin 
                        mulout = sopA * sopB;
                        out = mulout[63:32];
                end
                3'b010: begin 
                    
                        mulout = sopA * $signed({32'b0, opB});
                        out = mulout[63:32];
                end
                3'b011: begin 
                        mulout = opA * opB;
                        out = mulout[63:32];
                end
                3'b100: out = sopA / sopB;
                3'b101: out = opA / opB;
                3'b110: out = sopA % sopB;
                3'b111: out = opA % opB;
                default: begin 
                    halt <= 1'b1;
                    out = 32'b0;
                end
            endcase
        end else begin
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
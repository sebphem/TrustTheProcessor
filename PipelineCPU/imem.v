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
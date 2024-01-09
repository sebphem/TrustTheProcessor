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
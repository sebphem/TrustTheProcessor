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
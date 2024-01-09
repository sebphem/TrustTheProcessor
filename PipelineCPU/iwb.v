module IWB(
    input [31:0] PC_in_wb,
    input [31:0] Inst_in_wb,
    input [1:0] WBSel_in_wb,
    input [31:0] LoadExtended_in_wb,
    input [31:0] Immediate_in_wb,
    input [31:0] ALUOutput_in_wb,
    input [4:0] Rdst_in_wb,
    input halt_in_wb,
    output [4:0] Rw_out_wb,
    output [31:0] Di_out_wb,
    input WEN,
    input CLK,
    input RST);

assign Di_out_wb = (WBSel_in_wb == `WBSel_ALU) ? ALUOutput_in_wb : (WBSel_in_wb == `WBSel_PC4) ? PC_in_wb + 4 : (WBSel_in_wb == `WBSel_Mem) ? LoadExtended_in_wb : Immediate_in_wb;
assign Rw_out_wb = (halt_in_wb)? 0 : Rdst_in_wb;
endmodule
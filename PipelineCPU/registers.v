module IF_ID_Register(
    input [31:0] PC_if,
    input [31:0] Inst_if,
    input halt_if,
    input valid_if,
    output reg valid_id,
    output reg halt_id,
    output reg [31:0] PC_id,
    output reg [31:0] Inst_id,
    input stall,
    input squash,
    input WEN, 
    input CLK, 
    input RST);
    always @ (negedge CLK or negedge RST)
        if(squash) begin
            halt_id <= 0;
            PC_id <= 0;
            Inst_id <= 32'h00000013;
            valid_id <= 1'b1;
        end else if (stall) begin
            // Latch values
        end else if (!RST || (valid_if == 1'b0)) begin
            halt_id <= 0;
            PC_id <= 0;
            Inst_id <= 0;
            valid_id <= 0;
        end else if (!WEN) begin
            valid_id <= valid_if;
            halt_id <= halt_if & valid_if;
            PC_id <= PC_if;
            Inst_id <= Inst_if;
        end
endmodule // IF_ID_Register

module ID_EX_Register(
    input [31:0] PC_id,
    input [31:0] Inst_id,
    input MemRW_id, 
    input RWrEn_id,
    input [1:0] ALUOp_id,
    input [1:0] ALUSrc_id,
    input [4:0] RegDst_id,
    input [2:0] ImmSel_id,
    input ASel_id,
    input BSel_id,
    input JMP_id,
    input BR_id,
    input [1:0] WBSel_id,
    input [31:0] Immediate_id,
    input [1:0] MemSize_id,
    input [31:0] Rdata1_id,
    input [31:0] Rdata2_id,
    input [4:0] Rsrc1_id,
    input [4:0] Rsrc2_id,
    input halt_id,
    input stall,
    input valid_id,
    output reg valid_ex,
    output reg [31:0] PC_ex,
    output reg [31:0] Inst_ex,
    output reg MemRW_ex,
    output reg RWrEn_ex,
    output reg [1:0] ALUOp_ex,
    output reg [1:0] ALUSrc_ex,
    output reg [4:0] RegDst_ex,
    output reg [2:0] ImmSel_ex,
    output reg [31:0] Rdata1_ex,
    output reg [31:0] Rdata2_ex,
    output reg ASel_ex,
    output reg BSel_ex,
    output reg JMP_ex,
    output reg BR_ex,
    output reg [1:0] WBSel_ex,
    output reg [31:0] Immediate_ex,
    output reg [1:0] MemSize_ex,
    output reg [4:0] Rsrc1_ex,
    output reg [4:0] Rsrc2_ex,
    output reg halt_ex,
    input squash,
    input WEN, 
    input CLK, 
    input RST);
    wire [2:0] test;
    always @ (negedge CLK or negedge RST) begin
        if(squash) begin
            valid_ex <= 0;
            PC_ex <= 0;
            Inst_ex <= 32'h00000013;
            MemRW_ex <= 1;
            RWrEn_ex <= 1;
            ALUOp_ex <= 0;
            ALUSrc_ex <= 0;
            RegDst_ex <= 0;
            ImmSel_ex <= 0;
            ASel_ex <= 0;
            BSel_ex <= 0;
            JMP_ex <= 0;
            BR_ex <= 0;
            WBSel_ex <= 0;
            Immediate_ex <= 0;
            MemSize_ex <= 0;
            Rdata1_ex <= 0;
            Rdata2_ex <= 0;
            Rsrc1_ex <= 0;
            Rsrc2_ex <= 0;
            halt_ex <= 0;
        end else if (stall) begin
            // latch values
        end else if (!RST || (valid_id == 1'b0)) begin
            valid_ex <= 0;
            PC_ex <= 0;
            Inst_ex <= 0;
            MemRW_ex <= 0;
            RWrEn_ex <= 0;
            ALUOp_ex <= 0;
            ALUSrc_ex <= 0;
            RegDst_ex <= 0;
            ImmSel_ex <= 0;
            ASel_ex <= 0;
            BSel_ex <= 0;
            JMP_ex <= 0;
            BR_ex <= 0;
            WBSel_ex <= 0;
            Immediate_ex <= 0;
            MemSize_ex <= 0;
            Rdata1_ex <= 0;
            Rdata2_ex <= 0;
            Rsrc1_ex <= 0;
            Rsrc2_ex <= 0;
            halt_ex <= 0;
        end else if (!WEN) begin
            valid_ex <= valid_id;
            PC_ex <= PC_id;
            Inst_ex <= Inst_id;
            MemRW_ex <= MemRW_id;
            RWrEn_ex <= RWrEn_id;
            ALUOp_ex <= ALUOp_id;
            ALUSrc_ex <= ALUSrc_id;
            RegDst_ex <= RegDst_id;
            ImmSel_ex <= ImmSel_id;
            ASel_ex <= ASel_id;
            BSel_ex <= BSel_id;
            JMP_ex <= JMP_id;
            BR_ex <= BR_id;
            WBSel_ex <= WBSel_id;
            Immediate_ex <= Immediate_id;
            MemSize_ex <= MemSize_id;
            Rdata1_ex <= Rdata1_id;
            Rdata2_ex <= Rdata2_id;
            Rsrc1_ex <= Rsrc1_id;
            Rsrc2_ex <= Rsrc2_id;
            halt_ex <= halt_id && valid_id;
        end
    end
endmodule // ID_EX_Register

module EX_MEM_Register(
    input [31:0] PC_ex,
    input [31:0] Inst_ex,
    input MemRW_ex,
    input RWrEn_ex,
    input MemToReg_ex,
    input BranchCondTrue_ex,  
    input [1:0] WBSel_ex,
    input [1:0] MemSize_ex,
    input [31:0] ALUOutput_ex,
    input [31:0] Immediate_ex,
    input [4:0] Rdst_ex,
    input [31:0] Rdata2_ex,
    input halt_ex,
    input valid_ex,
    output reg valid_mem,
    output reg [31:0] PC_mem,
    output reg [31:0] Inst_mem,
    output reg MemRW_mem,
    output reg RWrEn_mem,
    output reg BranchCondTrue_mem,
    output reg [1:0] WBSel_mem,
    output reg [1:0] MemSize_mem,
    output reg [31:0] ALUoutput_mem,
    output reg [31:0] Immediate_mem,
    output reg [4:0] Rdst_mem,
    output reg [31:0] Rdata2_mem,
    output reg halt_mem,
    input squash,
    input WEN, 
    input CLK, 
    input RST);
    always @ (negedge CLK or negedge RST)
        if (!RST || !valid_ex) begin
            valid_mem <= 0;
            PC_mem <= 0;
            Inst_mem <= 0;
            MemRW_mem <= 1;
            RWrEn_mem <= 0;
            BranchCondTrue_mem <= 0;
            WBSel_mem <= 0;
            MemSize_mem <= 0;
            ALUoutput_mem <= 0;
            Rdst_mem <= 0;
            Rdata2_mem <= 0;
            Immediate_mem <= 0;
            halt_mem <= 0;
        end 
        else if (squash) begin
            valid_mem <= 1;
            PC_mem <= PC_ex;
            Inst_mem <= 32'h00000013;
            MemRW_mem <= `MemRW_Read;
            RWrEn_mem <= `RWrEn_Disable;
            BranchCondTrue_mem <= 1'b0;
            WBSel_mem <= `WBSel_PC4;
            MemSize_mem <= `SIZE_WORD;
            ALUoutput_mem <= 32'h00000000;
            Immediate_mem <= 32'h00000000;
            Rdst_mem <= 5'b00000;
            Rdata2_mem <= 32'h00000000;
            halt_mem <= 1'b0;
        end else if (!WEN) begin
            valid_mem <= valid_ex;
            PC_mem <= PC_ex;
            Inst_mem <= Inst_ex;
            MemRW_mem <= MemRW_ex;
            RWrEn_mem <= RWrEn_ex;
            BranchCondTrue_mem <= BranchCondTrue_ex;
            WBSel_mem <= WBSel_ex;
            MemSize_mem <= MemSize_ex;
            ALUoutput_mem <= ALUOutput_ex;
            Immediate_mem <= Immediate_ex;
            Rdst_mem <= Rdst_ex;
            Rdata2_mem <= Rdata2_ex;
            halt_mem <= halt_ex && valid_ex;
        end
endmodule // EX_MEM_Register

module MEM_WB_Register(
    input [31:0] PC_mem,
    input [31:0] Inst_mem,
    input MemRW_mem,
    input RWrEn_mem,
    input [1:0] WBSel_mem,
    input [31:0] LoadExtended_mem,
    input [31:0] Immediate_mem,
    input [31:0] ALUOutput_mem,
    input [4:0] Rdst_mem,
    input halt_mem,
    input valid_mem,
    output reg valid_wb,
    output reg [31:0] PC_wb,
    output reg [31:0] Inst_wb,
    output reg MemRW_wb,
    output reg RWrEn_wb,
    output reg [1:0] WBSel_wb,
    output reg [31:0] LoadExtended_wb,
    output reg [31:0] Immediate_wb,
    output reg [31:0] ALUOutput_wb,
    output reg [4:0] Rdst_wb,
    output reg halt_wb,
    input WEN, 
    input CLK, 
    input RST);
    always @ (negedge CLK or negedge RST)
        if (!RST || !valid_mem) begin
            valid_wb <= 0;
            PC_wb <= 0;
            Inst_wb <= 0;
            WBSel_wb <= 0;
            LoadExtended_wb <= 0;
            Immediate_wb <= 0;
            ALUOutput_wb <= 0;
            halt_wb <= 0;
            Rdst_wb <= 0;
            RWrEn_wb <= 1;
        end else if (!WEN) begin
            valid_wb <= valid_mem;
            PC_wb <= PC_mem;
            Inst_wb <= Inst_mem;
            WBSel_wb <= WBSel_mem;
            LoadExtended_wb <= LoadExtended_mem;
            Immediate_wb <= Immediate_mem;  
            ALUOutput_wb <= ALUOutput_mem;
            Rdst_wb <= Rdst_mem;
            RWrEn_wb <= RWrEn_mem;
            halt_wb <= halt_mem && valid_mem;
        end
endmodule // MEM_WB_Register
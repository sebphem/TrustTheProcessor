`define ALU_A_REG 1'b0
`define ALU_B_IMM 1'b1
`define WB_ALU 2'b00
`define I_IMM 3'b001


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
            halt_E <= halt_D;;
            didBranch_E <= didBranch_D;
            // if not just reset, this program is not NEW
            NEW_OUT <= NEW_IN;
        end
endmodule

module EX_MEM_data_reg(WEN, CLK, RST, NEW, ALUresult_E, RegBData_E, Immediate_E, PC_Plus4_E, Rdst_E, InstWord_E,
                        ALUresult_M, RegBData_M, Immediate_M, PC_Plus4_M, Rdst_M, InstWord_M,
                        // nop
                        );
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
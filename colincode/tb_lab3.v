// Testbench for Northwestern - CompEng 361 - Lab2

module tb;
   reg clk, rst;
   reg exit;
   wire halt;
   

   // CPU instantiation
   PipelinedCPU CPU (halt, clk,rst);

   // Clock Period = 10 time units
   always
     #5 clk = ~clk;

   always @(posedge clk)
     if (halt)
       exit = 1;
   
   initial begin
    $dumpfile("waveform.vcd");
    $dumpvars(0, tb);

      // Clock and reset steup
      #0 rst = 1; clk = 0; exit =0;
      #0 rst = 0;
      #0 rst = 1;

      // Load program
      #0 $readmemh("mem_in.hex", CPU.IF.IMEM.Mem);
      #0 $readmemh("mem_in.hex", CPU.MEM.DMEM.Mem);
      #0 $readmemh("regs_in.hex", CPU.ID.RF.Mem);


      // regwrdata_wb=%x regwr_en_wb=%x rdst_wb=%x rdst_data_m=%x rdst_data_w=%x rdst_data_in_w=%x

      // Feel free to modify to inspect whatever you want


      // #0 $monitor($time,, "PC=%08x pcsel=%x pc.target addr pc=%08x pc+4=%08x targetpc=%08x\n IR=%08x halt=%x exit=%x IF_ID_Stall=%x \n ID\n   RegA=%x RegA Val=%x RegB=%x RegB Val=%x Rdst=%x halt=%x regwr=%x\n   controlunit regwr=%x\n EX\n   Rdst=%x Rdst_Data=%x RegA_Data=%x RegB_DATA=%x ALU_OUT=%x halt=%x regwr=%x \n   Branch Predictor pc=%08x target addr=%08x pc_sel=%x isnew=%x doBranch=%x funct3=%x opcode=%x\n Forwarding\n   RegA_ID_name=%x RegA_VAL_ID=%x RegB_ID_name=%x RegB_VAL_ID =%x\n    EX RDest=%x RDest_VAL=%x Reg_WR_EN=%x opcode=%x\n    MEM RDest=%x RDest_VAL=%x Reg_WR_EN=%x opcode=%x \n Forwarding out:\n   regA=%x regB=%x\n MEM\n   ALUresult_in=%x RegBData=%x M[%x] val put in: %x val gotten out: %x memwr=%x\n   Rdst=%x rdst_val=%x halt=%x regwr=%x opcode=%x\n WB\n   regwr=%x \n",
      //             //IF
      //             CPU.IF.PC, CPU.IF.PCSel, CPU.IF.targetAddr, CPU.IF.PC_Plus4, CPU.IF.Target_PC,
      //             CPU.IF.InstWord, halt, exit, CPU.IF_ID_stall_out,
      //             //ID
      //             CPU.ID.RegA, CPU.ID.RegAData, CPU.ID.RegB, CPU.ID.RegBData, CPU.ID_EX_data.Rdst_D, CPU.ID.halt_ID_out, CPU.ID.RegWrEn, CPU.ID.CU.RWrEn,
      //             // CPU.ID.RegWrData_WB, CPU.ID.RegWrEn_WB, CPU.ID.Rdst_WB, CPU.MEM_WB_data.Rdst_Data_M,CPU.MEM_WB_data.Rdst_Data_W, CPU.Rdst_Data_in_W,
      //             //ex
      //             CPU.EX.Rdst_out, CPU.EX.Rdst_Data_out, CPU.EX.RegAData, CPU.EX.RegBData, CPU.EX.ALUout, CPU.EX.halt_EX_out, CPU.EX.RegWrEn_out,
      //             CPU.EX.PC, CPU.EX.targetAddr, CPU.EX.PCsel, CPU.EX.isNew, CPU.EX.doBranch, CPU.EX.funct3, CPU.EX.opcode,
      //             //forwarding
      //             CPU.Forwarding.RegA_ID, CPU.Forwarding.RegA_ID_cur, CPU.Forwarding.RegB_ID, CPU.Forwarding.RegB_ID_cur,
      //             CPU.Forwarding.Rdst_EX_Name, CPU.Forwarding.Rdst_EX_Data, CPU.Forwarding.RegWREN_EX, CPU.Forwarding.opcode_EX,
      //             CPU.Forwarding.Rdst_MEM_Name, CPU.Forwarding.Rdst_MEM_Data, CPU.Forwarding.RegWREN_MEM, CPU.Forwarding.opcode_MEM,
      //             CPU.Forwarding.RegA_ID_Data,  CPU.Forwarding.RegB_ID_Data,
      //             //MEM
      //             CPU.MEM.ALUresult_in, CPU.MEM.RegBData, CPU.MEM.DMEM.Addr, CPU.MEM.DMEM.DataIn, CPU.MEM.DMEM.DataOut, CPU.MEM.DMEM.WEN,
      //             CPU.MEM.Rdst_out, CPU.MEM.Rdst_Data_out,
      //             CPU.MEM.halt_MEM_out, CPU.MEM.RegWrEn_out,
      //             CPU.MEM.InstWord[6:0],
      //             //WB
      //             CPU.MEM_WB_ctrl.RegWrEn_W);


      #0 $monitor($time,, "PC=%08x IR=%08x halt=%x exit=%x", CPU.IF.PC, CPU.IF.InstWord, halt, exit);
      // Exit???
      wait(exit);
      // Dump registers
      #0 $writememh("regs_out.hex", CPU.ID.RF.Mem);

      // Dump memory
      #0 $writememh("mem_out.hex", CPU.MEM.DMEM.Mem);

      $finish;      
   end
endmodule // tb


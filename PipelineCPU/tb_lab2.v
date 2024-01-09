// Testbench for Northwestern - CompEng 361 - Lab2
module tb;
   reg clk, rst;
   reg exit;
   wire halt;
   

   // Single Cycle CPU instantiation
   SingleCycleCPU CPU (halt, clk,rst);

   // Clock Period = 10 time units
   always
     #5 clk = ~clk;

   always @(posedge clk)
     if (halt)
       exit = 1;
   
   initial begin
      // Clock and reset steup
      #0 rst = 1; clk = 0; exit =0;
      #0 rst = 0;
      #0 rst = 1;

      // Load program
      #0 $readmemh("mem_in.hex", CPU.fetchStage.IMEM.Mem);
      #0 $readmemh("mem_in.hex", CPU.memoryUnit.DMEM.Mem);
      #0 $readmemh("regs_in.hex", CPU.decoder.RF.Mem);
      #0 $dumpfile("output.vcd");
      #0 $dumpvars(0, CPU);
      // Feel free to modify to inspect whatever you want
      #0 $monitor($time,, "PC=%08x IR=%08x halt=%x exit=%x", CPU.fetchStage.PC, CPU.fetchStage.instr_out_if, halt, exit);

      // Exit???
      wait(exit);
      
      // Dump registers
      #0 $writememh("regs_out.hex", CPU.decoder.RF.Mem);

      // Dump memory
      #0 $writememh("mem_out.hex", CPU.memoryUnit.DMEM.Mem);
      $finish;      
   end
   

endmodule // tb


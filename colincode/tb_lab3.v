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

      // Feel free to modify to inspect whatever you want
      #0 $monitor($time,, "PC=%08x IR=%08x halt=%x exit=%x IF_Stall=%x I_Stall=%x", CPU.IF.PC, CPU.IF.InstWord, halt, exit, CPU.IF_stall_out, CPU.ID_stall_out);

      // Exit???
      wait(exit);
      
      // Dump registers
      #0 $writememh("regs_out.hex", CPU.ID.RF.Mem);

      // Dump memory
      #0 $writememh("mem_out.hex", CPU.MEM.DMEM.Mem);

      $finish;      
   end
endmodule // tb


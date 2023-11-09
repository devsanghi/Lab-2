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
      #0 $readmemh("mem_in.hex", CPU.IMEM.Mem);
      #0 $readmemh("mem_in.hex", CPU.DMEM.Mem);
      #0 $readmemh("regs_in.hex", CPU.RF.Mem);

      // Feel free to modify to inspect whatever you want
      // #0 $monitor($time,, "PC=%08x IR=%08x halt=%x exit=%x", CPU.PC, CPU.InstWord, halt, exit);
      #0 $monitor($time,, "Rsrc1=%04x, Rdata1=%08x, Rsrc2 = %04x, Rdata2 = %08x, pc = %08X, ALUop2=%08x, ALUres = %08x, NPC = %08x, Rdst=%04x, RWdata=%04x, DataWord = %08x, zero = %04x, neg = %04x, immext: %04x, pcimm: %04x, memsize: %04x, wren: %04x", CPU.Rsrc1, CPU.Rdata1, CPU.Rsrc2, CPU.Rdata2, CPU.PC, CPU.ALUop2, CPU.ALUresult, CPU.NPC, CPU.Rdst, CPU.RWrdata, CPU.DataWord, CPU.zero, CPU.negative, CPU.Imm_extended, CPU.Bimm13, CPU.MemSize, CPU.MemWrEn);

      // Exit???
      wait(exit);
      
      // Dump registers
      #0 $writememh("regs_out.hex", CPU.RF.Mem);

      // Dump memory
      #0 $writememh("mem_out.hex", CPU.DMEM.Mem);

      $finish;      
   end
   

endmodule // tb


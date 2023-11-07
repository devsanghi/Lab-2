// Testbench for Northwestern - CompEng 361 - Lab2

module tb;
   reg clk, rst;
   wire halt;

   // Single Cycle CPU instantiation
   SingleCycleCPU CPU (halt, clk, rst);

   // Clock Period = 10 time units
   // (stops when halt is asserted)  
   always
     #5 clk = ~clk & !halt;

   initial begin
      // Clock and reset setup
      #0 rst = 1; clk = 0;
      $display("Initial reset and clock setup at time %t", $time);
      
      #0 rst = 0;
      $display("Reset deasserted at time %t", $time);

      #0 rst = 1;
      $display("Reset asserted again at time %t", $time);

      // Load program
      $display("Loading memory and register contents at time %t", $time);
      #0 $readmemh("mem_in.hex", CPU.IMEM.Mem);
      #0 $readmemh("regs_in.hex", CPU.RF.Mem);

      // Feel free to modify to inspect whatever you want
      #0 $monitor($time,, "PC=%08x IR=%08x", CPU.PC, CPU.InstWord);

      // Exits when halt is asserted
      $display("Starting simulation at time %t", $time);
      wait(halt);
      $display("Halt detected at time %t", $time);

      // Dump registers
      $display("Dumping registers to regs_out.hex at time %t", $time);
      #0 $writememh("regs_out.hex", CPU.RF.Mem);

      // Dump memory
      $display("Dumping memory to mem_out.hex at time %t", $time);
      #0 $writememh("mem_out.hex", CPU.DMEM.Mem);

      // End simulation
      $display("Simulation finished at time %t", $time);
      $finish;      
   end
endmodule


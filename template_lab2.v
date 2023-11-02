// Template for Northwestern - CompEng 361 - Lab2
// Groupname: Dev + George
// NetIDs: dds9623; gam3940

// Some useful defines...please add your own
`define OPCODE_COMPUTE    7'b0110011 // R-type
`define OPCODE_BRANCH     7'b1100011 // B-type
`define OPCODE_IMMEDIATE  7'b0010011 // I-type
`define OPCODE_LOAD       7'b0000011 // Load-type
`define OPCODE_STORE      7'b0100011 // S-type Store Type
`define OPCODE_JAL        7'b1101111 // J-type
`define OPCODE_JALR       7'b1100111 // I-type
`define OPCODE_LUI        7'b0110111 // U-type LUI
`define OPCODE_AUIPC      7'b0010111 // U-type AUIPC

`define SIZE_BYTE  2'b00
`define SIZE_HWORD 2'b01
`define SIZE_WORD  2'b10
// For Execution Unit R-Type instructions
`define FUNC_ADD      3'b000
`define AUX_FUNC_ADD  7'b0000000
`define AUX_FUNC_SUB  7'b0100000
`define FUNC_SLL 3'b001
`define FUNC_SLT 3'b010
`define FUNC_SLTU 3'b011
`define FUNC_XOR 3'b100
`define FUNC_SRL 3'b101
`define AUX_FUNC_SRL  7'b0000000
`define AUX_FUNC_SRA  7'b0100000
`define FUNC_OR 3'b110
`define FUNC_AND 3'b111
// For Execution Unit I-Type instructions
`define FUNC_ADDI  3'b000
`define FUNC_SLTI  3'b010
`define FUNC_XORI  3'b100
`define FUNC_ORI   3'b110
`define FUNC_ANDI  3'b111
`define FUNC_SLLI  3'b001
`define FUNC_SRLI  3'b101
`define AUX_FUNC_SRLI  7'b0000000
`define AUX_FUNC_SRAI  7'b0100000
// For Execution Unit Load/Store instructions
`define FUNC_LB    3'b000
`define FUNC_LH    3'b001
`define FUNC_LW    3'b010
`define FUNC_LBU   3'b100
`define FUNC_LHU   3'b101
`define FUNC_SB    3'b000
`define FUNC_SH    3'b001
`define FUNC_SW    3'b010
// For Execution Unit Branch instructions
`define FUNC_BEQ   3'b000
`define FUNC_BNE   3'b001
`define FUNC_BLT   3'b100
`define FUNC_BGE   3'b101
`define FUNC_BLTU  3'b110
`define FUNC_BGEU  3'b111
// For Execution Unit Jump instructions
`define FUNC_JAL   3'b010
`define FUNC_JALR  3'b011



module SingleCycleCPU(halt, clk, rst);
   output halt;
   input clk, rst;

   wire [31:0] PC, InstWord;
   wire [31:0] DataAddr, StoreData, DataWord;
   wire [1:0]  MemSize;
   wire        MemWrEn;
   
   wire [4:0]  Rsrc1, Rsrc2, Rdst;
   wire [31:0] Rdata1, Rdata2, RWrdata;
   wire        RWrEn;

   wire [31:0] NPC, PC_Plus_4;
   wire [6:0]  opcode;

   // needed for instruction decode
   wire [6:0]  funct7;
   wire [2:0]  funct3;
   wire [11:0] Iimm12;
   wire [11:0] Simm12;
   wire [11:0] Bimm12;
   wire [19:0] Uimm20;
   wire [19:0] Jimm20;

   // outputs from each individual execution unit // idk if we need it, maybe only use RWrdata and MUX before??
   wire [31:0] exe_r_out, exe_i_out, exe_l_out, exe_s_out, exe_b_out, exe_j_out;

   // Only support R-TYPE ADD and SUB
   /// Need to extend for all functions -- do at end
   assign halt = !((opcode == `OPCODE_COMPUTE) && (funct3 == `FUNC_ADD) &&
		   ((funct7 == `AUX_FUNC_ADD) || (funct7 == `AUX_FUNC_SUB)));
     
   // System State (everything is neg assert)
   InstMem IMEM(.Addr(PC), .Size(`SIZE_WORD), .DataOut(InstWord), .CLK(clk));
   DataMem DMEM(.Addr(DataAddr), .Size(MemSize), .DataIn(StoreData), .DataOut(DataWord), .WEN(MemWrEn), .CLK(clk));

   RegFile RF(.AddrA(Rsrc1), .DataOutA(Rdata1), 
	      .AddrB(Rsrc2), .DataOutB(Rdata2), 
	      .AddrW(Rdst), .DataInW(RWrdata), .WenW(RWrEn), .CLK(clk));

   Reg PC_REG(.Din(NPC), .Qout(PC), .WEN(1'b0), .CLK(clk), .RST(rst));

   // Instruction Decode
   assign opcode = InstWord[6:0];   
   assign Rdst = InstWord[11:7]; 
   assign Rsrc1 = InstWord[19:15]; 
   assign Rsrc2 = InstWord[24:20];
   assign funct3 = InstWord[14:12];  // R-Type, I-Type, S-Type
   assign funct7 = InstWord[31:25];  // R-Type
   assign Iimm12 = InstWord[31:20]; // I-Type
   assign Simm12 = {InstWord[31:25], InstWord[11:7]}; // S-Type
   assign Bimm12 = {InstWord[31], InstWord[7], InstWord[30:25], InstWord[11:8], 1'b0}; // B-Type
   assign Uimm20 = {InstWord[31:12], 12'b0}; // U-Type
   assign Jimm20 = {InstWord[31], InstWord[19:12], InstWord[20], InstWord[30:21], 1'b0}; // J-Type

   // mux to see which execution unit needs to be called
   ExecUnitMUX ExecSel(.EUSel(ExecSelector), .op(opcode));

   .EUSel


   ExecutionUnitR exe_r_unit(.out(exe_r_out), .opA(opA), .opB(opB), .func(func), .auxFunc(auxFunc));
   ExecutionUnitI exe_i_unit(.out(exe_i_out), .opA(opA), .func(func), .imm12(imm12));
   ExecutionUnitL exe_l_unit(.out(exe_l_out), .opA(opA), .func(func), .imm12(imm12));
   ExecutionUnitS exe_s_unit(.out(exe_s_out), .opA(opA), .opB(opB), .func(func), .imm5(imm5), .imm6(imm6));
   ExecutionUnitB exe_b_unit(.out(exe_b_out), .opA(opA), .opB(opB), .func(func), .imm5(imm5), .imm6(imm6));
   ExecutionUnitJ exe_j_unit(.out(exe_j_out), .opA(opA), .func(func), .imm20(imm20));
   ExecutionUnitU exe_u_unit(.out(exe_u_out), .func(func), .imm20(imm20));
   



   assign MemWrEn = 1'b1; // Change this to allow stores
   assign RWrEn = 1'b0;  // Every instruction will write to the register file

   // Hardwired to support R-Type instructions -- please add muxes and other control signals
   ExecutionUnit EU(.out(RWrdata), .opA(Rdata1), .opB(Rdata2), .func(funct3), .auxFunc(funct7));

   // Fetch Address Datapath
   assign PC_Plus_4 = PC + 4;
   assign NPC = PC_Plus_4;

   // increment PC call module at the bottom of page (PC_INC) -- need to first fix module
   
endmodule // SingleCycleCPU

// MUX for Execution Unit

module ExecUnitMUX(ExecSelector, opcode)
   output [6:0] ExecSelector;
   input [6:0]  opcode;

   assign (opcode == `OPCODE_COMPUTE) ? (ExecSelector = `OPCODE_COMPUTE) :
          (opcode == `OPCODE_IMMEDIATE) ? (ExecSelector = `OPCODE_IMMEDIATE) :
          (opcode == `OPCODE_LOAD) ? (ExecSelector = `OPCODE_LOAD) :
          (opcode == `OPCODE_STORE) ? (ExecSelector = `OPCODE_STORE) :
          (opcode == `OPCODE_BRANCH) ? (ExecSelector = `OPCODE_BRANCH) :
          (opcode == `OPCODE_JAl) ? (ExecSelector = `OPCODE_JAl) :
          (opcode == `OPCODE_JAlR) ? (ExecSelector = `OPCODE_JAlR) :
          (opcode == `OPCODE_LUI) ? (ExecSelector = `OPCODE_LUI) :
          (opcode == `OPCODE_AUIPC) ? (ExecSelector = `OPCODE_AUIPC) :
          0;

endmodule // ExecUnitMUX

// Execution unit R-Type instructions
module ExecutionUnitR(out, opA, opB, func, auxFunc);
   output [31:0] out;
   input [31:0]  opA, opB;
   input [2:0] 	 func;
   input [6:0] 	 auxFunc;

   assign out =(func == `FUNC_ADD) ? ((auxFunc == `AUX_FUNC_ADD) ? (opA + opB) : (auxFunc == `AUX_FUNC_SUB) ? (opA - opB) : 0) :
               (func == `FUNC_SLL)     ? (opA << opB[4:0]) :
               (func == `FUNC_SLT)     ? ((opA < opB) ? 32'b1 : 0) :
               (func == `FUNC_SLTU)    ? (($unsigned(opA) < $unsigned(opB)) ? 32'b1 : 0) :
               (func == `FUNC_XOR)     ? (opA ^ opB) :
               (func == `FUNC_SRL) ? ((auxFunc == `AUX_FUNC_SRL) ? (opA >> opB[4:0]) : (auxFunc == `AUX_FUNC_SRA) ? ($signed(opA) >>> opB[4:0]) : 0) :
               (func == `FUNC_OR)      ? (opA | opB) :
               (func == `FUNC_AND)     ? (opA & opB) : 
               0;  

endmodule // ExecutionUnitR

// Execution unit I-Type instructions
module ExecutionUnitI(out, opA, func, imm12);
   output [31:0] out;
   input [31:0]  opA;
   input [2:0] 	 func;
   input [11:0] 	 imm12;

   wire [11:5] auxFunc;
   wire [4:0]  shamt

   assign auxFunc = imm12[11:5];
   assign shamt = imm12[4:0];

   assign out = (func == `FUNC_ADDI) ? (opA + imm12) :
               (func == `FUNC_SLTI)     ? ((opA < imm12) ? 32'b1 : 0) :
               (func == `FUNC_XORI)     ? (opA ^ imm12) :
               (func == `FUNC_ORI)    ? (opA | imm12) :
               (func == `FUNC_ANDI)     ? (opA & imm12) :
               (func == `FUNC_SLLI) ? (opA << shamt) :
               (func == `FUNC_SRLI)      ? ((auxFunc == `AUX_FUNC_SRLI) ? (opA >> shamt) : (auxFunc == `AUX_FUNC_SRAI) ? ($signed(opA) >>> shamt) : 0) :
               0;  

endmodule // ExecutionUnitI

// Execution unit Load instructions
module ExecutionUnitL(out, opA, func, imm12);
   output [31:0] out;
   input [31:0]  opA;
   input [2:0] 	 func;
   input [11:0] 	 imm12;

   assign out = (func == `FUNC_LB) ? ($signed((opA + imm12)[0:7])) :
               (func == `FUNC_LH)     ? ((opA + imm12)[0:15]) :
               (func == `FUNC_LW)     ? ((opA + imm12)[0:31]) :
               (func == `FUNC_LBU)    ? ($unsigned((opA + imm12)[0:7])) :
               (func == `FUNC_LHU)     ? ($unsigned((opA + imm12)[0:15])) :
               0;  

endmodule // ExecutionUnitL

// Execution unit Store instructions
module ExecutionUnitS(out, opA, opB, func, imm5, imm6);
   output [31:0] out;
   input [31:0]  opA, opB;
   input [2:0] 	 func;
   input [4:0] 	 imm5;
   input [11:5] 	 imm6;

   wire [11:0] imm12;
   
   assign imm12 = {imm6, imm5};

   assign out = (func == `FUNC_SB) ? (opA + imm12) :
               (func == `FUNC_SH)     ? (opA + imm12) :
               (func == `FUNC_SW)     ? (opA + imm12) :
               0;  

endmodule // ExecutionUnitS

// Execution unit Branch instructions
module ExecutionUnitB(out, opA, opB, func, imm5, imm6);
   output [31:0] out;
   input [31:0]  opA, opB;
   input [2:0] 	 func;
   input [4:0] 	 imm5;
   input [11:5] 	 imm6;

   wire [11:0] imm12;
   
   assign imm12 = {imm6, imm5};

   assign out = (func == `FUNC_BEQ) ? (opA == opB) ? (1) :
               (func == `FUNC_BNE)     ? (opA != opB) ? (1) :
               (func == `FUNC_BLT)     ? (opA < opB) ? (1) :
               (func == `FUNC_BGE)     ? (opA >= opB) ? (1) :
               (func == `FUNC_BLTU)     ? ($unsigned(opA) < $unsigned(opB)) ? (1) :
               (func == `FUNC_BGEU)     ? ($unsigned(opA) >= $unsigned(opB)) ? (1) :
               0;  

endmodule // ExecutionUnitB

// Execution unit Jump instructions
module ExecutionUnitJ(out, opA, func, imm20);
   output [31:0] out;
   input [31:0]  opA;
   input [6:0] opcode;
   input [2:0] 	 func;
   input [19:0] 	 imm20;

   assign out = (func == `FUNC_JAL) ? (imm20) :
               (func == `FUNC_JALR)     ? (opA + imm20) :
               0;

endmodule // ExecutionUnitJ

// Execution unit U-Type instructions
module ExecutionUnitU(out, func, imm20);
   output [31:0] out;
   input [2:0] 	 func;
   input [19:0] 	 imm20;

   assign out = (func == `FUNC_LUI) ? (imm20) :
               (func == `FUNC_AUIPC)     ? (imm20) :
               0;

endmodule // ExecutionUnitU               

// PC Incrementor
module PC_INC(PC, b, z, ne, i, OUT_PC);

// b is branch flag
// z is zero flag
// i is immediate
//ne is not equal sign for branching
//OUT_PC is the signal that goes back into PC

input[31:0] PC;
input b, z, ne;
input [15:0] i;
output[31:0] OUT_PC;

wire null1;
wire[29:0] out1;  
//out1 goes to add2 to add with immediate
//out1 goes to mux1 sel0
adder_n #(.n(30)) add1(pc[31:2], {29'b0, 1'b1}, 0, out1, null1);

//sign extend into a wire called ext
wire[29:0] ext;
//ext goes into add2
ext_30 ext1(i, ext);

wire null2;
wire[29:0] out2;
//out2 goes to mux1 sel1
adder_n #(.n(30)) add2(out1, ext, 0, out2, null2);

wire[29:0] temp;
branch_mux mux1(out1, out2, b, z, ne, temp);
assign OUT_PC = {temp, 2'b0};

//this function is supposed to increase pc and branch if needed
endmodule
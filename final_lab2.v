// Template for Northwestern - CompEng 361 - Lab2
// Groupname: Dev + George
// NetIDs: dds9623; gam3940

// DEFINES
// Opcodes
`define OPCODE_COMPUTE    7'b0110011 // R-type
`define OPCODE_BRANCH     7'b1100011 // B-type
`define OPCODE_IMMEDIATE  7'b0010011 // I-type
`define OPCODE_LOAD       7'b0000011 // Load-type
`define OPCODE_STORE      7'b0100011 // S-type Store Type
`define OPCODE_JAL        7'b1101111 // J-type
`define OPCODE_JALR       7'b1100111 // I-type
`define OPCODE_LUI        7'b0110111 // U-type LUI
`define OPCODE_AUIPC      7'b0010111 // U-type AUIPC
// Other
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
`define FUNC_JALR  3'b000
// ALU Control
`define ALU_ADD  4'b0000
`define ALU_SUB  4'b0001
`define ALU_AND   4'b0010
`define ALU_OR    4'b0011
`define ALU_XOR   4'b0100
`define ALU_SLL   4'b0101
`define ALU_SRL   4'b0110
`define ALU_SRA   4'b0111
`define ALU_SLT   4'b1000
`define ALU_SLTU  4'b1001
`define ALU_NOP   4'b1111

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////           
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////   

//Single Cycle CPU
module SingleCycleCPU(halt, clk, rst);
    output halt;
    input clk, rst;

    wire [31:0] PC, InstWord;
    wire [31:0] DataAddr, StoreData, DataWord;
    wire [1:0]  MemSize;
    wire        MemWrEn;
    
    wire [4:0]  Rsrc1, Rsrc2, Rdst;
    wire [31:0] Rdata1, Rdata2, RWrdata, RWrdata_0;
    wire        RWrEn;

    wire [31:0] NPC, PC_Plus_4;
    wire PCSrc; //
    wire BranchAnd; //
    wire [6:0]  opcode;
    wire [1:0] nPC_sel; //
    wire [3:0] ALUctr; //   
    

    // needed for instruction decode
    wire [6:0]  funct7;
    wire [2:0]  funct3;
    wire [11:0] Iimm12;
    wire [11:0] Simm12;
    wire [12:0] Bimm13;
    wire [19:0] Uimm20;
    wire [20:0] Jimm21;

    wire [31:0] extended_Iimm;
    wire [31:0] extended_Simm;
    wire [31:0] extended_Bimm;
    wire [31:0] extended_Uimm;
    wire [31:0] extended_Jimm;
    wire [31:0] Imm_extended;

    wire [31:0] ALUop2;
    wire [31:0] ALUresult;

    wire [31:0] PC_Imm;
    


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 0) HALT
    
    assign halt = !((opcode == `OPCODE_LUI) ||
                (opcode == `OPCODE_AUIPC) ||
                ((opcode == `OPCODE_JAL) && ($signed(PC + extended_Jimm) % 4 == 0) && ($signed(PC + extended_Jimm) >= 0) && ($signed(PC + extended_Jimm) <= 16'h0400)) ||
                ((opcode == `OPCODE_JALR) && ($signed(Rdata1 + extended_Iimm) % 4 == 0) && ($signed(Rdata1 + extended_Iimm) >= 0) && ($signed(Rdata1 + extended_Iimm) <= 16'h0400)) ||
                ((opcode == `OPCODE_BRANCH) && ($signed(PC + extended_Bimm) % 4 == 0) && ($signed(PC + extended_Bimm) >= 0) && ($signed(PC + extended_Bimm) <= 16'h0400) && 
                (funct3 == `FUNC_BEQ || funct3 == `FUNC_BGE || funct3 == `FUNC_BGEU || funct3 == `FUNC_BLT || funct3 == `FUNC_BLTU || funct3 == `FUNC_BNE)) ||
                ((opcode == `OPCODE_LOAD) && (funct3 == `FUNC_LB) && ($signed(Rdata1 + extended_Iimm) >= 0) && ($signed(Rdata1 + extended_Iimm) <= 16'h0400)) ||
                ((opcode == `OPCODE_LOAD) && (funct3 == `FUNC_LH || funct3 == `FUNC_LHU) && ($signed(Rdata1 + extended_Iimm) % 2 == 0) && ($signed(Rdata1 + extended_Iimm) >= 0) && ($signed(Rdata1 + extended_Iimm) <= 16'h0400)) ||
                ((opcode == `OPCODE_LOAD) && (funct3 == `FUNC_LW) && ($signed(Rdata1 + extended_Iimm) % 4 == 0) && ($signed(Rdata1 + extended_Iimm) >= 0) && ($signed(Rdata1 + extended_Iimm) <= 16'h0400)) ||
                ((opcode == `OPCODE_LOAD) && (funct3 == `FUNC_LBU) && ($signed(Rdata1 + extended_Simm) >= 0) && ($signed(Rdata1 + extended_Simm) <= 16'h0400)) ||
                ((opcode == `OPCODE_STORE) && (funct3 == `FUNC_SB) && ($signed(Rdata1 + extended_Simm) >= 0) && ($signed(Rdata1 + extended_Simm) <= 16'h0400)) ||
                ((opcode == `OPCODE_STORE) && (funct3 == `FUNC_SH) && ($signed(Rdata1 + extended_Simm) % 2 == 0) && ($signed(Rdata1 + extended_Simm) >= 0) && ($signed(Rdata1 + extended_Simm) <= 16'h0400)) ||
                ((opcode == `OPCODE_STORE) && (funct3 == `FUNC_SW) && ($signed(Rdata1 + extended_Simm) % 4 == 0) && ($signed(Rdata1 + extended_Simm) >= 0) && ($signed(Rdata1 + extended_Simm) <= 16'h0400)) ||
                ((opcode == `OPCODE_IMMEDIATE) && (funct3 == `FUNC_ADDI || funct3 == `FUNC_SLTI || funct3 == `FUNC_XORI || funct3 == `FUNC_ORI || funct3 == `FUNC_ANDI)) ||
                ((opcode == `OPCODE_IMMEDIATE) && (funct3 == `FUNC_SLLI) && (funct7 == 7'b0000000)) ||
                ((opcode == `OPCODE_IMMEDIATE) && (funct3 == `FUNC_SRLI) && (funct7 == `AUX_FUNC_SRLI)) ||
                ((opcode == `OPCODE_IMMEDIATE) && (funct3 == `FUNC_SRLI) && (funct7 == `AUX_FUNC_SRAI)) ||
                ((opcode == `OPCODE_COMPUTE) && (funct3 == `FUNC_ADD) && (funct7 == `AUX_FUNC_ADD)) ||
                ((opcode == `OPCODE_COMPUTE) && (funct3 == `FUNC_ADD) && (funct7 == `AUX_FUNC_SUB)) ||
                ((opcode == `OPCODE_COMPUTE) && (funct3 == `FUNC_SLT)) ||
                ((opcode == `OPCODE_COMPUTE) && (funct3 == `FUNC_SLTU)) ||
                ((opcode == `OPCODE_COMPUTE) && (funct3 == `FUNC_XOR)) ||
                ((opcode == `OPCODE_COMPUTE) && (funct3 == `FUNC_OR)) ||
                ((opcode == `OPCODE_COMPUTE) && (funct3 == `FUNC_AND) && (funct7 == 7'b0000000)));

            
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////           
    // 1) FETCH
    // System State (everything is neg assert)
    InstMem IMEM(.Addr(PC), .Size(`SIZE_WORD), .DataOut(InstWord), .CLK(clk));
   
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////           
    // 2) DECODE
    assign opcode = InstWord[6:0];   
    assign Rdst = InstWord[11:7]; 
    assign Rsrc1 = InstWord[19:15]; 
    assign Rsrc2 = InstWord[24:20];
    assign funct3 = InstWord[14:12];  // R-Type, I-Type, S-Type
    assign funct7 = InstWord[31:25];  // R-Type
    assign Iimm12 = InstWord[31:20]; // I-Type
    assign Simm12 = {InstWord[31:25], InstWord[11:7]}; // S-Type
    assign Bimm13 = {InstWord[31], InstWord[7], InstWord[30:25], InstWord[11:8], 1'b0}; // B-Type
    assign Uimm20 = {InstWord[31:12]}; // U-Type
    assign Jimm21 = {InstWord[31], InstWord[19:12], InstWord[20], InstWord[30:21], 1'b0}; // J-Type

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////           
    // 3) CONTROL
    // ControlUnit CUNIT1(.opcode(opcode), .funct3(funct3), .funct7(funct7), .MemSize(MemSize),
    //         .nPC_sel(nPC_sel), .RWrEn(RWrEn), .RegDst(RegDst), .ExtOp(ExtOp), .ALUSrc(ALUSrc), .ALUctr(ALUctr), .MemWrEn(MemWrEn), .MemtoReg(MemtoReg));

    ControlUnit CUNIT1(.opcode(opcode), .funct3(funct3), .funct7(funct7), .MemSize(MemSize),
            .nPC_sel(nPC_sel), .RWrEn(RWrEn), .ExtOp(ExtOp), .ALUSrc(ALUSrc), .ALUctr(ALUctr), .MemWrEn(MemWrEn), .MemtoReg(MemtoReg));
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////           
    // 4) Read Regs a,b from file
    RegFile RF(.AddrA(Rsrc1), .DataOutA(Rdata1), 
            .AddrB(Rsrc2), .DataOutB(Rdata2), 
            .AddrW(Rdst), .DataInW(RWrdata), .WenW(RWrEn), .CLK(clk));

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////           
    // 5) Extender
    Extender EX1(.Iimm12(Iimm12), .Simm12(Simm12), .Bimm13(Bimm13), .Uimm20(Uimm20), .Jimm21(Jimm21), .ExtOp(ExtOp), .opcode(opcode),
            .extended_Iimm(extended_Iimm), .extended_Simm(extended_Simm), .extended_Bimm(extended_Bimm), .extended_Uimm(extended_Uimm), .extended_Jimm(extended_Jimm), .Imm_extended(Imm_extended));
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////           
    // 6) Mux for op2 or immediate
    MuxI MUXI1(.a(Rdata2), .b(Imm_extended), .sel(ALUSrc), .out(ALUop2));

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////           
    // 7) Execute
    ALU ALU1(.a(Rdata1), .b(ALUop2), .ALUctr(ALUctr), .result(ALUresult), .zero(zero), .negative(negative));//, .overflow(overflow));

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////           
    // 8) we have PC, PC+4, and immediate
    AdderPC APC1(.PC(PC), .out(PC_Plus_4));

    AndGate AND1(.a(nPC_sel[0]), .b(zero), .out(BranchAnd));
    OrGate OR1(.a(nPC_sel[1]), .b(BranchAnd), .out(PCSrc));

    AdderPCImm API1(.PC(PC), .Imm(Imm_extended), .PC_Imm(PC_Imm));
    MuxI MUXI2(.a(PC_Plus_4), .b(PC_Imm), .sel(PCSrc), .out(NPC));
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////           
    // 9) Memory
    DataMem DMEM(.Addr(ALUresult), .Size(MemSize), .DataIn(Rdata2), .DataOut(DataWord), .WEN(MemWrEn), .CLK(clk));

    // load, data out is info from mem to be put into rd
    // store, data out does nothing

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////           
    // 10) Update Regs
    MuxI MUXI3(.a(ALUresult), .b(DataWord), .sel(MemtoReg), .out(RWrdata_0));
    MuxI MUXI4(.a(RWrdata_0), .b(PC_Plus_4), .sel(nPC_sel[1]), .out(RWrdata));

    // RegFile RF2(.AddrA(Rsrc1), .DataOutA(Rdata1), 
    //         .AddrB(Rsrc2), .DataOutB(Rdata2), 
    //         .AddrW(Rdst), .DataInW(RWrdata), .WenW(RWrEn), .CLK(clk));   
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////           
    // 11) PC Update
    Reg PC_REG(.Din(NPC), .Qout(PC), .WEN(1'b0), .CLK(clk), .RST(rst));

endmodule // SingleCycleCPU

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////           
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////           
// MODULES

// Control Unit
module ControlUnit(
    input [6:0] opcode,
    input [2:0] funct3,
    input [6:0] funct7,
    output reg [1:0]  MemSize,
    output reg [1:0] nPC_sel,
    output reg RWrEn,
    //output reg RegDst,
    output reg ExtOp,
    output reg ALUSrc,
    output reg [3:0] ALUctr,
    output reg MemWrEn,
    output reg MemtoReg
    );

    // Control signal settings for different instruction types
    always @(*) begin
        // Default control signal values
        MemSize = `SIZE_WORD;
        nPC_sel = 2'b00;
        RWrEn = 1;
        // RegDst = 0;
        ExtOp = 1; // Assume sign extension by default
        ALUSrc = 0;
        ALUctr = `ALU_ADD; // Default ALU operation is ADD
        MemWrEn = 0;
        MemtoReg = 0;

        case (opcode)
            `OPCODE_LUI: begin
                RWrEn = 0;
                ALUSrc = 1; // Source is immediate
                ALUctr = `ALU_NOP; // No operation needed, just load immediate
                // MemtoReg = 0; // Select ALU result
            end
            `OPCODE_AUIPC: begin
                RWrEn = 0;
                ALUSrc = 1; // Source is immediate
                ALUctr = `ALU_ADD; // Add to PC
                // MemtoReg = 0; // Select ALU result
            end
            `OPCODE_JAL: begin
                nPC_sel = 2'b10; // JAL address
                RWrEn = 0;
                // RegDst = 1; // Write to rd
                // ALUctr = `ALU_ADD; // PC + 4
                // MemtoReg = 0; // Select ALU result
            end
            `OPCODE_JALR: begin
                nPC_sel = 2'b11; // JALR address
                RWrEn = 0;
                // RegDst = 1; // Write to rd
                // ALUctr = `ALU_ADD; // PC + 4
                // MemtoReg = 0; // Select ALU result
            end
            `OPCODE_BRANCH: begin
                nPC_sel = 2'b01; // Branch address
                // Select the branch decision control signal based on funct3
                case (funct3)
                    `FUNC_BEQ: ALUctr = `ALU_SUB; // For beq, subtract and check zero
                    `FUNC_BNE: ALUctr = `ALU_SUB; // For bne, subtract and check non-zero
                    `FUNC_BLT: ALUctr = `ALU_SLT; // For blt, set less than
                    `FUNC_BGE: ALUctr = `ALU_SLT; // For bge, set less than and negate
                    `FUNC_BLTU: ALUctr = `ALU_SLTU; // For bltu, set less than unsigned
                    `FUNC_BGEU: ALUctr = `ALU_SLTU; // For bgeu, set less than unsigned and negate
                endcase
                case (funct3)
                    `FUNC_BEQ: ExtOp = 1;
                    `FUNC_BNE: ExtOp = 1;
                    `FUNC_BLT: ExtOp = 1;
                    `FUNC_BGE: ExtOp = 1;
                    `FUNC_BLTU: ExtOp = 0;
                    `FUNC_BGEU: ExtOp = 0;
                endcase
            end
            `OPCODE_LOAD: begin
                RWrEn = 0;
                ALUSrc = 1; // Load uses immediate
                MemtoReg = 1; // Load data from memory
                case (funct3)
                    `FUNC_LB: MemSize = `SIZE_BYTE;
                    `FUNC_LH: MemSize = `SIZE_HWORD;
                    `FUNC_LW: MemSize = `SIZE_WORD;
                    `FUNC_LBU: MemSize = `SIZE_BYTE;
                    `FUNC_LHU: MemSize = `SIZE_HWORD;
                endcase
                case (funct3)
                    `FUNC_LB: ExtOp = 1;
                    `FUNC_LH: ExtOp = 1;
                    `FUNC_LW: ExtOp = 1;
                    `FUNC_LBU: ExtOp = 0;
                    `FUNC_LHU: ExtOp = 0;
                endcase
                // ALU operation is ADD for address calculation
            end
            `OPCODE_STORE: begin
                MemWrEn = 1;
                ALUSrc = 1; // Store uses immediate
                case (funct3)
                    `FUNC_SB: MemSize = `SIZE_BYTE;
                    `FUNC_SH: MemSize = `SIZE_HWORD;
                    `FUNC_SW: MemSize = `SIZE_WORD;
                endcase
                // ALU operation is ADD for address calculation
            end
            `OPCODE_IMMEDIATE: begin
                RWrEn = 0;
                ALUSrc = 1; // I-type uses immediate
                // Decoding the exact ALU operation based on funct3 and funct7
                case (funct3)
                    `FUNC_ADDI: ALUctr = `ALU_ADD;
                    `FUNC_SLTI: ALUctr = `ALU_SLT;
                    `FUNC_XORI: ALUctr = `ALU_XOR;
                    `FUNC_ORI: ALUctr = `ALU_OR;
                    `FUNC_ANDI: ALUctr = `ALU_AND;
                    `FUNC_SLLI: ALUctr = `ALU_SLL;
                    `FUNC_SRLI: ALUctr = (funct7 == `AUX_FUNC_SRAI) ? `ALU_SRA : `ALU_SRL;
                endcase
            end
            `OPCODE_COMPUTE: begin
                RWrEn = 0;
                //RegDst = 1; // R-type uses rd
                // Decoding the exact ALU operation based on funct3 and funct7
                case (funct3)
                    `FUNC_ADD: ALUctr = (funct7 == `AUX_FUNC_SUB) ? `ALU_SUB : `ALU_ADD;
                    `FUNC_SLL: ALUctr = `ALU_SLL;
                    `FUNC_SLT: ALUctr = `ALU_SLT;
                    `FUNC_SLTU: ALUctr = `ALU_SLTU;
                    `FUNC_XOR: ALUctr = `ALU_XOR;
                    `FUNC_SRL: ALUctr = (funct7 == `AUX_FUNC_SRA) ? `ALU_SRA : `ALU_SRL;
                    `FUNC_OR: ALUctr = `ALU_OR;
                    `FUNC_AND: ALUctr = `ALU_AND;
                endcase
            end
            // Add additional cases for other opcodes if necessary
        endcase
    end

endmodule // ControlUnit

// ALU
module ALU(
    input [31:0] a,
    input [31:0] b,
    input [3:0] ALUctr,
    output reg [31:0] result,
    output zero,
    output negative
    // output overflow do we need overflow??
    );

    // Internal signals for signed and unsigned comparisons
    wire signed [31:0] signed_a = a;
    wire signed [31:0] signed_b = b;

    // Detect overflow for addition and subtraction
    // wire overflow_add = (~(a[31] ^ b[31]) & (a[31] ^ result[31]));
    // wire overflow_sub = ((a[31] ^ b[31]) & (a[31] ^ result[31]));

    // Determine the result based on the ALU control signal
    always @(*) begin
        case (ALUctr)
            `ALU_ADD: begin
                result = a + b;
                //overflow = overflow_add;
            end
            `ALU_SUB: begin
                result = a - b;
                //overflow = overflow_sub;
            end
            `ALU_AND: result = a & b;
            `ALU_OR:  result = a | b;
            `ALU_XOR: result = a ^ b;
            `ALU_SLL: result = a << b[4:0]; // only use the lower 5 bits of b
            `ALU_SRL: result = a >> b[4:0]; // only use the lower 5 bits of b
            `ALU_SRA: result = signed_a >>> b[4:0]; // arithmetic shift right
            `ALU_SLT: result = signed_a < signed_b ? 32'b1 : 32'b0;
            `ALU_SLTU: result = a < b ? 32'b1 : 32'b0;
            `ALU_NOP: result = b; // For LUI, just pass the operand through
            default: result = 32'b0; // should not happen
        endcase
    end

    // Zero flag
    assign zero = (result == 32'b0);

    // Negative flag
    assign negative = result[31];

endmodule

// Sign Extend
module Extender(
    input [11:0] Iimm12, Simm12, 
    input [12:0] Bimm13, 
    input [19:0] Uimm20, 
    input [20:0] Jimm21,
    input ExtOp,
    input [6:0] opcode,
    output reg [31:0] extended_Iimm,
    output reg [31:0] extended_Simm,
    output reg [31:0] extended_Bimm,
    output reg [31:0] extended_Uimm,
    output reg [31:0] extended_Jimm,
    output reg [31:0] Imm_extended
    );

    always @(*) begin
        if (ExtOp) begin // Sign extend
            extended_Iimm = {{20{Iimm12[11]}}, Iimm12};
            extended_Simm = {{20{Simm12[11]}}, Simm12};
            extended_Bimm = {{19{Bimm13[12]}}, Bimm13};
            extended_Uimm = {Uimm20, 12'b0};
            extended_Jimm = {{11{Jimm21[20]}}, Jimm21};
        end else begin
            extended_Iimm = {20'b0, Iimm12};
            extended_Simm = {20'b0, Simm12};
            extended_Bimm = {19'b0, Bimm13};
            extended_Uimm = {Uimm20, 12'b0};
            extended_Jimm = {11'b0, Jimm21};
        end
        case (opcode)  
            `OPCODE_LUI: Imm_extended = extended_Uimm;
            `OPCODE_AUIPC: Imm_extended = extended_Uimm;
            `OPCODE_JAL: Imm_extended = extended_Jimm;
            `OPCODE_JALR: Imm_extended = extended_Iimm;
            `OPCODE_BRANCH: Imm_extended = extended_Bimm;
            `OPCODE_LOAD: Imm_extended = extended_Iimm;
            `OPCODE_STORE: Imm_extended = extended_Simm;
            `OPCODE_IMMEDIATE: Imm_extended = extended_Iimm;
            default: Imm_extended = 32'b0;
        endcase
    end
endmodule // Extender

// And gate
module AndGate(
    input a,
    input b,
    output out
    );

    assign out = a & b;
endmodule // AndGate

// Or gate
module OrGate(
    input a,
    input b,
    output out
    );

    assign out = a | b;
endmodule

// Immediate Mux
module MuxI(
    input [31:0] a,
    input [31:0] b,
    input sel,
    output reg [31:0] out
    );

    always @(*) begin
        if (sel) begin
            out = b;
        end else begin
            out = a;
        end
    end
endmodule // MuxI

// Adder for PC
module AdderPC(
    input [31:0] PC,
    output reg [31:0] out
    );

    always @(*) begin
        out = PC + 32'h20;
    end
endmodule

// Adder for PC and immediate
module AdderPCImm(
    input [31:0] PC,
    input [31:0] Imm,
    output reg [31:0] PC_Imm
    );

    always @(*) begin
        PC_Imm = PC + Imm;
    end
endmodule

// Library Modules for Northwestern - CompEng 361 - Lab2

module InstMem(Addr, Size, DataOut, CLK);
   input [31:0] Addr;
   input [1:0] 	Size;
   output [31:0] DataOut;
   reg [31:0] DataOut;   
   input 	CLK;
   reg [7:0] 	Mem[0:1024];
   wire [31:0] 	AddrW;

   // Addresses are word aligned
   assign AddrW = Addr & 32'hfffffffc;

   // Little endian 
   always @ *
     DataOut = {Mem[AddrW+3], Mem[AddrW+2], 
		Mem[AddrW+1], Mem[AddrW]};
      
endmodule // InstMem


module DataMem(Addr, Size, DataIn, DataOut, WEN, CLK);
   input [31:0] Addr;
   input [1:0] 	Size;   
   input [31:0] DataIn;   
   output [31:0] DataOut;
   reg [31:0] DataOut;   
   input      WEN, CLK;
   reg [7:0] 	Mem[0:1024];

   wire [31:0] 	AddrH, AddrW;

   assign AddrH = Addr & 32'hfffffffe;
   assign AddrW = Addr & 32'hfffffffc;

   always @ * 
     DataOut = (Size == 2'b00) ? {4{Mem[Addr]}} :
	       ((Size == 2'b01) ? {2{Mem[AddrH+1],Mem[AddrH]}} :
		{Mem[AddrW+3], Mem[AddrW+2], Mem[AddrW+1], Mem[AddrW]});
   
   always @ (negedge CLK)
     if (!WEN) begin
	case (Size)
	  2'b00: begin // Write byte
	     Mem[Addr] <= DataIn[7:0];
	  end
	  2'b01: begin  // Write halfword
	     Mem[AddrH] <= DataIn[7:0];
	     Mem[AddrH+1] <= DataIn[15:8];
	  end
	  2'b10, 2'b11: begin // Write word
	     Mem[AddrW] <= DataIn[7:0];
	     Mem[AddrW+1] <= DataIn[15:8];
	     Mem[AddrW+2] <= DataIn[23:16];
	     Mem[AddrW+3] <= DataIn[31:24];
	  end
	endcase // case (Size)
     end // if (!WEN)
      
endmodule // DataMem

module RegFile(AddrA, DataOutA,
	       AddrB, DataOutB,
	       AddrW, DataInW, WenW, CLK);
   input [4:0] AddrA, AddrB, AddrW;
   output [31:0] DataOutA, DataOutB;
   reg [31:0] DataOutA, DataOutB;   
   input [31:0]  DataInW;
   input 	 WenW, CLK;
   reg [31:0] 	 Mem[0:31];
   
   always @ * begin
      // Remember that x0 == 0
      DataOutA = (AddrA == 0) ? 32'h00000000 : Mem[AddrA];
      DataOutB = (AddrB == 0) ? 32'h00000000 : Mem[AddrB]; 
   end

   always @ (negedge CLK) begin
     if (!WenW) begin
       Mem[AddrW] <= DataInW;
     end
      Mem[0] <= 0; // Enforce the invariant that x0 = 0
   end
   
endmodule // RegFile


module Reg(Din, Qout, WEN, CLK, RST);
   parameter width = 32;
   input [width-1:0] Din;
   output [width-1:0] Qout;
   input 	      WEN, CLK, RST;

   reg [width-1:0]    Qout;
   
   always @ (negedge CLK or negedge RST)
     if (!RST)
       Qout <= 0;
     else
       if (!WEN)
	 Qout <= Din;
  
endmodule // Reg


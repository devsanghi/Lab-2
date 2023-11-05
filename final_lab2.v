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
`define ALU_ADD  4'b0000;
`define ALU_SUB  4'b0001;
`define ALU_AND   4'b0010;
`define ALU_OR    4'b0011;
`define ALU_XOR   4'b0100;
`define ALU_SLL   4'b0101;
`define ALU_SRL   4'b0110;
`define ALU_SRA   4'b0111;
`define ALU_SLT   4'b1000;
`define ALU_SLTU  4'b1001;
`define ALU_NOP   4'b1111;

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
    wire [31:0] Rdata1, Rdata2, RWrdata;
    wire        RWrEn;

    wire [31:0] NPC, PC_Plus_4;
    wire [6:0]  opcode;

    // needed for instruction decode
    wire [6:0]  funct7;
    wire [2:0]  funct3;
    wire [11:0] Iimm12;
    wire [11:0] Simm12;
    wire [11:0] Bimm13;
    wire [19:0] Uimm20;
    wire [19:0] Jimm21;

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 0) HALT
    // Only support R-TYPE ADD and SUB
    //////// Need to extend for all functions -- do at end
    assign halt = !((opcode == `OPCODE_COMPUTE) && (funct3 == `FUNC_ADD) &&
            ((funct7 == `AUX_FUNC_ADD) || (funct7 == `AUX_FUNC_SUB)));
            
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////           
    // 1) FETCH
    // System State (everything is neg assert)
    InstMem IMEM(.Addr(PC), .Size(`SIZE_WORD), .DataOut(InstWord), .CLK(clk));
    // DataMem DMEM(.Addr(DataAddr), .Size(MemSize), .DataIn(StoreData), .DataOut(DataWord), .WEN(MemWrEn), .CLK(clk));

    // RegFile RF(.AddrA(Rsrc1), .DataOutA(Rdata1), 
    //         .AddrB(Rsrc2), .DataOutB(Rdata2), 
    //         .AddrW(Rdst), .DataInW(RWrdata), .WenW(RWrEn), .CLK(clk));

    Reg PC_REG(.Din(NPC), .Qout(PC), .WEN(1'b0), .CLK(clk), .RST(rst));

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
    assign Uimm20 = {InstWord[31:12], 12'b0}; // U-Type
    assign Jimm21 = {InstWord[31], InstWord[19:12], InstWord[20], InstWord[30:21], 1'b0}; // J-Type

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////           
    // 3) CONTROL
    ControlUnit CUNIT1(.opcode(opcode), .funct3(funct3), .funct7(funct7), .MemSize(MemSize),
            .nPC_sel(nPC_sel), .RegWr(RegWr), .RegDst(RegDst), .ExtOp(ExtOp), .ALUSrc(ALUSrc), .ALUctr(ALUctr), .MemWrEn(MemWrEn), .MemtoReg(MemtoReg));

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////           
    // 4) Read Regs a,b from file
    RegFile RF1(.AddrA(Rsrc1), .DataOutA(Rdata1), 
            .AddrB(Rsrc2), .DataOutB(Rdata2), 
            .AddrW(Rdst), .DataInW(RWrdata), .WenW(RWrEn), .CLK(clk));

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////           
    // 4) Execute
    ALU ALU1(.a(Rdata1), .b(Rdata2), .ALUctr(ALUctr), .result(ALUresult), .zero(zero), .negative(negative), .overflow(overflow));

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////           
    // 5) Memory
    DataMem DMEM(.Addr(ALUresult), .Size(MemSize), .DataIn(StoreData), .DataOut(DataWord), .WEN(MemWrEn), .CLK(clk));

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////           
    // 6) Update Regs
    RegFile RF3()
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////           
    // 7) PC Update




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
    output reg RegWr,
    output reg RegDst,
    output reg ExtOp,
    output reg ALUSrc,
    output reg [3:0] ALUctr,
    output reg MemWrEn,
    output reg MemtoReg
    );

    // Control signal settings for different instruction types
    always @(*) begin
        // Default control signal values
        nPC_sel = 2'b00;
        RegWr = 0;
        RegDst = 0;
        ExtOp = 1; // Assume sign extension by default
        ALUSrc = 0;
        ALUctr = ALU_ADD; // Default ALU operation is ADD
        MemWrEn = 0;
        MemtoReg = 0;

        case (opcode)
            `OPCODE_LUI: begin
                RegWr = 1;
                ALUSrc = 1; // Source is immediate
                ALUctr = ALU_NOP; // No operation needed, just load immediate
                MemtoReg = 0; // Select ALU result
            end
            `OPCODE_AUIPC: begin
                RegWr = 1;
                ALUSrc = 1; // Source is immediate
                ALUctr = ALU_ADD; // Add to PC
                MemtoReg = 0; // Select ALU result
            end
            `OPCODE_JAL: begin
                nPC_sel = 2'b10; // JAL address
                RegWr = 1;
                RegDst = 1; // Write to rd
                ALUctr = ALU_ADD; // PC + 4
                MemtoReg = 0; // Select ALU result
            end
            `OPCODE_JALR: begin
                nPC_sel = 2'b11; // JALR address
                RegWr = 1;
                RegDst = 1; // Write to rd
                ALUctr = ALU_ADD; // PC + 4
                MemtoReg = 0; // Select ALU result
            end
            `OPCODE_BRANCH: begin
                nPC_sel = 2'b01; // Branch address
                // Branch instructions do not write to the register file
                RegWr = 0;
                // Select the branch decision control signal based on funct3
                case (funct3)
                    `FUNC_BEQ: ALUctr = ALU_SUB; // For beq, subtract and check zero
                    `FUNC_BNE: ALUctr = ALU_SUB; // For bne, subtract and check non-zero
                    `FUNC_BLT: ALUctr = ALU_SLT; // For blt, set less than
                    `FUNC_BGE: ALUctr = ALU_SLT; // For bge, set less than and negate
                    `FUNC_BLTU: ALUctr = ALU_SLTU; // For bltu, set less than unsigned
                    `FUNC_BGEU: ALUctr = ALU_SLTU; // For bgeu, set less than unsigned and negate
                endcase
            end
            `OPCODE_LOAD: begin
                RegWr = 1;
                ALUSrc = 1; // Load uses immediate
                MemtoReg = 1; // Load data from memory
                case (funct3)
                    `FUNC_LB: MemSize = `SIZE_BYTE;
                    `FUNC_LH: MemSize = `SIZE_HWORD;
                    `FUNC_LW: MemSize = `SIZE_WORD;
                    `FUNC_LBU: MemSize = `SIZE_BYTE;
                    `FUNC_LHU: MemSize = `SIZE_HWORD;
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
                RegWr = 1;
                ALUSrc = 1; // I-type uses immediate
                // Decoding the exact ALU operation based on funct3 and funct7
                case (funct3)
                    `FUNC_ADDI: ALUctr = ALU_ADD;
                    `FUNC_SLTI: ALUctr = ALU_SLT;
                    `FUNC_XORI: ALUctr = ALU_XOR;
                    `FUNC_ORI: ALUctr = ALU_OR;
                    `FUNC_ANDI: ALUctr = ALU_AND;
                    `FUNC_SLLI: ALUctr = ALU_SLL;
                    `FUNC_SRLI: ALUctr = (funct7 == `AUX_FUNC_SRAI) ? ALU_SRA : ALU_SRL;
                endcase
            end
            `OPCODE_COMPUTE: begin
                RegWr = 1;
                RegDst = 1; // R-type uses rd
                // Decoding the exact ALU operation based on funct3 and funct7
                case (funct3)
                    `FUNC_ADD: ALUctr = (funct7 == `AUX_FUNC_SUB) ? ALU_SUB : ALU_ADD;
                    `FUNC_SLL: ALUctr = ALU_SLL;
                    `FUNC_SLT: ALUctr = ALU_SLT;
                    `FUNC_SLTU: ALUctr = ALU_SLTU;
                    `FUNC_XOR: ALUctr = ALU_XOR;
                    `FUNC_SRL: ALUctr = (funct7 == `AUX_FUNC_SRA) ? ALU_SRA : ALU_SRL;
                    `FUNC_OR: ALUctr = ALU_OR;
                    `FUNC_AND: ALUctr = ALU_AND;
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
    input ALUSrc,
    input [11:0] Iimm12,
    input [11:0] Simm12,
    input [12:0] Bimm13,
    input [19:0] Uimm20,
    input [20:0] Jimm21,
    output reg [31:0] result,
    output zero,
    output negative,
    output overflow
    );

    // Temporary fix until we implement the extop to determine if we sign extend
    wire [31:0] extended_Iimm = {{20{Iimm12[11]}}, Iimm12};
    wire [31:0] extended_Simm = {{20{Simm12[11]}}, Simm12};
    wire [31:0] extended_Bimm = {{19{Bimm13[12]}}, Bimm13, 1'b0};
    wire [31:0] extended_Uimm = {Uimm20, 12'b0};
    wire [31:0] extended_Jimm = {{11{Jimm21[20]}}, Jimm21, 1'b0};

    function [31:0] select_operand2;
        input [3:0] ALUctr;
        input ALUSrc;
        input [31:0] b;
        input [31:0] Iimm;
        input [31:0] Simm;
        input [31:0] Bimm;
        input [31:0] Uimm;
        input [31:0] Jimm;
        begin
            if (ALUSrc) begin
                case (ALUctr)
                    ALU_ADD,
                    ALU_SUB,
                    ALU_SLT,
                    ALU_SLTU: select_operand2 = Iimm; // I-Type operations
                    ALU_LUI:  select_operand2 = Uimm; // U-Type operation for LUI
                    // Add other cases if necessary for different types of immediates
                    default: select_operand2 = 32'b0; // Fallback if no immediate is selected
                endcase
            end else begin
                select_operand2 = b; // Use the value from register b if ALUSrc is 0
            end
        end
    endfunction

    wire [31:0] operand2 = select_operand2(ALUctr, ALUSrc, b, extended_Iimm, extended_Simm, extended_Bimm, extended_Uimm, extended_Jimm);

    // Internal signals for signed and unsigned comparisons
    wire signed [31:0] signed_a = a;
    wire signed [31:0] signed_b = operand2;
    wire [31:0] unsigned_a = a;
    wire [31:0] unsigned_b = operand2;

    // Detect overflow for addition and subtraction
    wire overflow_add = (~(a[31] ^ b[31]) & (a[31] ^ result[31]));
    wire overflow_sub = ((a[31] ^ b[31]) & (a[31] ^ result[31]));

    // Determine the result based on the ALU control signal
    always @(*) begin
        case (ALUctr)
            ALU_ADD: begin
                result = a + b;
                overflow = overflow_add;
            end
            ALU_SUB: begin
                result = a - b;
                overflow = overflow_sub;
            end
            ALU_AND: result = a & operand2;
            ALU_OR:  result = a | operand2;
            ALU_XOR: result = a ^ operand2;
            ALU_SLL: result = a << operand2[4:0]; // only use the lower 5 bits of b
            ALU_SRL: result = a >> operand2[4:0]; // only use the lower 5 bits of b
            ALU_SRA: result = signed_a >>> operand2[4:0]; // arithmetic shift right
            ALU_SLT: result = signed_a < signed_b ? 32'b1 : 32'b0;
            ALU_SLTU: result = unsigned_a < unsigned_b ? 32'b1 : 32'b0;
            ALU_NOP: result = a; // For LUI, just pass the operand through
            default: result = 32'b0; // should not happen
        endcase
    end

    // Zero flag
    assign zero = (result == 32'b0);

    // Negative flag
    assign negative = result[31];

endmodule

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

// Control Unit
module ControlUnit(
    input [6:0] opcode,
    input [2:0] funct3,
    input [6:0] funct7,
    output reg [1:0]  MemSize,
    output reg [1:0] nPC_sel,
    output reg RWrEn,
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
        MemSize = `SIZE_WORD; ///////////////////////////////////////////
        nPC_sel = 2'b00;
        RWrEn = 0;
        RegDst = 0;
        ExtOp = 1; // Assume sign extension by default
        ALUSrc = 0;
        ALUctr = `ALU_ADD; // Default ALU operation is ADD
        MemWrEn = 0;
        MemtoReg = 0;

        case (opcode)
            `OPCODE_LUI: begin
                RWrEn = 1;
                ALUSrc = 1; // Source is immediate
                ALUctr = `ALU_NOP; // No operation needed, just load immediate
                MemtoReg = 0; // Select ALU result
            end
            `OPCODE_AUIPC: begin
                RWrEn = 1;
                ALUSrc = 1; // Source is immediate
                ALUctr = `ALU_ADD; // Add to PC
                MemtoReg = 0; // Select ALU result
            end
            `OPCODE_JAL: begin
                nPC_sel = 2'b10; // JAL address
                RWrEn = 1;
                RegDst = 1; // Write to rd
                ALUctr = `ALU_ADD; // PC + 4
                MemtoReg = 0; // Select ALU result
            end
            `OPCODE_JALR: begin
                nPC_sel = 2'b11; // JALR address
                RWrEn = 1;
                RegDst = 1; // Write to rd
                ALUctr = `ALU_ADD; // PC + 4
                MemtoReg = 0; // Select ALU result
            end
            `OPCODE_BRANCH: begin
                nPC_sel = 2'b01; // Branch address
                // Branch instructions do not write to the register file
                RWrEn = 0; //////////////////// delete
                // Select the branch decision control signal based on funct3
                case (funct3)
                    `FUNC_BEQ: ALUctr = `ALU_SUB; // For beq, subtract and check zero
                    `FUNC_BNE: ALUctr = `ALU_SUB; // For bne, subtract and check non-zero
                    `FUNC_BLT: ALUctr = `ALU_SLT; // For blt, set less than
                    `FUNC_BGE: ALUctr = `ALU_SLT; // For bge, set less than and negate
                    `FUNC_BLTU: ALUctr = `ALU_SLTU; // For bltu, set less than unsigned
                    `FUNC_BGEU: ALUctr = `ALU_SLTU; // For bgeu, set less than unsigned and negate
                endcase
            end
            `OPCODE_LOAD: begin
                RWrEn = 1;
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
                RWrEn = 1;
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
                RWrEn = 1;
                RegDst = 1; // R-type uses rd
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
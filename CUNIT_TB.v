`timescale 1ns / 1ps

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

module ControlUnit_tb;

    // Inputs
    reg [6:0] opcode;
    reg [2:0] funct3;
    reg [6:0] funct7;

    // Outputs
    wire [1:0] MemSize;
    wire [1:0] nPC_sel;
    wire RWrEn;
    wire RegDst;
    wire ExtOp;
    wire ALUSrc;
    wire [3:0] ALUctr;
    wire MemWrEn;
    wire MemtoReg;

    // Test vector signals
    reg [1:0] expected_MemSize [0:9];
    reg [1:0] expected_nPC_sel [0:9];
    reg expected_RWrEn [0:9];
    reg expected_RegDst [0:9];
    reg expected_ExtOp [0:9];
    reg expected_ALUSrc [0:9];
    reg [3:0] expected_ALUctr [0:9];
    reg expected_MemWrEn [0:9];
    reg expected_MemtoReg [0:9];

    integer i;
    integer count;

    // Instantiate the Unit Under Test (UUT)
    ControlUnit uut (
        .opcode(opcode), 
        .funct3(funct3), 
        .funct7(funct7), 
        .MemSize(MemSize), 
        .nPC_sel(nPC_sel), 
        .RWrEn(RWrEn), 
        .RegDst(RegDst), 
        .ExtOp(ExtOp), 
        .ALUSrc(ALUSrc), 
        .ALUctr(ALUctr), 
        .MemWrEn(MemWrEn), 
        .MemtoReg(MemtoReg)
    );

    initial begin
        // Initialize the test vectors with expected values
        // ADD
        expected_MemSize[0] = 2'b00;
        expected_nPC_sel[0] = 2'b00;
        expected_RWrEn[0] = 1;
        expected_RegDst[0] = 1;
        expected_ExtOp[0] = 0;
        expected_ALUSrc[0] = 0;
        expected_ALUctr[0] = `ALU_ADD;
        expected_MemWrEn[0] = 0;
        expected_MemtoReg[0] = 0;
        // SUB
        expected_MemSize[1] = 2'b00;
        expected_nPC_sel[1] = 2'b00;
        expected_RWrEn[1] = 1;
        expected_RegDst[1] = 1;
        expected_ExtOp[1] = 0;
        expected_ALUSrc[1] = 0;
        expected_ALUctr[1] = `ALU_SUB;
        expected_MemWrEn[1] = 0;
        expected_MemtoReg[1] = 0;
        // XOR
        expected_MemSize[2] = 2'b00
        expected_nPC_sel[2] = 2'b00
        expected_RWrEn[2] = 1
        expected_RegDst[2] = 1
        expected_ExtOp[2] = 0
        expected_ALUSrc[2] = 0
        expected_ALUctr[2] = `ALU_XOR
        expected_MemWrEn[2] = 0
        expected_MemtoReg[2] = 0
        // OR
        expected_MemSize[3] = 2'b00
        expected_nPC_sel[3] = 2'b00
        expected_RWrEn[3] = 1
        expected_RegDst[3] = 1
        expected_ExtOp[3] = 0
        expected_ALUSrc[3] = 0
        expected_ALUctr[3] = `ALU_OR
        expected_MemWrEn[3] = 0
        expected_MemtoReg[3] = 0
        

        


        // Define opcode, funct3, and funct7 for each test vector
        inp_opcode[3:0]={`OPCODE_COMPUTE,`OPCODE_COMPUTE,`OPCODE_COMPUTE,`OPCODE_COMPUTE}//,`OPCODE_COMPUTE,`OPCODE_COMPUTE,`OPCODE_COMPUTE,`OPCODE_COMPUTE,`OPCODE_COMPUTE,`OPCODE_COMPUTE}
        inp_funct3[3:0]={`FUNC_ADD,`FUNC_ADD,`FUNC_XOR,`FUNC_OR}//,`FUNC_AND,`FUNC_SLL,`FUNC_SRL,`FUNC_SRA,`FUNC_SLT,`FUNC_SLTU}
        inp_funct7[3:0]={`AUX_FUNC_ADD,`AUX_FUNC_SUB,7'b0000000,7'b0000000}//,7'b0000000,7'b0000000,`AUX_FUNC_SRL,`AUX_FUNC_SRA,7'b0000000,7'b0000000}

        // Initialize count
        count = 0;

        // Apply test vectors
        for (i = 0; i < 10; i = i + 1) begin
            // Set the opcode, funct3, and funct7 to the values for the current test
            // ...
            opcode = inp_opcode[i];
            funct3 = inp_funct3[i];
            funct7 = inp_funct7[i];

            #10; // Wait for output to settle

            $display("Test #%d", count);

            // Check all outputs against expected values
            if ((MemSize !== expected_MemSize[i]) || 
                (nPC_sel !== expected_nPC_sel[i]) ||
                (RWrEn !== expected_RWrEn[i]) ||
                // ... Check all other signals
                ) begin
                $display("Test failed for vector %d", i);
                // ... You can print out the specifics of what failed
            end else begin
                $display("Test passed for vector %d", i);
            end
            count = count + 1;
        end

        $finish; // End simulation
    end

endmodule

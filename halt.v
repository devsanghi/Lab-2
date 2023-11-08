    assign halt = !((opcode == `OPCODE_LUI) ||
        (opcode == `OPCODE_AUIPC) ||
        // for jal, check that the effective jump address is a multiple of 4 and that it's within [0, 1024]
        ((opcode == `OPCODE_JAL) && ($signed(PC + extended_Jimm) % 4 == 0) && ($signed(PC + extended_Jimm) >= 0) && ($signed(PC + extended_Jimm) <= 16'h0400)) || 
        // for jalr, check that effective jump address is a multiple of 4 and that it's within [0, 1024]
        ((opcode == `OPCODE_JALR) && ($signed(Rdata1 + extended_Iimm) % 4 == 0) && ($signed(Rdata1 + extended_Iimm) >= 0) && ($signed(Rdata1 + extended_Iimm) <= 16'h0400)) || 
        // for branching instructions, check that branching address is a multiple of 4 and that it's within [0, 1024]
        ((opcode == `OPCODE_BRANCH) && ($signed(PC + extended_Bimm) % 4 == 0) && ($signed(PC + extended_Bimm) >= 0) && ($signed(PC + extended_Bimm) <= 16'h0400) && (funct3 == `FUNC_BEQ || funct3 == `FUNC_BGE || funct3 == `FUNC_BGEU || funct3 == `FUNC_BLT || funct3 == `FUNC_BLTU || funct3 == `FUNC_BNE)) ||
        ((opcode == `OPCODE_LOAD) && (funct3 == `FUNC_LB) && ($signed(Rdata1 + extended_Iimm) >= 0) && ($signed(Rdata1 + extended_Iimm) <= 16'h0400)) ||
        // for lh or lhu, check that load address is a multiple of 2 and that it's within [0, 1024]
        ((opcode == `OPCODE_LOAD) && (funct3 == `FUNC_LH || funct3 == `FUNC_LHU) && ($signed(Rdata1 + extended_Iimm) % 2 == 0) && ($signed(Rdata1 + extended_Iimm) >= 0) && ($signed(Rdata1 + extended_Iimm) <= 16'h0400)) ||
        // for lw, check that load address is a multiple of 4 and that it's within [0, 1024]
        ((opcode == `OPCODE_LOAD) && (funct3 == `FUNC_LW) && ($signed(Rdata1 + extended_Iimm) % 4 == 0) && ($signed(Rdata1 + extended_Iimm) >= 0) && ($signed(Rdata1 + extended_Iimm) <= 16'h0400)) ||
        ((opcode == `OPCODE_LOAD) && (funct3 == `FUNC_LBU) && ($signed(Rdata1 + extended_Simm) >= 0) && ($signed(Rdata1 + extended_Simm) <= 16'h0400)) || 
        ((opcode == `OPCODE_STORE) && (funct3 == `FUNC_SB) && ($signed(Rdata1 + extended_Simm) >= 0) && ($signed(Rdata1 + extended_Simm) <= 16'h0400)) ||
        // for sh, check that the store address is a multiple of 2 and that it's within [0, 1024]
        ((opcode == `OPCODE_STORE) && (funct3 == `FUNC_SH) && ($signed(Rdata1 + extended_Simm) % 2 == 0) && ($signed(Rdata1 + extended_Simm) >= 0) && ($signed(Rdata1 + extended_Simm) <= 16'h0400)) ||
        // for sw, check that the store address is a multiple of 4 and that it's within [0, 1024]
        ((opcode == `OPCODE_STORE) && (funct3 == `FUNC_SW) && ($signed(Rdata1 + extended_Simm) % 4 == 0) && ($signed(Rdata1 + extended_Simm) >= 0) && ($signed(Rdata1 + extended_Simm) <= 16'h0400)) ||
        ((opcode == `OPCODE_IMMEDIATE) && (funct3 == `FUNC_ADDI || funct3 == `FUNC_SLTI || funct3 == FUNC_XORI || funct3 == `FUNC_ORI || funct3 == `FUNC_ANDI)) ||
        // shift instructions slli, srli, and srai all also have a funct7 encoded in the upper 7 bits of the 12-bit immediate
        ((opcode == `OPCODE_IMMEDIATE) && (funct3 == `FUNC_SLLI) && (funct7 == 7'b0000000)) ||
        ((opcode == `OPCODE_IMMEDIATE) && (funct3 == `FUNC_SRLI) && (funct7 == `AUX_FUNC_SRLI)) ||
        ((opcode == `OPCODE_IMMEDIATE) && (funct3 == `FUNC_SRLI) && (funct7 == `AUX_FUNC_SRAI)) ||
        // all register compute instructions also have a funct7 input
        ((opcode == `OPCODE_COMPUTE) && (funct3 == `FUNC_ADD) && (funct7 == `AUX_FUNC_ADD)) ||
        ((opcode == `OPCODE_COMPUTE) && (funct3 == `FUNC_ADD) && (funct7 == `AUX_FUNC_SUB)) ||
        ((opcode == `OPCODE_COMPUTE) && (funct3 == `FUNC_SLL) && (funct7 == 7'b0000000)) ||
        ((opcode == `OPCODE_COMPUTE) && (funct3 == `FUNC_SLT) && (funct7 == 7'b0000000)) ||
        ((opcode == `OPCODE_COMPUTE) && (funct3 == `FUNC_SLTU) && (funct7 == 7'b0000000)) ||
        ((opcode == `OPCODE_COMPUTE) && (funct3 == `FUNC_XOR) && (funct7 == 7'b0000000)) ||
        ((opcode == `OPCODE_COMPUTE) && (funct3 == `FUNC_SRL) && (funct7 == `AUX_FUNC_SRL)) ||
        ((opcode == `OPCODE_COMPUTE) && (funct3 == `FUNC_SRL) && (funct7 == `AUX_FUNC_SRA)) ||
        ((opcode == `OPCODE_COMPUTE) && (funct3 == `FUNC_OR) && (funct7 == 7'b0000000)) ||
        ((opcode == `OPCODE_COMPUTE) && (funct3 == `FUNC_AND) && (funct7 == 7'b0000000)));
// ALU

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
`timescale 1ns / 1ps

module ALU_tb;

    reg [63:0] a, b;
    reg [3:0] ALUctr;
    wire [63:0] result;
    wire zero, negative;

    // Instantiate the ALU module
    ALU alu_instance (.a(a), .b(b), .ALUctr(ALUctr), .result(result), .zero(zero), .negative(negative));

    // File handle
    integer file, r;
    // Error flag
    integer eof;

    // Test case storage
    reg [3:0] test_ALUctr;
    reg [63:0] test_opA, test_opB;
    reg [63:0] test_expected_result;

    initial begin
        // Open file with test vectors
        file = $fopen("execute-trace.txt", "r");
        if (file == 0) begin
            $display("Testbench Error: could not open test vectors file.");
            $finish;
        end

        // Read and apply test vectors
        while (!$feof(file)) begin
            eof = $fscanf(file, "%b\t%h\t%h\t%h\n", test_ALUctr, test_opA, test_opB, test_expected_result);
            if (eof > 0) begin
                ALUctr = test_ALUctr;
                a = test_opA;
                b = test_opB;
                #10; // Wait for ALU to process
                // Check the result and flags
                if ((result !== test_expected_result) || (zero !== (result == 0)) || (negative !== result[63])) begin
                    $display("Test failed: ALUctr=%b, opA=%h, opB=%h, expected=%h, got=%h", test_ALUctr, test_opA, test_opB, test_expected_result, result);
                end else begin
                    $display("Test passed: ALUctr=%b, opA=%h, opB=%h", test_ALUctr, test_opA, test_opB);
                end
                //#10; // Wait time between tests
            end
        end

        $fclose(file);
        $finish; // End simulation
    end

endmodule

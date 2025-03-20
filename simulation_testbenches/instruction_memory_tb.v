`timescale 1ns/1ps

module tb_instr_memory;
    // Testbench signals
    reg [31:0] addr;
    wire [31:0] data;

    // Expected instruction data (matches the mem in the DUT)
    reg [31:0] expected_instructions [0:63];
    integer i;

    // Instantiate the instruction memory
    instr_memory uut (
        .addr(addr),
        .data(data)
    );

    // Initialize expected instructions (same as in the DUT)
    initial begin
        expected_instructions[0]  = 32'h005302b3; // ADD x5, x6, x5 (R-type)
        expected_instructions[1]  = 32'h00510293; // ADDI x5, x2, 5 (I-type)
        expected_instructions[2]  = 32'h0000a283; // LW x5, 0(x1) (Load)
        expected_instructions[3]  = 32'h0050a223; // SW x5, 0(x1) (Store)
        expected_instructions[4]  = 32'h00528463; // BEQ x5, x5, 8 (Branch)
        expected_instructions[5]  = 32'h00000013; // NOP
        expected_instructions[6]  = 32'h008000ef; // JAL x1, 8 (Jump)
        expected_instructions[7]  = 32'h00000013; // NOP
        expected_instructions[8]  = 32'h000080e7; // JALR x1, 0(x1) (Jump)
        expected_instructions[9]  = 32'h000052b7; // LUI x5, 5 (Load Upper Immediate)
        expected_instructions[10] = 32'h00005297; // AUIPC x5, 5 (Add Upper Immediate to PC)
        for (i = 11; i < 64; i = i + 1) begin
            expected_instructions[i] = 32'h00000013; // NOP
        end
    end

    // Test variables
    integer errors = 0;
    integer test_count = 0;
    reg [8*50:1] test_name;

    // Test stimulus
    initial begin
        $dumpfile("tb_instr_memory.vcd");
        $dumpvars(0, tb_instr_memory);

        // Initialize inputs
        addr = 32'h0;

        // Test Case 1: Fetch instruction at address 0 (ADD)
        $display("Test Case 1: Fetch Address 0 (ADD)");
        addr = 32'h0;
        #10;
        check_outputs(expected_instructions[0], "Fetch Address 0 (ADD)");
        test_count = test_count + 1;

        // Test Case 2: Fetch instruction at address 4 (ADDI)
        $display("Test Case 2: Fetch Address 4 (ADDI)");
        addr = 32'h4;
        #10;
        check_outputs(expected_instructions[1], "Fetch Address 4 (ADDI)");
        test_count = test_count + 1;

        // Test Case 3: Fetch instruction at address 8 (LW)
        $display("Test Case 3: Fetch Address 8 (LW)");
        addr = 32'h8;
        #10;
        check_outputs(expected_instructions[2], "Fetch Address 8 (LW)");
        test_count = test_count + 1;

        // Test Case 4: Fetch instruction at address 16 (BEQ)
        $display("Test Case 4: Fetch Address 16 (BEQ)");
        addr = 32'h10;
        #10;
        check_outputs(expected_instructions[4], "Fetch Address 16 (BEQ)");
        test_count = test_count + 1;

        // Test Case 5: Fetch instruction at address 24 (JAL)
        $display("Test Case 5: Fetch Address 24 (JAL)");
        addr = 32'h18;
        #10;
        check_outputs(expected_instructions[6], "Fetch Address 24 (JAL)");
        test_count = test_count + 1;

        // Test Case 6: Fetch instruction at address 32 (JALR)
        $display("Test Case 6: Fetch Address 32 (JALR)");
        addr = 32'h20;
        #10;
        check_outputs(expected_instructions[8], "Fetch Address 32 (JALR)");
        test_count = test_count + 1;

        // Test Case 7: Fetch instruction at address 40 (AUIPC)
        $display("Test Case 7: Fetch Address 40 (AUIPC)");
        addr = 32'h28;
        #10;
        check_outputs(expected_instructions[10], "Fetch Address 40 (AUIPC)");
        test_count = test_count + 1;

        // Test Case 8: Fetch NOP at address 44
        $display("Test Case 8: Fetch Address 44 (NOP)");
        addr = 32'h2C;
        #10;
        check_outputs(expected_instructions[11], "Fetch Address 44 (NOP)");
        test_count = test_count + 1;

        // Test Case 9: Fetch at boundary address 252
        $display("Test Case 9: Fetch Address 252 (NOP)");
        addr = 32'hFC;
        #10;
        check_outputs(expected_instructions[63], "Fetch Address 252 (NOP)");
        test_count = test_count + 1;

        // Summary
        $display("Test Summary: %0d tests run, %0d errors found", test_count, errors);
        if (errors == 0)
            $display("All tests passed successfully!");
        else
            $display("Test failed with %0d errors.", errors);

        $finish;
    end

    // Task to check outputs against expected values
    task check_outputs;
        input [31:0] exp_data;
        input [8*50:1] test_name;
        begin
            if (data !== exp_data) begin
                $display("ERROR in %0s: data = %h, expected %h", test_name, data, exp_data);
                errors = errors + 1;
            end
        end
    endtask
endmodule
`timescale 1ns/1ps

module tb_execute;
    // Testbench signals
    reg clk;
    reg rst;
    reg mispredict_flush;
    reg instruction_valid;
    reg [4:0] alu_control;
    reg [31:0] opa_mux_in;
    reg [31:0] opb_mux_in;
    reg [31:0] pc_address;
    reg [1:0] forward_a;
    reg [1:0] forward_b;
    reg [31:0] ex_data;
    reg [31:0] mem_data;
    wire [31:0] branch_target;
    wire branch_taken;
    wire [31:0] alu_out_address;

    // Instantiate the execute module
    execute uut (
        .clk(clk),
        .rst(rst),
        .mispredict_flush(mispredict_flush),
        .instruction_valid(instruction_valid),
        .alu_control(alu_control),
        .opa_mux_in(opa_mux_in),
        .opb_mux_in(opb_mux_in),
        .pc_address(pc_address),
        .branch_target(branch_target),
        .branch_taken(branch_taken),
        .forward_a(forward_a),
        .forward_b(forward_b),
        .ex_data(ex_data),
        .mem_data(mem_data),
        .alu_out_address(alu_out_address)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 10 ns clock period (100 MHz)
    end

    // Test variables
    integer errors = 0;
    integer test_count = 0;
    reg [8*50:1] test_name;

    // Test stimulus
    initial begin
        $dumpfile("tb_execute.vcd");
        $dumpvars(0, tb_execute);

        // Initialize inputs
        rst = 0;
        mispredict_flush = 0;
        instruction_valid = 0;
        alu_control = 5'b0;
        opa_mux_in = 32'h0;
        opb_mux_in = 32'h0;
        pc_address = 32'h0;
        forward_a = 2'b00;
        forward_b = 2'b00;
        ex_data = 32'h0;
        mem_data = 32'h0;

        // Test Case 1: Reset behavior
        $display("Test Case 1: Reset Behavior");
        rst = 1;
        #20;
        rst = 0;
        #10;
        check_outputs(32'h0, 32'h0, 0, "Reset Behavior");
        test_count = test_count + 1;

        // Test Case 2: Invalid instruction (instruction_valid = 0)
        $display("Test Case 2: Invalid Instruction");
        instruction_valid = 0;
        alu_control = 5'b00000; // ADD
        opa_mux_in = 32'h00000005;
        opb_mux_in = 32'h00000003;
        pc_address = 32'h00001000;
        forward_a = 2'b00;
        forward_b = 2'b00;
        #10;
        check_outputs(32'h0, 32'h0, 0, "Invalid Instruction");
        test_count = test_count + 1;

        // Test Case 3: Mispredict flush
        $display("Test Case 3: Mispredict Flush");
        instruction_valid = 1;
        mispredict_flush = 1;
        #10;
        mispredict_flush = 0;
        check_outputs(32'h0, 32'h0, 0, "Mispredict Flush");
        test_count = test_count + 1;

        // Test Case 4: ADD operation (no forwarding)
        $display("Test Case 4: ADD Operation (No Forwarding)");
        instruction_valid = 1;
        alu_control = 5'b00000;
        opa_mux_in = 32'h00000005;
        opb_mux_in = 32'h00000003;
        pc_address = 32'h00001000;
        forward_a = 2'b00;
        forward_b = 2'b00;
        #10;
        check_outputs(32'h00000008, 32'h00001003, 0, "ADD Operation (No Forwarding)");
        test_count = test_count + 1;

        // Test Case 5: SUB operation (forward from EX)
        $display("Test Case 5: SUB Operation (Forward from EX)");
        alu_control = 5'b01000;
        opa_mux_in = 32'h0000000A;
        opb_mux_in = 32'h0000000B;
        forward_a = 2'b01;
        forward_b = 2'b01;
        ex_data = 32'h00000005;
        mem_data = 32'h00000000;
        #10;
        check_outputs(32'h00000000, 32'h00001005, 0, "SUB Operation (Forward from EX)");
        test_count = test_count + 1;

        // Test Case 6: SLL operation (forward from MEM)
        $display("Test Case 6: SLL Operation (Forward from MEM)");
        alu_control = 5'b00001;
        opa_mux_in = 32'h0000000A;
        opb_mux_in = 32'h0000000B;
        forward_a = 2'b10;
        forward_b = 2'b10;
        ex_data = 32'h00000000;
        mem_data = 32'h00000002;
        #10;
        check_outputs(32'h00000008, 32'h00001002, 0, "SLL Operation (Forward from MEM)");
        test_count = test_count + 1;

        // Test Case 7: SLT operation (signed comparison)
        $display("Test Case 7: SLT Operation (Signed)");
        alu_control = 5'b00010;
        opa_mux_in = 32'hFFFFFFFE; // -2
        opb_mux_in = 32'h00000001; // 1
        forward_a = 2'b00;
        forward_b = 2'b00;
        #10;
        check_outputs(32'h00000001, 32'h00001001, 0, "SLT Operation (Signed)");
        test_count = test_count + 1;

        // Test Case 8: SLTU operation (unsigned comparison)
        $display("Test Case 8: SLTU Operation (Unsigned)");
        alu_control = 5'b00011;
        opa_mux_in = 32'hFFFFFFFE; // 4294967294
        opb_mux_in = 32'h00000001; // 1
        #10;
        check_outputs(32'h00000000, 32'h00001001, 0, "SLTU Operation (Unsigned)");
        test_count = test_count + 1;

        // Test Case 9: XOR operation
        $display("Test Case 9: XOR Operation");
        alu_control = 5'b00100;
        opa_mux_in = 32'h0000000F;
        opb_mux_in = 32'h00000005;
        #10;
        check_outputs(32'h0000000A, 32'h00001005, 0, "XOR Operation");
        test_count = test_count + 1;

        // Test Case 10: SRL operation
        $display("Test Case 10: SRL Operation");
        alu_control = 5'b00101;
        opa_mux_in = 32'h00000014;
        opb_mux_in = 32'h00000002;
        #10;
        check_outputs(32'h00000005, 32'h00001002, 0, "SRL Operation");
        test_count = test_count + 1;

        // Test Case 11: SRA operation
        $display("Test Case 11: SRA Operation");
        alu_control = 5'b01101;
        opa_mux_in = 32'hFFFFFFFE; // -2
        opb_mux_in = 32'h00000001;
        #10;
        check_outputs(32'hFFFFFFFF, 32'h00001001, 0, "SRA Operation");
        test_count = test_count + 1;

        // Test Case 12: OR operation
        $display("Test Case 12: OR Operation");
        alu_control = 5'b00110;
        opa_mux_in = 32'h0000000F;
        opb_mux_in = 32'h00000005;
        #10;
        check_outputs(32'h0000000F, 32'h00001005, 0, "OR Operation");
        test_count = test_count + 1;

        // Test Case 13: AND operation
        $display("Test Case 13: AND Operation");
        alu_control = 5'b00111;
        opa_mux_in = 32'h0000000F;
        opb_mux_in = 32'h00000005;
        #10;
        check_outputs(32'h00000005, 32'h00001005, 0, "AND Operation");
        test_count = test_count + 1;

        // Test Case 14: Invalid ALU control (default to ADD)
        $display("Test Case 14: Invalid ALU Control");
        alu_control = 5'b11111; // Invalid
        opa_mux_in = 32'h00000005;
        opb_mux_in = 32'h00000003;
        #10;
        check_outputs(32'h00000008, 32'h00001003, 0, "Invalid ALU Control");
        test_count = test_count + 1;

        // Test Case 15: Branch target with large immediate
        $display("Test Case 15: Branch Target with Large Immediate");
        alu_control = 5'b00000;
        opa_mux_in = 32'h00000000;
        opb_mux_in = 32'hFFFFF000; // Large negative immediate
        pc_address = 32'h00002000;
        #10;
        check_outputs(32'hFFFFF000, 32'h00001000, 0, "Branch Target with Large Immediate");
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
        input [31:0] exp_alu_out_address;
        input [31:0] exp_branch_target;
        input exp_branch_taken;
        input [8*50:1] test_name;
        begin
            if (alu_out_address !== exp_alu_out_address) begin
                $display("ERROR in %0s: alu_out_address = %h, expected %h", test_name, alu_out_address, exp_alu_out_address);
                errors = errors + 1;
            end
            if (branch_target !== exp_branch_target) begin
                $display("ERROR in %0s: branch_target = %h, expected %h", test_name, branch_target, exp_branch_target);
                errors = errors + 1;
            end
            if (branch_taken !== exp_branch_taken) begin
                $display("ERROR in %0s: branch_taken = %b, expected %b", test_name, branch_taken, exp_branch_taken);
                errors = errors + 1;
            end
        end
    endtask
endmodule
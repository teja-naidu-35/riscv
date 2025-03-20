`timescale 1ns/1ps

module tb_controlunit;
    // Testbench signals
    reg [6:0] opcode;
    reg [2:0] fun3;
    reg [6:0] fun7;
    reg valid;
    reg [31:0] op_a;
    reg [31:0] op_b;
    wire reg_write;
    wire load;
    wire store;
    wire jalr;
    wire branch_result;
    wire next_sel;
    wire [2:0] imm_sel;
    wire [1:0] mem_to_reg;
    wire [4:0] alu_control;
    wire [3:0] mem_mask;

    // Instantiate the controlunit
    controlunit uut (
        .opcode(opcode),
        .fun3(fun3),
        .fun7(fun7),
        .valid(valid),
        .reg_write(reg_write),
        .load(load),
        .store(store),
        .jalr(jalr),
        .branch_result(branch_result),
        .next_sel(next_sel),
        .imm_sel(imm_sel),
        .mem_to_reg(mem_to_reg),
        .alu_control(alu_control),
        .mem_mask(mem_mask),
        .op_a(op_a),
        .op_b(op_b)
    );

    // Test variables
    integer errors = 0;
    integer test_count = 0;
    reg [8*50:1] test_name;

    // Test stimulus
    initial begin
        $dumpfile("tb_controlunit.vcd");
        $dumpvars(0, tb_controlunit);

        // Initialize inputs
        opcode = 7'b0;
        fun3 = 3'b0;
        fun7 = 7'b0;
        valid = 0;
        op_a = 32'h0;
        op_b = 32'h0;

        // Test Case 1: Invalid input (valid = 0)
        $display("Test Case 1: Invalid Input (valid = 0)");
        valid = 0;
        opcode = 7'b0110011; // R-type
        fun3 = 3'b000;
        fun7 = 7'b0000000;
        #10;
        check_outputs(0, 0, 0, 0, 0, 0, 3'b000, 2'b00, 5'b00000, 4'b0000, "Invalid Input");
        test_count = test_count + 1;

        // Test Case 2: R-type ADD
        $display("Test Case 2: R-type ADD");
        valid = 1;
        opcode = 7'b0110011;
        fun3 = 3'b000;
        fun7 = 7'b0000000;
        #10;
        check_outputs(1, 0, 0, 0, 0, 0, 3'b000, 2'b00, 5'b00000, 4'b0000, "R-type ADD");
        test_count = test_count + 1;

        // Test Case 3: R-type SUB
        $display("Test Case 3: R-type SUB");
        opcode = 7'b0110011;
        fun3 = 3'b000;
        fun7 = 7'b0100000;
        #10;
        check_outputs(1, 0, 0, 0, 0, 0, 3'b000, 2'b00, 5'b01000, 4'b0000, "R-type SUB");
        test_count = test_count + 1;

        // Test Case 4: I-type ADDI
        $display("Test Case 4: I-type ADDI");
        opcode = 7'b0010011;
        fun3 = 3'b000;
        fun7 = 7'b0000000;
        #10;
        check_outputs(1, 0, 0, 0, 0, 0, 3'b000, 2'b00, 5'b00000, 4'b0000, "I-type ADDI");
        test_count = test_count + 1;

        // Test Case 5: Load LW
        $display("Test Case 5: Load LW");
        opcode = 7'b0000011;
        fun3 = 3'b010; // LW
        fun7 = 7'b0000000;
        #10;
        check_outputs(1, 1, 0, 0, 0, 0, 3'b000, 2'b01, 5'b00000, 4'b1111, "Load LW");
        test_count = test_count + 1;

        // Test Case 6: Load LH
        $display("Test Case 6: Load LH");
        opcode = 7'b0000011;
        fun3 = 3'b001; // LH
        #10;
        check_outputs(1, 1, 0, 0, 0, 0, 3'b000, 2'b01, 5'b00000, 4'b0011, "Load LH");
        test_count = test_count + 1;

        // Test Case 7: Store SW
        $display("Test Case 7: Store SW");
        opcode = 7'b0100011;
        fun3 = 3'b010; // SW
        #10;
        check_outputs(0, 0, 1, 0, 0, 0, 3'b001, 2'b00, 5'b00000, 4'b1111, "Store SW");
        test_count = test_count + 1;

        // Test Case 8: Branch BEQ (taken)
        $display("Test Case 8: Branch BEQ (taken)");
        opcode = 7'b1100011;
        fun3 = 3'b000; // BEQ
        op_a = 32'h00000005;
        op_b = 32'h00000005;
        #10;
        check_outputs(0, 0, 0, 0, 1, 0, 3'b010, 2'b00, 5'b00000, 4'b0000, "Branch BEQ (taken)");
        test_count = test_count + 1;

        // Test Case 9: Branch BNE (not taken)
        $display("Test Case 9: Branch BNE (not taken)");
        opcode = 7'b1100011;
        fun3 = 3'b001; // BNE
        op_a = 32'h00000005;
        op_b = 32'h00000005;
        #10;
        check_outputs(0, 0, 0, 0, 0, 0, 3'b010, 2'b00, 5'b00000, 4'b0000, "Branch BNE (not taken)");
        test_count = test_count + 1;

        // Test Case 10: JAL
        $display("Test Case 10: JAL");
        opcode = 7'b1101111;
        fun3 = 3'b000;
        fun7 = 7'b0000000;
        #10;
        check_outputs(1, 0, 0, 0, 0, 1, 3'b011, 2'b10, 5'b00000, 4'b0000, "JAL");
        test_count = test_count + 1;

        // Test Case 11: JALR
        $display("Test Case 11: JALR");
        opcode = 7'b1100111;
        fun3 = 3'b000;
        fun7 = 7'b0000000;
        #10;
        check_outputs(1, 0, 0, 1, 0, 1, 3'b000, 2'b10, 5'b00000, 4'b0000, "JALR");
        test_count = test_count + 1;

        // Test Case 12: LUI
        $display("Test Case 12: LUI");
        opcode = 7'b0110111;
        fun3 = 3'b000;
        fun7 = 7'b0000000;
        #10;
        check_outputs(1, 0, 0, 0, 0, 0, 3'b100, 2'b00, 5'b00000, 4'b0000, "LUI");
        test_count = test_count + 1;

        // Test Case 13: AUIPC
        $display("Test Case 13: AUIPC");
        opcode = 7'b0010111;
        fun3 = 3'b000;
        fun7 = 7'b0000000;
        #10;
        check_outputs(1, 0, 0, 0, 0, 0, 3'b100, 2'b00, 5'b00000, 4'b0000, "AUIPC");
        test_count = test_count + 1;

        // Test Case 14: Invalid opcode
        $display("Test Case 14: Invalid Opcode");
        opcode = 7'b1111111; // Invalid opcode
        #10;
        check_outputs(0, 0, 0, 0, 0, 0, 3'b000, 2'b00, 5'b00000, 4'b0000, "Invalid Opcode");
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
        input exp_reg_write, exp_load, exp_store, exp_jalr, exp_branch_result, exp_next_sel;
        input [2:0] exp_imm_sel;
        input [1:0] exp_mem_to_reg;
        input [4:0] exp_alu_control;
        input [3:0] exp_mem_mask;
        input [8*50:1] test_name;
        begin
            if (reg_write !== exp_reg_write) begin
                $display("ERROR in %0s: reg_write = %b, expected %b", test_name, reg_write, exp_reg_write);
                errors = errors + 1;
            end
            if (load !== exp_load) begin
                $display("ERROR in %0s: load = %b, expected %b", test_name, load, exp_load);
                errors = errors + 1;
            end
            if (store !== exp_store) begin
                $display("ERROR in %0s: store = %b, expected %b", test_name, store, exp_store);
                errors = errors + 1;
            end
            if (jalr !== exp_jalr) begin
                $display("ERROR in %0s: jalr = %b, expected %b", test_name, jalr, exp_jalr);
                errors = errors + 1;
            end
            if (branch_result !== exp_branch_result) begin
                $display("ERROR in %0s: branch_result = %b, expected %b", test_name, branch_result, exp_branch_result);
                errors = errors + 1;
            end
            if (next_sel !== exp_next_sel) begin
                $display("ERROR in %0s: next_sel = %b, expected %b", test_name, next_sel, exp_next_sel);
                errors = errors + 1;
            end
            if (imm_sel !== exp_imm_sel) begin
                $display("ERROR in %0s: imm_sel = %b, expected %b", test_name, imm_sel, exp_imm_sel);
                errors = errors + 1;
            end
            if (mem_to_reg !== exp_mem_to_reg) begin
                $display("ERROR in %0s: mem_to_reg = %b, expected %b", test_name, mem_to_reg, exp_mem_to_reg);
                errors = errors + 1;
            end
            if (alu_control !== exp_alu_control) begin
                $display("ERROR in %0s: alu_control = %b, expected %b", test_name, alu_control, exp_alu_control);
                errors = errors + 1;
            end
            if (mem_mask !== exp_mem_mask) begin
                $display("ERROR in %0s: mem_mask = %b, expected %b", test_name, mem_mask, exp_mem_mask);
                errors = errors + 1;
            end
        end
    endtask
endmodule
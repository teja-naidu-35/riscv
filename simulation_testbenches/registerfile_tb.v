module tb_registerfile;
    // Testbench signals
    reg clk;
    reg rst;
    reg en;
    reg [4:0] rs1;
    reg [4:0] rs2;
    reg [4:0] rd;
    reg [31:0] data;
    wire [31:0] op_a;
    wire [31:0] op_b;

    // Instantiate the registerfile
    registerfile uut (
        .clk(clk),
        .rst(rst),
        .en(en),
        .rs1(rs1),
        .rs2(rs2),
        .rd(rd),
        .data(data),
        .op_a(op_a),
        .op_b(op_b)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 10 ns clock period
    end

    // Test variables
    integer errors = 0;
    integer test_count = 0;
    reg [8*50:1] test_name;

    // Test stimulus
    initial begin
        $dumpfile("tb_registerfile.vcd");
        $dumpvars(0, tb_registerfile);

        // Initialize inputs
        rst = 0;
        en = 0;
        rs1 = 5'b0;
        rs2 = 5'b0;
        rd = 5'b0;
        data = 32'h0;

        // Test Case 1: Reset behavior
        $display("Test Case 1: Reset Behavior");
        rst = 1;
        #20;
        rst = 0;
        rs1 = 5'd1;
        rs2 = 5'd2;
        #10;
        check_outputs(32'h0, 32'h0, "Reset Behavior");
        test_count = test_count + 1;

        // Test Case 2: Write to register 1 and read
        $display("Test Case 2: Write to Register 1");
        en = 1;
        rd = 5'd1;
        data = 32'hDEADBEEF;
        #10;
        en = 0;
        rs1 = 5'd1;
        rs2 = 5'd2;
        #10;
        check_outputs(32'hDEADBEEF, 32'h0, "Write to Register 1");
        test_count = test_count + 1;

        // Test Case 3: Write to register 0 (should not write)
        $display("Test Case 3: Write to Register 0");
        en = 1;
        rd = 5'd0;
        data = 32'h12345678;
        #10;
        en = 0;
        rs1 = 5'd0;
        rs2 = 5'd1;
        #10;
        check_outputs(32'h0, 32'hDEADBEEF, "Write to Register 0");
        test_count = test_count + 1;

        // Test Case 4: Write to register 2 and read both registers
        $display("Test Case 4: Write to Register 2");
        en = 1;
        rd = 5'd2;
        data = 32'hCAFEBABE;
        #10;
        en = 0;
        rs1 = 5'd1;
        rs2 = 5'd2;
        #10;
        check_outputs(32'hDEADBEEF, 32'hCAFEBABE, "Write to Register 2");
        test_count = test_count + 1;

        // Test Case 5: Write disabled (en = 0)
        $display("Test Case 5: Write Disabled");
        en = 0;
        rd = 5'd3;
        data = 32'h55555555;
        #10;
        rs1 = 5'd3;
        rs2 = 5'd2;
        #10;
        check_outputs(32'h0, 32'hCAFEBABE, "Write Disabled");
        test_count = test_count + 1;

        // Test Case 6: Simultaneous read and write
        $display("Test Case 6: Simultaneous Read and Write");
        en = 1;
        rd = 5'd4;
        data = 32'hAAAAAAAA;
        rs1 = 5'd4;
        rs2 = 5'd1;
        #10;
        en = 0;
        #10;
        check_outputs(32'h0, 32'hDEADBEEF, "Simultaneous Read and Write (before write)");
        rs1 = 5'd4;
        #10;
        check_outputs(32'hAAAAAAAA, 32'hDEADBEEF, "Simultaneous Read and Write (after write)");
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
        input [31:0] exp_op_a;
        input [31:0] exp_op_b;
        input [8*50:1] test_name;
        begin
            if (op_a !== exp_op_a) begin
                $display("ERROR in %0s: op_a = %h, expected %h", test_name, op_a, exp_op_a);
                errors = errors + 1;
            end
            if (op_b !== exp_op_b) begin
                $display("ERROR in %0s: op_b = %h, expected %h", test_name, op_b, exp_op_b);
                errors = errors + 1;
            end
        end
    endtask
endmodule
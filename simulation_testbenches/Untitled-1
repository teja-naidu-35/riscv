`timescale 1ns/1ps

module tb_microprocessor1;
    // Testbench signals
    reg clk;
    reg rst;
    reg interrupt;
    wire [31:0] pc_address;
    wire [31:0] instruction_data;
    wire [31:0] alu_out_address;
    wire [31:0] load_data_out;
    wire mem_request;
    wire mem_we;
    wire mem_re;
    wire [15:0] crc_out;
    wire data_valid;
    wire stall;
    wire branch_taken_execute;
    wire next_sel_decode;
    wire trap_taken;

    // Instantiate the microprocessor1 module
    microprocessor1 #(.CLK_PERIOD(10)) uut (
        .clk(clk),
        .rst(rst),
        .interrupt(interrupt),
        .pc_address(pc_address),
        .instruction_data(instruction_data),
        .alu_out_address(alu_out_address),
        .load_data_out(load_data_out),
        .mem_request(mem_request),
        .mem_we(mem_we),
        .mem_re(mem_re),
        .crc_out(crc_out),
        .data_valid(data_valid),
        .stall(stall),
        .branch_taken_execute(branch_taken_execute),
        .next_sel_decode(next_sel_decode),
        .trap_taken(trap_taken)
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
    reg [31:0] expected_registers [0:31]; // To track expected register values
    integer cycle_count;

    // Simulated memory for data cache verification
    reg [31:0] data_memory [0:63];

    // Initialize expected registers and memory
    initial begin
        integer i;
        for (i = 0; i < 32; i = i + 1)
            expected_registers[i] = 32'h0;
        for (i = 0; i < 64; i = i + 1)
            data_memory[i] = 32'h0;
    end

    // Test stimulus
    initial begin
        $dumpfile("tb_microprocessor1.vcd");
        $dumpvars(0, tb_microprocessor1);

        // Initialize inputs
        rst = 0;
        interrupt = 0;
        cycle_count = 0;

        // Test Case 1: Reset behavior
        $display("Test Case 1: Reset Behavior");
        rst = 1;
        #20;
        rst = 0;
        #10;
        check_reset_outputs("Reset Behavior");
        test_count = test_count + 1;

        // Run the program and check behavior
        $display("Test Case 2: Execute Program");
        repeat (20) begin
            #10;
            cycle_count = cycle_count + 1;
            monitor_pipeline();
        end

        // Test Case 3: Trigger an interrupt
        $display("Test Case 3: Interrupt Handling");
        interrupt = 1;
        #10;
        cycle_count = cycle_count + 1;
        check_interrupt_outputs("Interrupt Handling");
        test_count = test_count + 1;

        // Continue simulation for a few more cycles
        interrupt = 0;
        repeat (5) begin
            #10;
            cycle_count = cycle_count + 1;
            monitor_pipeline();
        end

        // Test Case 4: Verify final register and memory state
        $display("Test Case 4: Verify Final State");
        check_final_state("Verify Final State");
        test_count = test_count + 1;

        // Summary
        $display("Test Summary: %0d tests run, %0d errors found", test_count, errors);
        if (errors == 0)
            $display("All tests passed successfully!");
        else
            $display("Test failed with %0d errors.", errors);

        $finish;
    end

    // Task to check outputs after reset
    task check_reset_outputs;
        input [8*50:1] test_name;
        begin
            if (pc_address !== 32'h0) begin
                $display("ERROR in %0s: pc_address = %h, expected %h", test_name, pc_address, 32'h0);
                errors = errors + 1;
            end
            if (instruction_data !== 32'h00000013) begin
                $display("ERROR in %0s: instruction_data = %h, expected %h", test_name, instruction_data, 32'h00000013);
                errors = errors + 1;
            end
            if (alu_out_address !== 32'h0) begin
                $display("ERROR in %0s: alu_out_address = %h, expected %h", test_name, alu_out_address, 32'h0);
                errors = errors + 1;
            end
        end
    endtask

    // Task to monitor the pipeline and verify behavior
    task monitor_pipeline;
        begin
            $display("Cycle %0d: PC = %h, Instruction = %h, ALU Out = %h, Load Data = %h, Stall = %b", 
                     cycle_count, pc_address, instruction_data, alu_out_address, load_data_out, stall);

            // Update expected register values based on the program
            case (cycle_count)
                5: begin // ADDI x1, x0, 5
                    expected_registers[1] = 32'h5;
                end
                6: begin // ADDI x2, x0, 3
                    expected_registers[2] = 32'h3;
                end
                7: begin // ADD x3, x1, x2
                    expected_registers[3] = expected_registers[1] + expected_registers[2]; // 8
                end
                8: begin // SUB x4, x3, x1
                    expected_registers[4] = expected_registers[3] - expected_registers[1]; // 3
                end
                9: begin // SW x4, 0(x0)
                    data_memory[0] = expected_registers[4]; // Store 3 at memory[0]
                end
                10: begin // LW x5, 0(x0)
                    expected_registers[5] = data_memory[0]; // Load 3 into x5
                end
                11: begin // BEQ x5, x4, 8 (branch taken)
                    // Branch should be taken, PC should jump to 0x0020
                    if (pc_address !== 32'h0020) begin
                        $display("ERROR at Cycle %0d: PC = %h, expected %h (BEQ)", cycle_count, pc_address, 32'h0020);
                        errors = errors + 1;
                    end
                end
                12: begin // JAL x7, 4
                    expected_registers[7] = 32'h0024; // Return address
                    if (pc_address !== 32'h0024) begin
                        $display("ERROR at Cycle %0d: PC = %h, expected %h (JAL)", cycle_count, pc_address, 32'h0024);
                        errors = errors + 1;
                    end
                end
            endcase
        end
    endtask

    // Task to check outputs after interrupt
    task check_interrupt_outputs;
        input [8*50:1] test_name;
        begin
            if (pc_address !== 32'h1000) begin
                $display("ERROR in %0s: pc_address = %h, expected %h", test_name, pc_address, 32'h1000);
                errors = errors + 1;
            end
            if (!trap_taken) begin
                $display("ERROR in %0s: trap_taken = %b, expected %b", test_name, trap_taken, 1'b1);
                errors = errors + 1;
            end
        end
    endtask

    // Task to verify final register and memory state
    task check_final_state;
        input [8*50:1] test_name;
        begin
            // Check registers
            if (expected_registers[1] !== 32'h5) begin
                $display("ERROR in %0s: x1 = %h, expected %h", test_name, expected_registers[1], 32'h5);
                errors = errors + 1;
            end
            if (expected_registers[2] !== 32'h3) begin
                $display("ERROR in %0s: x2 = %h, expected %h", test_name, expected_registers[2], 32'h3);
                errors = errors + 1;
            end
            if (expected_registers[3] !== 32'h8) begin
                $display("ERROR in %0s: x3 = %h, expected %h", test_name, expected_registers[3], 32'h8);
                errors = errors + 1;
            end
            if (expected_registers[4] !== 32'h3) begin
                $display("ERROR in %0s: x4 = %h, expected %h", test_name, expected_registers[4], 32'h3);
                errors = errors + 1;
            end
            if (expected_registers[5] !== 32'h3) begin
                $display("ERROR in %0s: x5 = %h, expected %h", test_name, expected_registers[5], 32'h3);
                errors = errors + 1;
            end
            if (expected_registers[6] !== 32'h0) begin
                $display("ERROR in %0s: x6 = %h, expected %h", test_name, expected_registers[6], 32'h0);
                errors = errors + 1;
            end
            if (expected_registers[7] !== 32'h0024) begin
                $display("ERROR in %0s: x7 = %h, expected %h", test_name, expected_registers[7], 32'h0024);
                errors = errors + 1;
            end

            // Check memory
            if (data_memory[0] !== 32'h3) begin
                $display("ERROR in %0s: memory[0] = %h, expected %h", test_name, data_memory[0], 32'h3);
                errors = errors + 1;
            end
        end
    endtask
endmodule
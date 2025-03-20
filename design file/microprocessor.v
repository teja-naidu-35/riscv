`timescale 1ns/1ps

module microprocessor (
    input wire clk,
    input wire rst,
    output wire [31:0] pc_address,
    output wire [31:0] instruction_data,
    output wire [31:0] alu_out_address,
    output wire [31:0] load_data_out,
    output wire mem_request,
    output wire mem_we,
    output wire mem_re,
    output wire [15:0] crc_out,
    output wire data_valid,
    output wire stall,
    output wire branch_taken_execute,
    output wire next_sel_decode,
    output wire trap_taken
);
    parameter CLK_PERIOD = 10; // Defined at top level for propagation

    // Internal wires
    wire [31:0] pre_address_pc, predicted_pc;
    wire [31:0] instruction_fetch, instruction_decode, instruction_execute, instruction_memory, instruction_memory_wb;
    wire [31:0] pre_pc_addr_decode, pre_pc_addr_execute, pre_pc_addr_memory, pre_pc_addr_memory_wb;
    wire load_decode, store_decode, jalr_decode, mispredict_flush;
    wire reg_write_decode, reg_write_execute, reg_write_memory, reg_write_memory_wb;
    wire [4:0] alu_control_decode, alu_control_execute;
    wire [1:0] mem_to_reg_decode, mem_to_reg_execute, mem_to_reg_memory, mem_to_reg_memory_wb;
    wire [4:0] rs1_decode, rs2_decode, rd_decode, rd_execute, rd_memory, rd_memory_wb;
    wire [31:0] opb_data_decode, opa_mux_out_decode, opb_mux_out_decode;
    wire [31:0] alu_out_execute, opb_data_execute, store_data_out;
    wire [1:0] forward_a, forward_b;
    wire [31:0] imm_mux_out;
    wire [31:0] i_immo, s_immo, sb_immo, uj_immo, u_immo;
    wire [2:0] imm_sel;
    wire instruction_mem_valid, data_mem_valid;
    wire [3:0] mem_mask;
    wire memory_error;
    wire [31:0] data_mem_in, data_mem_out;
    wire crc_valid, data_valid_mw;

    // Fetch Stage
    fetch u_fetch (
        .clk(clk),
        .rst(rst),
        .next_sel(next_sel_decode),
        .jalr(jalr_decode),
        .valid(instruction_mem_valid),
        .branch_result(branch_result_decode),
        .mispredict_flush(mispredict_flush),
        .predicted_pc(predicted_pc),
        .instruction_fetch(instruction_fetch),
        .address_out(pc_address),
        .pre_address_pc(pre_address_pc)
    );

    // Instruction Memory
    instruc_mem_top u_instruction_memory (
        .clk(clk),
        .rst(rst),
        .we(1'b0),
        .re(1'b1),
        .request(1'b1),
        .mask(4'b1111), // Fixed constant value
        .address(pc_address[9:2]),
        .data_in(32'b0),
        .valid(instruction_mem_valid),
        .data_out(instruction_fetch)
    );

    // Fetch Pipeline
    fetch_pipe u_fetch_pipe (
        .clk(clk),
        .rst(rst),
        .mispredict_flush(mispredict_flush),
        .pre_address_pc(pre_address_pc),
        .predicted_pc(predicted_pc),
        .instruction_fetch(instruction_fetch),
        .instruction(instruction_decode),
        .instruction_valid(instruction_mem_valid),
        .pre_address_out(pre_pc_addr_decode),
        .rd_out(rd_decode)
    );

    // Decode Stage
    decode u_decode (
        .clk(clk),
        .rst(rst),
        .valid(instruction_mem_valid),
        .reg_write_en_in(reg_write_memory_wb),
        .instruction(instruction_decode),
        .pc_address(pre_pc_addr_decode),
        .rd_wb_data(store_data_out),
        .rd_in(rd_decode),
        .rd_out(rd_execute),
        .load(load_decode),
        .store(store_decode),
        .jalr(jalr_decode),
        .next_sel(next_sel_decode),
        .reg_write_en_out(reg_write_decode),
        .mem_to_reg(mem_to_reg_decode),
        .branch_result(branch_result_decode),
        .alu_control(alu_control_decode),
        .opb_data(opb_data_decode),
        .opa_mux_out(opa_mux_out_decode),
        .opb_mux_out(opb_mux_out_decode),
        .rs1(rs1_decode),
        .rs2(rs2_decode),
        .u_imme(u_immo)
    );

    // Immediate Generator
    immediategen u_immediategen (
        .instr(instruction_decode),
        .i_imme(i_immo),
        .s_imme(s_immo),
        .sb_imme(sb_immo),
        .uj_imme(uj_immo),
        .u_imme(u_immo)
    );

    // Control Unit
    controlunit u_controlunit (
        .opcode(instruction_decode[6:0]),
        .fun3(instruction_decode[14:12]),
        .fun7(instruction_decode[31:25]),
        .valid(instruction_mem_valid),
        .reg_write(reg_write_decode),
        .load(load_decode),
        .store(store_decode),
        .jalr(jalr_decode),
        .branch_result(branch_result_decode),
        .next_sel(next_sel_decode),
        .imm_sel(imm_sel),
        .mem_to_reg(mem_to_reg_decode),
        .alu_control(alu_control_decode),
        .mem_mask(mem_mask),
        .op_a(op_a),
        .op_b(op_b)
    );

    // Register File
    registerfile u_registerfile (
        .clk(clk),
        .rst(rst),
        .en(reg_write_memory_wb && instruction_mem_valid),
        .rs1(rs1_decode),
        .rs2(rs2_decode),
        .rd(rd_memory_wb),
        .data(store_data_out),
        .op_a(op_a),
        .op_b(op_b)
    );

    // Immediate Mux
    mux3_8 u_mux3_8 (
        .a(i_immo),
        .b(s_immo),
        .c(sb_immo),
        .d(uj_immo),
        .e(u_immo),
        .sel(imm_sel),
        .out(imm_mux_out)
    );

    // Decode-to-Execute Pipeline
    decode_execute_pipe u_de_pipe (
        .clk(clk),
        .rst(rst),
        .mispredict_flush(mispredict_flush),
        .stall(stall),
        .instruction_in(instruction_decode),
        .alu_out_in(32'b0),
        .opb_data_in(opb_data_decode),
        .pre_pc_addr_in(pre_pc_addr_decode),
        .alu_control_in(alu_control_decode),
        .mem_to_reg_in(mem_to_reg_decode),
        .reg_write_in(reg_write_decode),
        .load_in(load_decode),
        .store_in(store_decode),
        .branch_taken_in(branch_result_decode),
        .rd_in(rd_execute),
        .instruction_out(instruction_execute),
        .alu_out_out(alu_out_execute),
        .opb_data_out(opb_data_execute),
        .pre_pc_addr_out(pre_pc_addr_execute),
        .alu_control_out(alu_control_execute),
        .mem_to_reg_out(mem_to_reg_execute),
        .reg_write_out(reg_write_execute),
        .load_out(load_decode),
        .store_out(store_decode),
        .branch_taken_out(branch_taken_execute),
        .rd_out(rd_execute)
    );

    // Forwarding Unit
    forwarding_unit u_forwarding (
        .rs1(rs1_decode),
        .rs2(rs2_decode),
        .ex_rd(rd_execute),
        .ex_reg_write(reg_write_execute),
        .mem_rd(rd_memory),
        .mem_reg_write(reg_write_memory),
        .forward_a(forward_a),
        .forward_b(forward_b)
    );

    // Execute Stage
    execute u_execute (
        .clk(clk),
        .rst(rst),
        .mispredict_flush(mispredict_flush),
        .instruction_valid(instruction_mem_valid),
        .alu_control(alu_control_execute),
        .opa_mux_in(opa_mux_out_decode),
        .opb_mux_in(opb_mux_out_decode),
        .pc_address(pre_pc_addr_execute),
        .branch_target(alu_out_execute),
        .branch_taken(branch_taken_execute),
        .forward_a(forward_a),
        .forward_b(forward_b),
        .ex_data(alu_out_execute),
        .mem_data(alu_out_address),
        .alu_out_address(alu_out_address)
    );

    // Execute-to-Memory Pipeline
    execute_memory_pipe u_em_pipe (
        .clk(clk),
        .rst(rst),
        .mispredict_flush(mispredict_flush),
        .instruction_in(instruction_execute),
        .alu_out_in(alu_out_execute),
        .opb_data_in(opb_data_execute),
        .pre_pc_addr_in(pre_pc_addr_execute),
        .mem_to_reg_in(mem_to_reg_execute),
        .reg_write_in(reg_write_execute),
        .rd_in(rd_execute),
        .instruction_out(instruction_memory),
        .alu_out_out(alu_out_address),
        .opb_data_out(store_data_out),
        .pre_pc_addr_out(pre_pc_addr_memory),
        .mem_to_reg_out(mem_to_reg_memory),
        .reg_write_out(reg_write_memory),
        .rd_out(rd_memory)
    );

    // Memory Stage
    memory_stage u_memory_stage (
        .clk(clk),
        .rst(rst),
        .we(mem_we),
        .re(mem_re),
        .request(mem_request),
        .addr(alu_out_address),
        .write_data(store_data_out),
        .mask(mem_mask),
        .read_data(load_data_out),
        .data_in(data_mem_in),
        .data_out(data_mem_out),
        .data_valid(data_valid),
        .crc_valid(crc_valid)
    );

    // Data Memory with Error Injection
    data_mem_top #(.CLK_PERIOD(CLK_PERIOD)) u_data_memory (
        .clk(clk),
        .rst(rst),
        .we(mem_we),
        .re(mem_re),
        .request(mem_request),
        .load(load_decode),
        .mask(mem_mask),
        .address(alu_out_address[9:2]),
        .data_in(store_data_out),
        .valid(data_mem_valid),
        .data_out(data_mem_in)
    );

    // Advanced CRC-16 Module
    crc16 u_crc16 (
        .clk(clk),
        .rst(rst),
        .data_in(data_mem_out[7:0]),
        .data_valid(crc_valid),
        .crc_out(crc_out)
    );

    // Memory-to-Write-Back Pipeline
    memory_writeback_pipe u_mw_pipe (
        .clk(clk),
        .rst(rst),
        .mispredict_flush(mispredict_flush),
        .instruction_in(instruction_memory),
        .alu_out_in(alu_out_address),
        .data_mem_out(load_data_out),
        .pre_pc_addr_in(pre_pc_addr_memory),
        .mem_to_reg_in(mem_to_reg_memory),
        .reg_write_in(reg_write_memory),
        .rd_in(rd_memory),
        .data_valid(data_valid),
        .instruction_out(instruction_memory_wb),
        .alu_out_out(alu_out_address),
        .data_mem_out_out(load_data_out),
        .pre_pc_addr_out(pre_pc_addr_memory_wb),
        .mem_to_reg_out(mem_to_reg_memory_wb),
        .reg_write_out(reg_write_memory_wb),
        .rd_out(rd_memory_wb),
        .data_valid_out(data_valid_mw)
    );

    // Write-Back Stage
    write_back u_write_back (
        .clk(clk),
        .rst(rst),
        .mispredict_flush(mispredict_flush),
        .memory_error(memory_error),
        .mem_to_reg(mem_to_reg_memory_wb),
        .alu_out(alu_out_address),
        .data_mem_out(load_data_out),
        .next_sel_address(pre_pc_addr_memory_wb + 4),
        .rd_sel_mux_out(store_data_out)
    );

    // Branch Predictor
    branch_predictor u_branch_predictor (
        .clk(clk),
        .rst(rst),
        .branch_taken(branch_taken_execute),
        .branch_target(alu_out_address),
        .pc_address(pre_address_pc),
        .predicted_pc(predicted_pc)
    );

    // Hazard Detection Unit
    hazard_detection_unit u_hazard (
        .rs1_decode(rs1_decode),
        .rs2_decode(rs2_decode),
        .ex_rd(rd_execute),
        .ex_reg_write(reg_write_execute),
        .ex_is_load(load_decode),
        .branch_taken(branch_taken_execute),
        .stall(stall),
        .flush_fetch(flush_fetch),
        .flush_decode(flush_decode),
        .flush_execute(flush_execute)
    );

    assign mispredict_flush = flush_execute && branch_taken_execute;
    assign memory_error = 0; // To be enhanced with CRC comparison
    assign trap_taken = memory_error || !instruction_mem_valid;
    assign mem_request = data_mem_valid;
    assign mem_we = store_decode && data_mem_valid;
    assign mem_re = load_decode && data_mem_valid;
    assign instruction_data = instruction_fetch;

    // Halt logic with trap support
    reg halt;
    always @(posedge clk or posedge rst) begin
        if (rst) halt <= 0;
        else if (trap_taken) begin
            $display("TRAP: Error or invalid instruction at time %0t, PC=%h", $time, pc_address);
            halt <= 1;
        end
    end
endmodule

// Submodule Definitions
module fetch (
    input wire clk,
    input wire rst,
    input wire next_sel,
    input wire jalr,
    input wire valid,
    input wire branch_result,
    input wire mispredict_flush,
    output wire [31:0] predicted_pc,
    output wire [31:0] instruction_fetch,
    output wire [31:0] address_out,
    output wire [31:0] pre_address_pc
);
    reg [31:0] pc;
    always @(posedge clk or posedge rst) begin
        if (rst) pc <= 0;
        else if (valid && !mispredict_flush) pc <= next_sel ? (jalr ? 32'h4 : pc + 4) : pc;
    end
    assign address_out = pc;
    assign pre_address_pc = pc;
    assign predicted_pc = pc + 4;
    assign instruction_fetch = 32'h0; // Driven by memory
endmodule

module fetch_pipe (
    input wire clk,
    input wire rst,
    input wire mispredict_flush,
    input wire [31:0] pre_address_pc,
    input wire [31:0] predicted_pc,
    input wire [31:0] instruction_fetch,
    output reg [31:0] instruction,
    output reg instruction_valid,
    output reg [31:0] pre_address_out,
    output reg [4:0] rd_out
);
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            instruction <= 0;
            instruction_valid <= 0;
            pre_address_out <= 0;
            rd_out <= 0;
        end else if (!mispredict_flush) begin
            instruction <= instruction_fetch;
            instruction_valid <= 1;
            pre_address_out <= pre_address_pc;
            rd_out <= instruction_fetch[11:7];
        end
    end
endmodule

module decode (
    input wire clk,
    input wire rst,
    input wire valid,
    input wire reg_write_en_in,
    input wire [31:0] instruction,
    input wire [31:0] pc_address,
    input wire [31:0] rd_wb_data,
    input wire [4:0] rd_in,
    output reg [4:0] rd_out,
    output reg load,
    output reg store,
    output reg jalr,
    output reg next_sel,
    output reg branch_result,
    output reg reg_write_en_out,
    output reg [4:0] alu_control,
    output reg [1:0] mem_to_reg,
    output reg [4:0] rs1,
    output reg [4:0] rs2,
    output reg [31:0] opb_data,
    output reg [31:0] opa_mux_out,
    output reg [31:0] opb_mux_out,
    input wire [31:0] u_imme
);
    always @(posedge clk or posedge rst) begin
        if (rst || !valid) begin
            load <= 0;
            store <= 0;
            jalr <= 0;
            next_sel <= 0;
            branch_result <= 0;
            reg_write_en_out <= 0;
            alu_control <= 0;
            mem_to_reg <= 0;
            rs1 <= 0;
            rs2 <= 0;
            opb_data <= 0;
            opa_mux_out <= 0;
            opb_mux_out <= 0;
            rd_out <= 0;
        end else begin
            load <= (instruction[6:0] == 7'b0000011);
            store <= (instruction[6:0] == 7'b0100011);
            jalr <= (instruction[6:0] == 7'b1100111);
            next_sel <= (instruction[6:0] == 7'b1101111);
            branch_result <= (instruction[6:0] == 7'b1100011) && (instruction[14:12] == 3'b000); // BEQ
            reg_write_en_out <= (instruction[6:0] == 7'b0110011) || (instruction[6:0] == 7'b0010011) || 
                              (instruction[6:0] == 7'b0000011) || (instruction[6:0] == 7'b1101111) || 
                              (instruction[6:0] == 7'b1100111);
            alu_control <= {instruction[30], instruction[14:12]}; // Handle R-type and I-type ALU ops
            mem_to_reg <= (instruction[6:0] == 7'b0000011) ? 2'b01 : 2'b00;
            rs1 <= instruction[19:15];
            rs2 <= instruction[24:20];
            opb_data <= 32'hx; // Driven by register file
            opa_mux_out <= next_sel ? pc_address : 32'hx;
            opb_mux_out <= u_imme;
            rd_out <= rd_in;
        end
    end
endmodule

module immediategen (
    input wire [31:0] instr,
    output wire [31:0] i_imme,
    output wire [31:0] s_imme,
    output wire [31:0] sb_imme,
    output wire [31:0] uj_imme,
    output wire [31:0] u_imme
);
    assign i_imme = {{20{instr[31]}}, instr[31:20]};
    assign s_imme = {{20{instr[31]}}, instr[31:25], instr[11:7]};
    assign sb_imme = {{19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
    assign uj_imme = {{11{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};
    assign u_imme = {instr[31:12], 12'b0};
endmodule

module controlunit (
    input wire [6:0] opcode,
    input wire [2:0] fun3,
    input wire [6:0] fun7,
    input wire valid,
    output reg reg_write,
    output reg load,
    output reg store,
    output reg jalr,
    output reg branch_result,
    output reg next_sel,
    output reg [2:0] imm_sel,
    output reg [1:0] mem_to_reg,
    output reg [4:0] alu_control,
    output reg [3:0] mem_mask,
    input wire [31:0] op_a,
    input wire [31:0] op_b
);
    always @(*) begin
        if (!valid) begin
            reg_write = 0;
            load = 0;
            store = 0;
            jalr = 0;
            branch_result = 0;
            next_sel = 0;
            imm_sel = 0;
            mem_to_reg = 0;
            alu_control = 0;
            mem_mask = 0;
        end else begin
            case (opcode)
                7'b0110011: begin // R-type
                    reg_write = 1;
                    load = 0;
                    store = 0;
                    jalr = 0;
                    branch_result = 0;
                    next_sel = 0;
                    imm_sel = 3'b000;
                    mem_to_reg = 2'b00;
                    alu_control = {fun7[5], fun3}; // e.g., ADD/SUB
                    mem_mask = 0;
                end
                7'b0010011: begin // I-type (ALU)
                    reg_write = 1;
                    load = 0;
                    store = 0;
                    jalr = 0;
                    branch_result = 0;
                    next_sel = 0;
                    imm_sel = 3'b000;
                    mem_to_reg = 2'b00;
                    alu_control = {1'b0, fun3}; // e.g., ADDI
                    mem_mask = 0;
                end
                7'b0000011: begin // Load
                    reg_write = 1;
                    load = 1;
                    store = 0;
                    jalr = 0;
                    branch_result = 0;
                    next_sel = 0;
                    imm_sel = 3'b000;
                    mem_to_reg = 2'b01;
                    alu_control = 0;
                    case (fun3)
                        3'b000: mem_mask = 4'b0001; // lb
                        3'b001: mem_mask = 4'b0011; // lh
                        3'b010: mem_mask = 4'b1111; // lw
                        default: mem_mask = 0;
                    endcase
                end
                7'b0100011: begin // Store
                    reg_write = 0;
                    load = 0;
                    store = 1;
                    jalr = 0;
                    branch_result = 0;
                    next_sel = 0;
                    imm_sel = 3'b001;
                    mem_to_reg = 2'b00;
                    alu_control = 0;
                    case (fun3)
                        3'b000: mem_mask = 4'b0001; // sb
                        3'b001: mem_mask = 4'b0011; // sh
                        3'b010: mem_mask = 4'b1111; // sw
                        default: mem_mask = 0;
                    endcase
                end
                7'b1100011: begin // Branch
                    reg_write = 0;
                    load = 0;
                    store = 0;
                    jalr = 0;
                    branch_result = (fun3 == 3'b000) ? (op_a == op_b) : (fun3 == 3'b001) ? (op_a != op_b) : 1'b0; // BEQ, BNE
                    next_sel = 0;
                    imm_sel = 3'b010;
                    mem_to_reg = 2'b00;
                    alu_control = 0;
                    mem_mask = 0;
                end
                7'b1101111: begin // JAL
                    reg_write = 1;
                    load = 0;
                    store = 0;
                    jalr = 0;
                    branch_result = 0;
                    next_sel = 1;
                    imm_sel = 3'b011;
                    mem_to_reg = 2'b00;
                    alu_control = 0;
                    mem_mask = 0;
                end
                7'b1100111: begin // JALR
                    reg_write = 1;
                    load = 0;
                    store = 0;
                    jalr = 1;
                    branch_result = 0;
                    next_sel = 1;
                    imm_sel = 3'b000;
                    mem_to_reg = 2'b00;
                    alu_control = 0;
                    mem_mask = 0;
                end
                default: begin
                    reg_write = 0;
                    load = 0;
                    store = 0;
                    jalr = 0;
                    branch_result = 0;
                    next_sel = 0;
                    imm_sel = 0;
                    mem_to_reg = 0;
                    alu_control = 0;
                    mem_mask = 0;
                end
            endcase
        end
    end
endmodule

module registerfile (
    input wire clk,
    input wire rst,
    input wire en,
    input wire [4:0] rs1,
    input wire [4:0] rs2,
    input wire [4:0] rd,
    input wire [31:0] data,
    output wire [31:0] op_a,
    output wire [31:0] op_b
);
    reg [31:0] reg_file [0:31];
    integer i;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i = 0; i < 32; i = i + 1) reg_file[i] <= 0;
        end else if (en && rd != 0) begin
            reg_file[rd] <= data;
        end
    end
    assign op_a = (rs1 != 0) ? reg_file[rs1] : 0;
    assign op_b = (rs2 != 0) ? reg_file[rs2] : 0;
endmodule

module mux3_8 (
    input wire [31:0] a,
    input wire [31:0] b,
    input wire [31:0] c,
    input wire [31:0] d,
    input wire [31:0] e,
    input wire [2:0] sel,
    output wire [31:0] out
);
    assign out = (sel == 3'b000) ? a :
                 (sel == 3'b001) ? b :
                 (sel == 3'b010) ? c :
                 (sel == 3'b011) ? d :
                 (sel == 3'b100) ? e : 32'h0;
endmodule

module decode_execute_pipe (
    input wire clk,
    input wire rst,
    input wire mispredict_flush,
    input wire stall,
    input wire [31:0] instruction_in,
    input wire [31:0] alu_out_in,
    input wire [31:0] opb_data_in,
    input wire [31:0] pre_pc_addr_in,
    input wire [4:0] alu_control_in,
    input wire [1:0] mem_to_reg_in,
    input wire reg_write_in,
    input wire load_in,
    input wire store_in,
    input wire branch_taken_in,
    input wire [4:0] rd_in,
    output reg [31:0] instruction_out,
    output reg [31:0] alu_out_out,
    output reg [31:0] opb_data_out,
    output reg [31:0] pre_pc_addr_out,
    output reg [4:0] alu_control_out,
    output reg [1:0] mem_to_reg_out,
    output reg reg_write_out,
    output reg load_out,
    output reg store_out,
    output reg branch_taken_out,
    output reg [4:0] rd_out
);
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            instruction_out <= 0;
            alu_out_out <= 0;
            opb_data_out <= 0;
            pre_pc_addr_out <= 0;
            alu_control_out <= 0;
            mem_to_reg_out <= 0;
            reg_write_out <= 0;
            load_out <= 0;
            store_out <= 0;
            branch_taken_out <= 0;
            rd_out <= 0;
        end else if (!mispredict_flush && !stall) begin
            instruction_out <= instruction_in;
            alu_out_out <= alu_out_in;
            opb_data_out <= opb_data_in;
            pre_pc_addr_out <= pre_pc_addr_in;
            alu_control_out <= alu_control_in;
            mem_to_reg_out <= mem_to_reg_in;
            reg_write_out <= reg_write_in;
            load_out <= load_in;
            store_out <= store_in;
            branch_taken_out <= branch_taken_in;
            rd_out <= rd_in;
        end
    end
endmodule

module execute (
    input wire clk,
    input wire rst,
    input wire mispredict_flush,
    input wire instruction_valid,
    input wire [4:0] alu_control,
    input wire [31:0] opa_mux_in,
    input wire [31:0] opb_mux_in,
    input wire [31:0] pc_address,
    output wire [31:0] branch_target,
    output reg branch_taken,
    input wire [1:0] forward_a,
    input wire [1:0] forward_b,
    input wire [31:0] ex_data,
    input wire [31:0] mem_data,
    output reg [31:0] alu_out_address
);
    reg [31:0] opa, opb;
    always @(*) begin
        case (forward_a)
            2'b01: opa = ex_data;
            2'b10: opa = mem_data;
            default: opa = opa_mux_in;
        endcase
        case (forward_b)
            2'b01: opb = ex_data;
            2'b10: opb = mem_data;
            default: opb = opb_mux_in;
        endcase
    end
    always @(posedge clk or posedge rst) begin
        if (rst || !instruction_valid) begin
            alu_out_address <= 0;
            branch_taken <= 0;
        end else if (!mispredict_flush) begin
            case (alu_control)
                5'b00000: alu_out_address <= opa + opb; // ADD
                5'b01000: alu_out_address <= opa - opb; // SUB
                5'b00001: alu_out_address <= opa << opb[4:0]; // SLL
                5'b00010: alu_out_address <= ($signed(opa) < $signed(opb)) ? 32'b1 : 32'b0; // SLT
                default: alu_out_address <= opa + opb;
            endcase
            branch_taken <= (alu_control == 5'b01000) && (opa == opb); // EQ for branch
        end
    end
    assign branch_target = pc_address + 4;
endmodule

module execute_memory_pipe (
    input wire clk,
    input wire rst,
    input wire mispredict_flush,
    input wire [31:0] instruction_in,
    input wire [31:0] alu_out_in,
    input wire [31:0] opb_data_in,
    input wire [31:0] pre_pc_addr_in,
    input wire [1:0] mem_to_reg_in,
    input wire reg_write_in,
    input wire [4:0] rd_in,
    output reg [31:0] instruction_out,
    output reg [31:0] alu_out_out,
    output reg [31:0] opb_data_out,
    output reg [31:0] pre_pc_addr_out,
    output reg [1:0] mem_to_reg_out,
    output reg reg_write_out,
    output reg [4:0] rd_out
);
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            instruction_out <= 0;
            alu_out_out <= 0;
            opb_data_out <= 0;
            pre_pc_addr_out <= 0;
            mem_to_reg_out <= 0;
            reg_write_out <= 0;
            rd_out <= 0;
        end else if (!mispredict_flush) begin
            instruction_out <= instruction_in;
            alu_out_out <= alu_out_in;
            opb_data_out <= opb_data_in;
            pre_pc_addr_out <= pre_pc_addr_in;
            mem_to_reg_out <= mem_to_reg_in;
            reg_write_out <= reg_write_in;
            rd_out <= rd_in;
        end
    end
endmodule

module memory_stage (
    input wire clk,
    input wire rst,
    output reg we,
    output reg re,
    output reg request,
    input wire [31:0] addr,
    input wire [31:0] write_data,
    input wire [3:0] mask,
    output reg [31:0] read_data,
    input wire [31:0] data_in,
    output reg [31:0] data_out,
    output reg data_valid,
    output reg crc_valid
);
    reg [1:0] mem_state;
    parameter IDLE = 2'b00, REQUEST = 2'b01, WAIT = 2'b10, COMPLETE = 2'b11;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            we <= 0;
            re <= 0;
            request <= 0;
            data_out <= 0;
            read_data <= 0;
            data_valid <= 0;
            crc_valid <= 0;
            mem_state <= IDLE;
        end else begin
            case (mem_state)
                IDLE: begin
                    if (mask != 0) begin
                        we <= |mask; // Active only if mask is non-zero
                        re <= 0;
                        request <= 1;
                        data_out <= write_data;
                        mem_state <= REQUEST;
                    end else if (mask == 0) begin
                        we <= 0;
                        re <= 1;
                        request <= 1;
                        mem_state <= REQUEST;
                    end else begin
                        we <= 0;
                        re <= 0;
                        request <= 0;
                    end
                    data_valid <= 0;
                    crc_valid <= 0;
                end
                REQUEST: begin
                    we <= 0;
                    re <= 0;
                    request <= 1;
                    mem_state <= WAIT;
                end
                WAIT: begin
                    if (data_in !== 32'hx) begin
                        read_data <= data_in;
                        data_valid <= 1;
                        crc_valid <= 1;
                        mem_state <= COMPLETE;
                    end
                end
                COMPLETE: begin
                    we <= 0;
                    re <= 0;
                    request <= 0;
                    data_valid <= 0;
                    crc_valid <= 0;
                    mem_state <= IDLE;
                end
            endcase
        end
    end
endmodule

module memory_writeback_pipe (
    input wire clk,
    input wire rst,
    input wire mispredict_flush,
    input wire [31:0] instruction_in,
    input wire [31:0] alu_out_in,
    input wire [31:0] data_mem_out,
    input wire [31:0] pre_pc_addr_in,
    input wire [1:0] mem_to_reg_in,
    input wire reg_write_in,
    input wire [4:0] rd_in,
    input wire data_valid,
    output reg [31:0] instruction_out,
    output reg [31:0] alu_out_out,
    output reg [31:0] data_mem_out_out,
    output reg [31:0] pre_pc_addr_out,
    output reg [1:0] mem_to_reg_out,
    output reg reg_write_out,
    output reg [4:0] rd_out,
    output reg data_valid_out
);
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            instruction_out <= 0;
            alu_out_out <= 0;
            data_mem_out_out <= 0;
            pre_pc_addr_out <= 0;
            mem_to_reg_out <= 0;
            reg_write_out <= 0;
            rd_out <= 0;
            data_valid_out <= 0;
        end else if (!mispredict_flush && data_valid) begin
            instruction_out <= instruction_in;
            alu_out_out <= alu_out_in;
            data_mem_out_out <= data_mem_out;
            pre_pc_addr_out <= pre_pc_addr_in;
            mem_to_reg_out <= mem_to_reg_in;
            reg_write_out <= reg_write_in;
            rd_out <= rd_in;
            data_valid_out <= 1;
        end else begin
            data_valid_out <= 0;
        end
    end
endmodule

module write_back (
    input wire clk,
    input wire rst,
    input wire mispredict_flush,
    input wire memory_error,
    input wire [1:0] mem_to_reg,
    input wire [31:0] alu_out,
    input wire [31:0] data_mem_out,
    input wire [31:0] next_sel_address,
    output reg [31:0] rd_sel_mux_out
);
    always @(posedge clk or posedge rst) begin
        if (rst) rd_sel_mux_out <= 0;
        else if (!mispredict_flush) rd_sel_mux_out <= (mem_to_reg == 2'b01) ? data_mem_out : alu_out;
    end
endmodule

module branch_predictor (
    input wire clk,
    input wire rst,
    input wire branch_taken,
    input wire [31:0] branch_target,
    input wire [31:0] pc_address,
    output wire [31:0] predicted_pc
);
    assign predicted_pc = pc_address + 4; // Simple prediction
endmodule

module hazard_detection_unit (
    input wire [4:0] rs1_decode,
    input wire [4:0] rs2_decode,
    input wire [4:0] ex_rd,
    input wire ex_reg_write,
    input wire ex_is_load,
    input wire branch_taken,
    output reg stall,
    output reg flush_fetch,
    output reg flush_decode,
    output reg flush_execute
);
    always @(*) begin
        stall = ex_is_load && ((rs1_decode == ex_rd) || (rs2_decode == ex_rd)) && ex_reg_write;
        flush_fetch = 0;
        flush_decode = 0;
        flush_execute = branch_taken;
    end
endmodule

module instruc_mem_top (
    input wire clk,
    input wire rst,
    input wire we,
    input wire re,
    input wire request,
    input wire [3:0] mask,
    input wire [7:0] address,
    input wire [31:0] data_in,
    output reg valid,
    output reg [31:0] data_out
);
    reg [31:0] mem [0:255];
    integer i;
    initial begin
        for (i = 0; i < 256; i = i + 1) mem[i] = 32'h0;
        mem[0] = 32'h00108093; // addi x1, x1, 1
        mem[1] = 32'h00210113; // addi x2, x2, 2
        mem[2] = 32'h00108133; // add x2, x1, x2
        mem[3] = 32'h0020A023; // sw x2, 0(x1)
        mem[4] = 32'h0000A103; // lw x2, 0(x1)
        mem[5] = 32'h00208163; // beq x1, x2, 4
        mem[6] = 32'h0000006F; // jal x0, 0
    end
    always @(posedge clk) begin
        if (re && request) begin
            data_out <= mem[address];
            valid <= 1;
        end else begin
            valid <= 0;
        end
    end
endmodule

module data_mem_top #(
    parameter CLK_PERIOD = 10
) (
    input wire clk,
    input wire rst,
    input wire we,
    input wire re,
    input wire request,
    input wire load,
    input wire [3:0] mask,
    input wire [7:0] address,
    input wire [31:0] data_in,
    output reg valid,
    output reg [31:0] data_out
);
    reg [31:0] mem [0:255];
    integer i;
    initial begin
        for (i = 0; i < 256; i = i + 1) mem[i] = 32'h0;
    end
    always @(posedge clk) begin
        if (we && request) mem[address] <= data_in;
        if (re && request && load) begin
            if ($time > (CLK_PERIOD * 45)) mem[address] <= mem[address] ^ 32'h00000001; // Inject error
            data_out <= mem[address];
            valid <= 1;
        end else valid <= 0;
    end
endmodule

module crc16 (
    input wire clk,
    input wire rst,
    input wire [7:0] data_in,
    input wire data_valid,
    output wire [15:0] crc_out
);
    reg [15:0] crc_reg;
    wire [15:0] next_crc;
    parameter POLY = 16'h8005; // CRC-16-IBM polynomial (x^16 + x^15 + x^2 + 1)

    // CRC calculation logic
    assign next_crc[0] = crc_reg[15] ^ data_in[0];
    assign next_crc[1] = crc_reg[14] ^ (data_in[1] ^ next_crc[0]);
    assign next_crc[2] = crc_reg[13] ^ (data_in[2] ^ next_crc[1]);
    assign next_crc[3] = crc_reg[12] ^ (data_in[3] ^ next_crc[2]);
    assign next_crc[4] = crc_reg[11] ^ (data_in[4] ^ next_crc[3]);
    assign next_crc[5] = crc_reg[10] ^ (data_in[5] ^ next_crc[4]);
    assign next_crc[6] = crc_reg[9]  ^ (data_in[6] ^ next_crc[5]);
    assign next_crc[7] = crc_reg[8]  ^ (data_in[7] ^ next_crc[6]);
    assign next_crc[8] = crc_reg[7]  ^ (next_crc[0] & POLY[0]);
    assign next_crc[9] = crc_reg[6]  ^ (next_crc[1] & POLY[1]);
    assign next_crc[10] = crc_reg[5] ^ (next_crc[2] & POLY[2]);
    assign next_crc[11] = crc_reg[4] ^ (next_crc[3] & POLY[3]);
    assign next_crc[12] = crc_reg[3] ^ (next_crc[4] & POLY[4]);
    assign next_crc[13] = crc_reg[2] ^ (next_crc[5] & POLY[5]);
    assign next_crc[14] = crc_reg[1] ^ (next_crc[6] & POLY[6]);
    assign next_crc[15] = crc_reg[0] ^ (next_crc[7] & POLY[7]);

    always @(posedge clk or posedge rst) begin
        if (rst) crc_reg <= 16'hFFFF; // Standard CRC-16-IBM initialization
        else if (data_valid) crc_reg <= next_crc;
    end

    assign crc_out = crc_reg; // Output as wire
endmodule

module forwarding_unit (
    input wire [4:0] rs1,
    input wire [4:0] rs2,
    input wire [4:0] ex_rd,
    input wire ex_reg_write, 
    input wire [4:0] mem_rd,
    input wire mem_reg_write,
    output reg [1:0] forward_a,
    output reg [1:0] forward_b
);
    always @(*) begin
        forward_a = 2'b00;
        forward_b = 2'b00;
        if (ex_reg_write && ex_rd != 0 && ex_rd == rs1) forward_a = 2'b01;
        if (mem_reg_write && mem_rd != 0 && mem_rd == rs1) forward_a = 2'b10;
        if (ex_reg_write && ex_rd != 0 && ex_rd == rs2) forward_b = 2'b01;
        if (mem_reg_write && mem_rd != 0 && mem_rd == rs2) forward_b = 2'b10;
    end
endmodule
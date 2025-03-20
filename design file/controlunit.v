`timescale 1ns/1ps

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
            imm_sel = 3'b0;
            mem_to_reg = 2'b0;
            alu_control = 5'b0;
            mem_mask = 4'b0;
        end else begin
            case (opcode)
                7'b0110011: begin // R-type (e.g., ADD, SUB)
                    reg_write = 1;
                    load = 0;
                    store = 0;
                    jalr = 0;
                    branch_result = 0;
                    next_sel = 0;
                    imm_sel = 3'b000;
                    mem_to_reg = 2'b00;
                    alu_control = {fun7[5], fun3}; // e.g., ADD: 00000, SUB: 01000
                    mem_mask = 4'b0;
                end
                7'b0010011: begin // I-type ALU (e.g., ADDI)
                    reg_write = 1;
                    load = 0;
                    store = 0;
                    jalr = 0;
                    branch_result = 0;
                    next_sel = 0;
                    imm_sel = 3'b000;
                    mem_to_reg = 2'b00;
                    alu_control = {1'b0, fun3}; // e.g., ADDI: 00000
                    mem_mask = 4'b0;
                end
                7'b0000011: begin // Load (e.g., LW)
                    reg_write = 1;
                    load = 1;
                    store = 0;
                    jalr = 0;
                    branch_result = 0;
                    next_sel = 0;
                    imm_sel = 3'b000;
                    mem_to_reg = 2'b01;
                    alu_control = 5'b00000;
                    mem_mask = (fun3 == 3'b000) ? 4'b0001 : // LB
                              (fun3 == 3'b001) ? 4'b0011 :  // LH
                              4'b1111;                      // LW
                end
                7'b0100011: begin // Store (e.g., SW)
                    reg_write = 0;
                    load = 0;
                    store = 1;
                    jalr = 0;
                    branch_result = 0;
                    next_sel = 0;
                    imm_sel = 3'b001;
                    mem_to_reg = 2'b00;
                    alu_control = 5'b00000;
                    mem_mask = (fun3 == 3'b000) ? 4'b0001 : // SB
                              (fun3 == 3'b001) ? 4'b0011 :  // SH
                              4'b1111;                      // SW
                end
                7'b1100011: begin // Branch (e.g., BEQ)
                    reg_write = 0;
                    load = 0;
                    store = 0;
                    jalr = 0;
                    next_sel = 0;
                    imm_sel = 3'b010;
                    mem_to_reg = 2'b00;
                    alu_control = 5'b00000;
                    mem_mask = 4'b0;
                    case (fun3)
                        3'b000: branch_result = (op_a == op_b);      // BEQ
                        3'b001: branch_result = (op_a != op_b);      // BNE
                        3'b100: branch_result = ($signed(op_a) < $signed(op_b)); // BLT
                        3'b101: branch_result = ($signed(op_a) >= $signed(op_b)); // BGE
                        3'b110: branch_result = (op_a < op_b);       // BLTU
                        3'b111: branch_result = (op_a >= op_b);      // BGEU
                        default: branch_result = 0;
                    endcase
                end
                7'b1101111: begin // JAL
                    reg_write = 1;
                    load = 0;
                    store = 0;
                    jalr = 0;
                    branch_result = 0;
                    next_sel = 1;
                    imm_sel = 3'b011;
                    mem_to_reg = 2'b10;
                    alu_control = 5'b00000;
                    mem_mask = 4'b0;
                end
                7'b1100111: begin // JALR
                    reg_write = 1;
                    load = 0;
                    store = 0;
                    jalr = 1;
                    branch_result = 0;
                    next_sel = 1;
                    imm_sel = 3'b000;
                    mem_to_reg = 2'b10;
                    alu_control = 5'b00000;
                    mem_mask = 4'b0;
                end
                7'b0110111: begin // LUI
                    reg_write = 1;
                    load = 0;
                    store = 0;
                    jalr = 0;
                    branch_result = 0;
                    next_sel = 0;
                    imm_sel = 3'b100;
                    mem_to_reg = 2'b00;
                    alu_control = 5'b00000;
                    mem_mask = 4'b0;
                end
                7'b0010111: begin // AUIPC
                    reg_write = 1;
                    load = 0;
                    store = 0;
                    jalr = 0;
                    branch_result = 0;
                    next_sel = 0;
                    imm_sel = 3'b100;
                    mem_to_reg = 2'b00;
                    alu_control = 5'b00000;
                    mem_mask = 4'b0;
                end
                default: begin
                    reg_write = 0;
                    load = 0;
                    store = 0;
                    jalr = 0;
                    branch_result = 0;
                    next_sel = 0;
                    imm_sel = 3'b0;
                    mem_to_reg = 2'b0;
                    alu_control = 5'b0;
                    mem_mask = 4'b0;
                end
            endcase
        end
    end
endmodule
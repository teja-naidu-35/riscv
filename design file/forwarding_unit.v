`timescale 1ns/1ps

module forwarding_unit (
    input wire [4:0] rs1,
    input wire [4:0] rs2,
    input wire [4:0] ex_rd,      // EX/MEM rd
    input wire ex_reg_write,     // EX/MEM regWrite
    input wire [4:0] mem_rd,     // MEM/WB rd
    input wire mem_reg_write,    // MEM/WB regWrite
    output reg [1:0] forward_a,  // Forwarding for rs1
    output reg [1:0] forward_b   // Forwarding for rs2
);
    always @(*) begin
        // Forwarding for rs1
        if (ex_reg_write && ex_rd != 0 && ex_rd == rs1) begin
            forward_a = 2'b01; // Forward from EX/MEM
        end else if (mem_reg_write && mem_rd != 0 && mem_rd == rs1) begin
            forward_a = 2'b10; // Forward from MEM/WB
        end else begin
            forward_a = 2'b00; // No forwarding
        end

        // Forwarding for rs2
        if (ex_reg_write && ex_rd != 0 && ex_rd == rs2) begin
            forward_b = 2'b01; // Forward from EX/MEM
        end else if (mem_reg_write && mem_rd != 0 && mem_rd == rs2) begin
            forward_b = 2'b10; // Forward from MEM/WB
        end else begin
            forward_b = 2'b00; // No forwarding
        end
    end
endmodule
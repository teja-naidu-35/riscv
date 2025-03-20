`timescale 1ns/1ps

module execute_memory_pipe (
    input wire clk,
    input wire rst,
    input wire mispredict_flush,
    input wire [31:0] instruction_in,
    input wire [31:0] alu_out_in,      // ALU result
    input wire [31:0] opb_data_in,     // op_b for stores
    input wire [31:0] pre_pc_addr_in,  // PC
    input wire [1:0] mem_to_reg_in,
    input wire reg_write_in,
    input wire [4:0] rd_in,
    output reg [31:0] instruction_out,
    output reg [31:0] ex_mem_ALUOut,   // Renamed: alu_out_out -> ex_mem_ALUOut
    output reg [31:0] ex_mem_rs2_data, // Renamed: opb_data_out -> ex_mem_rs2_data
    output reg [31:0] pre_pc_addr_out,
    output reg [1:0] mem_to_reg_out,
    output reg ex_mem_regWrite,        // Renamed: reg_write_out -> ex_mem_regWrite
    output reg [4:0] ex_mem_rd         // Renamed: rd_out -> ex_mem_rd
);
    always @(posedge clk or posedge rst) begin
        if (rst || mispredict_flush) begin
            instruction_out <= 32'b0;
            ex_mem_ALUOut <= 32'b0;
            ex_mem_rs2_data <= 32'b0;
            pre_pc_addr_out <= 32'b0;
            mem_to_reg_out <= 2'b0;
            ex_mem_regWrite <= 0;
            ex_mem_rd <= 5'b0;
        end else begin
            instruction_out <= instruction_in;
            ex_mem_ALUOut <= alu_out_in;
            ex_mem_rs2_data <= opb_data_in;
            pre_pc_addr_out <= pre_pc_addr_in;
            mem_to_reg_out <= mem_to_reg_in;
            ex_mem_regWrite <= reg_write_in;
            ex_mem_rd <= rd_in;
        end
    end
endmodule
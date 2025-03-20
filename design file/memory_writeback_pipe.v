`timescale 1ns/1ps

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
    output reg [31:0] mem_wb_ALUOut,    // Renamed: alu_out_out -> mem_wb_ALUOut
    output reg [31:0] mem_wb_memData,   // Renamed: data_mem_out_out -> mem_wb_memData
    output reg [31:0] pre_pc_addr_out,
    output reg [1:0] mem_to_reg_out,
    output reg mem_wb_regWrite,         // Renamed: reg_write_out -> mem_wb_regWrite
    output reg [4:0] mem_wb_rd,         // Renamed: rd_out -> mem_wb_rd
    output reg data_valid_out
);
    always @(posedge clk or posedge rst) begin
        if (rst || mispredict_flush) begin
            instruction_out <= 32'b0;
            mem_wb_ALUOut <= 32'b0;
            mem_wb_memData <= 32'b0;
            pre_pc_addr_out <= 32'b0;
            mem_to_reg_out <= 2'b0;
            mem_wb_regWrite <= 0;
            mem_wb_rd <= 5'b0;
            data_valid_out <= 0;
        end else begin
            instruction_out <= instruction_in;
            mem_wb_ALUOut <= alu_out_in;
            mem_wb_memData <= data_mem_out;
            pre_pc_addr_out <= pre_pc_addr_in;
            mem_to_reg_out <= mem_to_reg_in;
            mem_wb_regWrite <= reg_write_in;
            mem_wb_rd <= rd_in;
            data_valid_out <= data_valid;
        end
    end
endmodule
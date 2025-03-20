`timescale 1ns/1ps

module decode_execute_pipe (
    input wire clk,
    input wire rst,
    input wire mispredict_flush,
    input wire stall,
    input wire [31:0] instruction_in,
    input wire [31:0] alu_out_in,      // Not used (set to 0)
    input wire [31:0] opb_data_in,     // op_b for stores
    input wire [31:0] pre_pc_addr_in,  // PC from decode
    input wire [4:0] alu_control_in,
    input wire [1:0] mem_to_reg_in,
    input wire reg_write_in,
    input wire load_in,
    input wire store_in,
    input wire branch_taken_in,
    input wire [4:0] rd_in,
    input wire [31:0] opa_mux_in,      // Muxed operand A
    input wire [31:0] opb_mux_in,      // Muxed operand B
    output reg [31:0] instruction_out,
    output reg [31:0] id_ex_rs1_data,  // Renamed: opa_mux_out -> id_ex_rs1_data
    output reg [31:0] id_ex_rs2_data,  // Renamed: opb_mux_out -> id_ex_rs2_data
    output reg [31:0] id_ex_imm,       // Renamed: opb_data_out -> id_ex_imm
    output reg [31:0] pre_pc_addr_out,
    output reg [4:0] alu_control_out,
    output reg [1:0] mem_to_reg_out,
    output reg id_ex_regWrite,         // Renamed: reg_write_out -> id_ex_regWrite
    output reg id_ex_memWrite,         // Renamed: store_out -> id_ex_memWrite
    output reg id_ex_memRead,          // Renamed: load_out -> id_ex_memRead
    output reg id_ex_ALUSrc,           // Added: ALUSrc signal
    output reg id_ex_branch,           // Renamed: branch_taken_out -> id_ex_branch
    output reg [4:0] id_ex_rd          // Renamed: rd_out -> id_ex_rd
);
    always @(posedge clk or posedge rst) begin
        if (rst || mispredict_flush) begin
            instruction_out <= 32'b0;
            id_ex_rs1_data <= 32'b0;
            id_ex_rs2_data <= 32'b0;
            id_ex_imm <= 32'b0;
            pre_pc_addr_out <= 32'b0;
            alu_control_out <= 5'b0;
            mem_to_reg_out <= 2'b0;
            id_ex_regWrite <= 0;
            id_ex_memWrite <= 0;
            id_ex_memRead <= 0;
            id_ex_ALUSrc <= 0;
            id_ex_branch <= 0;
            id_ex_rd <= 5'b0;
        end else if (stall) begin
            // Hold values (no change)
        end else begin
            instruction_out <= instruction_in;
            id_ex_rs1_data <= opa_mux_in;
            id_ex_rs2_data <= opb_mux_in;
            id_ex_imm <= opb_data_in; // For store instructions
            pre_pc_addr_out <= pre_pc_addr_in;
            alu_control_out <= alu_control_in;
            mem_to_reg_out <= mem_to_reg_in;
            id_ex_regWrite <= reg_write_in;
            id_ex_memWrite <= store_in;
            id_ex_memRead <= load_in;
            id_ex_ALUSrc <= (instruction_in[6:0] == 7'b0010011 || instruction_in[6:0] == 7'b0000011 || 
                            instruction_in[6:0] == 7'b0100011 || instruction_in[6:0] == 7'b1100111 || 
                            instruction_in[6:0] == 7'b0110111 || instruction_in[6:0] == 7'b0010111);
            id_ex_branch <= branch_taken_in;
            id_ex_rd <= rd_in;
        end
    end
endmodule
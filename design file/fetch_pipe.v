`timescale 1ns/1ps

module fetch_pipe (
    input wire clk,
    input wire rst,
    input wire mispredict_flush,
    input wire [31:0] pre_address_pc,     // PC + 4 from fetch
    input wire [31:0] predicted_pc,       // Predicted PC (not used here)
    input wire [31:0] instruction_fetch,  // Instruction from cache
    input wire instruction_mem_valid,     // Valid signal from cache
    output reg [31:0] if_id_pc,           // Renamed: pre_address_out -> if_id_pc
    output reg [31:0] if_id_instr,        // Renamed: instruction -> if_id_instr
    output reg instruction_valid,
    output reg [4:0] rd_out
);
    always @(posedge clk or posedge rst) begin
        if (rst || mispredict_flush) begin
            if_id_pc <= 32'b0;
            if_id_instr <= 32'b0;
            instruction_valid <= 0;
            rd_out <= 5'b0;
        end else begin
            if_id_pc <= pre_address_pc;
            if_id_instr <= instruction_fetch;
            instruction_valid <= instruction_mem_valid;
            rd_out <= instruction_fetch[11:7];
        end
    end
endmodule
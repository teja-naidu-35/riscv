`timescale 1ns/1ps

module fetch (
    input wire clk,
    input wire rst,
    input wire next_sel,          // From decode (JAL)
    input wire jalr,              // From decode (JALR)
    input wire valid,             // Instruction memory valid
    input wire branch_result,     // Branch taken result from execute
    input wire mispredict_flush,  // Flush due to branch misprediction
    input wire [31:0] predicted_pc, // Predicted PC from branch predictor
    input wire stall,             // Stall from hazard detection
    input wire interrupt,         // Interrupt signal
    output reg [31:0] address_out, // PC to instruction cache
    output reg [31:0] pre_address_pc // PC + 4
);
    reg rst_delayed;

    // Delay reset to ensure proper initialization
    always @(posedge clk or posedge rst) begin
        if (rst) rst_delayed <= 1;
        else rst_delayed <= 0;
    end

    // PC management
    always @(posedge clk or posedge rst) begin
        if (rst || rst_delayed) begin
            address_out <= 32'b0;
            pre_address_pc <= 32'b0;
        end else if (interrupt) begin
            address_out <= 32'h1000; // Interrupt vector
            pre_address_pc <= 32'h1000;
        end else if (mispredict_flush) begin
            address_out <= predicted_pc;
            pre_address_pc <= predicted_pc + 4;
        end else if (valid && !stall) begin
            address_out <= (next_sel || jalr || branch_result) ? predicted_pc : address_out + 4;
            pre_address_pc <= address_out + 4;
        end
    end
endmodule
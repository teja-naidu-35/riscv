`timescale 1ns/1ps

module branch_predictor (
    input wire clk,
    input wire rst,
    input wire branch_taken,      // Actual branch result
    input wire [31:0] branch_target, // Branch target address
    input wire [31:0] pc_address,    // Current PC
    output reg [31:0] predicted_pc   // Predicted PC
);
    reg [1:0] predictor [0:63];
    reg [5:0] index;

    // Initialize predictor
    integer i;
    initial begin
        for (i = 0; i < 64; i = i + 1) begin
            predictor[i] = 2'b01; // Weakly not-taken
        end
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            predicted_pc <= 32'b0;
            for (i = 0; i < 64; i = i + 1) begin
                predictor[i] <= 2'b01;
            end
        end else begin
            index = pc_address[7:2];
            // Update predictor based on actual branch outcome
            if (branch_taken) begin
                case (predictor[index])
                    2'b00: predictor[index] <= 2'b01; // Strongly not-taken -> Weakly not-taken
                    2'b01: predictor[index] <= 2'b10; // Weakly not-taken -> Weakly taken
                    2'b10: predictor[index] <= 2'b11; // Weakly taken -> Strongly taken
                    2'b11: predictor[index] <= 2'b11; // Strongly taken -> Strongly taken
                endcase
            end else begin
                case (predictor[index])
                    2'b00: predictor[index] <= 2'b00; // Strongly not-taken -> Strongly not-taken
                    2'b01: predictor[index] <= 2'b00; // Weakly not-taken -> Strongly not-taken
                    2'b10: predictor[index] <= 2'b01; // Weakly taken -> Weakly not-taken
                    2'b11: predictor[index] <= 2'b10; // Strongly taken -> Weakly taken
                endcase
            end
            // Predict next PC
            predicted_pc <= (predictor[index][1]) ? branch_target : pc_address + 4;
        end
    end
endmodule
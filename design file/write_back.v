`timescale 1ns/1ps

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
        if (rst || memory_error || mispredict_flush) begin
            rd_sel_mux_out <= 32'b0;
        end else begin
            case (mem_to_reg)
                2'b00: rd_sel_mux_out <= alu_out;         // ALU result
                2'b01: rd_sel_mux_out <= data_mem_out;    // Memory data (load)
                2'b10: rd_sel_mux_out <= next_sel_address; // PC + 4 (JAL, JALR)
                default: rd_sel_mux_out <= 32'b0;
            endcase
        end
    end
endmodule
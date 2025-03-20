`timescale 1ns/1ps

module registerfile (
    input wire clk,
    input wire rst,
    input wire en,          // Write enable
    input wire [4:0] rs1,   // Source register 1
    input wire [4:0] rs2,   // Source register 2
    input wire [4:0] rd,    // Destination register
    input wire [31:0] data, // Write data
    output reg [31:0] op_a, // Data for rs1
    output reg [31:0] op_b  // Data for rs2
);
    reg [31:0] registers [0:31];
    integer i;

    // Initialize registers on reset
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i = 0; i < 32; i = i + 1) begin
                registers[i] <= 32'b0;
            end
        end else if (en && rd != 0) begin
            registers[rd] <= data;
        end
    end

    // Read data (combinational)
    always @(*) begin
        op_a = (rs1 == 0) ? 32'b0 : registers[rs1];
        op_b = (rs2 == 0) ? 32'b0 : registers[rs2];
    end
endmodule
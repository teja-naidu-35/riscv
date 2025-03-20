`timescale 1ns/1ps

module csr_unit (
    input wire clk,
    input wire rst,
    input wire interrupt,
    input wire [31:0] pc_address,
    output reg [31:0] csr_data,
    output reg interrupt_taken
);
    reg [31:0] mtvec;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            mtvec <= 32'h1000;
            csr_data <= 32'b0;
            interrupt_taken <= 0;
        end else if (interrupt) begin
            interrupt_taken <= 1;
            csr_data <= mtvec;
        end else begin
            interrupt_taken <= 0;
            csr_data <= mtvec;
        end
    end
endmodule
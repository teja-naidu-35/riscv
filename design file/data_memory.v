`timescale 1ns/1ps

module data_memory (
    input clk,
    input [31:0] addr,
    input [31:0] data_in,
    input we,
    input re,
    output reg [31:0] data_out
);
    reg [31:0] mem [0:63];
    
    // Preload data (will be overridden by testbench if needed)
    initial begin
        for (i = 0; i < 64; i = i + 1) begin
            mem[i] = i; // Data memory: 0, 1, 2, ..., 63
        end
    end

    integer i;
    always @(posedge clk) begin
        if (we) begin
            mem[addr[31:2]] <= data_in; // Word-aligned address
        end
    end

    always @(*) begin
        if (re) begin
            data_out = mem[addr[31:2]];
        end else begin
            data_out = 32'b0;
        end
    end
endmodule
`timescale 1ns/1ps

module data_cache #(
    parameter CLK_PERIOD = 10
) (
    input wire clk,
    input wire rst,
    input wire we,
    input wire re,
    input wire request,
    input wire load,
    input wire [3:0] mask,
    input wire [7:0] address, // addr[9:2]
    input wire [31:0] data_in,
    output reg valid,
    output reg [31:0] data_out
);
    reg [31:0] cache [0:63];
    reg [7:0] tags [0:63];
    reg [1:0] state;
    reg [31:0] mem_data;
    parameter IDLE = 2'b00, FETCH = 2'b01, WAIT = 2'b10;

    // Initialize cache and tags
    initial begin
        for (i = 0; i < 64; i = i + 1) begin
            cache[i] = 32'b0;
            tags[i] = 8'hFF; // Invalid tag
        end
    end

    integer i;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            valid <= 0;
            data_out <= 32'b0;
            state <= IDLE;
            for (i = 0; i < 64; i = i + 1) begin
                cache[i] <= 32'b0;
                tags[i] <= 8'hFF;
            end
        end else begin
            case (state)
                IDLE: begin
                    if (request) begin
                        if (tags[address[5:0]] == address[7:6] && re) begin
                            data_out <= cache[address[5:0]];
                            valid <= 1;
                        end else if (we) begin
                            if (mask[0]) cache[address[5:0]][7:0]  <= data_in[7:0];
                            if (mask[1]) cache[address[5:0]][15:8]  <= data_in[15:8];
                            if (mask[2]) cache[address[5:0]][23:16] <= data_in[23:16];
                            if (mask[3]) cache[address[5:0]][31:24] <= data_in[31:24];
                            tags[address[5:0]] <= address[7:6];
                            valid <= 1;
                        end else begin
                            state <= FETCH;
                            valid <= 0;
                        end
                    end else begin
                        valid <= 0;
                    end
                end
                FETCH: begin
                    // Simulate memory fetch (testbench will preload)
                    mem_data <= cache[address[5:0]]; // Use preloaded data
                    state <= WAIT;
                end
                WAIT: begin
                    cache[address[5:0]] <= mem_data;
                    tags[address[5:0]] <= address[7:6];
                    data_out <= mem_data;
                    valid <= 1;
                    state <= IDLE;
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule
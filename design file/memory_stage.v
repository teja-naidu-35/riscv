`timescale 1ns/1ps

module memory_stage (
    input wire clk,
    input wire rst,
    input wire we,          // Write enable
    input wire re,          // Read enable
    output reg request,     // Memory request
    input wire [31:0] addr, // Memory address
    input wire [31:0] write_data, // Data to write
    input wire [3:0] mask,  // Byte mask
    output reg [31:0] read_data, // Data read from memory
    input wire [31:0] data_in,   // Data from data cache
    output reg [31:0] data_out,  // Data to data cache
    output reg data_valid,       // Data valid signal
    output reg crc_valid         // CRC valid signal
);
    reg [1:0] mem_state;
    parameter IDLE = 2'b00, REQUEST = 2'b01, WAIT = 2'b10, COMPLETE = 2'b11;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            request <= 0;
            data_out <= 32'b0;
            read_data <= 32'b0;
            data_valid <= 0;
            crc_valid <= 0;
            mem_state <= IDLE;
        end else begin
            case (mem_state)
                IDLE: begin
                    if (we || re) begin
                        request <= 1;
                        data_out <= write_data;
                        mem_state <= REQUEST;
                    end else begin
                        request <= 0;
                        data_valid <= 0;
                        crc_valid <= 0;
                        mem_state <= IDLE;
                    end
                end
                REQUEST: begin
                    mem_state <= WAIT;
                end
                WAIT: begin
                    if (data_in !== 32'hx) begin
                        read_data <= data_in;
                        data_valid <= 1;
                        crc_valid <= 1;
                        mem_state <= COMPLETE;
                    end
                end
                COMPLETE: begin
                    request <= 0;
                    data_valid <= 0;
                    crc_valid <= 0;
                    mem_state <= IDLE;
                end
                default: mem_state <= IDLE;
            endcase
        end
    end
endmodule
`timescale 1ns/1ps

module crc16 (
    input wire clk,
    input wire rst,
    input wire [7:0] data_in,
    input wire data_valid,
    output reg [15:0] crc_out
);
    reg [15:0] crc;
    integer i;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            crc <= 16'hFFFF;
            crc_out <= 16'b0;
        end else if (data_valid) begin
            crc <= 16'hFFFF;
            for (i = 0; i < 8; i = i + 1) begin
                crc[0]  <= data_in[i] ^ crc[15];
                crc[1]  <= crc[0];
                crc[2]  <= crc[1];
                crc[3]  <= crc[2];
                crc[4]  <= crc[3];
                crc[5]  <= crc[4] ^ (data_in[i] ^ crc[15]);
                crc[6]  <= crc[5];
                crc[7]  <= crc[6];
                crc[8]  <= crc[7];
                crc[9]  <= crc[8];
                crc[10] <= crc[9];
                crc[11] <= crc[10];
                crc[12] <= crc[11] ^ (data_in[i] ^ crc[15]);
                crc[13] <= crc[12];
                crc[14] <= crc[13];
                crc[15] <= crc[14];
            end
            crc_out <= crc;
        end
    end
endmodule
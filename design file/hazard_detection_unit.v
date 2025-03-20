`timescale 1ns/1ps

module hazard_detection_unit (
    input wire [4:0] rs1_decode,
    input wire [4:0] rs2_decode,
    input wire [4:0] ex_rd,
    input wire ex_reg_write,
    input wire ex_is_load,
    input wire branch_taken,
    output reg stall,
    output reg flush_fetch,
    output reg flush_decode,
    output reg flush_execute
);
    always @(*) begin
        // Load-use hazard
        if (ex_is_load && ex_reg_write && ex_rd != 0 && (ex_rd == rs1_decode || ex_rd == rs2_decode)) begin
            stall = 1;
            flush_fetch = 0;
            flush_decode = 0;
            flush_execute = 0;
        end
        // Branch/jump hazard
        else if (branch_taken) begin
            stall = 0;
            flush_fetch = 1;
            flush_decode = 1;
            flush_execute = 1;
        end else begin
            stall = 0;
            flush_fetch = 0;
            flush_decode = 0;
            flush_execute = 0;
        end
    end
endmodule
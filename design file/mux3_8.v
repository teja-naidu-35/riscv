`timescale 1ns/1ps

module mux3_8 (
    input wire [31:0] a,    // I-type immediate
    input wire [31:0] b,    // S-type immediate
    input wire [31:0] c,    // SB-type immediate
    input wire [31:0] d,    // UJ-type immediate
    input wire [31:0] e,    // U-type immediate
    input wire [2:0] sel,   // Select signal
    output wire [31:0] out  // Selected immediate
);
    assign out = (sel == 3'b000) ? a :    // I-type
                 (sel == 3'b001) ? b :    // S-type
                 (sel == 3'b010) ? c :    // SB-type
                 (sel == 3'b011) ? d :    // UJ-type
                 (sel == 3'b100) ? e :    // U-type
                 32'b0;                   // Default
endmodule
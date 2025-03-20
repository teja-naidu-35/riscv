`timescale 1ns/1ps

module immediategen (
    input wire [31:0] instr,
    output wire [31:0] i_imme,  // I-type immediate
    output wire [31:0] s_imme,  // S-type immediate
    output wire [31:0] sb_imme, // SB-type immediate
    output wire [31:0] uj_imme, // UJ-type immediate
    output wire [31:0] u_imme   // U-type immediate
);
    assign i_imme  = {{20{instr[31]}}, instr[31:20]};                    // I-type (ADDI, LW, JALR)
    assign s_imme  = {{20{instr[31]}}, instr[31:25], instr[11:7]};      // S-type (SW)
    assign sb_imme = {{19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0}; // SB-type (BEQ)
    assign uj_imme = {{11{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0}; // UJ-type (JAL)
    assign u_imme  = {instr[31:12], 12'b0};                              // U-type (LUI, AUIPC)
endmodule
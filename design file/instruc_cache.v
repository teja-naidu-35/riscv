`timescale 1ns/1ps

module instr_memory (
    input [31:0] addr,
    output reg [31:0] data
);
    reg [31:0] mem [0:63];
    
    // Preload instructions
    initial begin
        mem[0]  = 32'h005302b3; // ADD x5, x6, x5 (R-type)
        mem[1]  = 32'h00510293; // ADDI x5, x2, 5 (I-type)
        mem[2]  = 32'h0000a283; // LW x5, 0(x1) (Load)
        mem[3]  = 32'h0050a223; // SW x5, 0(x1) (Store)
        mem[4]  = 32'h00528463; // BEQ x5, x5, 8 (Branch, taken)
        mem[5]  = 32'h00000013; // NOP
        mem[6]  = 32'h008000ef; // JAL x1, 8 (Jump)
        mem[7]  = 32'h00000013; // NOP
        mem[8]  = 32'h000080e7; // JALR x1, 0(x1) (Jump)
        mem[9]  = 32'h000052b7; // LUI x5, 5 (Load Upper Immediate)
        mem[10] = 32'h00005297; // AUIPC x5, 5 (Add Upper Immediate to PC)
        for (i = 11; i < 64; i = i + 1) begin
            mem[i] = 32'h00000013; // NOP
        end
    end

    integer i;
    always @(*) begin
        data = mem[addr[31:2]]; // Word-aligned address (divide by 4)
    end
endmodule
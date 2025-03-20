`timescale 1ns/1ps

module execute (
    input wire clk,
    input wire rst,
    input wire mispredict_flush,
    input wire instruction_valid,
    input wire [4:0] alu_control,
    input wire [31:0] opa_mux_in,      // id_ex_rs1_data
    input wire [31:0] opb_mux_in,      // id_ex_rs2_data or immediate
    input wire [31:0] pc_address,      // PC from decode_execute_pipe
    output reg [31:0] branch_target,   // Branch/Jump target
    output reg branch_taken,           // Branch taken signal
    input wire [1:0] forward_a,        // Forwarding for opa
    input wire [1:0] forward_b,        // Forwarding for opb
    input wire [31:0] ex_data,         // EX/MEM ALU result
    input wire [31:0] mem_data,        // MEM/WB result
    output reg [31:0] alu_out_address  // ALU result
);
    reg [31:0] opa, opb;

    // Forwarding logic
    always @(*) begin
        opa = (forward_a == 2'b01) ? ex_data : (forward_a == 2'b10) ? mem_data : opa_mux_in;
        opb = (forward_b == 2'b01) ? ex_data : (forward_b == 2'b10) ? mem_data : opb_mux_in;
    end

    // ALU and branch/jump logic
    always @(posedge clk or posedge rst) begin
        if (rst || !instruction_valid || mispredict_flush) begin
            alu_out_address <= 32'b0;
            branch_taken <= 0;
            branch_target <= 32'b0;
        end else begin
            case (alu_control)
                5'b00000: alu_out_address <= opa + opb;            // ADD, ADDI, LW, SW, JALR
                5'b01000: alu_out_address <= opa - opb;            // SUB
                5'b00001: alu_out_address <= opa << opb[4:0];      // SLL
                5'b00010: alu_out_address <= ($signed(opa) < $signed(opb)) ? 1 : 0; // SLT
                5'b00011: alu_out_address <= (opa < opb) ? 1 : 0;  // SLTU
                5'b00100: alu_out_address <= opa ^ opb;            // XOR
                5'b00101: alu_out_address <= opa >> opb[4:0];      // SRL
                5'b01101: alu_out_address <= $signed(opa) >>> opb[4:0]; // SRA
                5'b00110: alu_out_address <= opa | opb;            // OR
                5'b00111: alu_out_address <= opa & opb;            // AND
                default:  alu_out_address <= opa + opb;
            endcase
            branch_target <= pc_address + opb; // For branches and jumps
            branch_taken <= 0; // Set by controlunit and passed through pipeline
        end
    end
endmodule
`timescale 1ns/1ps

module decode (
    input wire clk,
    input wire rst,
    input wire valid,              // Instruction valid from fetch_pipe
    input wire reg_write_en_in,    // Write enable from writeback
    input wire [31:0] instruction, // Instruction from fetch_pipe
    input wire [31:0] pc_address,  // PC from fetch_pipe
    input wire [31:0] rd_wb_data,  // Writeback data
    input wire [4:0] rd_in,        // Destination register from fetch_pipe
    input wire [31:0] op_a,        // Register data from registerfile
    input wire [31:0] op_b,        // Register data from registerfile
    input wire [31:0] u_imme,      // U-type immediate from immediategen
    output reg [4:0] rd_out,       // Destination register to decode_execute_pipe
    output reg load,               // Load control signal
    output reg store,              // Store control signal
    output reg jalr,               // JALR control signal
    output reg next_sel,           // JAL control signal
    output reg branch_result,      // Branch control signal
    output reg reg_write_en_out,   // Register write enable
    output reg [4:0] alu_control,  // ALU control signal
    output reg [1:0] mem_to_reg,   // Memory-to-register control
    output reg [4:0] rs1,          // Source register 1
    output reg [4:0] rs2,          // Source register 2
    output reg [31:0] opb_data,    // op_b for store instructions
    output reg [31:0] opa_mux_out, // Muxed operand A
    output reg [31:0] opb_mux_out  // Muxed operand B
);
    always @(posedge clk or posedge rst) begin
        if (rst || !valid) begin
            rd_out <= 5'b0;
            load <= 0;
            store <= 0;
            jalr <= 0;
            next_sel <= 0;
            branch_result <= 0;
            reg_write_en_out <= 0;
            alu_control <= 5'b0;
            mem_to_reg <= 2'b0;
            rs1 <= 5'b0;
            rs2 <= 5'b0;
            opb_data <= 32'b0;
            opa_mux_out <= 32'b0;
            opb_mux_out <= 32'b0;
        end else begin
            rd_out <= rd_in;
            rs1 <= instruction[19:15];
            rs2 <= instruction[24:20];
            opb_data <= op_b;
            // Mux for operand A: PC for JAL, JALR, AUIPC; otherwise op_a
            opa_mux_out <= (instruction[6:0] == 7'b1101111 || instruction[6:0] == 7'b1100111 || 
                           instruction[6:0] == 7'b0010111) ? pc_address : op_a;
            // Mux for operand B: Immediate for I-type, S-type, JALR, LUI, AUIPC; otherwise op_b
            opb_mux_out <= (instruction[6:0] == 7'b0010011 || instruction[6:0] == 7'b0000011 || 
                           instruction[6:0] == 7'b0100011 || instruction[6:0] == 7'b1100111 || 
                           instruction[6:0] == 7'b0110111 || instruction[6:0] == 7'b0010111) ? u_imme : op_b;
        end
    end
endmodule
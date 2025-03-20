`timescale 1ns/1ps

module microprocessor1 (
    input clk,
    input rst,
    output [31:0] instr_addr,
    input [31:0] instr_data,
    output [31:0] data_addr,
    output [31:0] data_in,
    input [31:0] data_out,
    output mem_we,
    output mem_re
);
    // Pipeline registers
    // IF/ID
    reg [31:0] if_id_pc;
    reg [31:0] if_id_instr;

    // ID/EX
    reg [31:0] id_ex_pc;
    reg [31:0] id_ex_rs1_data;
    reg [31:0] id_ex_rs2_data;
    reg [31:0] id_ex_imm;
    reg id_ex_regWrite;
    reg id_ex_memWrite;
    reg id_ex_ALUSrc;
    reg [4:0] id_ex_rd;
    reg [2:0] id_ex_ALUOp;
    reg id_ex_branch;
    reg id_ex_jump;
    reg id_ex_jalr;

    // EX/MEM
    reg [31:0] ex_mem_ALUOut;
    reg [31:0] ex_mem_rs2_data;
    reg ex_mem_regWrite;
    reg ex_mem_memWrite;
    reg [4:0] ex_mem_rd;
    reg ex_mem_branch;
    reg ex_mem_jump;
    reg ex_mem_jalr;
    reg [31:0] ex_mem_branch_target;

    // MEM/WB
    reg [31:0] mem_wb_ALUOut;
    reg [31:0] mem_wb_memData;
    reg mem_wb_regWrite;
    reg [4:0] mem_wb_rd;

    // Fetch Stage
    reg [31:0] pc;
    assign instr_addr = pc;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pc <= 0;
            if_id_pc <= 0;
            if_id_instr <= 0;
        end else if (ex_mem_branch || ex_mem_jump || ex_mem_jalr) begin
            pc <= ex_mem_branch_target;
            if_id_pc <= 0; // Flush pipeline
            if_id_instr <= 0;
        end else begin
            pc <= pc + 4;
            if_id_pc <= pc;
            if_id_instr <= instr_data;
        end
    end

    // Decode Stage
    reg [31:0] regfile [0:31];
    wire [4:0] rs1 = if_id_instr[19:15];
    wire [4:0] rs2 = if_id_instr[24:20];
    wire [4:0] rd = if_id_instr[11:7];
    wire [6:0] opcode = if_id_instr[6:0];
    wire [2:0] funct3 = if_id_instr[14:12];
    wire [6:0] funct7 = if_id_instr[31:25];
    wire [31:0] imm;

    // Immediate generation
    assign imm = (opcode == 7'b0010011 || opcode == 7'b0000011) ? {{20{if_id_instr[31]}}, if_id_instr[31:20]} : // I-type (ADDI, LW)
                 (opcode == 7'b0100011) ? {{20{if_id_instr[31]}}, if_id_instr[31:25], if_id_instr[11:7]} : // S-type (SW)
                 (opcode == 7'b1100011) ? {{19{if_id_instr[31]}}, if_id_instr[31], if_id_instr[7], if_id_instr[30:25], if_id_instr[11:8], 1'b0} : // B-type (BEQ)
                 (opcode == 7'b1101111) ? {{11{if_id_instr[31]}}, if_id_instr[31], if_id_instr[19:12], if_id_instr[20], if_id_instr[30:21], 1'b0} : // J-type (JAL)
                 (opcode == 7'b1100111) ? {{20{if_id_instr[31]}}, if_id_instr[31:20]} : // I-type (JALR)
                 (opcode == 7'b0110111 || opcode == 7'b0010111) ? {if_id_instr[31:12], 12'b0} : // U-type (LUI, AUIPC)
                 32'b0;

    // Control signals
    wire regWrite = (opcode == 7'b0110011 || opcode == 7'b0010011 || opcode == 7'b0000011 || opcode == 7'b1101111 || opcode == 7'b1100111 || opcode == 7'b0110111 || opcode == 7'b0010111);
    wire memWrite = (opcode == 7'b0100011);
    wire ALUSrc = (opcode == 7'b0010011 || opcode == 7'b0000011 || opcode == 7'b0100011 || opcode == 7'b1100111 || opcode == 7'b0010111);
    wire [2:0] ALUOp = (opcode == 7'b0110011) ? 3'b000 : // ADD
                       (opcode == 7'b0010011) ? 3'b001 : // ADDI
                       (opcode == 7'b0000011 || opcode == 7'b0100011 || opcode == 7'b1100111) ? 3'b010 : // LW, SW, JALR (address calculation)
                       (opcode == 7'b1100011) ? 3'b011 : // BEQ
                       (opcode == 7'b1101111) ? 3'b100 : // JAL
                       (opcode == 7'b0110111) ? 3'b101 : // LUI
                       (opcode == 7'b0010111) ? 3'b110 : // AUIPC
                       3'b000;
    wire branch = (opcode == 7'b1100011);
    wire jump = (opcode == 7'b1101111);
    wire jalr = (opcode == 7'b1100111);

    // Register file
    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1) begin
            regfile[i] = 0;
        end
    end

    always @(posedge clk) begin
        if (mem_wb_regWrite && mem_wb_rd != 0) begin
            regfile[mem_wb_rd] <= (mem_wb_rd == id_ex_rd && id_ex_regWrite) ? ex_mem_ALUOut :
                                  (mem_wb_rd == ex_mem_rd && ex_mem_regWrite) ? mem_wb_ALUOut :
                                  mem_wb_memData != 0 ? mem_wb_memData : mem_wb_ALUOut;
        end
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            id_ex_pc <= 0;
            id_ex_rs1_data <= 0;
            id_ex_rs2_data <= 0;
            id_ex_imm <= 0;
            id_ex_regWrite <= 0;
            id_ex_memWrite <= 0;
            id_ex_ALUSrc <= 0;
            id_ex_rd <= 0;
            id_ex_ALUOp <= 0;
            id_ex_branch <= 0;
            id_ex_jump <= 0;
            id_ex_jalr <= 0;
        end else if (ex_mem_branch || ex_mem_jump || ex_mem_jalr) begin
            // Flush pipeline on branch/jump
            id_ex_pc <= 0;
            id_ex_rs1_data <= 0;
            id_ex_rs2_data <= 0;
            id_ex_imm <= 0;
            id_ex_regWrite <= 0;
            id_ex_memWrite <= 0;
            id_ex_ALUSrc <= 0;
            id_ex_rd <= 0;
            id_ex_ALUOp <= 0;
            id_ex_branch <= 0;
            id_ex_jump <= 0;
            id_ex_jalr <= 0;
        end else begin
            id_ex_pc <= if_id_pc;
            id_ex_rs1_data <= regfile[rs1];
            id_ex_rs2_data <= regfile[rs2];
            id_ex_imm <= imm;
            id_ex_regWrite <= regWrite;
            id_ex_memWrite <= memWrite;
            id_ex_ALUSrc <= ALUSrc;
            id_ex_rd <= rd;
            id_ex_ALUOp <= ALUOp;
            id_ex_branch <= branch;
            id_ex_jump <= jump;
            id_ex_jalr <= jalr;
        end
    end

    // Execute Stage
    wire [31:0] alu_in2 = id_ex_ALUSrc ? id_ex_imm : id_ex_rs2_data;
    wire [31:0] alu_out;
    assign alu_out = (id_ex_ALUOp == 3'b000) ? id_ex_rs1_data + id_ex_rs2_data : // ADD
                     (id_ex_ALUOp == 3'b001 || id_ex_ALUOp == 3'b010) ? id_ex_rs1_data + id_ex_imm : // ADDI, LW/SW/JALR address
                     (id_ex_ALUOp == 3'b011) ? (id_ex_rs1_data == id_ex_rs2_data ? id_ex_pc + id_ex_imm : id_ex_pc + 4) : // BEQ
                     (id_ex_ALUOp == 3'b100) ? id_ex_pc + id_ex_imm : // JAL
                     (id_ex_ALUOp == 3'b101) ? id_ex_imm : // LUI
                     (id_ex_ALUOp == 3'b110) ? id_ex_pc + id_ex_imm : // AUIPC
                     32'b0;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            ex_mem_ALUOut <= 0;
            ex_mem_rs2_data <= 0;
            ex_mem_regWrite <= 0;
            ex_mem_memWrite <= 0;
            ex_mem_rd <= 0;
            ex_mem_branch <= 0;
            ex_mem_jump <= 0;
            ex_mem_jalr <= 0;
            ex_mem_branch_target <= 0;
        end else begin
            ex_mem_ALUOut <= (id_ex_ALUOp == 3'b100 || id_ex_ALUOp == 3'b010) ? id_ex_pc + 4 : alu_out; // JAL/JALR return address
            ex_mem_rs2_data <= id_ex_rs2_data;
            ex_mem_regWrite <= id_ex_regWrite;
            ex_mem_memWrite <= id_ex_memWrite;
            ex_mem_rd <= id_ex_rd;
            ex_mem_branch <= id_ex_branch && (id_ex_rs1_data == id_ex_rs2_data);
            ex_mem_jump <= id_ex_jump;
            ex_mem_jalr <= id_ex_jalr;
            ex_mem_branch_target <= (id_ex_jalr) ? id_ex_rs1_data + id_ex_imm : alu_out;
        end
    end

    // Memory Stage
    assign data_addr = ex_mem_ALUOut;
    assign data_in = ex_mem_rs2_data;
    assign mem_we = ex_mem_memWrite;
    assign mem_re = ex_mem_regWrite && !ex_mem_memWrite && !ex_mem_branch && !ex_mem_jump && !ex_mem_jalr;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            mem_wb_ALUOut <= 0;
            mem_wb_memData <= 0;
            mem_wb_regWrite <= 0;
            mem_wb_rd <= 0;
        end else begin
            mem_wb_ALUOut <= ex_mem_ALUOut;
            mem_wb_memData <= data_out;
            mem_wb_regWrite <= ex_mem_regWrite;
            mem_wb_rd <= ex_mem_rd;
        end
    end

    // Writeback Stage (handled in regfile write above)
endmodule
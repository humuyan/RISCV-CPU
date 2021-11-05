`default_nettype none
`timescale 1ns / 1ps
`include "ops.vh"
`include "csr.vh"

module decoder(
    input wire[31:0]        inst,
    output wire[4:0]        reg_s,
    output wire[4:0]        reg_t, // reg_t or csr reg
    output wire[4:0]        reg_d,
    output reg[5:0]         op,
    output reg[31:0]        imm,
    output reg              imm_select
    );
    
    wire sign;
    wire[19:0] sign_ext;
    assign sign = inst[31];
    assign sign_ext = {20{sign}};
    assign reg_d = inst[11:7];
    assign reg_s = (inst[6:0] == 7'b0110111) ? 5'b00000 : inst[19:15]; // lui should have zero as s1
    assign reg_t = (inst[6:0] == 7'b1110011) ? csr_reg : inst[24:20]; // csr_reg will replace reg_t at CSR instructions
    reg[4:0] csr_reg;
    
    always_comb begin
        op = `OP_INVALID;
        imm = 32'h0;
        imm_select = 1'b0;
        case (inst[6:0])
            7'b0110011: begin // R type
                case ({inst[31:25], inst[14:12]}) // funct7 and funct3
                    10'b0000000_000: op = `OP_ADD;
                    10'b0000000_111: op = `OP_AND;
                    10'b0000000_110: op = `OP_OR;
                    10'b0000000_100: op = `OP_XOR;
                    default: begin end
                endcase
            end
            7'b0010011: begin // I type
                imm = {sign_ext, inst[31:20]};
                imm_select = 1'b1;
                case (inst[14:12])
                    3'b000: op = `OP_ADDI;
                    3'b111: op = `OP_ANDI;
                    3'b110: op = `OP_ORI;
                    3'b001: begin
                        if (inst[31:25] == 7'b0000000) op = `OP_SLLI;
                        else begin end
                    end
                    3'b101: begin
                        if (inst[31:25] == 7'b0000000) op = `OP_SRLI;
                        else begin end
                    end
                    default: begin end
                endcase
            end
            7'b0000011: begin 
                imm = {sign_ext, inst[31:20]};
                imm_select = 1'b1;
                case (inst[14:12])
                    3'b000: op = `OP_LB;
                    3'b010: op = `OP_LW;
                    default: begin end
                endcase
            end // I type (load)
            7'b0100011: begin // S type
                imm = {sign_ext, inst[31:25], inst[11:7]};
                imm_select = 1'b1;
                case (inst[14:12])
                    3'b000: op = `OP_SB;
                    3'b010: op = `OP_SW;
                    default: begin end
                endcase
            end
            7'b1100011: begin // B type
                imm = {sign_ext, inst[7], inst[30:25], inst[11:8], 1'b0};
                imm_select = 1'b1;
                case (inst[14:12])
                    3'b000: op = `OP_BEQ;
                    3'b001: op = `OP_BNE;
                    3'b100: op = `OP_BLT;
                    3'b101: op = `OP_BGE;
                    3'b110: op = `OP_BLTU;
                    3'b111: op = `OP_BGEU;
                    default: begin end
                endcase
            end
            7'b0110111, 7'b0010111: begin // U type
                imm = {inst[31:12], 12'b0};
                imm_select = 1'b1;
                case (inst[5])
                    1'b0: op = `OP_AUIPC;
                    1'b1: op = `OP_LUI;
                endcase
            end
            7'b1101111: begin // jal (J type)
                imm = {sign_ext[10:0], inst[31], inst[19:12], inst[20], inst[30:21], 1'b0};
                imm_select = 1'b1;
                op = `OP_JAL;
            end
            7'b1100111: begin
                imm = {sign_ext, inst[31:20]};
                imm_select = 1'b1;
                case (inst[14:12]) 
                    3'b000: op = `OP_JALR;
                    default: begin end
                endcase
            end
            7'b1110011: begin
                case (inst[31:20])
                    12'b001100000101: csr_reg = `CSR_MTVEC;
                    12'b001101000001: csr_reg = `CSR_MEPC;
                    12'b001101000010: csr_reg = `CSR_MCAUSE;
                    12'b001100000100: csr_reg = `CSR_MIE;
                    12'b001101000100: csr_reg = `CSR_MIP;
                    12'b001101000011: csr_reg = `CSR_MTVAL;
                    12'b001101000000: csr_reg = `CSR_MSCRATCH;
                    12'b001100000000: csr_reg = `CSR_MSTATUS;
                    default: csr_reg = `CSR_ZERO;
                endcase
            end
            // TODO: ecall ebreak
            default: begin end
        endcase
    end
endmodule



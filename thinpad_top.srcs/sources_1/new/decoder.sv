`default_nettype none
`timescale 1ns / 1ps
`include "ops.vh"

module decoder(
    input wire[31:0]        inst,
    output wire[4:0]        reg_s,
    output wire[4:0]        reg_t,
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
    assign reg_t = inst[24:20]; 
    // CLZ and PCNT do not need s2 (reg_t).
    
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
                    10'b0100100_001: op = `OP_SBCLR;
                    default: begin end
                endcase
            end
            7'b0010011: begin // I type
                imm = {sign_ext, inst[31:20]};
                
                imm_select = 1'b1;  // CLZ and PCNT do not need the operand b (imm)

                case (inst[14:12])
                    3'b000: op = `OP_ADDI;
                    3'b111: op = `OP_ANDI;
                    3'b110: op = `OP_ORI;
                    3'b001: begin  // SLLI, CLZ, PCNT
                        if (inst[31:25] == 7'b0000000) op = `OP_SLLI;
                        else if (inst[31:25] == 7'b0110000) begin
                            if (inst[24:20] == 5'b00000) op = `OP_CLZ;
                            else if(inst[24:20] == 5'b00010) op = `OP_PCNT;
                            else begin end
                        end
                        else begin end
                    end
                    3'b101: begin // In this case, inst[31:25] is 0. So there is no need to modify imm.
                        if (inst[31:25] == 7'b0000000) op = `OP_SRLI;
                        else begin end
                    end
                    default: begin end
                endcase
            end
            7'b0000011: begin  // lb, lw (I type)
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
            7'b1100111: begin  // jalr (I type)
                imm = {sign_ext, inst[31:20]};
                imm_select = 1'b1;
                case (inst[14:12]) 
                    3'b000: op = `OP_JALR;
                    default: begin end
                endcase
            end
            // TODO: ecall ebreak
            // TODO: CLZ, SBCLR, PCNT
            // if x == 0, CLZ(x) = -1
            default: begin end
        endcase
    end
endmodule



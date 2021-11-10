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
    output reg              imm_select,
    output reg[3:0]         cur_exception
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
        if (op == `OP_INVALID && inst != 32'h0) begin
            cur_exception = `EXCEPT_ILLEGAL_INST;
        end else begin
            cur_exception = `EXCEPT_NONE;
            case (op)
                `OP_ECALL: cur_exception = `EXCEPT_U_ECALL;
                `OP_EBREAK: cur_exception = `EXCEPT_EBREAK;
                `OP_MRET: cur_exception = `EXCEPT_MRET;
                default: begin end
            endcase
        end
    end
    
    always_comb begin
        op = `OP_INVALID;
        imm = 32'h0;
        imm_select = 1'b0;
        csr_reg = `CSR_ZERO;
        case (inst[6:0])
            7'b0110011: begin // R type
                case ({inst[31:25], inst[14:12]}) // funct7 and funct3
                    10'b0000000_000: op = `OP_ADD;
                    10'b0100000_000: op = `OP_SUB;
                    10'b0000000_001: op = `OP_SLL;
                    10'b0000000_010: op = `OP_SLT;
                    10'b0000000_011: op = `OP_SLTU;
                    10'b0000000_100: op = `OP_XOR;
                    10'b0100100_001: op = `OP_SBCLR;
                    10'b0000000_110: op = `OP_OR;
                    10'b0000000_111: op = `OP_AND;
                    10'b0000000_101: op = `OP_SRL;
                    10'b0100000_101: op = `OP_SRA;
                    default: begin end
                endcase
            end
            7'b0010011: begin // I type
                imm = {sign_ext, inst[31:20]};
                
                imm_select = 1'b1;  // CLZ and PCNT do not need the operand b (imm)

                case (inst[14:12])
                    3'b000: op = `OP_ADDI;
                    3'b010: op = `OP_SLTI;
                    3'b011: op = `OP_SLTIU;
                    3'b100: op = `OP_XORI;
                    3'b110: op = `OP_ORI;
                    3'b111: op = `OP_ANDI;
                    3'b001: begin
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
                        else if (inst[31:25] == 7'b0100000) op = `OP_SRAI;
                        else begin end
                    end
                    default: begin end
                endcase
            end
            7'b0000011: begin  // lb, lh, lw, lbu, lhu (I type)
                imm = {sign_ext, inst[31:20]};
                imm_select = 1'b1;
                case (inst[14:12])
                    3'b000: op = `OP_LB;
                    3'b001: op = `OP_LH;
                    3'b010: op = `OP_LW;
                    3'b100: op = `OP_LBU;
                    3'b101: op = `OP_LHU;
                    default: begin end
                endcase
            end // I type (load)
            7'b0100011: begin // S type
                imm = {sign_ext, inst[31:25], inst[11:7]};
                imm_select = 1'b1;
                case (inst[14:12])
                    3'b000: op = `OP_SB;
                    3'b001: op = `OP_SH;
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
                    12'b000110000000: csr_reg = `CSR_SATP;
                    12'b001100000011: csr_reg = `CSR_MIDELEG;
                    12'b001100000010: csr_reg = `CSR_MEDELEG;
                    12'b000100000000: csr_reg = `CSR_SSTATUS;
                    12'b000101000001: csr_reg = `CSR_SEPC;
                    12'b000101000010: csr_reg = `CSR_SCAUSE;
                    12'b000101000011: csr_reg = `CSR_STVAL;
                    12'b000100000101: csr_reg = `CSR_STVEC;
                    12'b000101000000: csr_reg = `CSR_SSCRATCH;
                    12'b000100000100: csr_reg = `CSR_SIE;
                    12'b000101000100: csr_reg = `CSR_SIP;
                    12'b110000000001: csr_reg = `CSR_MTIME;
                    12'b110010000001: csr_reg = `CSR_MTIMEH;
                    default: csr_reg = `CSR_ZERO;
                endcase
                case (inst[14:12])
                    3'b011: op = `OP_CSRRC;
                    3'b010: op = `OP_CSRRS;
                    3'b001: op = `OP_CSRRW;
                    3'b000: casez (inst[31:7])
                        25'b0000000000010000000000000: op = `OP_EBREAK;
                        25'b0000000000000000000000000: op = `OP_ECALL;
                        25'b0011000000100000000000000: op = `OP_MRET;
                        25'b0001001??????????00000000: op = `OP_SFENCE_VMA; // nop
                        default: begin end
                    endcase
                    default: begin end
                endcase
            end
            default: begin end
        endcase
    end
endmodule



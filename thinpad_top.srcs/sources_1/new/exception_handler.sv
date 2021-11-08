`default_nettype none
`timescale 1ns / 1ps
`include "csr.vh"

module exception_handler(
    input wire          clk,
    input wire          rst,
    input wire          mem_done,
    input wire[1:0]     mem_occupied_by,
    input wire          we,
    input wire[4:0]     waddr,
    input wire[31:0]    wdata,
    input wire[4:0]     raddr,
    input wire          mtip,
    input wire[3:0]     cur_exception,
    input wire[31:0]    cur_exception_pc,

    output wire[31:0]   mtvec,
    output wire[31:0]   mepc,
    output wire[31:0]   rdata,
    output wire         is_exception
    );

    localparam MEM_IF = 1;

    `define STATUS_MPP csrs[`CSR_MSTATUS][12:11]
    `define STATUS_MIE csrs[`CSR_MSTATUS][3]
    `define STATUS_MPIE csrs[`CSR_MSTATUS][7]

    `define MIE csrs[`CSR_MIE]
    `define MIP csrs[`CSR_MIP]
    
    reg is_exception_reg;
    assign is_exception = is_exception_reg;

    reg[31:0] csrs[0:31];
    assign mtvec = csrs[`CSR_MTVEC];
    assign mepc = csrs[`CSR_MEPC];
    assign rdata = raddr == 5'b0 ? 32'b0 : csrs[raddr];

    typedef enum reg[1:0] { USER, SUPERVISOR, MACHINE } mode_t;
    mode_t mode;

    typedef enum reg[31:0] {
        CAUSE_U_ECALL = 32'h00000008,
        CAUSE_EBREAK = 32'h00000003,
        CAUSE_ILLEGAL_INST = 32'h00000002,
        CAUSE_TIMEOUT = 32'h80000007
    } cause_t;

    typedef enum integer {
        IDX_TIMEOUT = 7
    } interrupt_index_t;

    always_comb begin
        is_exception_reg = 1'b0;
        case (cur_exception)
            `EXCEPT_U_ECALL: if (mode == USER) is_exception_reg = 1'b1;
            `EXCEPT_EBREAK: if (mode == USER) is_exception_reg = 1'b1;
            `EXCEPT_MRET: if (mode == MACHINE) is_exception_reg = 1'b1;
            default: begin end
        endcase
        if ((`STATUS_MIE || mode < MACHINE) && `MIE[IDX_TIMEOUT] && `MIP[IDX_TIMEOUT]) begin
            is_exception_reg = 1'b1;
        end
    end

    always_ff @(posedge clk or negedge rst) begin
        if (rst) begin
            mode <= MACHINE;
            csrs[0] <= 32'h00000000;
            csrs[1] <= 32'h00000000;
            csrs[2] <= 32'h00000000;
            csrs[3] <= 32'h00000000;
            csrs[4] <= 32'h00000000;
            csrs[5] <= 32'h00000000;
            csrs[6] <= 32'h00000000;
            csrs[7] <= 32'h00000000;
            csrs[8] <= 32'h00000000;
            csrs[9] <= 32'h00000000;
            csrs[10] <= 32'h00000000;
            csrs[11] <= 32'h00000000;
            csrs[12] <= 32'h00000000;
            csrs[13] <= 32'h00000000;
            csrs[14] <= 32'h00000000;
            csrs[15] <= 32'h00000000;
            csrs[16] <= 32'h00000000;
            csrs[17] <= 32'h00000000;
            csrs[18] <= 32'h00000000;
            csrs[19] <= 32'h00000000;
            csrs[20] <= 32'h00000000;
            csrs[21] <= 32'h00000000;
            csrs[22] <= 32'h00000000;
            csrs[23] <= 32'h00000000;
            csrs[24] <= 32'h00000000;
            csrs[25] <= 32'h00000000;
            csrs[26] <= 32'h00000000;
            csrs[27] <= 32'h00000000;
            csrs[28] <= 32'h00000000;
            csrs[29] <= 32'h00000000;
            csrs[30] <= 32'h00000000;
            csrs[31] <= 32'h00000000;
        end else begin
            if (mem_done && is_exception) begin
                case (cur_exception)
                    `EXCEPT_U_ECALL: begin
                        csrs[`CSR_MEPC] <= cur_exception_pc;
                        csrs[`CSR_MCAUSE] <= CAUSE_U_ECALL;
                        `STATUS_MPP <= mode;
                        mode <= MACHINE;
                    end
                    `EXCEPT_EBREAK: begin
                        csrs[`CSR_MEPC] <= cur_exception_pc;
                        csrs[`CSR_MCAUSE] <= CAUSE_EBREAK;
                        `STATUS_MPP <= mode;
                        mode <= MACHINE;
                    end
                    `EXCEPT_MRET: begin
                        `STATUS_MPP <= 2'b00;
                        mode <= mode_t'(`STATUS_MPP);
                    end
                    default: begin 
                        if ((`STATUS_MIE || mode < MACHINE) && 
                            `MIE[IDX_TIMEOUT] && `MIP[IDX_TIMEOUT]) begin // timeout
                            csrs[`CSR_MEPC] <= cur_exception_pc;
                            csrs[`CSR_MCAUSE] <= CAUSE_TIMEOUT;
                            `STATUS_MPP <= mode;
                            mode <= MACHINE;
                        end
                    end
                endcase
            // for accurate interrupt, I won't optimize the ram again.
            end else begin // handle other commands (csrrs etc.)
                if (mem_done && mem_occupied_by == MEM_IF && we && mode == MACHINE) csrs[waddr] <= wdata;
                // set MIP
                `MIP[IDX_TIMEOUT] <= mtip;
            end
        end
    end

endmodule

`default_nettype none
`timescale 1ns / 1ps
`include "csr.vh"

module exception_handler(
    input wire          clk,
    input wire          rst,
    input wire          mem_done,
    input wire          we,
    input wire[4:0]     waddr,
    input wire[31:0]    wdata,
    input wire[4:0]     raddr,
    input wire[3:0]     cur_exception,
    input wire[31:0]    cur_exception_pc,

    output wire[31:0]   mtvec,
    output wire[31:0]   rdata,
    output wire         is_exception
    );

    `define STATUS_MPP csrs[`CSR_MSTATUS][12:11]
    `define STATUS_MIE csrs[`CSR_MSTATUS][3]
    `define STATUS_MPIE csrs[`CSR_MSTATUS][7]

    `define MIE csrs[`CSR_MIE]
    `define MIP csrs[`CSR_MIP]
    
    reg is_exception_reg;
    assign is_exception = is_exception_reg;

    reg[31:0] csrs[0:31];
    assign mtvec = csrs[`CSR_MTVEC];
    assign rdata = csrs[raddr];

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
            `EXCEPT_U_ECALL: begin
                if (mode == USER) is_exception_reg = 1'b1;
            end
            `EXCEPT_EBREAK: begin
                if (mode == USER) is_exception_reg = 1'b1;
            end
            default: begin end
        endcase
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
            if (mem_done) begin // trigger current exception
                if (is_exception) begin
                    csrs[`CSR_MEPC] <= cur_exception_pc;
                    case (cur_exception)
                        `EXCEPT_U_ECALL: begin
                            mode <= MACHINE;
                            csrs[`CSR_MCAUSE] <= CAUSE_U_ECALL;
                        end
                        default: begin end
                    endcase
                end else if (`STATUS_MIE || mode < MACHINE) begin // maybe trigger interrupt
                    if (`MIE[IDX_TIMEOUT] && `MIP[IDX_TIMEOUT]) begin
                        csrs[`CSR_MEPC] <= cur_exception_pc;
                    end
                end   
            // for accurate interrupt, I won't optimize the ram again.
            end else begin // handle other commands (csrrs etc.)
                if (mode == MACHINE) csrs[waddr] <= wdata;
                // set MIP
                case (cur_exception)
                    `EXCEPT_TIMEOUT: begin
                        if (`MIE[IDX_TIMEOUT]) begin
                            `MIP[IDX_TIMEOUT] <= 1'b1; 
                        end
                    end
                    default: begin end
                endcase
            end
        end
    end

endmodule

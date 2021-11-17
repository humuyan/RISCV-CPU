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
    input wire          pipeline_empty,
    input wire[3:0]     cur_exception,
    input wire[31:0]    cur_exception_pc,
    input wire[31:0]    tval,
    input wire[63:0]    mtime,

    output wire[3:0]    timeout_exception,
    output wire         exception_handle_by_s,
    output wire[31:0]   mtvec,
    output wire[31:0]   mepc,
    output wire[31:0]   stvec,
    output wire[31:0]   sepc,
    output wire[31:0]   satp,
    output reg[31:0]    rdata,
    output wire[1:0]    mode_out,
    output wire         is_pending_exception,
    output wire[3:0]    pending_exception_out
    );

    localparam MEM_IF = 1;

    `define MSTATUS csrs[`CSR_MSTATUS]
    `define STATUS_MPP csrs[`CSR_MSTATUS][12:11]
    `define STATUS_MIE csrs[`CSR_MSTATUS][3]
    `define STATUS_MPIE csrs[`CSR_MSTATUS][7]
    `define STATUS_SPP csrs[`CSR_MSTATUS][8]
    `define STATUS_SIE csrs[`CSR_MSTATUS][1]
    `define STATUS_SPIE csrs[`CSR_MSTATUS][5]

    `define MIE csrs[`CSR_MIE]
    `define MIP csrs[`CSR_MIP]
    `define SIE csrs[`CSR_MIE]
    `define SIP csrs[`CSR_MIP]
    `define MIDELEG csrs[`CSR_MIDELEG]
    
    reg is_exception_reg;
    assign is_pending_exception = is_exception_reg;

    reg[3:0] pending_exception;
    assign pending_exception_out = pending_exception;
    reg[31:0] pending_exception_pc;

    assign timeout_exception = (m_timeout && pending_exception == `EXCEPT_NONE) ? `EXCEPT_M_TIMEOUT : 
                               (s_timeout && pending_exception == `EXCEPT_NONE) ? `EXCEPT_S_TIMEOUT :
                               `EXCEPT_NONE;

    reg[31:0] csrs[0:31];
    assign mtvec = csrs[`CSR_MTVEC];
    assign mepc = csrs[`CSR_MEPC];
    assign stvec = csrs[`CSR_STVEC];
    assign sepc = csrs[`CSR_SEPC];
    assign satp = csrs[`CSR_SATP];
    always_comb begin
        case (raddr)
            `CSR_ZERO: rdata = 32'b0;
            `CSR_MTIME: rdata = mtime[31:0];
            `CSR_MTIMEH: rdata = mtime[63:32];
            `CSR_SIE: rdata = csrs[`CSR_MIE];
            `CSR_SIP: rdata = csrs[`CSR_MIP];
            `CSR_SSTATUS: rdata = csrs[`CSR_MSTATUS];
            default: rdata = csrs[raddr];
        endcase
    end

    reg[31:0] pending_exception_tval;

    assign exception_handle_by_s = csrs[`CSR_MEDELEG][{1'b0, pending_exception}] || (pending_exception == `EXCEPT_S_TIMEOUT && `MIDELEG[IDX_S_TIMEOUT]);

    typedef enum reg[1:0] { USER, SUPERVISOR, MACHINE } mode_t;
    mode_t mode;
    assign mode_out = mode;

    typedef enum reg[31:0] {
        CAUSE_U_ECALL = 32'h00000008,
        CAUSE_S_ECALL = 32'h00000009,
        CAUSE_EBREAK = 32'h00000003,
        CAUSE_ILLEGAL_INST = 32'h00000002,
        CAUSE_INST_PAGE_FAULT = 32'h0000000C,
        CAUSE_LOAD_PAGE_FAULT = 32'h0000000D,
        CAUSE_STORE_PAGE_FAULT = 32'h0000000F,
        CAUSE_S_TIMEOUT = 32'h80000005,
        CAUSE_M_TIMEOUT = 32'h80000007
    } cause_t;

    typedef enum integer {
        IDX_M_TIMEOUT = 7,
        IDX_S_TIMEOUT = 5
    } interrupt_index_t;

    wire m_timeout, s_timeout;
    assign m_timeout = (`STATUS_MIE || mode < MACHINE) && `MIE[IDX_M_TIMEOUT] && (`MIP[IDX_M_TIMEOUT] || mtip);
    assign s_timeout = `SIE[IDX_S_TIMEOUT] && `SIP[IDX_S_TIMEOUT] && ((mode == SUPERVISOR && `STATUS_SIE) || mode < SUPERVISOR);
    always_comb begin
        is_exception_reg = 1'b0;
        // pending_exception or timeout, is_pending_exception will be 1.
        case (pending_exception)
            `EXCEPT_U_ECALL: if (mode == USER) is_exception_reg = 1'b1;
            `EXCEPT_S_ECALL: if (mode == SUPERVISOR) is_exception_reg = 1'b1;
            `EXCEPT_EBREAK: if (mode == USER) is_exception_reg = 1'b1;
            `EXCEPT_MRET: if (mode == MACHINE) is_exception_reg = 1'b1;
            `EXCEPT_SRET: if (mode == SUPERVISOR) is_exception_reg = 1'b1;
            `EXCEPT_INST_PAGE_FAULT: is_exception_reg = 1'b1;
            `EXCEPT_LOAD_PAGE_FAULT: is_exception_reg = 1'b1;
            `EXCEPT_STORE_PAGE_FAULT: is_exception_reg = 1'b1;
            `EXCEPT_ILLEGAL_INST: is_exception_reg = 1'b1;
            `EXCEPT_M_TIMEOUT: is_exception_reg = 1'b1;
            `EXCEPT_S_TIMEOUT: is_exception_reg = 1'b1;
            default: begin end
        endcase
    end

    always_ff @(posedge clk or negedge rst) begin
        if (rst) begin
            mode <= MACHINE;
            pending_exception <= `EXCEPT_NONE;
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
            // mem_done with cur_exception -> pending_exception && is_pending_exception -> pipeline poured ~ jump
            if (mem_done && is_pending_exception && pipeline_empty) begin
                // pipeline poured, handle exception.
                case (pending_exception)
                    `EXCEPT_U_ECALL, `EXCEPT_S_ECALL, `EXCEPT_EBREAK, `EXCEPT_ILLEGAL_INST, 
                    `EXCEPT_INST_PAGE_FAULT, `EXCEPT_LOAD_PAGE_FAULT, `EXCEPT_STORE_PAGE_FAULT: begin
                        if (exception_handle_by_s) begin // go to S state
                            `STATUS_SIE <= 1'b0;
                            `STATUS_SPIE <= `STATUS_SIE;
                            csrs[`CSR_SEPC] <= pending_exception_pc;
                            csrs[`CSR_STVAL] <= pending_exception_tval;
                            `STATUS_SPP <= mode[0];
                            mode <= SUPERVISOR;
                            case (pending_exception)
                                `EXCEPT_U_ECALL: csrs[`CSR_SCAUSE] <= CAUSE_U_ECALL;
                                `EXCEPT_S_ECALL: csrs[`CSR_SCAUSE] <= CAUSE_S_ECALL;
                                `EXCEPT_EBREAK: csrs[`CSR_SCAUSE] <= CAUSE_EBREAK;
                                `EXCEPT_ILLEGAL_INST: csrs[`CSR_SCAUSE] <= CAUSE_ILLEGAL_INST;
                                `EXCEPT_INST_PAGE_FAULT: csrs[`CSR_SCAUSE] <= CAUSE_INST_PAGE_FAULT;
                                `EXCEPT_LOAD_PAGE_FAULT: csrs[`CSR_SCAUSE] <= CAUSE_LOAD_PAGE_FAULT;
                                `EXCEPT_STORE_PAGE_FAULT: csrs[`CSR_SCAUSE] <= CAUSE_STORE_PAGE_FAULT;
                                default: begin end
                            endcase
                        end else begin // handle in M state
                            `STATUS_MIE <= 1'b0;
                            `STATUS_MPIE <= `STATUS_MIE;
                            csrs[`CSR_MEPC] <= pending_exception_pc;
                            csrs[`CSR_MTVAL] <= pending_exception_tval;
                            `STATUS_MPP <= mode;
                            mode <= MACHINE;
                            case (pending_exception)
                                `EXCEPT_U_ECALL: csrs[`CSR_MCAUSE] <= CAUSE_U_ECALL;
                                `EXCEPT_S_ECALL: csrs[`CSR_MCAUSE] <= CAUSE_S_ECALL;
                                `EXCEPT_EBREAK: csrs[`CSR_MCAUSE] <= CAUSE_EBREAK;
                                `EXCEPT_ILLEGAL_INST: csrs[`CSR_MCAUSE] <= CAUSE_ILLEGAL_INST;
                                `EXCEPT_INST_PAGE_FAULT: csrs[`CSR_MCAUSE] <= CAUSE_INST_PAGE_FAULT;
                                `EXCEPT_LOAD_PAGE_FAULT: csrs[`CSR_MCAUSE] <= CAUSE_LOAD_PAGE_FAULT;
                                `EXCEPT_STORE_PAGE_FAULT: csrs[`CSR_MCAUSE] <= CAUSE_STORE_PAGE_FAULT;
                                default: begin end
                            endcase
                        end
                    end
                    `EXCEPT_MRET: begin
                        `STATUS_MIE <= `STATUS_MPIE;
                        `STATUS_MPP <= 2'b00;
                        mode <= mode_t'(`STATUS_MPP);
                    end
                    `EXCEPT_SRET: begin
                        `STATUS_SIE <= `STATUS_SPIE;
                        `STATUS_SPP <= 1'b0;
                        mode <= mode_t'({1'b0, `STATUS_SPP});
                    end
                    `EXCEPT_M_TIMEOUT: begin
                        `STATUS_MIE <= 1'b0;
                        `STATUS_MPIE <= `STATUS_MIE;
                        csrs[`CSR_MEPC] <= pending_exception_pc;
                        csrs[`CSR_MCAUSE] <= CAUSE_M_TIMEOUT;
                        csrs[`CSR_MTVAL] <= 32'h0; // mtval is of no use here
                        `STATUS_MPP <= mode;
                        mode <= MACHINE;
                    end
                    `EXCEPT_S_TIMEOUT: begin
                        if (`MIDELEG[IDX_S_TIMEOUT]) begin
                            `STATUS_SIE <= 1'b0;
                            `STATUS_SPIE <= `STATUS_SIE;
                            csrs[`CSR_SEPC] <= pending_exception_pc;
                            csrs[`CSR_SCAUSE] <= CAUSE_S_TIMEOUT;
                            csrs[`CSR_STVAL] <= 32'h0;
                            `STATUS_SPP <= mode[0];
                            mode <= SUPERVISOR;
                        end else begin
                            `STATUS_MIE <= 1'b0;
                            `STATUS_MPIE <= `STATUS_MIE;
                            csrs[`CSR_MEPC] <= pending_exception_pc;
                            csrs[`CSR_MCAUSE] <= CAUSE_S_TIMEOUT;
                            csrs[`CSR_MTVAL] <= 32'h0;
                            `STATUS_MPP <= mode;
                            mode <= MACHINE;
                        end
                    end
                    default: begin end
                endcase
                pending_exception <= `EXCEPT_NONE;
                pending_exception_pc <= 32'h0;
            end else begin
                // raised exception, save it. (ready to handle exception, handle only one)
                if (mem_done && pending_exception == `EXCEPT_NONE) begin
                    if (cur_exception != `EXCEPT_NONE) begin
                        pending_exception <= cur_exception;
                        pending_exception_pc <= cur_exception_pc;
                        pending_exception_tval <= tval;                 
                    end
                end
                // handle other commands (csrrs etc.)
                if (mem_done && mem_occupied_by == MEM_IF && we && mode != USER) begin
                    case (waddr)
                        `CSR_SIE: csrs[`CSR_MIE] <= wdata;
                        `CSR_SIP: csrs[`CSR_MIP] <= wdata;
                        `CSR_SSTATUS: csrs[`CSR_MSTATUS] <= {`MSTATUS[31:9], wdata[8], `MSTATUS[7:6], wdata[5:4], `MSTATUS[3:2], wdata[1:0]};
                        default: csrs[waddr] <= wdata;
                    endcase
                end
                // set MIP
                `MIP[IDX_M_TIMEOUT] <= mtip;
            end
        end
    end

endmodule

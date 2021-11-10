`default_nettype none
`timescale 1ns / 1ps
`include "mem.vh"
`include "csr.vh"

module mem (
    input wire[4:0] inst,
    input wire[31:0] addr,
    input wire[31:0] data_in,
    input wire clk,
    input wire rst,
    input wire[1:0] mem_occupied_by,
    input wire[1:0] mode,
    input wire[31:0] satp,

    output wire[63:0] mtime_out,
    output reg[3:0] cur_exception,
    output wire mtip,
    output wire done,
    output wire idle,
    output wire[31:0] data_out,
    // ram
    inout wire[31:0] base_ram_data,  //BaseRAM数据，低8位与CPLD串口控制器共享
    output wire[19:0] base_ram_addr, //BaseRAM地址
    output wire[3:0] base_ram_be_n,
    output wire base_ram_ce_n,       //BaseRAM片选，低有效
    output wire base_ram_oe_n,       //BaseRAM读使能，低有效
    output wire base_ram_we_n,       //BaseRAM写使能，低有效

    inout wire[31:0] ext_ram_data,  //ExtRAM数据
    output wire[19:0] ext_ram_addr, //ExtRAM地址
    output wire[3:0] ext_ram_be_n,
    output wire ext_ram_ce_n,       //ExtRAM片选，低有效
    output wire ext_ram_oe_n,       //ExtRAM读使能，低有效
    output wire ext_ram_we_n,       //ExtRAM写使能，低有效

    // uart
    output wire uart_rdn,         //读串口信号，低有效
    output wire uart_wrn,         //写串口信号，低有效
    input wire uart_dataready,    //串口数据准备好
    input wire uart_tbre,         //发送数据标志
    input wire uart_tsre         //数据发送完毕标志
);

typedef enum reg[1:0] { USER, SUPERVISOR, MACHINE } mode_t;

reg[8:0] done_ram_enable;
reg[8:0] ram_enable;
reg[31:0] cur_addr;
reg[31:0] cur_data_in;
reg[31:0] cur_data_out;
reg[3:0] ram_be_n;
reg[63:0] mtime;
reg[63:0] mtimecmp;

assign mtime_out = mtime;

localparam IDLE = 9'b111111111;
localparam READ_BASE = 9'b100111111;
localparam READ_EXT = 9'b111100111;
localparam WRITE_BASE = 9'b101011111;
localparam WRITE_EXT = 9'b111101011;
localparam READ_UART = 9'b111111101;
localparam WRITE_UART = 9'b111111110;

assign base_ram_addr = cur_addr[19:0];
assign ext_ram_addr = cur_addr[19:0];
assign base_ram_data = ((ram_enable == WRITE_BASE) || (ram_enable == WRITE_UART)) ? cur_data_in : 32'bz;
assign ext_ram_data = (ram_enable == WRITE_EXT) ? cur_data_in : 32'bz;
assign data_out = cur_data_out;
assign base_ram_be_n = ram_be_n;
assign ext_ram_be_n = ram_be_n;

typedef enum reg[4:0] {
    S_IDLE,
    S_RW_RAM_U,
    S_RW_RAM_M,
    S_READ_UART,
    S_WRITE_UART,
    S_READ_UART_GAP,
    S_READ_UART_GAP_1,
    S_WRITE_UART_WAIT_TBRE,
    S_WRITE_UART_WAIT_TBRE_1,
    S_WRITE_UART_WAIT_TSRE,
    S_WRITE_UART_WAIT_TSRE_1,
    S_DONE_U,
    S_DONE_M,
    S_IDLE_U,
    S_READ_L1_PAGE_TABLE,
    S_READ_L1_PAGE_TABLE_1,
    S_READ_L2_PAGE_TABLE,
    S_READ_L2_PAGE_TABLE_1
} mem_state_t;
mem_state_t state, next_state;

assign {
    base_ram_ce_n, base_ram_oe_n, base_ram_we_n, 
    ext_ram_ce_n, ext_ram_oe_n, ext_ram_we_n,
    uart_rdn, uart_wrn
} = ram_enable[7:0];

wire base_byte_sign;
assign base_byte_sign = mapped[1:0] == 2'b00 ? base_ram_data[7] : 
                        (mapped[1:0] == 2'b01 ? base_ram_data[15] :
                        (mapped[1:0] == 2'b10 ? base_ram_data[23] : base_ram_data[31]));

wire[7:0] base_byte_data;
assign base_byte_data = mapped[1:0] == 2'b00 ? base_ram_data[7:0] : 
                        (mapped[1:0] == 2'b01 ? base_ram_data[15:8] :
                        (mapped[1:0] == 2'b10 ? base_ram_data[23:16] : base_ram_data[31:24]));

wire base_half_sign;
assign base_half_sign = mapped[1:0] == 2'b00 ? base_ram_data[15] : base_ram_data[31];

wire[15:0] base_half_data;
assign base_half_data = mapped[1:0] == 2'b00 ? base_ram_data[15:0] : base_ram_data[31:16];


wire ext_byte_sign;
assign ext_byte_sign = mapped[1:0] == 2'b00 ? ext_ram_data[7] : 
                       (mapped[1:0] == 2'b01 ? ext_ram_data[15] :
                       (mapped[1:0] == 2'b10 ? ext_ram_data[23] : ext_ram_data[31]));

wire[7:0] ext_byte_data;
assign ext_byte_data = mapped[1:0] == 2'b00 ? ext_ram_data[7:0] : 
                       (mapped[1:0] == 2'b01 ? ext_ram_data[15:8] :
                       (mapped[1:0] == 2'b10 ? ext_ram_data[23:16] : ext_ram_data[31:24]));

wire ext_half_sign;
assign ext_half_sign = mapped[1:0] == 2'b00 ? ext_ram_data[15] : ext_ram_data[31];

wire[15:0] ext_half_data;
assign ext_half_data = mapped[1:0] == 2'b00 ? ext_ram_data[15:0] : ext_ram_data[31:16];

typedef enum reg[1:0] { PAGE_NORMAL, PAGE_FAULT_READ, PAGE_FAULT_WRITE } page_status_t;
page_status_t page_status, done_page_status;
assign mtip = mtime >= mtimecmp;

localparam MEM_NONE = 0;
localparam MEM_IF = 1;
localparam MEM_MEM = 2;
always_comb begin
    cur_exception = `EXCEPT_NONE;
    case (mem_occupied_by)
        MEM_IF: if (mode != MACHINE && page_status == PAGE_FAULT_READ) cur_exception = `EXCEPT_INST_PAGE_FAULT;
        MEM_MEM: case (page_status)
            PAGE_FAULT_READ: cur_exception = `EXCEPT_LOAD_PAGE_FAULT;
            PAGE_FAULT_WRITE: cur_exception = `EXCEPT_STORE_PAGE_FAULT;
            default: begin end
        endcase
    endcase
end

wire page_enabled;
assign page_enabled = (mode_t'(mode) != MACHINE && satp[31] == `PAGING_SV32);
wire[31:0] mapped = page_enabled ? page_addr : addr;
reg[31:0] page_table_data;
reg[31:0] page_addr;

`define SATP_PPN satp[19:0]
`define PTE_PPN page_table_data[29:10]
always_comb begin
    case (state)
        S_IDLE_U, S_READ_L1_PAGE_TABLE, S_READ_L1_PAGE_TABLE_1: page_addr = {`SATP_PPN, addr[31:22], 2'b0};
        S_READ_L2_PAGE_TABLE, S_READ_L2_PAGE_TABLE_1: page_addr = {`PTE_PPN, addr[21:12], 2'b0}; 
        S_RW_RAM_U, S_DONE_U: page_addr = {`PTE_PPN, addr[11:0]};
        default: page_addr = 32'h0;
    endcase
end

always_comb begin
    cur_data_out = 32'b0;
    case (inst[3:2])
        `MEM_READ: casez (mapped)
            `MEM_BASE: begin
                case (inst[1:0])
                    `MEM_BYTE: cur_data_out = {{24{inst[4] == `SIGN_EXT ? base_byte_sign : 1'b0}}, base_byte_data};
                    `MEM_HALF: cur_data_out = {{16{inst[4] == `SIGN_EXT ? base_half_sign : 1'b0}}, base_half_data};
                    `MEM_WORD: cur_data_out = base_ram_data;
                    default: begin end
                endcase 
            end
            `MEM_EXT: begin
                case (inst[1:0])
                    `MEM_BYTE: cur_data_out = {{24{inst[4] == `SIGN_EXT ? ext_byte_sign : 1'b0}}, ext_byte_data};
                    `MEM_HALF: cur_data_out = {{16{inst[4] == `SIGN_EXT ? ext_half_sign : 1'b0}}, ext_half_data};
                    `MEM_WORD: cur_data_out = ext_ram_data;
                    default: begin end
                endcase
            end
            `MEM_UART: begin
                case (mapped[2:0])
                    3'b101: cur_data_out = { 26'b0, uart_tbre && uart_tsre, 4'b0, uart_dataready }; // state
                    3'b000: cur_data_out = { 24'b0, base_ram_data[7:0] }; // data
                    default: begin end
                endcase
            end
            `MEM_MTIME_LO: cur_data_out = mtime[31:0];
            `MEM_MTIME_HI: cur_data_out = mtime[63:32];
            `MEM_MTIMECMP_LO: cur_data_out = mtimecmp[31:0];
            `MEM_MTIMECMP_HI: cur_data_out = mtimecmp[63:32];
        endcase
        default: begin end
    endcase
end

assign done = ((~page_enabled) && next_state == S_RW_RAM_M) || (page_enabled && next_state == S_IDLE_U);
assign idle = (state == S_IDLE) ? 1'b1 : 1'b0;

reg[3:0] done_be_n;
// This logic is only for idle / read_ram / write_ram
always_comb begin
    ram_be_n = 4'b0000;
    ram_enable = IDLE;
    cur_addr = { 12'b0, mapped[21:2] };
    cur_data_in = data_in;
    next_state = S_DONE_M; // default, next state is DONE.
    page_status = PAGE_NORMAL;
    case (state)
        S_IDLE, S_RW_RAM_M, S_RW_RAM_U: begin
            case (state)
                S_IDLE: next_state = S_DONE_M;
                S_RW_RAM_M: next_state = S_DONE_M;
                S_RW_RAM_U: next_state = S_DONE_U;
                default: begin end
            endcase
            case (inst[3:2])
                `MEM_READ: casez (mapped)
                    `MEM_BASE: ram_enable = READ_BASE;
                    `MEM_EXT: ram_enable = READ_EXT;
                    `MEM_UART: begin
                        case (mapped[2:0])
                            3'b000: next_state = S_READ_UART; // Please annotate this if using async uart
                            default: begin end
                        endcase
                    end
                    `MEM_MTIME_LO, `MEM_MTIME_HI, `MEM_MTIMECMP_LO, `MEM_MTIMECMP_HI: begin end
                    default: page_status = PAGE_FAULT_READ;
                endcase
                `MEM_WRITE: casez (mapped)
                    `MEM_BASE, `MEM_EXT: begin
                        casez (mapped)
                            `MEM_BASE: ram_enable = WRITE_BASE;
                            `MEM_EXT: ram_enable = WRITE_EXT;
                            default: begin end
                        endcase
                        case (inst[1:0]) 
                            `MEM_BYTE: begin
                                case (mapped[1:0])
                                    2'b00: begin
                                        ram_be_n = 4'b1110;
                                        cur_data_in = {24'b0, data_in[7:0]}; 
                                    end
                                    2'b01: begin
                                        ram_be_n = 4'b1101;
                                        cur_data_in = {16'b0, data_in[7:0], 8'b0};
                                    end
                                    2'b10: begin
                                        ram_be_n = 4'b1011;
                                        cur_data_in = {8'b0, cur_data_in[7:0], 16'b0};
                                    end
                                    2'b11: begin
                                        ram_be_n = 4'b0111;
                                        cur_data_in = {cur_data_in[7:0], 24'b0};  
                                    end
                                endcase         
                            end
                            `MEM_HALF: begin
                                case (mapped[1:0])
                                    2'b00: begin
                                        ram_be_n = 4'b1100;
                                        cur_data_in = {16'b0, data_in[15:0]};
                                    end
                                    2'b10: begin
                                        ram_be_n = 4'b0011;
                                        cur_data_in = {data_in[15:0], 16'b0};
                                    end
                                    default: begin end // data must be assigned
                                endcase
                            end
                            default: begin end
                        endcase
                    end
                    `MEM_UART: begin
                        case (mapped[2:0])
                            3'b000: begin
                                ram_enable = WRITE_UART;
                                next_state = S_WRITE_UART; // Please annotate this if using async uart
                            end
                            default: begin end
                        endcase
                    end
                    `MEM_MTIME_LO, `MEM_MTIME_HI, `MEM_MTIMECMP_LO, `MEM_MTIMECMP_HI: begin end
                    default: page_status = PAGE_FAULT_WRITE;
                endcase
                default: begin end
            endcase
        end
        S_READ_UART: begin
            if (uart_dataready == 1) begin
                next_state = S_READ_UART_GAP;
            end else begin
                next_state = S_READ_UART;
            end
        end   
        S_READ_UART_GAP: begin
            ram_enable = READ_UART; // uart_dataready == 1
            next_state = S_READ_UART_GAP_1;
        end
        S_READ_UART_GAP_1: begin
            ram_enable = READ_UART;
            next_state = S_DONE_M;
        end
        S_WRITE_UART: begin
            ram_enable = WRITE_UART;
            next_state = S_WRITE_UART_WAIT_TBRE;
        end
        S_WRITE_UART_WAIT_TBRE: begin
            // ram_enable = IDLE;
            if (uart_tbre == 1) next_state = S_WRITE_UART_WAIT_TBRE_1; 
            else next_state = S_WRITE_UART_WAIT_TBRE;
        end
        S_WRITE_UART_WAIT_TBRE_1: begin
            next_state = S_WRITE_UART_WAIT_TSRE;
        end
        S_WRITE_UART_WAIT_TSRE: begin
            if (uart_tsre == 1) next_state = S_WRITE_UART_WAIT_TSRE_1;
            else next_state = S_WRITE_UART_WAIT_TSRE;
        end
        S_WRITE_UART_WAIT_TSRE_1: begin
            next_state = S_DONE_M;
        end
        S_DONE_M, S_DONE_U: begin
            ram_be_n = done_be_n;
            ram_enable = done_ram_enable;
            page_status = done_page_status;
            case (state)
                S_DONE_M: next_state = page_enabled ? S_READ_L1_PAGE_TABLE : S_RW_RAM_M;
                S_DONE_U: next_state = page_enabled ? S_IDLE_U : S_IDLE;
                default: begin end
            endcase
        end
        S_IDLE_U, S_READ_L1_PAGE_TABLE, S_READ_L1_PAGE_TABLE_1, S_READ_L2_PAGE_TABLE, S_READ_L2_PAGE_TABLE_1: begin
            case (state)
                S_IDLE_U: next_state = S_READ_L1_PAGE_TABLE;
                S_READ_L1_PAGE_TABLE: next_state = S_READ_L1_PAGE_TABLE_1;
                S_READ_L1_PAGE_TABLE_1: next_state = S_READ_L2_PAGE_TABLE;
                S_READ_L2_PAGE_TABLE: next_state = S_READ_L2_PAGE_TABLE_1;
                S_READ_L2_PAGE_TABLE_1: next_state = S_RW_RAM_U;
                default: next_state = S_DONE_U;
            endcase
            casez (page_addr)
                `MEM_BASE: ram_enable = READ_BASE;
                `MEM_EXT: ram_enable = READ_EXT;
                default: begin
                    case (inst[3:2])
                        `MEM_READ: page_status = PAGE_FAULT_READ;
                        `MEM_WRITE: page_status = PAGE_FAULT_WRITE;
                        default: page_status = PAGE_NORMAL;
                    endcase
                    next_state = S_DONE_U;
                end
            endcase
        end
        default: next_state = S_IDLE;
    endcase
end

always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
        state <= S_IDLE;
        mtime <= 64'h0;
        mtimecmp <= 64'hFFFFFFFF;
    end else begin
        state <= next_state;
        if (next_state == S_DONE_M || next_state == S_DONE_U) begin
            done_ram_enable <= ram_enable;
            done_be_n <= ram_be_n;
            done_page_status = page_status;
        end

        if (inst[3:2] == `MEM_WRITE) begin
            casez (mapped)
                `MEM_MTIME_LO: mtime <= { mtime[63:32], cur_data_in };
                `MEM_MTIME_HI: mtime <= { cur_data_in, mtime[31:0] };
                `MEM_MTIMECMP_LO: mtimecmp <= { mtimecmp[63:32], cur_data_in };
                `MEM_MTIMECMP_HI: mtimecmp <= { cur_data_in, mtimecmp[31:0] };
            endcase
        end else begin
            mtime <= mtime + 1;
        end

        // page data
        case (next_state)
            S_READ_L2_PAGE_TABLE, S_RW_RAM_U: begin
                casez (page_addr)
                    `MEM_BASE: page_table_data <= base_ram_data;
                    `MEM_EXT: page_table_data <= ext_ram_data;
                    default: page_table_data <= 32'h0;
                endcase
            end
            default: begin end
        endcase
        
    end
end

    
endmodule

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
    
    output reg[3:0] cur_exception,
    output wire done,
    output wire idle,
    output wire[31:0] data_out,
    output wire[3:0] state_out,
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

reg[8:0] done_ram_enable;
reg[8:0] ram_enable;
reg[31:0] cur_addr;
reg[31:0] cur_data_in;
reg[31:0] cur_data_out;
reg[3:0] ram_be_n;
reg[63:0] mtime;
reg[63:0] mtimecmp;

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
assign state_out = state;
assign base_ram_be_n = ram_be_n;
assign ext_ram_be_n = ram_be_n;

reg[3:0] state;
localparam S_IDLE = 0;
localparam S_RW_RAM = 1;
localparam S_READ_UART = 2;
localparam S_WRITE_UART = 3;
localparam S_READ_UART_GAP = 4;
localparam S_READ_UART_GAP_1 = 5;
localparam S_WRITE_UART_WAIT_TBRE = 6;
localparam S_WRITE_UART_WAIT_TBRE_1 = 7;
localparam S_WRITE_UART_WAIT_TSRE = 8;
localparam S_WRITE_UART_WAIT_TSRE_1 = 9;
localparam S_DONE = 10;

assign {
    base_ram_ce_n, base_ram_oe_n, base_ram_we_n, 
    ext_ram_ce_n, ext_ram_oe_n, ext_ram_we_n,
    uart_rdn, uart_wrn
} = ram_enable[7:0];

wire base_byte_sign;
assign base_byte_sign = addr[1:0] == 2'b00 ? base_ram_data[7] : 
                        (addr[1:0] == 2'b01 ? base_ram_data[15] :
                        (addr[1:0] == 2'b10 ? base_ram_data[23] : base_ram_data[31]));

wire[7:0] base_byte_data;
assign base_byte_data = addr[1:0] == 2'b00 ? base_ram_data[7:0] : 
                        (addr[1:0] == 2'b01 ? base_ram_data[15:8] :
                        (addr[1:0] == 2'b10 ? base_ram_data[23:16] : base_ram_data[31:24]));

wire base_half_sign;
assign base_half_sign = addr[1:0] == 2'b00 ? base_ram_data[15] : base_ram_data[31];

wire[15:0] base_half_data;
assign base_half_data = addr[1:0] == 2'b00 ? base_ram_data[15:0] : base_ram_data[31:16];


wire ext_byte_sign;
assign ext_byte_sign = addr[1:0] == 2'b00 ? ext_ram_data[7] : 
                       (addr[1:0] == 2'b01 ? ext_ram_data[15] :
                       (addr[1:0] == 2'b10 ? ext_ram_data[23] : ext_ram_data[31]));

wire[7:0] ext_byte_data;
assign ext_byte_data = addr[1:0] == 2'b00 ? ext_ram_data[7:0] : 
                       (addr[1:0] == 2'b01 ? ext_ram_data[15:8] :
                       (addr[1:0] == 2'b10 ? ext_ram_data[23:16] : ext_ram_data[31:24]));

wire ext_half_sign;
assign ext_half_sign = addr[1:0] == 2'b00 ? ext_ram_data[15] : ext_ram_data[31];

wire[15:0] ext_half_data;
assign ext_half_data = addr[1:0] == 2'b00 ? ext_ram_data[15:0] : ext_ram_data[31:16];

always_comb begin
    if (mtime >= mtimecmp) cur_exception = `EXCEPT_TIMEOUT;
    else cur_exception = `EXCEPT_NONE;
end

always_comb begin
    cur_data_out = 32'b0;
    case (inst[3:2])
        `MEM_READ: casez (addr)
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
                case (addr[2:0])
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
        default: begin 
            // TODO: raise PAGE_FAULT
        end
    endcase
end

assign done = next_state == S_RW_RAM ? 1'b1 : 1'b0;
assign idle = (state == S_IDLE) ? 1'b1 : 1'b0;

reg[3:0] done_be_n;
// Try to accelerate R&W of memory to 1 cycle.
reg[3:0] next_state;
// This logic is only for idle / read_ram / write_ram
always_comb begin
    ram_be_n = 4'b0000;
    ram_enable = IDLE;
    cur_addr = { 12'b0, addr[21:2] };
    cur_data_in = data_in;
    next_state = S_DONE; // default, next state is DONE.
    case (state)
        S_IDLE, S_RW_RAM: begin
            case (inst[3:2])
                `MEM_READ: casez (addr)
                    `MEM_BASE: ram_enable = READ_BASE;
                    `MEM_EXT: ram_enable = READ_EXT;
                    `MEM_UART: begin
                        case (addr[2:0])
                            3'b101: next_state = S_DONE; // state
                            3'b000: next_state = S_READ_UART; // Please annotate this if using async uart
                            default: next_state = S_DONE; // ignore other addresses except 0x10000000 and 0x10000005
                        endcase
                    end
                    `MEM_MTIME_LO, `MEM_MTIME_HI: next_state = S_DONE;
                    `MEM_MTIMECMP_LO, `MEM_MTIMECMP_HI: next_state = S_DONE;
                endcase
                `MEM_WRITE: casez (addr)
                    `MEM_BASE, `MEM_EXT: begin
                        casez (addr)
                            `MEM_BASE: ram_enable = WRITE_BASE;
                            `MEM_EXT: ram_enable = WRITE_EXT;
                            default: begin end
                        endcase
                        case (inst[1:0]) 
                            `MEM_BYTE: begin
                                case (addr[1:0])
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
                                case (addr[1:0])
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
                        case (addr[2:0])
                            3'b000: begin
                                ram_enable = WRITE_UART;
                                next_state = S_WRITE_UART; // Please annotate this if using async uart
                            end
                            default: next_state = S_DONE;
                        endcase
                    end
                    `MEM_MTIME_LO, `MEM_MTIME_HI: next_state = S_DONE;
                    `MEM_MTIMECMP_LO, `MEM_MTIMECMP_HI: next_state = S_DONE;
                endcase
                default: begin
                    next_state = S_DONE;        
                end
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
            next_state = S_DONE;
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
            next_state = S_DONE;
        end
        S_DONE: begin
            ram_be_n = done_be_n;
            ram_enable = done_ram_enable;
            next_state = S_RW_RAM;
        end
        default: next_state = S_IDLE;
    endcase
end

always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
        state <= S_IDLE;
    end else begin
        state <= next_state;
        if (next_state == S_DONE) begin
            done_ram_enable <= ram_enable;
            done_be_n <= ram_be_n;
        end
        else done_ram_enable <= IDLE;
        if (inst[3:2] == `MEM_WRITE) begin
            casez (addr)
                `MEM_MTIME_LO: mtime <= { mtime[63:32], cur_data_in };
                `MEM_MTIME_HI: mtime <= { cur_data_in, mtime[31:0] };
                `MEM_MTIMECMP_LO: mtimecmp <= { mtimecmp[63:32], cur_data_in };
                `MEM_MTIMECMP_HI: mtimecmp <= { cur_data_in, mtimecmp[31:0] };
            endcase
        end else begin
            mtime <= mtime + 1;
        end
    end
end
    
endmodule

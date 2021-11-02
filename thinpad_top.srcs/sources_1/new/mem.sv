`default_nettype none
`timescale 1ns / 1ps
`include "mem.vh"

module mem (
    input wire[3:0] inst,
    input wire[31:0] addr,
    input wire[31:0] data_in,
    input wire clk,
    input wire rst,
    
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

reg[8:0] ram_enable;
reg[31:0] cur_addr;
reg[31:0] cur_data_in;
reg[31:0] cur_data_out;

localparam IDLE = 9'b111111111;
localparam READ_BASE = 9'b100111111;
localparam READ_EXT = 9'b111100111;
localparam WRITE_BASE = 9'b101011111;
localparam WRITE_EXT = 9'b111101011;
localparam READ_UART = 9'b111111101;
localparam READ_UART_STATE = 9'b011111111;
localparam WRITE_UART = 9'b111111110;

assign base_ram_addr = cur_addr[19:0];
assign ext_ram_addr = cur_addr[19:0];
assign base_ram_data = ((ram_enable == WRITE_BASE) || (ram_enable == WRITE_UART)) ? cur_data_in : 32'bz;
assign ext_ram_data = (ram_enable == WRITE_EXT) ? cur_data_in : 32'bz;
assign data_out = cur_data_out;
assign state_out = state;
assign base_ram_be_n = 4'b0000;
assign ext_ram_be_n = 4'b0000;

reg[3:0] state;
localparam S_IDLE = 0;
localparam S_READ_RAM = 1;
localparam S_WRITE_RAM = 2;
localparam S_READ_UART = 3;
localparam S_WRITE_UART = 4;
localparam S_READ_UART_GAP = 5;
localparam S_READ_UART_GAP_2 = 6;
localparam S_WRITE_UART_WAIT_TBRE = 7;
localparam S_WRITE_UART_WAIT_TSRE = 8;
localparam S_READ_UART_STATE = 9;
localparam S_WRITE_RAM_BYTE_READ = 10;
localparam S_WRITE_RAM_BYTE_WRITE = 11;
localparam S_DONE = 12;


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


wire ext_byte_sign;
assign ext_byte_sign = addr[1:0] == 2'b00 ? ext_ram_data[7] : 
                       (addr[1:0] == 2'b01 ? ext_ram_data[15] :
                       (addr[1:0] == 2'b10 ? ext_ram_data[23] : ext_ram_data[31]));

wire[7:0] ext_byte_data;
assign ext_byte_data = addr[1:0] == 2'b00 ? ext_ram_data[7:0] : 
                       (addr[1:0] == 2'b01 ? ext_ram_data[15:8] :
                       (addr[1:0] == 2'b10 ? ext_ram_data[23:16] : ext_ram_data[31:24]));


always_comb begin
    cur_data_out = 32'b0;
    case (ram_enable)
        READ_BASE: case (inst[1:0])
            `MEM_BYTE: cur_data_out = {{24{base_byte_sign}}, base_byte_data};
            `MEM_WORD: cur_data_out = base_ram_data;
            default: begin end
        endcase
        READ_EXT: case (inst[1:0])
            `MEM_BYTE: cur_data_out = {{24{ext_byte_sign}}, ext_byte_data};
            `MEM_WORD: cur_data_out = ext_ram_data;
            default: begin end
        endcase
        READ_UART: cur_data_out = { 24'b0, base_ram_data[7:0] };
        READ_UART_STATE: cur_data_out = { 26'b0, uart_tbre && uart_tsre, 4'b0, uart_dataready };
        default: begin end
    endcase
end

assign done = (state == S_DONE || state == S_READ_RAM || state == S_WRITE_RAM) ? 1'b1 : 1'b0;
assign idle = (state == S_IDLE) ? 1'b1 : 1'b0;



always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
        ram_enable <= IDLE;
        cur_data_in <= 32'b0;
        cur_addr <= 32'b0;
        state <= S_IDLE;
    end else begin
        case (state)
            S_IDLE: begin
                cur_data_in <= data_in;
                cur_addr <= { 12'b0, addr[21:2] };
                case ({inst[3:2], addr[28], addr[22]})
                    `MEM_READ_BASE: begin
                        ram_enable <= READ_BASE;
                        state <= S_READ_RAM;
                    end
                    `MEM_WRITE_BASE: begin
                        case (inst[1:0]) 
                            `MEM_WORD: begin
                                ram_enable <= WRITE_BASE;
                                state <= S_WRITE_RAM;
                            end
                            `MEM_BYTE: begin
                                ram_enable <= READ_BASE; // read because I should read first and replace
                                case(addr[1:0])
                                    2'b00: cur_data_in <= {base_ram_data[31:8], cur_data_in[7:0]}; // little endian
                                    2'b01: cur_data_in <= {base_ram_data[31:16], cur_data_in[7:0], base_ram_data[7:0]};
                                    2'b10: cur_data_in <= {base_ram_data[31:24], cur_data_in[7:0], base_ram_data[15:0]};
                                    2'b11: cur_data_in <= {cur_data_in[7:0], base_ram_data[23:0]};                    
                                endcase
                                state <= S_WRITE_RAM_BYTE_READ;
                            end
                            default: begin end
                        endcase
                    end
                    `MEM_READ_EXT: begin
                        ram_enable <= READ_EXT;
                        state <= S_READ_RAM;
                    end
                    `MEM_WRITE_EXT: begin
                        case (inst[1:0]) 
                            `MEM_WORD: begin
                                ram_enable <= WRITE_EXT;
                                state <= S_WRITE_RAM;
                            end
                            `MEM_BYTE: begin
                                ram_enable <= READ_EXT; // read because I should read first and replace
                                state <= S_WRITE_RAM_BYTE_READ;
                            end
                            default: begin end
                        endcase
                    end
                    `MEM_READ_UART: begin
                        case (addr[2:0])
                            3'b101: begin
                                ram_enable <= READ_UART_STATE;
                                state <= S_READ_UART_STATE;
                            end
                            3'b000: state <= S_READ_UART;
                            default: state <= S_DONE; // ignore other addresses except 0x10000000 and 0x10000005
                        endcase
                    end
                    `MEM_WRITE_UART: begin
                        case (addr[2:0])
                            3'b000: begin
                                ram_enable <= WRITE_UART;
                                state <= S_WRITE_UART;
                            end
                            default: state <= S_DONE;
                        endcase
                    end
                    default: begin
                        ram_enable <= IDLE;
                        state <= S_DONE; // done -> idle                
                    end
                endcase
            end
            S_READ_RAM: begin
                ram_enable <= IDLE;
                state <= S_IDLE;
            end
            S_WRITE_RAM: begin
                ram_enable <= IDLE;
                state <= S_IDLE;
            end
            S_READ_UART: begin
                if (uart_dataready == 1) begin
                    ram_enable <= READ_UART;
                    state <= S_READ_UART_GAP;
                end else begin
                    state <= S_READ_UART;
                end
            end        
            S_READ_UART_GAP: state <= S_READ_UART_GAP_2;
            S_READ_UART_GAP_2: state <= S_DONE;
            S_WRITE_UART: begin
                ram_enable <= IDLE;
                state <= S_WRITE_UART_WAIT_TBRE;
            end 
            S_WRITE_UART_WAIT_TBRE: begin
                if (uart_tbre == 1) state <= S_WRITE_UART_WAIT_TSRE; 
                else state <= S_WRITE_UART_WAIT_TBRE;
            end
            S_WRITE_UART_WAIT_TSRE: begin
                if (uart_tsre == 1) state <= S_DONE;
                else state <= S_WRITE_UART_WAIT_TSRE;
            end
            S_READ_UART_STATE: begin
                state <= S_DONE;
            end
            S_WRITE_RAM_BYTE_READ: begin
                case (ram_enable)
                    READ_BASE: ram_enable <= WRITE_BASE;
                    READ_EXT: ram_enable <= WRITE_EXT;
                    default: begin end
                endcase
                state <= S_WRITE_RAM;
            end
            S_DONE: begin
                ram_enable <= IDLE;
                state <= S_IDLE;
            end
            default: begin end
        endcase
    end
end
    
endmodule

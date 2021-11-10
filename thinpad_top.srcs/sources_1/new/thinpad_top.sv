`default_nettype none
`include "alu.vh"
`include "ops.vh"
`include "mem.vh"
`include "csr.vh"

module thinpad_top(
    input wire clk_50M,           //50MHz 时钟输入
    input wire clk_11M0592,       //11.0592MHz 时钟输入（备用，可不用）

    input wire clock_btn,         //BTN5手动时钟按钮开关，带消抖电路，按下时为1
    input wire reset_btn,         //BTN6手动复位按钮开关，带消抖电路，按下时为1

    input  wire[3:0]  touch_btn,  //BTN1~BTN4，按钮开关，按下时为1
    input  wire[31:0] dip_sw,     //32位拨码开关，拨到“ON”时为1
    output wire[15:0] leds,       //16位LED，输出时1点亮
    output wire[7:0]  dpy0,       //数码管低位信号，包括小数点，输出1点亮
    output wire[7:0]  dpy1,       //数码管高位信号，包括小数点，输出1点亮

    //CPLD串口控制器信号
    output wire uart_rdn,         //读串口信号，低有效
    output wire uart_wrn,         //写串口信号，低有效
    input wire uart_dataready,    //串口数据准备好
    input wire uart_tbre,         //发送数据标志
    input wire uart_tsre,         //数据发送完毕标志

    //BaseRAM信号
    inout wire[31:0] base_ram_data,  //BaseRAM数据，低8位与CPLD串口控制器共享
    output wire[19:0] base_ram_addr, //BaseRAM地址
    output wire[3:0] base_ram_be_n,  //BaseRAM字节使能，低有效。如果不使用字节使能，请保持为0
    output wire base_ram_ce_n,       //BaseRAM片选，低有效
    output wire base_ram_oe_n,       //BaseRAM读使能，低有效
    output wire base_ram_we_n,       //BaseRAM写使能，低有效

    //ExtRAM信号
    inout wire[31:0] ext_ram_data,  //ExtRAM数据
    output wire[19:0] ext_ram_addr, //ExtRAM地址
    output wire[3:0] ext_ram_be_n,  //ExtRAM字节使能，低有效。如果不使用字节使能，请保持为0
    output wire ext_ram_ce_n,       //ExtRAM片选，低有效
    output wire ext_ram_oe_n,       //ExtRAM读使能，低有效
    output wire ext_ram_we_n,       //ExtRAM写使能，低有效

    //直连串口信号
    output wire txd,  //直连串口发送端
    input  wire rxd,  //直连串口接收端

    //Flash存储器信号，参考 JS28F640 芯片手册
    output wire [22:0]flash_a,      //Flash地址，a0仅在8bit模式有效，16bit模式无意义
    inout  wire [15:0]flash_d,      //Flash数据
    output wire flash_rp_n,         //Flash复位信号，低有效
    output wire flash_vpen,         //Flash写保护信号，低电平时不能擦除、烧写
    output wire flash_ce_n,         //Flash片选信号，低有效
    output wire flash_oe_n,         //Flash读使能信号，低有效
    output wire flash_we_n,         //Flash写使能信号，低有效
    output wire flash_byte_n,       //Flash 8bit模式选择，低有效。在使用flash的16位模式时请设为1

    //USB 控制器信号，参考 SL811 芯片手册
    output wire sl811_a0,
    //inout  wire[7:0] sl811_d,     //USB数据线与网络控制器的dm9k_sd[7:0]共享
    output wire sl811_wr_n,
    output wire sl811_rd_n,
    output wire sl811_cs_n,
    output wire sl811_rst_n,
    output wire sl811_dack_n,
    input  wire sl811_intrq,
    input  wire sl811_drq_n,

    //网络控制器信号，参考 DM9000A 芯片手册
    output wire dm9k_cmd,
    inout  wire[15:0] dm9k_sd,
    output wire dm9k_iow_n,
    output wire dm9k_ior_n,
    output wire dm9k_cs_n,
    output wire dm9k_pwrst_n,
    input  wire dm9k_int,

    //图像输出信号
    output wire[2:0] video_red,    //红色像素，3位
    output wire[2:0] video_green,  //绿色像素，3位
    output wire[1:0] video_blue,   //蓝色像素，2位
    output wire video_hsync,       //行同步（水平同步）信号
    output wire video_vsync,       //场同步（垂直同步）信号
    output wire video_clk,         //像素时钟输出
    output wire video_de           //行数据有效信号，用于区分消隐区
);

// 数码管连接关系示意图，dpy1同理
// p=dpy0[0] // ---a---
// c=dpy0[1] // |     |
// d=dpy0[2] // f     b
// e=dpy0[3] // |     |
// b=dpy0[4] // ---g---
// a=dpy0[5] // |     |
// f=dpy0[6] // e     c
// g=dpy0[7] // |     |
//           // ---d---  p

// 7段数码管译码器演示，将number用16进制显示在数码管上面
wire[7:0] number;
SEG7_LUT segL(.oSEG1(dpy0), .iDIG(number[3:0])); //dpy0是低位数码管
SEG7_LUT segH(.oSEG1(dpy1), .iDIG(number[7:4])); //dpy1是高位数码管
reg[7:0] num_reg;
assign number = num_reg;
assign leds = pc[15:0];

// interface to memory
reg[4:0] mem_inst;
reg[31:0] mem_addr;
reg[31:0] mem_data_in;
wire[31:0] mem_data_out;
wire[63:0] mtime;
wire mem_done;
wire mem_idle;
wire[3:0] mem_state;
reg[1:0] mem_occupied_by;

localparam MEM_NONE = 0;
localparam MEM_IF = 1;
localparam MEM_MEM = 2;

mem _mem(
    .inst(mem_inst),
    .addr(mem_addr),
    .mem_occupied_by(mem_occupied_by),
    .data_in(mem_data_in),
    .clk(clk_50M),
    .rst(reset_btn),
    .done(mem_done),
    .idle(mem_idle),
    .mode(mode),
    .satp(satp),
    .data_out(mem_data_out),
    .mtime_out(mtime),

    .base_ram_data(base_ram_data),
    .base_ram_addr(base_ram_addr),
    .base_ram_be_n(base_ram_be_n),
    .base_ram_ce_n(base_ram_ce_n),
    .base_ram_oe_n(base_ram_oe_n),
    .base_ram_we_n(base_ram_we_n),

    .ext_ram_data(ext_ram_data),
    .ext_ram_addr(ext_ram_addr),
    .ext_ram_be_n(ext_ram_be_n),
    .ext_ram_ce_n(ext_ram_ce_n),
    .ext_ram_oe_n(ext_ram_oe_n),
    .ext_ram_we_n(ext_ram_we_n),

    .uart_rdn(uart_rdn),
    .uart_wrn(uart_wrn),
    .uart_dataready(uart_dataready),
    .uart_tbre(uart_tbre),
    .uart_tsre(uart_tsre),
    .mtip(mtip),
    .cur_exception(mem_raising_exception)
);

reg[31:0] reg_inst;
wire[4:0] reg_s, reg_t, reg_d;
wire[5:0] id_exe_op;
wire[31:0] imm;
wire imm_select;

decoder _decoder(
    .inst(reg_inst),
    .reg_s(reg_s),
    .reg_t(reg_t),
    .reg_d(reg_d),
    .op(id_exe_op),
    .imm(imm),
    .imm_select(imm_select),
    .cur_exception(id_raising_exception)
);

reg[4:0] reg_waddr;
reg[31:0] reg_wdata;
reg reg_we;
wire[31:0] reg_rdata1;
wire[31:0] reg_rdata2;

regfile _regfile(
    .clk(clk_50M),
    .rst(reset_btn),
    .we(reg_we),
    .waddr(reg_waddr),
    .wdata(reg_wdata),
    .raddr1(reg_s),
    .rdata1(reg_rdata1),
    .raddr2(reg_t),
    .rdata2(reg_rdata2),
    .raddr3(reg_pred_s),
    .rdata3(reg_pred_s_data)
);

reg[4:0] exe_reg_d;
reg[`OP_LENGTH_1:0] exe_mem_op; // exe_mem_op <= id_exe_op;
reg[31:0] exe_imm;
reg exe_imm_select;

//interface to alu
reg[3:0] alu_op;
reg[31:0] exe_reg_s_val, exe_reg_t_val;
wire[31:0] exe_result;
wire[3:0] exe_flags;

// alu
always_comb begin
    case (exe_mem_op)
        `OP_ADD, `OP_ADDI, `OP_AUIPC, `OP_BEQ, `OP_BNE, `OP_SB, `OP_SW, `OP_LUI, `OP_JAL, `OP_JALR, `OP_LB, `OP_LW, `OP_LH, `OP_LBU, `OP_LHU, `OP_SH, `OP_CSRRC, `OP_CSRRS, `OP_CSRRW, `OP_BLT, `OP_BLTU, `OP_BGE, `OP_BGEU: alu_op = `ADD;
        `OP_AND, `OP_ANDI: alu_op = `AND;
        `OP_OR, `OP_ORI: alu_op = `OR;
        `OP_CLZ: alu_op = `CLZ;
        `OP_PCNT: alu_op = `PCNT;
        `OP_SBCLR: alu_op = `SBCLR;
        `OP_SLLI, `OP_SLL: alu_op = `SLL;
        `OP_SRLI, `OP_SRL: alu_op = `SRL;
        `OP_XOR, `OP_XORI: alu_op = `XOR;
        `OP_SLTU, `OP_SLTIU: alu_op = `SLTU;
        `OP_SRA, `OP_SRAI: alu_op = `SRA;
        `OP_SUB: alu_op = `SUB;
        `OP_SLT, `OP_SLTI: alu_op = `SLT;
        default: alu_op = `ZERO;
    endcase
end

reg[31:0] pc, next_pc; // next pc is the real instruction after exe_mem_pc!
reg[31:0] id_exe_pc, exe_mem_pc, mem_wb_pc;
// pc in exe state
reg is_jump_op, pc_jumping;

always_comb begin
    pc_jumping = 1'b0;
    is_jump_op = 1'b0;
    case (exe_mem_op)
        `OP_BEQ, `OP_BNE, `OP_BLT, `OP_BGE, `OP_BLTU, `OP_BGEU: begin
            is_jump_op = 1'b1;
            case (exe_mem_op)
                `OP_BEQ: pc_jumping = (raw_exe_reg_s_val == raw_exe_reg_t_val);
                `OP_BNE: pc_jumping = (raw_exe_reg_s_val != raw_exe_reg_t_val);
                `OP_BLT: pc_jumping = ($signed(raw_exe_reg_s_val) < $signed(raw_exe_reg_t_val));
                `OP_BGE: pc_jumping = ($signed(raw_exe_reg_t_val) >= $signed(raw_exe_reg_t_val));
                `OP_BLTU: pc_jumping = (raw_exe_reg_s_val < raw_exe_reg_t_val);
                `OP_BGEU: pc_jumping = (raw_exe_reg_s_val >= raw_exe_reg_t_val);
                default: pc_jumping = 1'b0;
            endcase
            if (pc_jumping) next_pc = exe_result; 
            else next_pc = exe_mem_pc + 32'h4;
        end
        `OP_JAL, `OP_JALR: begin
            is_jump_op = 1'b1;
            pc_jumping = 1'b1;
            next_pc = exe_result;
        end
        default: next_pc = exe_mem_pc + 32'h4;
    endcase
end

wire[4:0] reg_pred_s;
wire[31:0] reg_pred_s_data;
wire[31:0] raw_reg_pred_s_data;
assign raw_reg_pred_s_data = (wb_reg_d == reg_pred_s && wb_reg_d != 5'b0) ? wb_exe_result : 
                            ((mem_reg_d == reg_pred_s && mem_reg_d != 5'b0) ? mem_exe_result :
                             (exe_reg_d == reg_pred_s && exe_reg_d != 5'b0) ? exe_result : reg_pred_s_data);
wire[31:0] pred_pc;

reg[31:0] id_exe_pred_pc, exe_mem_pred_pc;
// pred pc
branch_pred _branch_pred(
    .clk(clk_50M),
    .mem_done(mem_done),
    .is_jump_op(is_jump_op),
    .last_jump_pc(exe_mem_pc[4:2]),
    .last_jump_result(pc_jumping),
    .inst(mem_data_out),
    .pc(pc),
    .reg_s_val(raw_reg_pred_s_data),
    .reg_s(reg_pred_s),
    .pred_pc(pred_pc)
);

// mem
always_comb begin
    mem_data_in = 32'b0;
    mem_addr = 32'b0;
    mem_inst = `MEM_IDLE_IDLE;
    if (mem_occupied_by == MEM_MEM) begin
        case (mem_wb_op)
            `OP_LB, `OP_LW, `OP_LH, `OP_LBU, `OP_LHU: begin
                mem_data_in = 32'b0;
                mem_addr = mem_exe_result;
                case (mem_wb_op)
                    `OP_LB: mem_inst = `MEM_READ_BYTE;
                    `OP_LBU: mem_inst = `MEM_READ_BYTE_UNSIGNED;
                    `OP_LH: mem_inst = `MEM_READ_HALF;
                    `OP_LHU: mem_inst = `MEM_READ_HALF_UNSIGNED;
                    `OP_LW: mem_inst = `MEM_READ_WORD;
                    default: begin end
                endcase
            end
            `OP_SB, `OP_SW, `OP_SH: begin
                mem_data_in = (wb_reg_d == mem_reg_t && wb_reg_d != 5'b0) ? wb_exe_result : mem_exe_reg_t_val;
                mem_addr = mem_exe_result;
                case (mem_wb_op)
                    `OP_SB: mem_inst = `MEM_WRITE_BYTE;
                    `OP_SH: mem_inst = `MEM_WRITE_HALF;
                    `OP_SW: mem_inst = `MEM_WRITE_WORD;
                    default: begin end
                endcase
            end
            default: begin end
        endcase
    end else if (mem_occupied_by == MEM_IF) begin
        mem_addr = pc;
        mem_inst = `MEM_READ_WORD;
    end else begin
        mem_inst = `MEM_IDLE_IDLE;
    end
end

// wb
always_comb begin
    case (mem_wb_op)
        `OP_ADD, `OP_ADDI, `OP_AND, `OP_ANDI, `OP_AUIPC, `OP_LUI, `OP_OR, `OP_ORI, `OP_SLLI, `OP_SRLI, `OP_XOR, `OP_LB, `OP_LW, `OP_LH, `OP_LBU, `OP_LHU, `OP_SLTU, `OP_CSRRC, `OP_CSRRS, `OP_CSRRW, `OP_SLTIU, `OP_XORI, `OP_SRA, `OP_SRAI, `OP_SUB, `OP_SLL, `OP_SRL, `OP_CLZ, `OP_PCNT, `OP_SBCLR, `OP_SLT, `OP_SLTI: begin
            reg_waddr = mem_reg_d;
            reg_wdata = mem_exe_result;
            reg_we = 1'b1;
        end
        `OP_JAL, `OP_JALR: begin
            reg_waddr = mem_reg_d;
            reg_wdata = mem_wb_pc + 32'h4;
            reg_we = 1'b1;
        end
        default: begin
            reg_waddr = 5'b0;
            reg_wdata = 32'b0;
            reg_we = 1'b0;
        end
    endcase
end

wire[31:0] raw_exe_reg_s_val, raw_exe_reg_t_val;
assign raw_exe_reg_s_val = ((mem_reg_d == exe_reg_s && mem_reg_d != 5'b0) ? mem_exe_result : exe_reg_s_val);
assign raw_exe_reg_t_val = ((mem_reg_d == exe_reg_t && mem_reg_d != 5'b0) ? mem_exe_result : exe_reg_t_val);

wire exe_is_csr_op;
assign exe_is_csr_op = (exe_mem_op == `OP_CSRRC || exe_mem_op == `OP_CSRRS || exe_mem_op == `OP_CSRRW);
wire exe_is_branch_op;
assign exe_is_branch_op = (exe_mem_op == `OP_BEQ || exe_mem_op == `OP_BNE || exe_mem_op == `OP_AUIPC || exe_mem_op == `OP_JAL || exe_mem_op == `OP_BLT || exe_mem_op == `OP_BLTU || exe_mem_op == `OP_BGE || exe_mem_op == `OP_BGEU);
alu _alu(
    .op(alu_op),
    .a(exe_is_branch_op ? exe_mem_pc : (exe_is_csr_op ? 32'b0 : raw_exe_reg_s_val)), // csr op needs all zero to get csr value
    .b(exe_imm_select ? exe_imm : 
       (exe_is_csr_op && csr_waddr == exe_reg_t && csr_waddr != 5'b0 ? csr_wdata : raw_exe_reg_t_val)), // csr is moved to EXE so I need to handle data dependency
    .r(exe_result),
    .flags(exe_flags)
);

reg[3:0] id_exception, exe_exception, mem_exception;
wire[3:0] id_raising_exception, mem_raising_exception;
reg[3:0] cur_exception;
reg[31:0] cur_exception_pc;

always_comb begin
    cur_exception_pc = mem_wb_pc;
    if (mem_exception != `EXCEPT_NONE) begin
        cur_exception = mem_exception;
    end else begin
        cur_exception = `EXCEPT_NONE;
    end
end

wire[31:0] mtvec, mepc, satp, csr_val;
wire is_pending_exception, mtip;
reg csr_we;
reg[4:0] csr_waddr;
reg[31:0] csr_wdata;
wire[3:0] pending_exception;

typedef enum reg[1:0] { USER, SUPERVISOR, MACHINE } mode_t;
wire[1:0] mode;

always_comb begin
    case (exe_mem_op)
        `OP_CSRRC: begin
            csr_we = 1'h1;
            csr_waddr = exe_reg_t;
            csr_wdata = csr_val & raw_exe_reg_s_val;
        end
        `OP_CSRRS: begin
            csr_we = 1'h1;
            csr_waddr = exe_reg_t;
            csr_wdata = csr_val | raw_exe_reg_s_val;
        end
        `OP_CSRRW: begin
            csr_we = 1'h1;
            csr_waddr = exe_reg_t;
            csr_wdata = raw_exe_reg_s_val;
        end
        default: begin
            csr_we = 1'h0;
            csr_waddr = 5'h0;
            csr_wdata = 32'h0;
        end
    endcase
end
exception_handler _exception_handler(
    .clk(clk_50M),
    .rst(reset_btn),
    .mem_done(mem_done),
    .mem_occupied_by(mem_occupied_by),
    .we(csr_we),
    .waddr(csr_waddr),
    .wdata(csr_wdata),
    .raddr(exe_reg_t),
    .cur_exception(cur_exception),
    .cur_exception_pc(cur_exception_pc),
    .mtime(mtime),
    .mtvec(mtvec),
    .mepc(mepc),
    .satp(satp),
    .rdata(csr_val),
    .mtip(mtip),
    .mode_out(mode),
    .pipeline_empty(pipeline_empty),
    .is_pending_exception(is_pending_exception),
    .pending_exception_out(pending_exception)
);

wire is_csr_op;
assign is_csr_op = (id_exe_op == `OP_CSRRC || id_exe_op == `OP_CSRRS || id_exe_op == `OP_CSRRW);

reg[31:0] id_reg_s_val, id_reg_t_val;
always_comb begin
    id_reg_s_val = reg_rdata1;
    id_reg_t_val = is_csr_op ? csr_val : reg_rdata2;
end

reg[`OP_LENGTH_1:0] mem_wb_op;
reg[4:0] exe_reg_s, exe_reg_t;
reg[31:0] mem_exe_reg_s_val, mem_exe_reg_t_val;
reg[31:0] mem_exe_result;
reg[4:0] mem_reg_s, mem_reg_t, mem_reg_d;
reg[31:0] wb_exe_result;
reg[4:0] wb_reg_s, wb_reg_t, wb_reg_d;

localparam INST_INVALID = 32'b0;

wire pipeline_empty;
assign pipeline_empty = (mem_wb_op == `OP_INVALID) && (exe_mem_op == `OP_INVALID) && 
                        (id_exe_op == `OP_INVALID) && (reg_inst == INST_INVALID);

wire pred_wrong;
assign pred_wrong = is_jump_op && (next_pc != exe_mem_pred_pc);

wire next_if_nop, next_id_nop, next_exe_nop, next_mem_nop;
// for delayed exception handling, all the exceptions will be handled in MEM state.
// next PC is 32'h0 (no need to fetch)
assign next_if_nop = (is_pending_exception && (!pipeline_empty)) || cur_exception != `EXCEPT_NONE;
// kill cur IF
assign next_id_nop = (is_pending_exception) || cur_exception != `EXCEPT_NONE || pred_wrong;
// kill cur ID
assign next_exe_nop = cur_exception != `EXCEPT_NONE || pred_wrong;
// kill cur EXE
assign next_mem_nop = cur_exception != `EXCEPT_NONE;

wire pipeline_flow;
assign pipeline_flow = (mem_occupied_by == MEM_IF) || (cur_exception != `EXCEPT_NONE); // halt current IF is OK (if exception happens, we don't need to fetch new instruction.)


always_ff @(posedge clk_50M or posedge reset_btn) begin
    if (reset_btn) begin
        reg_inst <= INST_INVALID;
        exe_mem_op <= `OP_INVALID;
        pc <= 32'h80000000;
    end else begin
        if (mem_done) begin
            if (pipeline_flow) begin      
                // PC (in exe state)
                id_exe_pred_pc <= pred_pc;
                exe_mem_pred_pc <= id_exe_pred_pc;

                if (next_if_nop) begin
                    pc <= pc; // if I change PC then page table may be no effect.
                end else begin
                    if (is_pending_exception && pipeline_empty) begin
                        case (pending_exception)
                            `EXCEPT_MRET: pc <= mepc;
                            default: pc <= mtvec;
                        endcase                    
                    end else if (pred_wrong) begin
                        pc <= next_pc;
                    end else begin
                        pc <= pred_pc;
                    end
                end

                if (next_id_nop) begin
                    reg_inst <= INST_INVALID;
                    id_exe_pc <= 32'h0;
                    id_exception <= `EXCEPT_NONE;
                end else begin
                    reg_inst <= mem_data_out;
                    id_exe_pc <= pc;
                    id_exception <= mem_raising_exception; // currently in IF (page fault...)
                end

                if (next_exe_nop) begin
                    exe_mem_op <= `OP_INVALID;
                    exe_mem_pc <= 32'h0;
                    exe_reg_s_val <= 0;
                    exe_reg_t_val <= 0;
                    exe_reg_s <= 0;
                    exe_reg_t <= 0;
                    exe_reg_d <= 0;
                    exe_imm <= 0;
                    exe_imm_select <= 0;
                    exe_exception <= `EXCEPT_NONE;
                end else begin
                    exe_mem_op <= id_exe_op;
                    exe_mem_pc <= id_exe_pc;
                    exe_reg_s_val <= id_reg_s_val;
                    exe_reg_t_val <= id_reg_t_val;
                    exe_reg_s <= reg_s;
                    exe_reg_t <= reg_t;
                    exe_reg_d <= reg_d;
                    exe_imm <= imm;
                    exe_imm_select <= imm_select;
                    exe_exception <= (id_exception != `EXCEPT_NONE) ? id_exception : id_raising_exception;
                end

                if (next_mem_nop) begin
                    mem_wb_op <= `OP_INVALID;
                    mem_wb_pc <= 32'h0;
                    mem_exe_reg_s_val <= 0;
                    mem_exe_reg_t_val <= 0;
                    mem_reg_s <= 0;
                    mem_reg_t <= 0;
                    mem_reg_d <= 0;
                    mem_occupied_by <= MEM_IF;
                    mem_exception <= `EXCEPT_NONE;
                end else begin
                    mem_wb_pc <= exe_mem_pc;
                    mem_wb_op <= exe_mem_op;
                    mem_exe_reg_s_val <= exe_reg_s_val;
                    mem_exe_reg_t_val <= exe_reg_t_val;
                    mem_exe_result <= exe_result;
                    mem_reg_s <= exe_reg_s;
                    mem_reg_t <= exe_reg_t;
                    mem_reg_d <= exe_reg_d;
                    if (exe_exception != `EXCEPT_NONE) begin // aka next mem_exception (if triggers exception previously, then no need to query MEM.)
                        mem_occupied_by <= MEM_IF;
                    end else begin
                        case (exe_mem_op) // aka next mem_wb_op
                            `OP_LB, `OP_LW, `OP_LH, `OP_SH, `OP_LHU, `OP_LBU, `OP_SB, `OP_SW: mem_occupied_by <= MEM_MEM;
                            default: mem_occupied_by <= MEM_IF;
                        endcase
                    end
                    mem_exception <= exe_exception; // mem_exception will be handled in else branch
                end
                
                wb_exe_result <= mem_exe_result;
                wb_reg_s <= mem_reg_s;
                wb_reg_t <= mem_reg_t;
                wb_reg_d <= mem_reg_d;  
            end else begin // occupied by mem, no need to flow the pipeline
                mem_occupied_by <= MEM_IF;
                mem_exe_result <= mem_data_out; // save it in case it changes
                mem_exception <= mem_raising_exception;
            end
        end else begin end
    end
end

endmodule

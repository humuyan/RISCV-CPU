`default_nettype none
`timescale 1ns / 1ps
`include "ops.vh"

module branch_pred (
    input wire clk,
    input wire is_jump_op,
    input wire[2:0] last_jump_pc, // 4 to 2
    input wire last_jump_result,
    input wire[31:0] inst,
    input wire[31:0] pc,
    input wire[31:0] reg_s_val,
    output wire[4:0] reg_s,
    output wire[31:0] pred_pc
);

// history, I don't need to reset it
reg[7:0][2:0] history_results = {8{3'b0}};
wire[2:0] history_result;
assign history_result = history_results[pc[4:2]]; // for PC[1:0] == 00 always

wire sign;
wire[19:0] sign_ext;
assign sign = inst[31];
assign sign_ext = {20{sign}};

// two cases
wire[31:0] jump_pc;
wire[31:0] no_jump_pc;
reg[31:0] jump_pc_reg;
assign jump_pc = jump_pc_reg;
assign no_jump_pc = pc + 32'h4;

reg pc_jumping;
assign reg_s = inst[11:7];
assign pred_pc = pc_jumping ? jump_pc : no_jump_pc;
always_comb begin
    // copied from decoder.sv
    jump_pc_reg = pc + 32'h4;
    pc_jumping = 0;
    // case (inst[6:0])
    //     7'b1100011: begin // B type
    //         jump_pc_reg = pc + {sign_ext, inst[7], inst[30:25], inst[11:8], 1'b0}; // pc + imm
    //         // branch: use last results to assume pc
    //         case(history_result)
    //             3'b000, 3'b001, 3'b100, 3'b101: pc_jumping = 0;
    //             3'b010, 3'b011, 3'b110, 3'b111: pc_jumping = 1;
    //             default: pc_jumping = 0;
    //         endcase
    //     end
    //     7'b1101111: begin // jal (J type)
    //         jump_pc_reg = pc + {sign_ext[10:0], inst[31], inst[19:12], inst[20], inst[30:21], 1'b0}; // pc + imm
    //         pc_jumping = 1;
    //     end
    //     7'b1100111: begin // I assume that the reg val is correct (I don't believe there will be much collision in jalr)
    //         jump_pc_reg = reg_s_val + {sign_ext, inst[31:20]};
    //         pc_jumping = 1;
    //     end
    //     default: begin end
    // endcase
    // // check if legal
    // if (jump_pc_reg == 32'h10000000) begin
    //     jump_pc_reg = pc + 32'h4;
    // end
end

always_ff @(posedge clk) begin
    if (is_jump_op) begin
       history_results[last_jump_pc] <= { history_results[last_jump_pc][1:0], last_jump_result };
    end
end
endmodule

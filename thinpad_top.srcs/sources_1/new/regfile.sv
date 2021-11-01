`default_nettype none
`timescale 1ns / 1ps

module regfile(
    input wire          clk,
    input wire          rst,
    input wire          we,
    input wire[4:0]     waddr,
    input wire[31:0]    wdata,
    
    input wire[4:0]     raddr1,
    output reg[31:0]    rdata1,
    input wire[4:0]     raddr2,
    output reg[31:0]    rdata2
    );
    
    reg[31:0] registers[0:31];
    
    always_ff @(posedge clk or posedge rst) begin
        if(rst) begin
            registers[0] = 32'h00000000;
            registers[1] = 32'h00000000;
            registers[2] = 32'h00000000;
            registers[3] = 32'h00000000;
            registers[4] = 32'h00000000;
            registers[5] = 32'h00000000;
            registers[6] = 32'h00000000;
            registers[7] = 32'h00000000;
            registers[8] = 32'h00000000;
            registers[9] = 32'h00000000;
            registers[10] = 32'h00000000;
            registers[11] = 32'h00000000;
            registers[12] = 32'h00000000;
            registers[13] = 32'h00000000;
            registers[14] = 32'h00000000;
            registers[15] = 32'h00000000;
            registers[16] = 32'h00000000;
            registers[17] = 32'h00000000;
            registers[18] = 32'h00000000;
            registers[19] = 32'h00000000;
            registers[20] = 32'h00000000;
            registers[21] = 32'h00000000;
            registers[22] = 32'h00000000;
            registers[23] = 32'h00000000;
            registers[24] = 32'h00000000;
            registers[25] = 32'h00000000;
            registers[26] = 32'h00000000;
            registers[27] = 32'h00000000;
            registers[28] = 32'h00000000;
            registers[29] = 32'h00000000;
            registers[30] = 32'h00000000;
            registers[31] = 32'h00000000;
        end
        else if (we) begin
            registers[waddr] <= wdata;
        end
    end
    
    always_comb begin
        if (raddr1 == 5'b0) rdata1 = 32'b0;
        else rdata1 = registers[raddr1];
    end
    
    always_comb begin
        if (raddr2 == 5'b0) rdata2 = 32'b0;
        else rdata2 = registers[raddr2];
    end
endmodule

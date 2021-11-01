`default_nettype none
`timescale 1ns / 1ps
`include "alu.vh"

module alu(
    input wire[3:0]        op,
    input wire[31:0]       a,
    input wire[31:0]       b,
    output wire[31:0]      r,
    output wire[3:0]       flags
    );
        
    reg zf,cf,sf,vf;
    reg[31:0] result;
    
    assign flags = {zf,cf,sf,vf};
    assign r = result;
    
    always_comb begin
        zf = 0;
        cf = 0;
        sf = 0;
        vf = 0;
        case (op)
            `ADD: begin
                result = a + b;
                if (result < a)
                    cf = 1'b1;
                else
                    cf = 1'b0;
                if ((result[31] != a[31]) && (a[31] == b[31]))
                    vf = 1'b1;
                else
                    vf = 1'b0;
            end
            
            `SUB: begin
                result = a - b;
                if (result > a)
                    cf = 1'b1;
                else
                    cf = 1'b0;
                if ((result[31] != a[31]) && (a[31] == b[31]))
                    vf = 1'b1;
                else
                    vf = 1'b0;
            end
            `AND: result = a & b;
            `OR: result = a | b;
            `XOR: result = a ^ b;
            `NOT: result = ~a;
            `SLL: result = a << b;
            `SRL: result = a >> b;
            `SRA: result = $signed(a) >>> b;                
            `ROL: result = (a << b) | (a >> (32 - b)); 
            default: result = 0;
        endcase
        
        zf = result == 0? 1'b1: 1'b0;
        if (result[31] == 1'b1) 
            sf = 1'b1;
        else
            sf = 1'b0;
    end

endmodule

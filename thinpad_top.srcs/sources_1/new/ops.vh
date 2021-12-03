`define OP_LENGTH 7
`define OP_LENGTH_1 6
`define OP_INVALID `OP_LENGTH'h0
`define OP_ADD `OP_LENGTH'h1
`define OP_ADDI `OP_LENGTH'h2
`define OP_AND `OP_LENGTH'h3
`define OP_ANDI `OP_LENGTH'h4
`define OP_AUIPC `OP_LENGTH'h5
`define OP_BEQ `OP_LENGTH'h6
`define OP_BNE `OP_LENGTH'h7
`define OP_JAL `OP_LENGTH'h8
`define OP_JALR `OP_LENGTH'h9
`define OP_LB `OP_LENGTH'hA
`define OP_LUI `OP_LENGTH'hB
`define OP_LW `OP_LENGTH'hC
`define OP_OR `OP_LENGTH'hD
`define OP_ORI `OP_LENGTH'hE
`define OP_SB `OP_LENGTH'hF
`define OP_SLLI `OP_LENGTH'h10
`define OP_SRLI `OP_LENGTH'h11
`define OP_SW `OP_LENGTH'h12
`define OP_XOR `OP_LENGTH'h13
`define OP_BLT `OP_LENGTH'h14
`define OP_BGE `OP_LENGTH'h15
`define OP_BLTU `OP_LENGTH'h16
`define OP_BGEU `OP_LENGTH'h17
`define OP_CLZ `OP_LENGTH'h18
`define OP_SBCLR `OP_LENGTH'h19
`define OP_PCNT `OP_LENGTH'h20
`define OP_LH `OP_LENGTH'h21
`define OP_LBU `OP_LENGTH'h22
`define OP_LHU `OP_LENGTH'h23
`define OP_SH `OP_LENGTH'h24
`define OP_SLTU `OP_LENGTH'h25
`define OP_SLTIU `OP_LENGTH'h26
`define OP_XORI `OP_LENGTH'h27
`define OP_SRA `OP_LENGTH'h28
`define OP_SRAI `OP_LENGTH'h29
`define OP_SUB `OP_LENGTH'h2a
`define OP_SLL `OP_LENGTH'h2b
`define OP_SRL `OP_LENGTH'h2c
`define OP_SLT `OP_LENGTH'h2d
`define OP_SLTI `OP_LENGTH'h2e

`define OP_SRET `OP_LENGTH'h2f
`define OP_CSRRC `OP_LENGTH'h30
`define OP_CSRRS `OP_LENGTH'h31
`define OP_CSRRW `OP_LENGTH'h32
`define OP_EBREAK `OP_LENGTH'h33
`define OP_ECALL `OP_LENGTH'h34
`define OP_MRET `OP_LENGTH'h35
`define OP_SFENCE_VMA `OP_LENGTH'h36
`define OP_CSRRCI `OP_LENGTH'h37
`define OP_CSRRSI `OP_LENGTH'h38
`define OP_CSRRWI `OP_LENGTH'h39
`define OP_ADD16 `OP_LENGTH'h40
`define CSR_ZERO 5'h0
`define CSR_MTVEC 5'h1
`define CSR_MEPC 5'h2
`define CSR_MCAUSE 5'h3
`define CSR_MIE 5'h4
`define CSR_MIP 5'h5
`define CSR_MTVAL 5'h6
`define CSR_MSCRATCH 5'h7
`define CSR_MSTATUS 5'h8

`define EXCEPT_NONE 4'h0
`define EXCEPT_U_ECALL 4'h1
`define EXCEPT_EBREAK 4'h2
`define EXCEPT_TIMEOUT 4'h3
`define EXCEPT_ILLEGAL_INST 4'h4
`define EXCEPT_INST_PAGE_FAULT 4'h5
`define EXCEPT_LOAD_PAGE_FAULT 4'h6
`define EXCEPT_STORE_PAGE_FAULT 4'h7

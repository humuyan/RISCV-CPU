`define CSR_ZERO 5'h0
`define CSR_MTVEC 5'h1
`define CSR_MEPC 5'h2
`define CSR_MCAUSE 5'h3
// sie == mie, sip == mip
`define CSR_MIE 5'h4
`define CSR_MIP 5'h5
`define CSR_MTVAL 5'h6
`define CSR_MSCRATCH 5'h7
// sstatus == mstatus
`define CSR_MSTATUS 5'h8
`define CSR_SATP 5'h9
`define CSR_MIDELEG 5'hA
`define CSR_MEDELEG 5'hB
`define CSR_STVEC 5'hC
`define CSR_SEPC 5'hD
`define CSR_SCAUSE 5'hE
`define CSR_STVAL 5'hF
`define CSR_SSCRATCH 5'h10

`define CSR_SIE 5'h11
`define CSR_SIP 5'h12
`define CSR_SSTATUS 5'h13

`define CSR_MTIME 5'h1E
`define CSR_MTIMEH 5'h1F


`define PAGING_BARE 1'h0
`define PAGING_SV32 1'h1

`define EXCEPT_NONE 4'h0

// DON'T touch the ID's here for they have special meanings (Exception code)
`define EXCEPT_ILLEGAL_INST 4'h2
`define EXCEPT_EBREAK 4'h3
`define EXCEPT_U_ECALL 4'h8
`define EXCEPT_S_ECALL 4'h9
`define EXCEPT_INST_PAGE_FAULT 4'hC
`define EXCEPT_LOAD_PAGE_FAULT 4'hD
`define EXCEPT_STORE_PAGE_FAULT 4'hF

// they are useless exception codes so borrow them.
`define EXCEPT_MRET 4'h4
`define EXCEPT_SRET 4'h5
`define EXCEPT_M_TIMEOUT 4'h6
`define EXCEPT_S_TIMEOUT 4'h7
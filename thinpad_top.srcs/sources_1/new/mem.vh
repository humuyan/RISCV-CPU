// inst
`define MEM_IDLE 2'b00
`define MEM_READ 2'b01
`define MEM_WRITE 2'b10

`define MEM_BYTE 2'b00
`define MEM_WORD 2'b11

`define MEM_BASE 32'b1000000000??????????????????????
`define MEM_EXT  32'b1000000001??????????????????????
`define MEM_UART 32'b00010000000000000000000000000???
`define MEM_MTIME_LO 32'h200BFF8
`define MEM_MTIME_HI 32'h200BFFC
`define MEM_MTIMECMP_LO 32'h2004000
`define MEM_MTIMECMP_HI 32'h2004004

`define MEM_IDLE_IDLE {`MEM_IDLE, `MEM_IDLE}
`define MEM_READ_BYTE {`MEM_READ, `MEM_BYTE}
`define MEM_READ_WORD {`MEM_READ, `MEM_WORD}
`define MEM_WRITE_BYTE {`MEM_WRITE, `MEM_BYTE}
`define MEM_WRITE_WORD {`MEM_WRITE, `MEM_WORD}
// inst
`define MEM_IDLE 2'b00
`define MEM_READ 2'b01
`define MEM_WRITE 2'b10

`define MEM_BYTE 2'b00
`define MEM_WORD 2'b11

`define MEM_BASE 2'b00
`define MEM_EXT 2'b01
`define MEM_UART 2'b10

// inst {addr[28], addr[22]}
`define MEM_READ_BASE {`MEM_READ, `MEM_BASE}
`define MEM_WRITE_BASE {`MEM_WRITE, `MEM_BASE}
`define MEM_READ_EXT {`MEM_READ, `MEM_EXT}
`define MEM_WRITE_EXT {`MEM_WRITE, `MEM_EXT}
`define MEM_READ_UART {`MEM_READ, `MEM_UART}
`define MEM_WRITE_UART {`MEM_WRITE, `MEM_UART}

`define MEM_IDLE_IDLE {`MEM_IDLE, `MEM_IDLE}
`define MEM_READ_BYTE {`MEM_READ, `MEM_BYTE}
`define MEM_READ_WORD {`MEM_READ, `MEM_WORD}
`define MEM_WRITE_BYTE {`MEM_WRITE, `MEM_BYTE}
`define MEM_WRITE_WORD {`MEM_WRITE, `MEM_WORD}
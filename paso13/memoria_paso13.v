/**
 * Paso 13: Creando un procesador RISC-V
 *         Subrutinas 1 (usando ensamblador (asm) estandar RISC-V)
 * Listo*
 */

 module Memory (
   input             clk,
   input      [31:0] mem_addr,  // direccion a ser leida
   output reg [31:0] mem_rdata, // datos leidos desde memoria
   input             mem_rstrb  // se setea en "high" cuando el procesador quiere leer
);

    reg [31:0] MEM [0:255]; 

`ifdef BENCH
    localparam slow_bit=15;
`else
    localparam slow_bit=19;
`endif


`include "../paso07/riscv_assembly.v"
    integer L0_   = 4;
    integer wait_ = 20;
    integer L1_   = 28;

    initial begin
        ADD(x10,x0,x0);
    Label(L0_); 
        ADDI(x10,x10,1);
        JAL(x1,LabelRef(wait_)); // call(wait_)
        JAL(zero,LabelRef(L0_)); // jump(l0_)

        EBREAK();

    Label(wait_);
        ADDI(x11,x0,1);
        SLLI(x11,x11,slow_bit);
    Label(L1_);
        ADDI(x11,x11,-1);
        BNE(x11,x0,LabelRef(L1_));
        JALR(x0,x1,0);

        endASM();
    end

    always @(posedge clk) begin
        if(mem_rstrb) begin
            mem_rdata <= MEM[mem_addr[31:2]];
        end
    end
endmodule
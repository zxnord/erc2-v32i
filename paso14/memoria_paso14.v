/**
 * Paso 14: Creando un procesador RISC-V
 *         Subrutinas 2 (usando pseudo-instrucciones RISC-V)
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


`include "../paso7/riscv_assembly.v"
    integer L0_   = 4;
    integer wait_ = 24;
    integer L1_   = 32;

    initial begin
        LI(a0,0);
    Label(L0_); 
        ADDI(a0,a0,1);
        CALL(LabelRef(wait_)); 
        J(LabelRef(L0_)); 

        EBREAK();

    Label(wait_);
        LI(a1,1);
        SLLI(a1,a1,slow_bit);
    Label(L1_);
        ADDI(a1,a1,-1);
        BNEZ(a1,LabelRef(L1_));
        RET();

        endASM();
    end

    always @(posedge clk) begin
        if(mem_rstrb) begin
            mem_rdata <= MEM[mem_addr[31:2]];
        end
    end
endmodule
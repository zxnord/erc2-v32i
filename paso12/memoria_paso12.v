/**
 * Step 12: Creating a RISC-V processor
 *         Subroutines 1 (using standard RISC-V asm)
 *         LUT optimization, gaining a lot of space !
 * DONE*
 */

module Memory (
   input             clk,
   input      [31:0] mem_addr,  // address to be read
   output reg [31:0] mem_rdata, // data read from memory
   input             mem_rstrb  // goes high when processor wants to read
);

    reg [31:0] MEM [0:255]; 

`ifdef BENCH
    localparam slow_bit=13;
`else
    localparam slow_bit=19;
`endif

   
`include "../paso7/riscv_assembly.v"
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
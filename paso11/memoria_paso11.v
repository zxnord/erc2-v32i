/**
 * Step 11: Creating a RISC-V processor
 *         Separate memory
 * DONE*
 */

module Memory (
    input             clk,
    input      [31:0] mem_addr,  // address to be read
    output reg [31:0] mem_rdata, // data read from memory
    input             mem_rstrb  // goes high when processor wants to read
);

    reg [31:0] MEM [0:255]; 

`include "../paso07/riscv_assembly.v"
    integer L0_=8;
    integer L1_=16;
    initial begin
        /*ADD(x1,x0,x0);
        ADDI(x2,x0,31);
    Label(L0_); 
        ADDI(x1,x1,1); 
        BNE(x1, x2, LabelRef(L0_));
        EBREAK();

        endASM();*/

        ADD(x1, x0, x0);
        ADDI(x2, x0, 32);
    Label(L0_); 
        ADDI(x1, x1, 1); 
        BNE(x1, x2, LabelRef(L0_));
    Label(L1_);
        ADDI(x1, x1, -1);
        BNE(x1, x0, LabelRef(L1_));
        JAL(x0, LabelRef(L0_));
        endASM();
    end

    always @(posedge clk) begin
        if(mem_rstrb) begin
            mem_rdata <= MEM[mem_addr[31:2]];
        end
    end
endmodule
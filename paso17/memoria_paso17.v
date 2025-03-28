/**
 * Paso 17: Creando un procesador RISC-V
 *         Enviando data por puerto serial
 * Listo*
 */

module Memory (
    input             clk,
    input      [31:0] mem_addr,  // direccion a ser leida
    output reg [31:0] mem_rdata, // datos leidos desde memoria
    input             mem_rstrb, // se setea en "high" cuando el procesador quiere leer 
    input      [31:0] mem_wdata, // datos a escribir (STORE)
    input      [3:0]  mem_wmask  // Mascara de escritura para diferenciar SW, SB, SH
);

    reg [31:0] MEM [0:1535]; // 1536 4-bytes words = 6 Kb of RAM in total

`ifdef BENCH
    localparam slow_bit=12;
`else
    localparam slow_bit=17;
`endif

    // Memory-mapped IO in IO page, 1-hot addressing in word address.   
    localparam IO_LEDS_bit      = 0;  // W five leds
    localparam IO_UART_DAT_bit  = 1;  // W data to send (8 bits) 
    localparam IO_UART_CNTL_bit = 2;  // R status. bit 9: busy sending

    // Converts an IO_xxx_bit constant into an offset in IO page.
    function [31:0] IO_BIT_TO_OFFSET;
        input [31:0] bitid;
        begin
            IO_BIT_TO_OFFSET = 1 << (bitid + 2);
        end
    endfunction

`include "../paso07/riscv_assembly.v"
    integer    L0_      = 12;
    integer    L1_      = 20;
    integer    L2_      = 52;      
    integer    wait_    = 104;
    integer    wait_L0_ = 112;
    integer    putc_    = 124; 
    integer    putc_L0_ = 132;

    initial begin
        LI(sp,32'h1800);   // End of RAM, 6kB
        LI(gp,32'h400000); // IO page

    Label(L0_);

        // Count from 0 to 15 on the LEDs      
        LI(s0,16); // upper bound of loop
        LI(a0,0);
    Label(L1_);
        SW(a0,gp,IO_BIT_TO_OFFSET(IO_LEDS_bit));
        CALL(LabelRef(wait_));
        ADDI(a0,a0,1);
        BNE(a0,s0,LabelRef(L1_));

        // Send abcdef...xyz to the UART
        LI(s0,26); // upper bound of loop     
        LI(a0,"a");
        LI(s1,0);
    Label(L2_);
        CALL(LabelRef(putc_));
        ADDI(a0,a0,1);
        ADDI(s1,s1,1);
        BNE(s1,s0,LabelRef(L2_));

        // CR;LF
        LI(a0,13);
        CALL(LabelRef(putc_));
        LI(a0,10);
        CALL(LabelRef(putc_));

        J(LabelRef(L0_));

        EBREAK(); // I systematically keep it before functions
                  // in case I decide to remove the loop...

    Label(wait_);
        LI(t0,1);
        SLLI(t0,t0,slow_bit);
    Label(wait_L0_);
        ADDI(t0,t0,-1);
        BNEZ(t0,LabelRef(wait_L0_));
        RET();

    Label(putc_);
        // Send character to UART
        SW(a0,gp,IO_BIT_TO_OFFSET(IO_UART_DAT_bit));
        // Read UART status, and loop until bit 9 (busy sending)
        // is zero.
        LI(t0,1<<9);
    Label(putc_L0_);
        LW(t1,gp,IO_BIT_TO_OFFSET(IO_UART_CNTL_bit));     
        AND(t1,t1,t0);
        BNEZ(t1,LabelRef(putc_L0_));
        RET();

        endASM();
    end

    wire [29:0] word_addr = mem_addr[31:2];

    always @(posedge clk) begin
        if(mem_rstrb) begin
            mem_rdata <= MEM[word_addr];
        end
        if(mem_wmask[0]) MEM[word_addr][ 7:0 ] <= mem_wdata[ 7:0 ];
        if(mem_wmask[1]) MEM[word_addr][15:8 ] <= mem_wdata[15:8 ];
        if(mem_wmask[2]) MEM[word_addr][23:16] <= mem_wdata[23:16];
        if(mem_wmask[3]) MEM[word_addr][31:24] <= mem_wdata[31:24];	 
    end
endmodule
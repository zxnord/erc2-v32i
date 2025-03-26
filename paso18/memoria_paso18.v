/**
 * Paso 18: Creando un procesador RISC-V
 *         Enviando data compleja por puerto serial
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

`include "../paso7/riscv_assembly.v"

    `define mandel_shift 10
    `define mandel_mul (1 << `mandel_shift)
    `define xmin (-2*`mandel_mul)
    `define xmax ( 2*`mandel_mul)
    `define ymin (-2*`mandel_mul)
    `define ymax ( 2*`mandel_mul)	
    `define dx ((`xmax-`xmin)/80)
    `define dy ((`ymax-`ymin)/80)
    `define norm_max (4 << `mandel_shift)

    integer    mandelstart_ = 12;
    integer    blink_       = 16;
    integer    loop_y_      = 76;
    integer    loop_x_      = 84;
    integer    loop_Z_      = 96;
    integer    exit_Z_      = 188;
    integer    wait_        = 264;
    integer    wait_L0_     = 272;
    integer    putc_        = 284; 
    integer    putc_L0_     = 292;
    integer    mulsi3_      = 308;
    integer    mulsi3_L0_   = 316;
    integer    mulsi3_L1_   = 328;

    integer    colormap_    = 344;

    // X,Y         : s0,s1
    // Cr,Ci       : s2,s3
    // Zr,Zi       : s4,s5
    // Zrr,2Zri,Zii: s6,s7,s8
    // cnt: s10
    // 128: s11

    initial begin
        LI(sp,32'h1800);   // End of RAM, 6kB
        LI(gp,32'h400000); // IO page

    Label(mandelstart_);

        // Blink 5 times.
        LI(s0,5);
    Label(blink_);
        LI(a0,5);
        SW(a0,gp,IO_BIT_TO_OFFSET(IO_LEDS_bit));
        CALL(LabelRef(wait_));
        LI(a0,10);
        SW(a0,gp,IO_BIT_TO_OFFSET(IO_LEDS_bit));
        CALL(LabelRef(wait_));
        ADDI(s0,s0,-1);
        BNEZ(s0,LabelRef(blink_));
        LI(a0,0);
        SW(a0,gp,IO_BIT_TO_OFFSET(IO_LEDS_bit));


        LI(s1,0);
        LI(s3,`xmin);
        LI(s11,80);
      
    Label(loop_y_);
        LI(s0,0);
        LI(s2,`ymin);

    Label(loop_x_);
        MV(s4,s2); // Z <- C
        MV(s5,s3);

        LI(s10,9); // iter <- 9

    Label(loop_Z_);
        MV(a0,s4); // Zrr  <- (Zr*Zr) >> mandel_shift
        MV(a1,s4);
        CALL(LabelRef(mulsi3_));
        SRLI(s6,a0,`mandel_shift);
        MV(a0,s4); // Zri <- (Zr*Zi) >> (mandel_shift-1)
        MV(a1,s5);
        CALL(LabelRef(mulsi3_));
        SRAI(s7,a0,`mandel_shift-1);
        MV(a0,s5); // Zii <- (Zi*Zi) >> (mandel_shift)
        MV(a1,s5);
        CALL(LabelRef(mulsi3_));
        SRLI(s8,a0,`mandel_shift);
        SUB(s4,s6,s8); // Zr <- Zrr - Zii + Cr  
        ADD(s4,s4,s2);
        ADD(s5,s7,s3); // Zi <- 2Zri + Cr

        ADD(s6,s6,s8); // if norm > norm max, exit loop
        LI(s7,`norm_max);
        BGT(s6,s7,LabelRef(exit_Z_));

        ADDI(s10,s10,-1);  // iter--, loop if non-zero
        BNEZ(s10,LabelRef(loop_Z_));

    Label(exit_Z_);
        LI(a0,colormap_);
        ADD(a0,a0,s10);
        LBU(a0,a0,0);
        CALL(LabelRef(putc_));

        ADDI(s0,s0,1);
        ADDI(s2,s2,`dx);
        BNE(s0,s11,LabelRef(loop_x_));

        LI(a0," ");
        CALL(LabelRef(putc_));
        LI(a0,"\n");
        CALL(LabelRef(putc_));

        ADDI(s1,s1,1);
        ADDI(s3,s3,`dy);
        BNE(s1,s11,LabelRef(loop_y_));


        J(LabelRef(mandelstart_));

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

        // Mutiplication routine,
        // Input in a0 and a1
        // Result in a0
    Label(mulsi3_);
        MV(a2,a0);
        LI(a0,0);
    Label(mulsi3_L0_); 
        ANDI(a3,a1,1);
        BEQZ(a3,LabelRef(mulsi3_L1_)); 
        ADD(a0,a0,a2);
    Label(mulsi3_L1_);
        SRLI(a1,a1,1);
        SLLI(a2,a2,1);
        BNEZ(a1,LabelRef(mulsi3_L0_));
        RET();

    Label(colormap_);
        DATAB(" ",".",",",":");
        DATAB(";","o","x","%");
        DATAB("#","@", 0 , 0 );
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
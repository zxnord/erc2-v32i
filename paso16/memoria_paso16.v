/**
 * Paso 16: Creando un procesador RISC-V
 *         Store
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

    reg [31:0] MEM [0:255]; 

`ifdef BENCH
    localparam slow_bit=11;
`else
    localparam slow_bit=17;
`endif


`include "../paso07/riscv_assembly.v"
    integer L0_   = 12;
    integer L1_   = 40;
    integer wait_ = 64;
    integer L2_   = 72;

    initial begin
        LI(a0,0);
        // Copy 16 bytes from adress 400
        // to address 800
        LI(s1,16);      
        LI(s0,0);         
    Label(L0_); 
        LB(a1,s0,400);
        SB(a1,s0,800);       
        CALL(LabelRef(wait_));
        ADDI(s0,s0,1); 
        BNE(s0,s1, LabelRef(L0_));

        // Read 16 bytes from adress 800
        LI(s0,0);
    Label(L1_);
        LB(a0,s0,800); // a0 (=x10) is plugged to the LEDs
        CALL(LabelRef(wait_));
        ADDI(s0,s0,1); 
        BNE(s0,s1, LabelRef(L1_));
        EBREAK();

    Label(wait_);
        LI(t0,1);
        SLLI(t0,t0,slow_bit);
    Label(L2_);
        ADDI(t0,t0,-1);
        BNEZ(t0,LabelRef(L2_));
        RET();

        endASM();

        // Note: index 100 (word address)
        //     corresponds to 
        // address 400 (byte address)
        MEM[100] = {8'h4, 8'h3, 8'h2, 8'h1};
        MEM[101] = {8'h8, 8'h7, 8'h6, 8'h5};
        MEM[102] = {8'hc, 8'hb, 8'ha, 8'h9};
        MEM[103] = {8'hff, 8'hf, 8'he, 8'hd};
    end

    wire [29:0] word_addr = mem_addr[31:2];

    always @(posedge clk) begin
        if(mem_rstrb) begin
            mem_rdata <= MEM[mem_addr[31:2]];
        end
        if(mem_wmask[0]) MEM[word_addr][ 7:0 ] <= mem_wdata[ 7:0 ];
        if(mem_wmask[1]) MEM[word_addr][15:8 ] <= mem_wdata[15:8 ];
        if(mem_wmask[2]) MEM[word_addr][23:16] <= mem_wdata[23:16];
        if(mem_wmask[3]) MEM[word_addr][31:24] <= mem_wdata[31:24];	
    end
endmodule
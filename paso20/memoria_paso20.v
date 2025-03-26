/**
 * Paso 20: Creando un procesador RISC-V
 *         Ejecutando codigo assembly compilado con GCC
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

    initial begin
        $readmemh("FIRMWARE/paso20_fix.hex",MEM);
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
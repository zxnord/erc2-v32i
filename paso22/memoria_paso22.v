/**
 * Paso 22: Creando un procesador RISC-V
 *         Habilitando SPI Flash
 * Listo*
 */

module Memory (
    input             clk,       // reloj de sistema
    input      [31:0] mem_addr,  // direccion a ser leida o escrita
    output reg [31:0] mem_rdata, // datos a ser leidos
    input             mem_rstrb, // strobe de lectura. Alto cuando se lee
    input      [31:0] mem_wdata, // datos a ser escritos
    input      [3:0]  mem_wmask	// mascara para leer los 4 bytes (1=byte de escritura)
);
    reg [31:0] MEM [0:9209]; // 294720 4-bytes words = 287 Kb of RAM in total

    initial begin
        $readmemh("FIRMWARE/paso22.hex",MEM);
    end

    wire [10:0] word_addr = mem_addr[12:2];

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


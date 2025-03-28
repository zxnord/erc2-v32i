/**
 * Paso 22: Creando un procesador RISC-V
 *         Habilitando SPI Flash
 * Listo*
 */

`default_nettype none
`include "../paso02/clockworks.v"

`include "memoria_paso22.v"
`include "procesador_paso22.v"
`include "spi_flash.v"
`include "../paso17/emitter_uart.v"


module SOC (
    input  clk_25mhz,   // reloj de sistema 
    input  rst,         // boton de reset
    output [7:0] led,   // LEDs de sistema (en la placa)
    input  ftdi_rxd,    // recepcion de data UART 
    output ftdi_txd,    // transmision de data UART
    output flash_clk,   // SPI flash clock
    output flash_csn,   // SPI flash chip select (active low)
    output flash_mosi,  // SPI flash IO pins
    input  flash_miso   // SPI flash IO pins
);

    wire clk;
    wire resetn;
    //wire flash_clk;

    wire [31:0] mem_addr;
    wire [31:0] mem_rdata;
    wire        mem_rbusy;
    wire mem_rstrb;
    wire [31:0] mem_wdata;
    wire [3:0]  mem_wmask;

    Processor CPU(
        .clk(clk),
        .resetn(resetn),		 
        .mem_addr(mem_addr),
        .mem_rdata(mem_rdata),
        .mem_rstrb(mem_rstrb),
        .mem_rbusy(mem_rbusy),
        .mem_wdata(mem_wdata),
        .mem_wmask(mem_wmask)
    );
   
    wire [31:0] RAM_rdata;
    wire [29:0] mem_wordaddr = mem_addr[31:2];
    wire isSPIFlash  = mem_addr[23];      
    wire isIO        = mem_addr[23:22] == 2'b01;
    wire isRAM = !(mem_addr[23] | mem_addr[22]);
    wire mem_wstrb = |mem_wmask;

    Memory RAM(
        .clk(clk),
        .mem_addr(mem_addr),
        .mem_rdata(RAM_rdata),
        .mem_rstrb(isRAM & mem_rstrb),
        .mem_wdata(mem_wdata),
        .mem_wmask({4{isRAM}}&mem_wmask)
    );

    wire [31:0] SPIFlash_rdata;
    wire SPIFlash_rbusy;
    MappedSPIFlash SPIFlash(
        .clk(clk),
        .word_address(mem_wordaddr[19:0]),
        .rdata(SPIFlash_rdata),
        .rstrb(isSPIFlash & mem_rstrb),
        .rbusy(SPIFlash_rbusy),
        .CLK(flash_clk),
        .CS_N(flash_csn),
        .MISO(flash_miso),
        .MOSI(flash_mosi)
    );

    // Memory-mapped IO in IO page, 1-hot addressing in word address.   
    localparam IO_LEDS_bit      = 0;  // W five leds
    localparam IO_UART_DAT_bit  = 1;  // W data to send (8 bits) 
    localparam IO_UART_CNTL_bit = 2;  // R status. bit 9: busy sending

    always @(posedge clk) begin
        if(isIO & mem_wstrb & mem_wordaddr[IO_LEDS_bit]) begin
            led <= mem_wdata[7:0];
//	 $display("Value sent to LEDS: %b %d %d",mem_wdata,mem_wdata,$signed(mem_wdata));
        end
    end

    wire uart_valid = isIO & mem_wstrb & mem_wordaddr[IO_UART_DAT_bit];
    wire uart_ready;

    corescore_emitter_uart #(
        .clk_freq_hz(25000000),
        .baud_rate(115200)
    ) UART(
        .i_clk(clk),
        .i_rst(!resetn),
        .i_data(mem_wdata[7:0]),
        .i_valid(uart_valid),
        .o_ready(uart_ready),
        .o_uart_tx(ftdi_txd)
    );

    wire [31:0] IO_rdata = 
                mem_wordaddr[IO_UART_CNTL_bit] ? { 22'b0, !uart_ready, 9'b0}
                                               : 32'b0;

    assign mem_rdata = isRAM      ? RAM_rdata :
                       isSPIFlash ? SPIFlash_rdata : 
                                    IO_rdata ;

    assign mem_rbusy = SPIFlash_rbusy;

`ifdef BENCH
    always @(posedge clk) begin
        if(uart_valid) begin
            $write("%c", mem_wdata[7:0] );
            $fflush(32'h8000_0001);
        end
    end
`endif

    // Gearbox and reset circuitry.
    Clockworks CW(
        .CLK(clk_25mhz),
        .RESET(rst),
        .clk(clk),
        .resetn(resetn)
    );
endmodule

/**
 * Paso 16: Creando un procesador RISC-V
 *         Store
 * Listo*
 */

`default_nettype none
`include "../paso02/clockworks.v"

`include "memoria_paso16.v"
`include "procesador_paso16.v"

module SOC (
    input  clk_25mhz,   // reloj de sistema 
    input  rst,         // boton de reset
    output [7:0] led,   // LEDs de sistema (en la placa)
    input  ftdi_rxd,    // recepcion de data UART 
    output ftdi_txd     // transmision de data UART
);

    wire clk;
    wire resetn;

    Memory RAM(
        .clk(clk),
        .mem_addr(mem_addr),
        .mem_rdata(mem_rdata),
        .mem_rstrb(mem_rstrb),
        .mem_wdata(mem_wdata),
        .mem_wmask(mem_wmask)
    );

    wire [31:0] mem_addr;
    wire [31:0] mem_rdata;
    wire [31:0] mem_wdata;
    wire mem_rstrb;
    wire [31:0] x10;
    wire [3:0] mem_wmask;

    Processor CPU(
        .clk(clk),
        .resetn(resetn),
        .mem_addr(mem_addr),
        .mem_rdata(mem_rdata),
        .mem_rstrb(mem_rstrb),
        .mem_wdata(mem_wdata),
        .mem_wmask(mem_wmask),
        .x10(x10)
    );

    assign led = x10[7:0];

    // Gearbox and reset circuitry.
    Clockworks CW(
        .CLK(clk_25mhz),
        .RESET(rst),
        .clk(clk),
        .resetn(resetn)
    );

    assign ftdi_txd  = 1'b0; // not used for now
endmodule
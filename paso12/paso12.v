/**
 * Step 12: Creating a RISC-V processor
 *         Subroutines 1 (using standard RISC-V asm)
 *         LUT optimization, gaining a lot of space !
 * DONE*
 */

`default_nettype none
`include "../paso02/clockworks.v"

`include "memoria_paso12.v"
`include "procesador_paso12.v"


module SOC (
    input  clk_25mhz,   // system clock 
    input  rst,         // reset button
    output [7:0] led,   // system LEDs
    input  ftdi_rxd,    // UART receive
    output ftdi_txd     // UART transmit
);

    wire clk;
    wire resetn;

    Memory RAM(
        .clk(clk),
        .mem_addr(mem_addr),
        .mem_rdata(mem_rdata),
        .mem_rstrb(mem_rstrb)
    );

    wire [31:0] mem_addr;
    wire [31:0] mem_rdata;
    wire mem_rstrb;
    wire [31:0] x10;

    Processor CPU(
        .clk(clk),
        .resetn(resetn),		 
        .mem_addr(mem_addr),
        .mem_rdata(mem_rdata),
        .mem_rstrb(mem_rstrb),
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
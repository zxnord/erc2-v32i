/**
 * Paso 2: Blinker lento
 * Listo
 */

`include "clockworks.v"

module SOC (
    input  clk_25mhz,
    input  rst,         // reset button
    output [7:0] led,   // system LEDs
    input  ftdi_rxd,    // UART receive
    output ftdi_txd     // UART transmit
);

    wire clk;    // internal clock
    wire resetn; // internal reset signal, goes low on reset
    reg [31:0] counter = 0;

    // A blinker that counts on 5 bits, wired to the 5 LEDs
    reg [7:0] count = 0;
    always @(posedge clk) begin
        count <= !resetn ? 0 : count + 1; //habilitar cuando reset sea realmente un boton mapeado.
        //count <= count + 1;
    end

    // Clock gearbox (to let you see what happens)
    // and reset circuitry (to workaround an
    // initialization problem with Ice40)
    Clockworks #(
        .SLOW(22) // Divide clock frequency by 2^21
    )CW(
        .CLK(clk_25mhz),
        .RESET(rst),
        .clk(clk),
        .resetn(resetn)
    );

    assign led = count;
    assign ftdi_txd  = 1'b0; // not used for now   
endmodule
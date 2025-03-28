/**
 * Step 3: Display a led pattern "animation" stored in BRAM.
 * DONE*
 */

`default_nettype none

`include "../paso02/clockworks.v"

module SOC (
    input  clk_25mhz,   // reloj de sistema
    input  rst,         // reset button
    output [7:0] led,   // system LEDs
    input  ftdi_rxd,    // UART receive
    output ftdi_txd     // UART transmit
);

    wire clk;    // internal clock
    wire resetn; // internal reset signal, goes low on reset
   
    reg [2:0] PC = 0;
    reg [2:0] MEM [0:20];
    initial begin
        MEM[0]  = 3'b111;
        MEM[1]  = 3'b110;
        MEM[2]  = 3'b111;
        MEM[3]  = 3'b101;
        MEM[4]  = 3'b111;
        MEM[5]  = 3'b011;
        MEM[6]  = 3'b111;
        MEM[7]  = 3'b110;
        MEM[8]  = 3'b111;
        MEM[9]  = 3'b100;
        MEM[10] = 3'b111;
        MEM[11] = 3'b001;
        MEM[12] = 3'b111;
        MEM[13] = 3'b010;
        MEM[14] = 3'b111;
        MEM[15] = 3'b001;
        MEM[16] = 3'b111;
        MEM[17] = 3'b100;
        MEM[18] = 3'b111;
        MEM[19] = 3'b000;
        MEM[20] = 3'b111;
    end

   reg [4:0] leds = 0;
   assign led = leds;

   always @(posedge clk) begin
      leds <= MEM[PC];
      PC <= (!resetn || PC==20) ? 0 : (PC+1);
      //PC <= PC==20 ? 0 : (PC+1);
   end

   // Gearbox and reset circuitry.
   Clockworks #(
     .SLOW(25) // Divide clock frequency by 2^25
   )CW(
     .CLK(clk_25mhz),
     .RESET(rst),
     .clk(clk),
     .resetn(resetn)
   );
   
   assign ftdi_txd  = 1'b0; // not used for now   
endmodule
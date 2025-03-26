   /**
 * Step 1: Blinker
 * DONE
 */

`default_nettype none

module SOC (
        input clk_25mhz,
        input  btn,      
        output [7:0] led, 
        input  ftdi_rxd,        
        output ftdi_txd         
   );

   reg [7:0] count = 0;
   always @(posedge clk_25mhz) begin
      count <= count + 1;
   end
   assign led = count;
   assign ftdi_txd  = 1'b0; // not used for now

   endmodule
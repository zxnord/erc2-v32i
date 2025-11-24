// Hardware Page Table Walker
`default_nettype none

module ptw(
    input  wire clk,
    input  wire reset,

    // Interface with CPU
    input  wire        start,          // CPU signals PTW to start
    input  wire [19:0] fault_vpn,      // VPN that caused the fault
    input  wire [21:0] satp_ppn,       // PPN of the L1 page table from SATP register
    output reg         done,           // PTW signals completion
    output reg [31:0]  pte_out,        // The final L0 Page Table Entry
    output wire [3:0]  debug_ptw_state_out,

    // Memory Interface (PTW takes over the memory bus)
    input  wire [31:0] mem_rdata_in,
    output reg [31:0]  mem_address_out,
    output reg         mem_read_enable_out
);

    assign debug_ptw_state_out = state;

    // FSM States - More robust version with explicit wait states
    localparam IDLE            = 4'd0;
    localparam L1_ADDR_CALC    = 4'd1;
    localparam L1_WAIT         = 4'd2;
    localparam L1_FETCH        = 4'd3;
    localparam L0_ADDR_CALC    = 4'd4;
    localparam L0_WAIT         = 4'd5;
    localparam L0_FETCH        = 4'd6;
    localparam DONE_STATE      = 4'd7;
    // FAULT_STATE removed

    reg [3:0] state = IDLE;

    // Internal registers
    reg [31:0] pte_l1_reg; // To store the fetched L1 PTE

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= IDLE;
            done <= 1'b0;
            mem_read_enable_out <= 1'b0;
        end else begin
            done <= 1'b0; // Default to not done

            case (state)
                IDLE: begin
                    mem_read_enable_out <= 1'b0;
                    if (start) begin
                        state <= L1_ADDR_CALC;
                    end
                end

                L1_ADDR_CALC: begin
                    // Calculate address of L1 PTE and start read
                    mem_address_out <= {satp_ppn, 12'b0} + {fault_vpn[19:10], 2'b00};
                    mem_read_enable_out <= 1'b1;
                    state <= L1_WAIT;
                end

                L1_WAIT: begin
                    // Wait one cycle for memory to respond. De-assert read enable.
                    mem_read_enable_out <= 1'b0;
                    state <= L1_FETCH;
                end

                L1_FETCH: begin
                    // Latch the data from memory
                    pte_l1_reg <= mem_rdata_in;
                    state <= L0_ADDR_CALC;
                end

                L0_ADDR_CALC: begin
                    // Check if L1 PTE is valid and not a leaf
                    if (pte_l1_reg[0] == 1'b0 || pte_l1_reg[3:1] != 3'b000) begin
                        // Invalid or a leaf PTE, this is a page fault
                        pte_out <= 32'h0; // Indicate error with invalid PTE
                        state <= DONE_STATE; // Go directly to DONE_STATE on fault
                    end else begin
                        // Calculate address of L0 PTE and start read
                        mem_address_out <= {pte_l1_reg[31:10], 12'b0} + {fault_vpn[9:0], 2'b00};
                        mem_read_enable_out <= 1'b1;
                        state <= L0_WAIT;
                    end
                end

                L0_WAIT: begin
                    // Wait one cycle for memory
                    mem_read_enable_out <= 1'b0;
                    state <= L0_FETCH;
                end

                L0_FETCH: begin
                    // Latch the final L0 PTE
                    pte_out <= mem_rdata_in;
                    state <= DONE_STATE;
                end

                DONE_STATE: begin
                    done <= 1'b1;
                    state <= IDLE;
                end

                default: begin
                    state <= IDLE;
                end
            endcase
        end
    end

endmodule
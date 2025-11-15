// Sequential, multi-cycle TLB lookup MMU
// This is a debugging version. It is slow but simple.
`default_nettype none

module mmu(
    input  wire clk,
    input  wire reset,

    // CPU Interface
    input  wire        start_lookup, // Pulse to start a new lookup
    input  wire [31:0] virt_addr,
    output reg [31:0] phys_addr,
    output reg         lookup_done,  // High for one cycle when lookup is complete
    output reg         tlb_hit,
    output reg         tlb_miss,

    // SATP from CPU
    input wire [31:0] satp,

    // TLB Write Interface (from CSRs)
    input wire [31:0] tlb_vpn_in,
    input wire [31:0] tlb_ppn_perms_in,
    input wire [31:0] tlb_write_index
);

    localparam TLB_ENTRIES = 16;

    // --- TLB Storage ---
    reg [31:0] tlb_vpn [0:TLB_ENTRIES-1];
    reg [31:0] tlb_ppn_perms [0:TLB_ENTRIES-1];
    reg tlb_valid [0:TLB_ENTRIES-1];

    // --- MMU FSM ---
    localparam IDLE   = 2'd0;
    localparam LOOKUP = 2'd1;
    localparam HIT    = 2'd2;
    localparam MISS   = 2'd3;
    reg [1:0] state = IDLE;

    reg [4:0] lookup_counter; // To iterate through TLB entries

    // --- Address decoding ---
    wire mmu_enable = ((satp >> 31) & 1);
    wire [19:0] vpn = virt_addr[31:12];
    wire [11:0] offset = virt_addr[11:0];

    // --- Combinational logic for lookup ---
    wire [19:0] ppn = tlb_ppn_perms[lookup_counter][29:10];

    // --- TLB Write Logic ---
    // Write is synchronous
    always @(posedge clk) begin
        if (tlb_write_index[4]) begin // Use a bit in the index as a write enable trigger
            tlb_vpn[tlb_write_index[3:0]] <= tlb_vpn_in;
            tlb_ppn_perms[tlb_write_index[3:0]] <= tlb_ppn_perms_in;
            tlb_valid[tlb_write_index[3:0]] <= 1'b1;
        end
    end

    // --- Main FSM and Lookup Logic ---
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= IDLE;
            lookup_done <= 1'b0;
            tlb_hit <= 1'b0;
            tlb_miss <= 1'b0;
            phys_addr <= 32'h0;
            lookup_counter <= 5'd0;
        end else begin
            // Default outputs
            lookup_done <= 1'b0;
            tlb_hit <= 1'b0;
            tlb_miss <= 1'b0;

            case (state)
                IDLE: begin
                    if (start_lookup) begin
                        if (!mmu_enable) begin
                            // MMU is off, treat as an immediate hit (identity mapping)
                            phys_addr <= virt_addr;
                            state <= HIT;
                        end else begin
                            // Start sequential search
                            lookup_counter <= 5'd0;
                            state <= LOOKUP;
                        end
                    end
                end

                LOOKUP: begin
                    // Check one entry per cycle
                    if (tlb_valid[lookup_counter] && tlb_vpn[lookup_counter][19:0] == vpn) begin
                        // Found a match
                        phys_addr <= {ppn, offset};
                        state <= HIT;
                    end else if (lookup_counter == TLB_ENTRIES - 1) begin
                        // Reached the end without a match
                        state <= MISS;
                    end else begin
                        // Continue to next entry
                        lookup_counter <= lookup_counter + 1;
                    end
                end

                HIT: begin
                    tlb_hit <= 1'b1;
                    lookup_done <= 1'b1;
                    state <= IDLE;
                end

                MISS: begin
                    tlb_miss <= 1'b1;
                    lookup_done <= 1'b1;
                    state <= IDLE;
                end
            endcase
        end
    end

    // Initialization for simulation
    integer i;
    initial begin
        for (i=0; i<TLB_ENTRIES; i=i+1) begin
            tlb_vpn[i] = 32'b0;
            tlb_ppn_perms[i] = 32'b0;
            tlb_valid[i] = 1'b0;
        end
    end

endmodule

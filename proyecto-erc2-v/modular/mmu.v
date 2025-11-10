`default_nettype none

module mmu (
    input wire clk,
    input wire reset,

    // Interface with processor
    input wire [31:0] virt_addr,
    output wire [31:0] phys_addr,

    // Interface with satp register
    input wire [31:0] satp,

    // MMU control
    input wire mmu_enable,

    // TLB outputs
    output wire tlb_hit,
    output wire tlb_miss,

    // TLB write interface
    input wire [31:0] tlb_vpn_in,
    input wire [31:0] tlb_ppn_perms,
    input wire [31:0] tlb_write_index
);

    localparam TLB_ENTRIES = 4;

    // TLB Entry: {valid, vpn, ppn, permissions}
    reg [TLB_ENTRIES-1:0] tlb_valid;
    reg [19:0] tlb_vpn [0:TLB_ENTRIES-1];
    reg [19:0] tlb_ppn [0:TLB_ENTRIES-1];
    reg [2:0] tlb_perms [0:TLB_ENTRIES-1]; // R, W, X

    wire [19:0] vpn = virt_addr[31:12];
    wire [11:0] offset = virt_addr[11:0];

    // TLB lookup
    wire [TLB_ENTRIES-1:0] tlb_hits;
    assign tlb_hits[0] = tlb_valid[0] && (tlb_vpn[0] == vpn);
    assign tlb_hits[1] = tlb_valid[1] && (tlb_vpn[1] == vpn);
    assign tlb_hits[2] = tlb_valid[2] && (tlb_vpn[2] == vpn);
    assign tlb_hits[3] = tlb_valid[3] && (tlb_vpn[3] == vpn);

    wire tlb_hit_any = |tlb_hits;
    assign tlb_hit = tlb_hit_any;
    assign tlb_miss = mmu_enable && !tlb_hit_any;

    // Find the matching entry
    wire [19:0] ppn_found = tlb_hits[0] ? tlb_ppn[0] :
                          tlb_hits[1] ? tlb_ppn[1] :
                          tlb_hits[2] ? tlb_ppn[2] :
                          tlb_hits[3] ? tlb_ppn[3] :
                          20'h0;

    wire [31:0] translated_addr = {ppn_found, offset};

    assign phys_addr = mmu_enable ? (tlb_hit ? translated_addr : virt_addr) : virt_addr; // For now, on miss, return virt_addr

    reg prev_tlb_write_trigger;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            prev_tlb_write_trigger <= 1'b0;
        end else begin
            prev_tlb_write_trigger <= tlb_write_index[0];
        end
    end

    wire write_trigger_edge = tlb_write_index[0] && !prev_tlb_write_trigger;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            tlb_valid <= 0;
            tlb_vpn[0] <= 0;
            tlb_ppn[0] <= 0;
            tlb_perms[0] <= 0;
            tlb_vpn[1] <= 0;
            tlb_ppn[1] <= 0;
            tlb_perms[1] <= 0;
            tlb_vpn[2] <= 0;
            tlb_ppn[2] <= 0;
            tlb_perms[2] <= 0;
            tlb_vpn[3] <= 0;
            tlb_ppn[3] <= 0;
            tlb_perms[3] <= 0;
        end else if (write_trigger_edge) begin
            tlb_valid[tlb_write_index[3:2]] <= tlb_ppn_perms[0];
            tlb_vpn[tlb_write_index[3:2]]   <= tlb_vpn_in[19:0];
            tlb_ppn[tlb_write_index[3:2]]   <= tlb_ppn_perms[31:10];
            tlb_perms[tlb_write_index[3:2]] <= tlb_ppn_perms[3:1];
        end
    end

endmodule

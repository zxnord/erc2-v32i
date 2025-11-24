// Monolithic processor with inlined MMU logic for debugging synthesis issues.
`default_nettype none

module procesador_monolithic(
    input  wire clk,
    input  wire reset,

    // Memory Interface
    input  wire [31:0] instruction_in,
    input  wire [31:0] mem_rdata_in,
    output wire [31:0] instruction_address_out,
    output wire [31:0] mem_address_out,
    output wire [31:0] mem_wdata_out,
    output wire        mem_wenable_out,

    // Debug
    output wire [7:0]  debug_out
);

    // --- Register File ---
    (* ram_style = "registers" *) // Force synthesis tool to use distributed registers for combinatorial reads
    reg [31:0] registers [0:31];

    // --- FSM & Core Registers ---
    reg [31:0] pc = 32'h0;
    reg [31:0] instruction_reg;
    reg [1:0] priv_mode = 2'b11;

    localparam FETCH_START        = 5'd0;
    localparam FETCH_WAIT         = 5'd1;
    localparam DECODE_PREP        = 5'd24; // New state to pipeline decode fields
    localparam EXECUTE_DECODE     = 5'd2;
    localparam EXECUTE_TRANSLATE  = 5'd22;
    localparam EXECUTE_DISPATCH   = 5'd23;
    localparam ALU_EXEC           = 5'd3;
    localparam BRANCH_EXEC        = 5'd4;
    localparam JAL_EXEC           = 5'd5;
    localparam JALR_EXEC          = 5'd6;
    localparam MEM_SETUP          = 5'd7;
    localparam MEM_WAIT           = 5'd8;
    localparam AMO_WRITE          = 5'd9;
    localparam MUL_START          = 5'd10;
    localparam MUL_WAIT           = 5'd11;
    localparam DIV_START          = 5'd12;
    localparam DIV_WAIT           = 5'd13;
    localparam CSR_EXEC           = 5'd14;
    localparam SFENCE_EXEC        = 5'd15;
    localparam MRET_EXEC          = 5'd16;
    localparam SRET_EXEC          = 5'd17;
    localparam ECALL_EXEC         = 5'd18;
    localparam EXCEPTION          = 5'd19;
    localparam PTW_START          = 5'd20;
    localparam PTW_WAIT           = 5'd21;
    
    reg [4:0] state = FETCH_START;
    reg [4:0] next_state; // Pipelined FSM state register

    // Pipeline registers
    reg [6:0] opcode_p;
    reg [2:0] funct3_p;
    reg [6:0] funct7_p;
    reg [11:0] funct12_p;
    reg [4:0] inst_type_reg;
    reg i_tlb_miss_reg, d_tlb_miss_reg;
    reg d_tlb_miss_check_reg;

    reg [4:0] sfence_counter = 5'd0;
    reg [31:0] amo_read_data_reg;
    reg [31:0] amo_addr_reg;
    reg [31:0] amo_rs2_data_reg;
    reg is_load_reg, is_store_reg, is_atomic_reg;
    reg [4:0] rdId_reg;

    reg [4:0] exception_cause_reg;
    reg is_inst_fault_reg;
    reg is_store_fault_reg;

    // --- CSR Registers ---
    reg [31:0] mstatus_reg, mepc_reg, mcause_reg, mtvec_reg, mtval_reg;
    reg [31:0] sstatus_reg, sepc_reg, scause_reg, stvec_reg, satp_reg, stval_reg;
    reg [31:0] tlb_vpn_reg, tlb_ppn_perms_reg, tlb_write_index_reg;
    reg [31:0] csr_rdata_reg;

    // --- Conditional Memory Delay for PTW ---
    wire ptw_active = (state == PTW_WAIT);
    reg [31:0] mem_rdata_delayed_reg;
    wire [31:0] mem_rdata_final;

    always @(posedge clk) begin
        mem_rdata_delayed_reg <= mem_rdata_in;
    end
    assign mem_rdata_final = ptw_active ? mem_rdata_delayed_reg : mem_rdata_in;

    // --- Instruction Decode ---
    // Stage 0: Raw fields from instruction_reg
    wire [11:0] csr_addr = instruction_reg[31:20];
    wire [6:0] opcode = instruction_reg[6:0];
    wire [4:0] rdId   = instruction_reg[11:7];
    wire [4:0] rs1Id  = instruction_reg[19:15];
    wire [4:0] rs2Id  = instruction_reg[24:20];
    wire [2:0] funct3 = instruction_reg[14:12];
    wire [6:0] funct7 = instruction_reg[31:25];
    wire [11:0] funct12 = instruction_reg[31:20];
    wire [31:0] imm_i = {{21{instruction_reg[31]}}, instruction_reg[30:20]};
    wire [31:0] imm_u = {instruction_reg[31:12], 12'b0};
    wire [31:0] imm_j = {{12{instruction_reg[31]}}, instruction_reg[19:12], instruction_reg[20], instruction_reg[30:21], 1'b0};
    wire [31:0] imm_b = {{20{instruction_reg[31]}}, instruction_reg[7], instruction_reg[30:25], instruction_reg[11:8], 1'b0};
    wire [31:0] imm_s = {{21{instruction_reg[31]}}, instruction_reg[30:25], instruction_reg[11:7]};

    // Stage 1: Decode flags from pipelined fields (opcode_p, funct3_p, etc.)
    wire is_load_p      = (opcode_p == 7'b0000011);
    wire is_store_p     = (opcode_p == 7'b0100011);
    wire is_alu_imm_p   = (opcode_p == 7'b0010011);
    wire is_alu_reg_p   = (opcode_p == 7'b0110011) && (funct7_p == 7'b0000000 || funct7_p == 7'b0100000);
    wire is_lui_p       = (opcode_p == 7'b0110111);
    wire is_jal_p       = (opcode_p == 7'b1101111);
    wire is_branch_p    = (opcode_p == 7'b1100011);
    wire is_auipc_p     = (opcode_p == 7'b0010111);
    wire is_jalr_p      = (opcode_p == 7'b1100111);
    wire is_system_p    = (opcode_p == 7'b1110011);
    wire is_atomic_p    = 1'b0; // A-extension DISABLED for debugging
    wire is_m_extension_p = (opcode_p == 7'b0110011) && (funct7_p == 7'b0000001);
    wire is_mul_p        = is_m_extension_p && (funct3_p == 3'b000);
    wire is_ecall_p = is_system_p && (funct12_p == 12'h000);
    wire is_sret_p = is_system_p && (funct12_p == 12'h102);
    wire is_mret_p = is_system_p && (funct12_p == 12'h302);
    wire is_sfence_vma_p = is_system_p && (funct12_p == 12'h120);
    wire is_csr_p = is_system_p && !is_ecall_p && !is_sret_p && !is_mret_p && !is_sfence_vma_p;
    wire is_illegal_p = !is_load_p && !is_store_p && !is_alu_imm_p && !is_alu_reg_p && !is_lui_p && !is_jal_p && !is_branch_p && !is_auipc_p && !is_jalr_p && !is_system_p && !is_m_extension_p && !is_atomic_p;

    // --- Register Access ---
    wire [31:0] rs1_data = (rs1Id == 5'b0) ? 32'b0 : registers[rs1Id];
    wire [31:0] rs2_data = (rs2Id == 5'b0) ? 32'b0 : registers[rs2Id];

    // --- Branch Logic ---
    reg branch_taken;
    always @(*) begin
        case (funct3)
            3'b000: branch_taken = (rs1_data == rs2_data); // BEQ
            3'b001: branch_taken = (rs1_data != rs2_data); // BNE
            3'b100: branch_taken = ($signed(rs1_data) < $signed(rs2_data)); // BLT
            3'b101: branch_taken = ($signed(rs1_data) >= $signed(rs2_data)); // BGE
            3'b110: branch_taken = (rs1_data < rs2_data);     // BLTU
            3'b111: branch_taken = (rs1_data >= rs2_data);    // BGEU
            default: branch_taken = 1'b0;
        endcase
    end

    // --- ALU & Data Path ---
    wire [31:0] alu_op2 = is_alu_reg_p ? rs2_data : imm_i;
    wire [4:0] shamt = alu_op2[4:0];
    reg [31:0] rd_wdata;
    wire rd_wenable = is_alu_reg_p || is_alu_imm_p || is_lui_p || is_jal_p || is_load_p || is_auipc_p || is_jalr_p || is_atomic_p;

    wire [4:0] amo_funct5 = instruction_reg[31:27];
    reg [31:0] amo_wdata;
    always @(*) begin
        case (amo_funct5)
            5'b00001: amo_wdata = amo_rs2_data_reg; // AMOSWAP
            5'b00000: amo_wdata = amo_read_data_reg + amo_rs2_data_reg; // AMOADD
            5'b00100: amo_wdata = amo_read_data_reg ^ amo_rs2_data_reg; // AMOXOR
            5'b01100: amo_wdata = amo_read_data_reg & amo_rs2_data_reg; // AMOAND
            5'b01000: amo_wdata = amo_read_data_reg | amo_rs2_data_reg; // AMOOR
            default:  amo_wdata = amo_rs2_data_reg; // Default to swap
        endcase
    end

    always @(*) begin
        if (is_load_p) rd_wdata = mem_rdata_final;
        else if (is_atomic_p) rd_wdata = amo_read_data_reg;
        else if (is_lui_p) rd_wdata = imm_u;
        else if (is_auipc_p) rd_wdata = pc + imm_u;
        else if (is_jal_p || is_jalr_p) rd_wdata = pc + 4;
        else if (is_m_extension_p) begin
            case (funct3_p)
                3'b000: rd_wdata = mul_result;
                3'b100: rd_wdata = div_quotient;
                3'b101: rd_wdata = div_quotient;
                3'b110: rd_wdata = div_remainder;
                3'b111: rd_wdata = div_remainder;
                default: rd_wdata = 32'hdeadbeef;
            endcase
        end
        else begin // Standard ALU
            case (funct3_p)
                3'b000: rd_wdata = (is_alu_reg_p && funct7_p[5]) ? (rs1_data - alu_op2) : (rs1_data + alu_op2);
                3'b001: rd_wdata = rs1_data << shamt;
                3'b010: rd_wdata = ($signed(rs1_data) < $signed(alu_op2)) ? 1 : 0;
                3'b011: rd_wdata = (rs1_data < alu_op2) ? 1 : 0;
                3'b100: rd_wdata = rs1_data ^ alu_op2;
                3'b101: rd_wdata = funct7_p[5] ? ($signed(rs1_data) >>> shamt) : (rs1_data >> shamt);
                3'b110: rd_wdata = rs1_data | alu_op2;
                3'b111: rd_wdata = rs1_data & alu_op2;
                default: rd_wdata = 32'b0;
            endcase
        end
    end

    // =================================================================
    // --- INLINED MMU LOGIC ---
    // =================================================================
    localparam TLB_ENTRIES = 16;
    (* ram_style = "registers" *) reg [31:0] tlb_vpn [0:TLB_ENTRIES-1];
    (* ram_style = "registers" *) reg [31:0] tlb_ppn_perms [0:TLB_ENTRIES-1];
    (* ram_style = "registers" *) reg tlb_valid [0:TLB_ENTRIES-1];

    wire mmu_enable = 1'b0; // S-extension DISABLED for debugging
    
    wire [19:0] i_vpn = pc[31:12];
    wire [11:0] i_offset = pc[11:0];
    wire [TLB_ENTRIES-1:0] i_tlb_hits;
    wire i_tlb_hit = |i_tlb_hits;
    wire i_tlb_miss = mmu_enable && !i_tlb_hit;
    wire [31:0] i_phys_addr;
    wire [19:0] i_ppn;

    genvar k_gen;
    generate for (k_gen = 0; k_gen < TLB_ENTRIES; k_gen = k_gen + 1) begin : i_tlb_check
        assign i_tlb_hits[k_gen] = tlb_valid[k_gen] && (tlb_vpn[k_gen][31:12] == i_vpn);
    end endgenerate
    
    wire [31:0] i_ppn_muxed [0:TLB_ENTRIES];
    assign i_ppn_muxed[0] = 32'h0;
    generate for (k_gen = 0; k_gen < TLB_ENTRIES; k_gen = k_gen + 1) begin : i_ppn_logic
        assign i_ppn_muxed[k_gen+1] = i_tlb_hits[k_gen] ? tlb_ppn_perms[k_gen] : i_ppn_muxed[k_gen];
    end endgenerate
    assign i_ppn = i_ppn_muxed[TLB_ENTRIES][29:10];
    assign i_phys_addr = mmu_enable ? {i_ppn, i_offset} : pc;

    wire [31:0] data_virt_addr = rs1_data + (is_load_p ? imm_i : (is_store_p ? imm_s : 32'd0));
    wire [19:0] d_vpn = data_virt_addr[31:12];
    wire [11:0] d_offset = data_virt_addr[11:0];
    wire [TLB_ENTRIES-1:0] d_tlb_hits;
    wire d_tlb_hit = |d_tlb_hits;
    wire d_tlb_miss = mmu_enable && !d_tlb_hit;
    wire [31:0] d_phys_addr;
    wire [19:0] d_ppn;

    genvar l_gen;
    generate for (l_gen = 0; l_gen < TLB_ENTRIES; l_gen = l_gen + 1) begin : d_tlb_check
        assign d_tlb_hits[l_gen] = tlb_valid[l_gen] && (tlb_vpn[l_gen][31:12] == d_vpn);
    end endgenerate

    wire [31:0] d_ppn_muxed [0:TLB_ENTRIES];
    assign d_ppn_muxed[0] = 32'h0;
    generate for (l_gen = 0; l_gen < TLB_ENTRIES; l_gen = l_gen + 1) begin : d_ppn_logic
        assign d_ppn_muxed[l_gen+1] = d_tlb_hits[l_gen] ? tlb_ppn_perms[l_gen] : d_ppn_muxed[l_gen];
    end endgenerate
    assign d_ppn = d_ppn_muxed[TLB_ENTRIES][29:10];
    assign d_phys_addr = mmu_enable ? {d_ppn, d_offset} : data_virt_addr;

    // --- M-Extension & PTW Modules ---
    wire mul_start = (state == MUL_START);
    wire div_start = (state == DIV_START);
    wire mul_busy, div_busy;
    wire [31:0] mul_result, div_quotient, div_remainder;
    multiplier i_multiplier ( .clk(clk), .reset(reset), .start(mul_start), .rs1_data(rs1_data), .rs2_data(rs2_data), .result(mul_result), .busy(mul_busy) );
    divider i_divider ( .clk(clk), .reset(reset), .start(div_start), .dividend(rs1_data), .divisor(rs2_data), .quotient_out(div_quotient), .remainder_out(div_remainder), .busy(div_busy) );

    reg [19:0] fault_vpn_reg;
    wire ptw_start = (state == PTW_START);
    wire ptw_done;
    wire [31:0] ptw_pte_out;
    wire [31:0] ptw_mem_address_out;
    wire ptw_mem_read_enable_out;
    wire [3:0] ptw_internal_state;
    ptw i_ptw ( .clk(clk), .reset(reset), .start(ptw_start), .fault_vpn(fault_vpn_reg), .satp_ppn(satp_reg[21:0]), .done(ptw_done), .pte_out(ptw_pte_out), .debug_ptw_state_out(ptw_internal_state), .mem_rdata_in(mem_rdata_final), .mem_address_out(ptw_mem_address_out), .mem_read_enable_out(ptw_mem_read_enable_out) );

    // --- Memory Bus Arbitration ---
    wire mem_op_active = (state == MEM_SETUP || state == MEM_WAIT || state == AMO_WRITE);
    wire amo_write_active = (state == AMO_WRITE);
    assign mem_address_out = ptw_active ? ptw_mem_address_out : (mem_op_active ? amo_addr_reg : i_phys_addr);
    assign mem_wdata_out   = amo_write_active ? amo_wdata : amo_rs2_data_reg;
    assign mem_wenable_out = ptw_active ? 1'b0 : ((is_store_reg && state == MEM_WAIT) || amo_write_active);

    // --- Debug Outputs ---
    assign debug_out = {4'b1010, state[3:0]};

    // --- CSR Logic ---
    wire [31:0] csr_wdata_calculated = (funct3[2] == 1'b0) ? rs1_data : (funct3[1] == 1'b0) ? (csr_rdata_reg | rs1_data) : (csr_rdata_reg & ~rs1_data);
    wire [31:0] csr_rdata = (csr_addr == 12'h300) ? mstatus_reg : (csr_addr == 12'h341) ? mepc_reg : (csr_addr == 12'h342) ? mcause_reg : (csr_addr == 12'h305) ? mtvec_reg : (csr_addr == 12'h343) ? mtval_reg : (csr_addr == 12'h180) ? satp_reg : (csr_addr == 12'h7C0) ? tlb_vpn_reg : (csr_addr == 12'h7C1) ? tlb_ppn_perms_reg : (csr_addr == 12'h7C2) ? tlb_write_index_reg : 32'h0;

    // --- FSM Sequential Logic ---
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc <= 32'h0;
            state <= FETCH_START;
            instruction_reg <= 32'h13; // NOP
            priv_mode <= 2'b11;
            mstatus_reg <= 32'h0; mepc_reg <= 32'h0; mcause_reg <= 32'h0; mtvec_reg <= 32'h0; mtval_reg <= 32'h0;
            satp_reg <= 32'h0;
            is_load_reg <= 1'b0; is_store_reg <= 1'b0; is_atomic_reg <= 1'b0;
            sfence_counter <= 5'd0;
        end else begin
            state <= next_state;

            case (state)
                FETCH_WAIT: instruction_reg <= instruction_in;
                
                DECODE_PREP: begin
                    opcode_p <= opcode;
                    funct3_p <= funct3;
                    funct7_p <= funct7;
                    funct12_p <= funct12;
                end

                EXECUTE_DECODE: begin
                    inst_type_reg <= inst_type;
                end

                EXECUTE_TRANSLATE: begin
                    i_tlb_miss_reg <= i_tlb_miss;
                    d_tlb_miss_reg <= d_tlb_miss;
                    d_tlb_miss_check_reg <= is_load_p || is_store_p || is_atomic_p;

                    if (i_tlb_miss) begin
                        fault_vpn_reg <= i_vpn;
                        is_inst_fault_reg <= 1'b1;
                    end
                    if (d_tlb_miss && (is_load_p || is_store_p || is_atomic_p)) begin
                        fault_vpn_reg <= d_vpn;
                        is_inst_fault_reg <= 1'b0;
                        is_store_fault_reg <= is_store_p || is_atomic_p;
                    end
                    if (is_load_p || is_store_p || is_atomic_p) begin
                        amo_addr_reg <= d_phys_addr;
                        amo_rs2_data_reg <= rs2_data;
                        rdId_reg <= rdId;
                        is_load_reg <= is_load_p;
                        is_store_reg <= is_store_p;
                        is_atomic_reg <= is_atomic_p;
                    end
                end

                ALU_EXEC: begin if (rd_wenable && rdId != 0) registers[rdId] <= rd_wdata; pc <= pc + 4; end
                BRANCH_EXEC: pc <= pc + imm_b;
                JAL_EXEC: begin if (rdId != 0) registers[rdId] <= pc + 4; pc <= pc + imm_j; end
                JALR_EXEC: begin if (rdId != 0) registers[rdId] <= pc + 4; pc <= (rs1_data + imm_i) & 32'hFFFFFFFE; end
                MRET_EXEC: begin pc <= mepc_reg; priv_mode <= mstatus_reg[12:11]; end
                SRET_EXEC: begin pc <= sepc_reg; priv_mode <= sstatus_reg[9:8]; end
                ECALL_EXEC: begin exception_cause_reg <= 8 + priv_mode; mtval_reg <= pc; end

                MEM_WAIT: begin
                    if (is_load_reg) begin if (rdId_reg != 0) registers[rdId_reg] <= mem_rdata_final; pc <= pc + 4;
                    end else if (is_store_reg) begin pc <= pc + 4;
                    end else if (is_atomic_reg) begin amo_read_data_reg <= mem_rdata_final;
                    end else begin pc <= pc + 4; end
                end

                AMO_WRITE: begin if (rdId_reg != 0) registers[rdId_reg] <= amo_read_data_reg; pc <= pc + 4; end
        
                EXCEPTION: begin mepc_reg <= pc; mcause_reg <= {27'b0, exception_cause_reg}; mstatus_reg[12:11] <= priv_mode; priv_mode <= 2'b11; pc <= mtvec_reg; end
        
                CSR_EXEC: begin
                    if (rdId != 0) registers[rdId] <= csr_rdata;
                    if (funct3 != 3'b000) begin
                        case(csr_addr)
                            12'h300: mstatus_reg <= csr_wdata_calculated; 12'h341: mepc_reg <= csr_wdata_calculated; 12'h342: mcause_reg <= csr_wdata_calculated; 12'h305: mtvec_reg <= csr_wdata_calculated; 12'h343: mtval_reg <= csr_wdata_calculated; 12'h180: satp_reg <= csr_wdata_calculated; 12'h7C0: tlb_vpn_reg <= csr_wdata_calculated; 12'h7C1: tlb_ppn_perms_reg <= csr_wdata_calculated; 12'h7C2: tlb_write_index_reg <= csr_wdata_calculated;
                        endcase
                    end
                    pc <= pc + 4;
                end
        
                SFENCE_EXEC: begin
                    tlb_valid[sfence_counter] <= 1'b0;
                    if (sfence_counter == TLB_ENTRIES - 1) begin pc <= pc + 4; sfence_counter <= 5'd0;
                    end else begin sfence_counter <= sfence_counter + 1; end
                end
        
                PTW_WAIT: begin
                    if (ptw_done && !ptw_pte_out[0]) begin exception_cause_reg <= is_inst_fault_reg ? 12 : (is_store_fault_reg ? 15 : 13); mtval_reg <= {fault_vpn_reg, 12'b0};
                    end else if (ptw_done && ptw_pte_out[0]) begin tlb_vpn[tlb_write_index_reg[3:0]] <= {fault_vpn_reg, 12'b0}; tlb_ppn_perms[tlb_write_index_reg[3:0]] <= ptw_pte_out; tlb_valid[tlb_write_index_reg[3:0]] <= 1'b1; tlb_write_index_reg <= tlb_write_index_reg + 1; end
                end

                MUL_WAIT: begin if (!mul_busy) begin if (rdId != 0) registers[rdId] <= rd_wdata; pc <= pc + 4; end end
                DIV_WAIT: begin if (!div_busy) begin if (rdId != 0) registers[rdId] <= rd_wdata; pc <= pc + 4; end end
            endcase
        end
    end

    // --- FSM Combinatorial Logic ---
    localparam INST_TYPE_ILLEGAL = 5'd1;
    localparam INST_TYPE_MEM = 5'd2;
    localparam INST_TYPE_M_EXT = 5'd3;
    localparam INST_TYPE_ECALL = 5'd4;
    localparam INST_TYPE_CSR = 5'd5;
    localparam INST_TYPE_SFENCE = 5'd6;
    localparam INST_TYPE_BRANCH = 5'd7;
    localparam INST_TYPE_JAL = 5'd8;
    localparam INST_TYPE_JALR = 5'd9;
    localparam INST_TYPE_MRET = 5'd10;
    localparam INST_TYPE_SRET = 5'd11;
    localparam INST_TYPE_ALU = 5'd12;

    reg [4:0] inst_type;
    always @(*) begin
        if (is_illegal_p) inst_type = INST_TYPE_ILLEGAL;
        else if (is_load_p || is_store_p || is_atomic_p) inst_type = INST_TYPE_MEM;
        else if (is_m_extension_p) inst_type = INST_TYPE_M_EXT;
        else if (is_ecall_p) inst_type = INST_TYPE_ECALL;
        else if (is_csr_p) inst_type = INST_TYPE_CSR;
        else if (is_sfence_vma_p) inst_type = INST_TYPE_SFENCE;
        else if (is_branch_p && branch_taken) inst_type = INST_TYPE_BRANCH;
        else if (is_jal_p) inst_type = INST_TYPE_JAL;
        else if (is_jalr_p) inst_type = INST_TYPE_JALR;
        else if (is_mret_p) inst_type = INST_TYPE_MRET;
        else if (is_sret_p) inst_type = INST_TYPE_SRET;
        else inst_type = INST_TYPE_ALU;
    end

    always @(*) begin
        next_state = state;
        case (state)
            FETCH_START: next_state = FETCH_WAIT;
            FETCH_WAIT: next_state = DECODE_PREP;
            DECODE_PREP: next_state = EXECUTE_DECODE;
            EXECUTE_DECODE: next_state = EXECUTE_TRANSLATE;
            EXECUTE_TRANSLATE: next_state = EXECUTE_DISPATCH;

            EXECUTE_DISPATCH: begin
                if (i_tlb_miss_reg) next_state = PTW_START;
                else if (d_tlb_miss_reg && d_tlb_miss_check_reg) next_state = PTW_START;
                else begin
                    case(inst_type_reg)
                        INST_TYPE_ILLEGAL: next_state = EXCEPTION;
                        INST_TYPE_MEM:     next_state = MEM_SETUP;
                        INST_TYPE_M_EXT:   if (is_mul_p) next_state = MUL_START; else next_state = DIV_START;
                        INST_TYPE_ECALL:   next_state = ECALL_EXEC;
                        INST_TYPE_CSR:     next_state = CSR_EXEC;
                        INST_TYPE_SFENCE:  next_state = SFENCE_EXEC;
                        INST_TYPE_BRANCH:  next_state = BRANCH_EXEC;
                        INST_TYPE_JAL:     next_state = JAL_EXEC;
                        INST_TYPE_JALR:    next_state = JALR_EXEC;
                        INST_TYPE_MRET:    next_state = MRET_EXEC;
                        INST_TYPE_SRET:    next_state = SRET_EXEC;
                        INST_TYPE_ALU:     next_state = ALU_EXEC;
                        default:           next_state = EXCEPTION;
                    endcase
                end
            end

            ALU_EXEC, BRANCH_EXEC, JAL_EXEC, JALR_EXEC, MRET_EXEC, SRET_EXEC, CSR_EXEC: next_state = FETCH_START;
            ECALL_EXEC, EXCEPTION: next_state = FETCH_START;
            SFENCE_EXEC: if (sfence_counter == TLB_ENTRIES - 1) next_state = FETCH_START; else next_state = SFENCE_EXEC;
            MEM_SETUP: next_state = MEM_WAIT;
            MEM_WAIT: if (is_load_reg || is_store_reg) next_state = FETCH_START; else if (is_atomic_reg) next_state = AMO_WRITE; else next_state = FETCH_START;
            AMO_WRITE: next_state = FETCH_START;
            PTW_START: next_state = PTW_WAIT;
            PTW_WAIT: if (ptw_done) if (ptw_pte_out[0]) next_state = FETCH_START; else next_state = EXCEPTION;
            MUL_START: next_state = MUL_WAIT;
            MUL_WAIT: if (!mul_busy) next_state = FETCH_START;
            DIV_START: next_state = DIV_WAIT;
            DIV_WAIT: if (!div_busy) next_state = FETCH_START;
        endcase
    end

    // Initialization for simulation
    initial begin
        $readmemh("zero_registers.hex", registers);
        $readmemh("zero_tlb_vpn.hex", tlb_vpn);
        $readmemh("zero_tlb_ppn_perms.hex", tlb_ppn_perms);
        tlb_valid[0] <= 1'b0; tlb_valid[1] <= 1'b0; tlb_valid[2] <= 1'b0; tlb_valid[3] <= 1'b0;
        tlb_valid[4] <= 1'b0; tlb_valid[5] <= 1'b0; tlb_valid[6] <= 1'b0; tlb_valid[7] <= 1'b0;
        tlb_valid[8] <= 1'b0; tlb_valid[9] <= 1'b0; tlb_valid[10] <= 1'b0; tlb_valid[11] <= 1'b0;
        tlb_valid[12] <= 1'b0; tlb_valid[13] <= 1'b0; tlb_valid[14] <= 1'b0; tlb_valid[15] <= 1'b0;
    end

endmodule
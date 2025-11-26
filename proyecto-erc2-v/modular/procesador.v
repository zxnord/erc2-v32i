`default_nettype none

// procesador.v - Version 3.17 (DEBUG)
// DEBUG: Restaurando rd_wdata_comb.

module procesador(
    input  wire clk,
    input  wire reset,

    // Interfaz con la memoria
    input  wire [31:0] instruction_in,
    input  wire [31:0] mem_rdata_in,
    output wire [31:0] instruction_address_out,
    output wire [31:0] mem_address_out,
    output wire [31:0] mem_wdata_out,
    output wire        mem_wenable_out,
    output wire [7:0]  debug_out
);

    // --- Banco de Registros ---
    (* ram_style = "registers" *)
    reg [31:0] registers [0:31];

    // --- FSM & Registros de Pipeline ---
    reg [31:0] pc = 32'h0;
    reg [31:0] instruction_reg;
    reg [1:0] priv_mode = 2'b11; // Inicia en Machine Mode

    // Estados de la FSM
    localparam FETCH_START        = 5'd0;
    localparam FETCH_WAIT         = 5'd1;
    localparam DECODE_PREP        = 5'd2;
    localparam EXECUTE_DECODE     = 5'd3;
    localparam EXECUTE_TRANSLATE  = 5'd4;
    localparam EXECUTE_DISPATCH   = 5'd5;
    localparam ALU_EXEC           = 5'd6;
    localparam BRANCH_EXEC        = 5'd7;
    localparam JAL_EXEC           = 5'd8;
    localparam JALR_EXEC          = 5'd9;
    localparam MEM_SETUP          = 5'd10;
    localparam MEM_WAIT           = 5'd11;
    localparam AMO_WRITE          = 5'd12;
    localparam MUL_START          = 5'd13;
    localparam MUL_WAIT           = 5'd14;
    localparam DIV_START          = 5'd15;
    localparam DIV_WAIT           = 5'd16;
    localparam EXCEPTION          = 5'd17;
    localparam CSR_READ           = 5'd18;
    localparam SFENCE_EXEC        = 5'd19;
    localparam SRET_EXEC          = 5'd20;
    localparam MRET_EXEC          = 5'd21;
    localparam ECALL_EXEC         = 5'd22;
    localparam PTW_START          = 5'd23;
    localparam PTW_WAIT           = 5'd24;
    localparam MUL_DONE           = 5'd25;
    localparam DIV_DONE           = 5'd26;
    localparam LUI_EXEC           = 5'd27;
    localparam CSR_WRITE          = 5'd28;
    localparam CSR_CALC           = 5'd29;
    
    (* keep = 1 *) reg [4:0] state = FETCH_START;
    (* keep = 1 *) reg [4:0] next_state;

    // Registros de Pipeline
    reg [6:0] opcode_p;
    reg [2:0] funct3_p;
    reg [6:0] funct7_p;
    reg [11:0] funct12_p;
    reg [31:0] rs1_data_p, rs2_data_p;
    reg [31:0] imm_u_p, imm_i_p, imm_s_p;
    reg [4:0] inst_type_reg;
    reg i_tlb_miss_reg, d_tlb_miss_reg;
    reg d_tlb_miss_check_reg;
    reg [31:0] csr_rdata_p;
    reg [31:0] rd_wdata_csr_p;
    reg [31:0] csr_wdata_p;

    // Registros para operaciones de memoria y atomicas
    reg [31:0] amo_read_data_reg;
    reg [31:0] amo_addr_reg;
    reg [31:0] amo_rs2_data_reg;
    reg is_load_reg, is_store_reg, is_atomic_reg, is_lr_reg, is_sc_reg, is_amo_reg;
    reg [4:0] rdId_reg;
    reg sc_success_reg;
    reg [31:0] reservation_addr;
    reg reservation_valid = 1'b0;

    // Registros para Excepciones y SFENCE
    reg [4:0] exception_cause_reg;
    reg is_inst_fault_reg;
    reg is_store_fault_reg;
    reg [4:0] sfence_counter = 5'd0;

    // --- CSRs (Machine and Supervisor) ---
    reg [31:0] mstatus_reg, mepc_reg, mcause_reg, mtvec_reg, mtval_reg;
    reg [31:0] sstatus_reg, sepc_reg, scause_reg, stvec_reg, stval_reg, satp_reg;
    reg [31:0] tlb_vpn_reg, tlb_ppn_perms_reg, tlb_write_index_reg; // Custom CSRs for TLB access

    // --- Decodificacion ---
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

    wire is_atomic_op_p = (opcode_p == 7'b0101111) && (funct3_p == 3'b010);
    wire is_lr_p        = is_atomic_op_p && (funct7_p[6:2] == 5'b00010);
    wire is_sc_p        = is_atomic_op_p && (funct7_p[6:2] == 5'b00011);
    wire is_amo_p       = is_atomic_op_p && !is_lr_p && !is_sc_p;

    // --- Lectura de Registros y Logica de Salto ---
    wire [31:0] rs1_data = (rs1Id == 5'b0) ? 32'b0 : registers[rs1Id];
    wire [31:0] rs2_data = (rs2Id == 5'b0) ? 32'b0 : registers[rs2Id];
    
    reg branch_taken;
    always @(*) begin
        case (funct3_p) // Correctly pipelined
            3'b000: branch_taken = (rs1_data_p == rs2_data_p);
            3'b001: branch_taken = (rs1_data_p != rs2_data_p);
            3'b100: branch_taken = ($signed(rs1_data_p) < $signed(rs2_data_p));
            3'b101: branch_taken = ($signed(rs1_data_p) >= $signed(rs2_data_p));
            3'b110: branch_taken = (rs1_data_p < rs2_data_p);
            3'b111: branch_taken = (rs1_data_p >= rs2_data_p);
            default: branch_taken = 1'b0;
        endcase
    end

    // --- ALU y Logica de Escritura ---
    wire [31:0] alu_op2 = (opcode_p == 7'b0110011) ? rs2_data_p : imm_i_p; // Pipelined
    wire [4:0] shamt = alu_op2[4:0];
    wire [31:0] rd_wdata;
    reg [31:0] rd_wdata_comb;
    assign rd_wdata = rd_wdata_comb;
    wire rd_wenable; 

    reg [31:0] amo_result;
    always @(*) begin
        case (funct7_p[6:2]) 
            5'b00001: amo_result = rs2_data_p; 5'b00000: amo_result = amo_read_data_reg + rs2_data_p;
            5'b00100: amo_result = amo_read_data_reg ^ rs2_data_p; 5'b01100: amo_result = amo_read_data_reg & rs2_data_p;
            5'b01000: amo_result = amo_read_data_reg | rs2_data_p; 5'b10000: amo_result = ($signed(amo_read_data_reg) < $signed(rs2_data_p)) ? amo_read_data_reg : rs2_data_p;
            5'b10100: amo_result = ($signed(amo_read_data_reg) > $signed(rs2_data_p)) ? amo_read_data_reg : rs2_data_p;
            5'b11000: amo_result = (amo_read_data_reg < rs2_data_p) ? amo_read_data_reg : rs2_data_p;
            5'b11100: amo_result = (amo_read_data_reg > rs2_data_p) ? amo_read_data_reg : rs2_data_p;
            default:  amo_result = amo_read_data_reg;
        endcase
    end

    // --- MMU / TLB ---
    localparam TLB_ENTRIES = 16;
    (* ram_style = "registers" *) reg [31:0] tlb_vpn [0:TLB_ENTRIES-1];
    (* ram_style = "registers" *) reg [31:0] tlb_ppn_perms [0:TLB_ENTRIES-1];
    (* ram_style = "registers" *) reg tlb_valid [0:TLB_ENTRIES-1];

    wire mmu_enable = (satp_reg[31]) && (priv_mode < 2'b11);
    
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

    wire [31:0] data_virt_addr = is_atomic_op_p ? rs1_data_p : (rs1_data_p + ((opcode_p == 7'b0000011) ? imm_i_p : imm_s_p));
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
    wire mul_busy, div_busy;
    wire [31:0] mul_result, div_quotient, div_remainder;
    multiplier mul_unit ( .clk(clk), .reset(reset), .start(state == MUL_START), .rs1_data(rs1_data_p), .rs2_data(rs2_data_p), .result(mul_result), .busy(mul_busy) );
    divider div_unit ( .clk(clk), .reset(reset), .start(state == DIV_START), .dividend(rs1_data_p), .divisor(rs2_data_p), .quotient_out(div_quotient), .remainder_out(div_remainder), .busy(div_busy) );

    reg [19:0] fault_vpn_reg;
    wire ptw_start = (state == PTW_START);
    wire ptw_done;
    wire [31:0] ptw_pte_out;
    wire [31:0] ptw_mem_address_out;
    wire ptw_mem_read_enable_out;
    ptw i_ptw ( .clk(clk), .reset(reset), .start(ptw_start), .fault_vpn(fault_vpn_reg), .satp_ppn(satp_reg[21:0]), .done(ptw_done), .pte_out(ptw_pte_out), .mem_rdata_in(mem_rdata_in), .mem_address_out(ptw_mem_address_out), .mem_read_enable_out(ptw_mem_read_enable_out) );

    // --- Memory Bus Arbitration ---
    wire ptw_active = (state == PTW_WAIT);
    wire mem_op_active = (state == MEM_SETUP || state == MEM_WAIT || state == AMO_WRITE);
    assign mem_address_out = ptw_active ? ptw_mem_address_out : (mem_op_active ? amo_addr_reg : i_phys_addr);
    assign mem_wdata_out   = is_store_reg ? amo_rs2_data_reg : amo_result;
    assign mem_wenable_out = (is_store_reg && state == MEM_WAIT) || (state == AMO_WRITE && (!is_sc_reg || sc_success_reg));

    // --- CSR Logic ---
    // DEBUG: Scorched earth
    wire [31:0] csr_wdata = 32'h0;
    
    always @(*) begin
        // DEBUG: Restoring rd_wdata_comb
        rd_wdata_comb = 32'h0; // Default
        case (inst_type_reg)
            INST_TYPE_MEM: begin
                if (is_load_reg) rd_wdata_comb = mem_rdata_in;
                else if (is_lr_reg) rd_wdata_comb = mem_rdata_in;
                else if (is_sc_reg) rd_wdata_comb = sc_success_reg ? 32'h0 : 32'h1;
                else if (is_amo_reg) rd_wdata_comb = amo_read_data_reg;
            end
            INST_TYPE_ALU: begin
                if (opcode_p == 7'b0010111) rd_wdata_comb = pc + imm_u_p; // AUIPC
                else begin
                    case (funct3_p)
                        3'b000: rd_wdata_comb = (opcode_p == 7'b0110011 && funct7_p[5]) ? (rs1_data_p - alu_op2) : (rs1_data_p + alu_op2);
                        3'b001: rd_wdata_comb = rs1_data_p << shamt;
                        3'b010: rd_wdata_comb = ($signed(rs1_data_p) < $signed(alu_op2)) ? 1 : 0;
                        3'b011: rd_wdata_comb = (rs1_data_p < alu_op2) ? 1 : 0;
                        3'b100: rd_wdata_comb = rs1_data_p ^ alu_op2;
                        3'b101: rd_wdata_comb = funct7_p[5] ? ($signed(rs1_data_p) >>> shamt) : (rs1_data_p >> shamt);
                        3'b110: rd_wdata_comb = rs1_data_p | alu_op2;
                        3'b111: rd_wdata_comb = rs1_data_p & alu_op2;
                        default: rd_wdata_comb = 32'h0;
                    endcase
                end
            end
            INST_TYPE_JAL, INST_TYPE_JALR: rd_wdata_comb = pc + 4;
            INST_TYPE_M_EXT: begin
                if (funct3_p[1] == 1'b0) rd_wdata_comb = div_quotient;
                else rd_wdata_comb = div_remainder;
            end
            INST_TYPE_CSR: rd_wdata_comb = csr_rdata_p;
        endcase
    end
    assign rd_wenable = (inst_type_reg == INST_TYPE_ALU) || (inst_type_reg == INST_TYPE_MEM) || (inst_type_reg == INST_TYPE_JAL) || (inst_type_reg == INST_TYPE_JALR) || (inst_type_reg == INST_TYPE_CSR);

    // --- FSM Secuencial ---
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc <= 32'h0; state <= FETCH_START; instruction_reg <= 32'h13; priv_mode <= 2'b11;
            mstatus_reg <= 32'h0; mepc_reg <= 32'h0; mcause_reg <= 32'h0; mtvec_reg <= 32'h0; mtval_reg <= 32'h0;
            sstatus_reg <= 32'h0; sepc_reg <= 32'h0; scause_reg <= 32'h0; stvec_reg <= 32'h0; stval_reg <= 32'h0; satp_reg <= 32'h0;
            reservation_valid <= 1'b0; sfence_counter <= 5'd0;
        end else begin
            state <= next_state;
            if (mem_wenable_out && !is_sc_reg && reservation_valid && (mem_address_out == reservation_addr)) reservation_valid <= 1'b0;

            case (state)
                FETCH_WAIT: instruction_reg <= instruction_in;
                DECODE_PREP: begin
                    {opcode_p, funct3_p, funct7_p, funct12_p} <= {opcode, funct3, funct7, funct12};
                    {rs1_data_p, rs2_data_p} <= {rs1_data, rs2_data};
                    {imm_u_p, imm_i_p, imm_s_p} <= {imm_u, imm_i, imm_s};
                end
                EXECUTE_DECODE: inst_type_reg <= inst_type;
                EXECUTE_TRANSLATE: begin
                    i_tlb_miss_reg <= i_tlb_miss; d_tlb_miss_reg <= d_tlb_miss;
                    d_tlb_miss_check_reg <= (opcode_p == 7'b0000011) || (opcode_p == 7'b0100011) || is_atomic_op_p;
                    if (i_tlb_miss) {fault_vpn_reg, is_inst_fault_reg} <= {i_vpn, 1'b1};
                    if (d_tlb_miss && d_tlb_miss_check_reg) {fault_vpn_reg, is_inst_fault_reg, is_store_fault_reg} <= {d_vpn, 1'b0, ((opcode_p == 7'b0100011) || is_atomic_op_p)};
                    amo_addr_reg <= d_phys_addr; amo_rs2_data_reg <= rs2_data_p; rdId_reg <= rdId;
                    {is_load_reg, is_store_reg, is_atomic_reg, is_lr_reg, is_sc_reg, is_amo_reg} <= {(opcode_p == 7'b0000011), (opcode_p == 7'b0100011), is_atomic_op_p, is_lr_p, is_sc_p, is_amo_p};
                end
                EXECUTE_DISPATCH: if (is_sc_reg) sc_success_reg <= reservation_valid && (reservation_addr == amo_addr_reg);
                LUI_EXEC: begin if (rdId_reg != 0) registers[rdId_reg] <= imm_u_p; pc <= pc + 4; end
                ALU_EXEC: begin if (rd_wenable && rdId_reg != 0) registers[rdId_reg] <= rd_wdata; pc <= pc + 4; end
                BRANCH_EXEC: pc <= pc + imm_b;
                JAL_EXEC: begin if (rd_wenable && rdId_reg != 0) registers[rdId_reg] <= pc + 4; pc <= pc + imm_j; end
                JALR_EXEC: begin if (rd_wenable && rdId_reg != 0) registers[rdId_reg] <= pc + 4; pc <= (rs1_data_p + imm_i_p) & 32'hFFFFFFFE; end
                MRET_EXEC: begin pc <= mepc_reg; priv_mode <= mstatus_reg[12:11]; end
                SRET_EXEC: begin pc <= sepc_reg; priv_mode <= sstatus_reg[9:8]; end
                ECALL_EXEC: begin exception_cause_reg <= 8 + priv_mode; mtval_reg <= pc; end
                MEM_WAIT: begin
                    if (is_load_reg) begin if (rdId_reg != 0) registers[rdId_reg] <= mem_rdata_in; if (is_lr_reg) {reservation_addr, reservation_valid} <= {amo_addr_reg, 1'b1}; pc <= pc + 4;
                    end else if (is_store_reg) begin if (is_sc_reg && sc_success_reg) reservation_valid <= 1'b0; pc <= pc + 4;
                    end else if (is_amo_reg) amo_read_data_reg <= mem_rdata_in;
                end
                AMO_WRITE: begin if (rdId_reg != 0) registers[rdId_reg] <= rd_wdata; pc <= pc + 4; end
                EXCEPTION: begin mepc_reg <= pc; mcause_reg <= {27'b0, exception_cause_reg}; mstatus_reg[12:11] <= priv_mode; priv_mode <= 2'b11; pc <= mtvec_reg; end
                CSR_READ: begin case (funct12_p) 12'h300: csr_rdata_p <= mstatus_reg; 12'h341: csr_rdata_p <= mepc_reg; 12'h342: csr_rdata_p <= mcause_reg; 12'h305: csr_rdata_p <= mtvec_reg; 12'h343: csr_rdata_p <= mtval_reg; 12'h100: csr_rdata_p <= sstatus_reg; 12'h141: csr_rdata_p <= sepc_reg; 12'h142: csr_rdata_p <= scause_reg; 12'h105: csr_rdata_p <= stvec_reg; 12'h143: csr_rdata_p <= stval_reg; 12'h180: satp_reg <= csr_rdata_p; default: csr_rdata_p <= 32'h0; endcase; end
                CSR_CALC: begin rd_wdata_csr_p <= rd_wdata; csr_wdata_p <= csr_wdata; end
                CSR_WRITE: begin if (rd_wenable && rdId_reg != 0) registers[rdId_reg] <= rd_wdata_csr_p; if (funct3_p != 3'b000) case(funct12_p) 12'h300: mstatus_reg <= csr_wdata_p; 12'h341: mepc_reg <= csr_wdata_p; 12'h342: mcause_reg <= csr_wdata_p; 12'h305: mtvec_reg <= csr_wdata_p; 12'h343: mtval_reg <= csr_wdata_p; 12'h100: sstatus_reg <= csr_wdata_p; 12'h141: sepc_reg <= csr_wdata_p; 12'h142: scause_reg <= csr_wdata_p; 12'h105: stvec_reg <= csr_wdata_p; 12'h143: stval_reg <= csr_wdata_p; 12'h180: satp_reg <= csr_wdata_p; endcase; pc <= pc + 4; end
                SFENCE_EXEC: begin tlb_valid[sfence_counter] <= 1'b0; if (sfence_counter == TLB_ENTRIES - 1) {pc, sfence_counter} <= {pc + 4, 5'd0}; else sfence_counter <= sfence_counter + 1; end
                PTW_WAIT: if (ptw_done) if (!ptw_pte_out[0]) {exception_cause_reg, mtval_reg} <= {is_inst_fault_reg ? 12 : (is_store_fault_reg ? 15 : 13), {fault_vpn_reg, 12'b0}}; else {tlb_vpn[tlb_write_index_reg[3:0]], tlb_ppn_perms[tlb_write_index_reg[3:0]], tlb_valid[tlb_write_index_reg[3:0]], tlb_write_index_reg} <= {{fault_vpn_reg, 12'b0}, ptw_pte_out, 1'b1, tlb_write_index_reg + 1};
                MUL_WAIT: begin end // Do nothing, just wait
                MUL_DONE: begin if (rdId_reg != 0) registers[rdId_reg] <= mul_result; pc <= pc + 4; end
                DIV_WAIT: begin end // Do nothing, just wait
                DIV_DONE: begin if (rdId_reg != 0) registers[rdId_reg] <= (funct3_p[1] ? div_remainder : div_quotient); pc <= pc + 4; end
            endcase
        end
    end

    // --- FSM Combinatorial Logic ---
    localparam INST_TYPE_ILLEGAL=5'd1, INST_TYPE_MEM=5'd2, INST_TYPE_M_EXT=5'd3, INST_TYPE_ECALL=5'd4, INST_TYPE_CSR=5'd5, INST_TYPE_SFENCE=5'd6, INST_TYPE_BRANCH=5'd7, INST_TYPE_JAL=5'd8, INST_TYPE_JALR=5'd9, INST_TYPE_MRET=5'd10, INST_TYPE_SRET=5'd11, INST_TYPE_ALU=5'd12;
    reg [4:0] inst_type;
    always @(*) begin
        inst_type = INST_TYPE_ILLEGAL; // Default
        case (opcode_p)
            7'b0110111: inst_type = INST_TYPE_ALU; // LUI
            7'b0010111: inst_type = INST_TYPE_ALU; // AUIPC
            7'b1101111: inst_type = INST_TYPE_JAL;
            7'b1100111: inst_type = INST_TYPE_JALR;
            7'b1100011: if (branch_taken) inst_type = INST_TYPE_BRANCH; else inst_type = INST_TYPE_ALU; // Treat not-taken branch as NOP (ALU)
            7'b0000011: inst_type = INST_TYPE_MEM; // LOAD
            7'b0100011: inst_type = INST_TYPE_MEM; // STORE
            7'b0010011: inst_type = INST_TYPE_ALU; // ALU-IMM
            7'b0110011: begin
                if (funct7_p == 7'b0000001) inst_type = INST_TYPE_M_EXT;
                else inst_type = INST_TYPE_ALU;
            end
            7'b1110011: begin // SYSTEM
                case(funct12_p)
                    12'h000: inst_type = INST_TYPE_ECALL;
                    12'h102: inst_type = INST_TYPE_SRET;
                    12'h302: inst_type = INST_TYPE_MRET;
                    12'h120: inst_type = INST_TYPE_SFENCE;
                    default: inst_type = INST_TYPE_CSR;
                endcase
            end
            7'b0101111: inst_type = INST_TYPE_MEM; // ATOMIC
            default: inst_type = INST_TYPE_ILLEGAL;
        endcase
    end

    always @(*) begin
        next_state = state;
        case (state)
            FETCH_START: next_state = FETCH_WAIT; FETCH_WAIT: next_state = DECODE_PREP; DECODE_PREP: next_state = EXECUTE_DECODE;
            EXECUTE_DECODE: next_state = EXECUTE_TRANSLATE; EXECUTE_TRANSLATE: next_state = EXECUTE_DISPATCH;
            EXECUTE_DISPATCH:
                if (opcode_p == 7'b0110111) next_state = LUI_EXEC; // LUI Override
                else if (i_tlb_miss_reg) next_state = PTW_START; 
                else if (d_tlb_miss_reg && d_tlb_miss_check_reg) next_state = PTW_START; 
                else case(inst_type_reg)
                    INST_TYPE_ILLEGAL: next_state = EXCEPTION;
                    INST_TYPE_MEM: next_state = MEM_SETUP;
                    INST_TYPE_M_EXT: if (funct3_p == 3'b000) next_state = MUL_START; else next_state = DIV_START;
                    INST_TYPE_ECALL: next_state = ECALL_EXEC;
                    INST_TYPE_CSR: next_state = CSR_READ;
                    INST_TYPE_SFENCE: next_state = SFENCE_EXEC;
                    INST_TYPE_BRANCH: next_state = BRANCH_EXEC;
                    INST_TYPE_JAL: next_state = JAL_EXEC;
                    INST_TYPE_JALR: next_state = JALR_EXEC;
                    INST_TYPE_MRET: next_state = MRET_EXEC;
                    INST_TYPE_SRET: next_state = SRET_EXEC;
                    INST_TYPE_ALU: next_state = ALU_EXEC;
                    default: next_state = EXCEPTION;
                endcase
            LUI_EXEC, ALU_EXEC, BRANCH_EXEC, JAL_EXEC, JALR_EXEC, MRET_EXEC, SRET_EXEC, CSR_WRITE, MUL_DONE, DIV_DONE: next_state = FETCH_START;
            CSR_READ: next_state = CSR_CALC;
            CSR_CALC: next_state = CSR_WRITE;
            ECALL_EXEC, EXCEPTION: next_state = EXCEPTION; // Halt on exception
            SFENCE_EXEC: if (sfence_counter == TLB_ENTRIES - 1) next_state = FETCH_START; else next_state = SFENCE_EXEC;
            MEM_SETUP: next_state = MEM_WAIT;
            MEM_WAIT: if (is_load_reg || (is_store_reg && !is_sc_reg)) next_state = FETCH_START; else if (is_sc_reg || is_amo_reg) next_state = AMO_WRITE; else next_state = FETCH_START;
            AMO_WRITE: next_state = FETCH_START;
            PTW_START: next_state = PTW_WAIT;
            PTW_WAIT: if (ptw_done) if (!ptw_pte_out[0]) {exception_cause_reg, mtval_reg} <= {is_inst_fault_reg ? 12 : (is_store_fault_reg ? 15 : 13), {fault_vpn_reg, 12'b0}}; else {tlb_vpn[tlb_write_index_reg[3:0]], tlb_ppn_perms[tlb_write_index_reg[3:0]], tlb_valid[tlb_write_index_reg[3:0]], tlb_write_index_reg} <= {{fault_vpn_reg, 12'b0}, ptw_pte_out, 1'b1, tlb_write_index_reg + 1};
            MUL_START: next_state = MUL_WAIT; MUL_WAIT: if (!mul_busy) next_state = MUL_DONE;
            DIV_START: next_state = DIV_WAIT; DIV_WAIT: if (!div_busy) next_state = DIV_DONE;
        endcase
    end
    assign debug_out = {priv_mode, state[4:0], 1'b0};
endmodule

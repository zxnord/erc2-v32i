// Procesador modificado para operar con una MMU secuencial (multi-ciclo)
`default_nettype none

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
    output wire [3:0]  debug_state_out // Debug output for FSM state
);

    // --- Banco de Registros ---
    reg [31:0] registers [0:31];

    // --- Registros Secuenciales y FSM ---
    reg [31:0] pc = 32'h00000000;
    reg [31:0] instruction_reg;

    reg [1:0] priv_mode = 2'b11; // M-mode

    localparam FETCH_START          = 4'd0;
    localparam FETCH_WAIT           = 4'd1;
    localparam EXECUTE              = 4'd2;
    localparam MEM_SETUP            = 4'd3; // New bubble state
    localparam MEM_START            = 4'd4;
    localparam MEM_WAIT             = 4'd5;
    localparam MEM_LOAD             = 4'd6;
    localparam MEM_STORE            = 4'd7;
    localparam EXCEPTION            = 4'd8;
    localparam CSR_WRITE            = 4'd9;
    localparam CSR_STALL            = 4'd10;
    localparam PAGE_FAULT_EXCEPTION = 4'd11;
    // MUL/DIV/ATOMIC states can be added here if needed
    reg [3:0] state = FETCH_START;

    reg [4:0] exception_cause_reg;

    // --- CSR Registers ---
    reg [31:0] mstatus_reg, mepc_reg, mcause_reg, mtvec_reg, mtval_reg;
    reg [31:0] sstatus_reg, sepc_reg, scause_reg, stvec_reg, satp_reg, stval_reg;
    reg [31:0] tlb_vpn_reg, tlb_ppn_perms_reg, tlb_write_index_reg;

    // --- Decodificación (basada en la instrucción registrada) ---
    wire [6:0] opcode = instruction_reg[6:0];
    wire [4:0] rdId   = instruction_reg[11:7];
    wire [4:0] rs1Id  = instruction_reg[19:15];
    wire [4:0] rs2Id  = instruction_reg[24:20];
    wire [2:0] funct3 = instruction_reg[14:12];
    wire [6:0] funct7 = instruction_reg[31:25];
    wire [31:0] imm_i = {{21{instruction_reg[31]}}, instruction_reg[30:20]};
    wire [31:0] imm_u = {instruction_reg[31:12], 12'b0};
    wire [31:0] imm_j = {{12{instruction_reg[31]}}, instruction_reg[19:12], instruction_reg[20], instruction_reg[30:21], 1'b0};
    wire [31:0] imm_b = {{20{instruction_reg[31]}}, instruction_reg[7], instruction_reg[30:25], instruction_reg[11:8], 1'b0};
    wire [31:0] imm_s = {{21{instruction_reg[31]}}, instruction_reg[30:25], instruction_reg[11:7]};

    // --- Tipos de Instrucción ---
    wire is_load      = (opcode == 7'b0000011);
    wire is_store     = (opcode == 7'b0100011);
    wire is_alu_imm   = (opcode == 7'b0010011);
    wire is_alu_reg   = (opcode == 7'b0110011) && (funct7 == 7'b0000000 || funct7 == 7'b0100000);
    wire is_lui       = (opcode == 7'b0110111);
    wire is_jal       = (opcode == 7'b1101111);
    wire is_branch    = (opcode == 7'b1100011);
    wire is_auipc     = (opcode == 7'b0010111);
    wire is_jalr      = (opcode == 7'b1100111);
    wire is_system    = (opcode == 7'b1110011);
    wire is_atomic    = 0; // Not supported in this simplified version
    wire is_mul       = 0; // Not supported in this simplified version
    wire is_div_rem   = 0; // Not supported in this simplified version

    wire [11:0] funct12 = instruction_reg[31:20];
    wire is_ecall = is_system && (funct12 == 12'h000) && (funct3 == 3'b000);
    wire is_ebreak = is_system && (funct12 == 12'h001) && (funct3 == 3'b000);
    wire is_sret = is_system && (funct12 == 12'h102) && (funct3 == 3'b000);
    wire is_mret = is_system && (funct12 == 12'h302) && (funct3 == 3'b000);
    wire is_csr = is_system && !is_ecall && !is_ebreak && !is_sret && !is_mret;
    wire is_sfence_vma = is_system && (funct12 == 12'h120) && (funct3 == 3'b000);

    wire is_illegal = !is_load && !is_store && !is_alu_imm && !is_alu_reg && !is_lui && !is_jal && !is_branch && !is_auipc && !is_jalr && !is_system;

    // --- Lectura de Registros ---
    wire [31:0] rs1_data = (rs1Id == 5'b0) ? 32'b0 : registers[rs1Id];
    wire [31:0] rs2_data = (rs2Id == 5'b0) ? 32'b0 : registers[rs2Id];

    // --- Lógica de Salto Condicional ---
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

    // --- ALU y Lógica de Escritura ---
    wire [31:0] alu_op2 = is_alu_reg ? rs2_data : imm_i;
    wire [4:0] shamt = alu_op2[4:0];
    reg [31:0] rd_wdata;
    wire rd_wenable = is_alu_reg || is_alu_imm || is_lui || is_jal || is_load || is_auipc || is_jalr;

    always @(*) begin
        if (is_load) rd_wdata = mem_rdata_in;
        else if (is_lui) rd_wdata = imm_u;
        else if (is_auipc) rd_wdata = pc + imm_u;
        else if (is_jal || is_jalr) rd_wdata = pc + 4;
        else begin
            case (funct3)
                3'b000: rd_wdata = (is_alu_reg && funct7[5]) ? (rs1_data - alu_op2) : (rs1_data + alu_op2);
                3'b001: rd_wdata = rs1_data << shamt;
                3'b010: rd_wdata = ($signed(rs1_data) < $signed(alu_op2)) ? 1 : 0;
                3'b011: rd_wdata = (rs1_data < alu_op2) ? 1 : 0;
                3'b100: rd_wdata = rs1_data ^ alu_op2;
                3'b101: rd_wdata = funct7[5] ? ($signed(rs1_data) >>> shamt) : (rs1_data >> shamt);
                3'b110: rd_wdata = rs1_data | alu_op2;
                3'b111: rd_wdata = rs1_data & alu_op2;
                default: rd_wdata = 32'b0;
            endcase
        end
    end

    // --- MMU Instantiation ---
    reg i_mmu_start, d_mmu_start;
    wire i_mmu_done, d_mmu_done;
    wire i_mmu_hit, d_mmu_hit;
    wire i_mmu_miss, d_mmu_miss;
    wire [31:0] i_mmu_phys_addr, d_mmu_phys_addr;
    wire [31:0] data_virt_addr = rs1_data + (is_load ? imm_i : imm_s);

    mmu i_mmu (
        .clk(clk), .reset(reset), .start_lookup(i_mmu_start), .virt_addr(pc),
        .phys_addr(i_mmu_phys_addr), .lookup_done(i_mmu_done), .tlb_hit(i_mmu_hit), .tlb_miss(i_mmu_miss),
        .satp(satp_reg), .tlb_vpn_in(tlb_vpn_reg), .tlb_ppn_perms_in(tlb_ppn_perms_reg), .tlb_write_index(tlb_write_index_reg)
    );
    mmu d_mmu (
        .clk(clk), .reset(reset), .start_lookup(d_mmu_start), .virt_addr(data_virt_addr),
        .phys_addr(d_mmu_phys_addr), .lookup_done(d_mmu_done), .tlb_hit(d_mmu_hit), .tlb_miss(d_mmu_miss),
        .satp(satp_reg), .tlb_vpn_in(tlb_vpn_reg), .tlb_ppn_perms_in(tlb_ppn_perms_reg), .tlb_write_index(tlb_write_index_reg)
    );

    // --- Salidas a la Memoria/SoC ---
    assign debug_state_out = state;
    assign instruction_address_out = i_mmu_phys_addr;
    assign mem_address_out   = d_mmu_phys_addr;
    assign mem_wdata_out     = rs2_data;
    assign mem_wenable_out   = (state == MEM_STORE);

    // --- CSR Logic ---
    wire [11:0] csr_addr = instruction_reg[31:20];
    reg [31:0] csr_rdata_reg;
    wire [31:0] csr_rdata = (csr_addr == 12'h300) ? mstatus_reg : (csr_addr == 12'h341) ? mepc_reg :
                          (csr_addr == 12'h342) ? mcause_reg : (csr_addr == 12'h305) ? mtvec_reg :
                          (csr_addr == 12'h343) ? mtval_reg : (csr_addr == 12'h180) ? satp_reg :
                          (csr_addr == 12'h7C0) ? tlb_vpn_reg : (csr_addr == 12'h7C1) ? tlb_ppn_perms_reg :
                          (csr_addr == 12'h7C2) ? tlb_write_index_reg : 32'h0;
    wire [31:0] csr_wdata_calculated = (funct3[2] == 1'b0) ? rs1_data :
                                     (funct3[1] == 1'b0) ? (csr_rdata_reg | rs1_data) :
                                     (csr_rdata_reg & ~rs1_data);

    // --- Lógica Secuencial Principal ---
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc <= 32'h0; state <= FETCH_START; instruction_reg <= 32'h13; priv_mode <= 2'b11;
            mstatus_reg <= 32'h0; mepc_reg <= 32'h0; mcause_reg <= 32'h0; mtvec_reg <= 32'h0; mtval_reg <= 32'h0;
            satp_reg <= 32'h0;
        end else begin
            // Defaults
            i_mmu_start <= 1'b0;
            d_mmu_start <= 1'b0;

            case (state)
                FETCH_START: begin
                    i_mmu_start <= 1'b1;
                    state <= FETCH_WAIT;
                end

                FETCH_WAIT: begin
                    if (i_mmu_done) begin
                        if (i_mmu_miss) begin
                            exception_cause_reg <= 12; // Instruction page fault
                            mtval_reg <= pc;
                            state <= PAGE_FAULT_EXCEPTION;
                        end else begin
                            instruction_reg <= instruction_in;
                            state <= EXECUTE;
                        end
                    end
                end

                EXECUTE: begin
                    if (is_illegal) begin
                        exception_cause_reg <= 2; mtval_reg <= instruction_reg; state <= EXCEPTION;
                    end else if (is_ecall) begin
                        exception_cause_reg <= 8 + priv_mode; mtval_reg <= pc; state <= EXCEPTION;
                    end else if (is_load) begin
                        state <= MEM_SETUP;
                    end else if (is_store) begin
                        state <= MEM_SETUP;
                    end else if (is_branch && branch_taken) begin
                        pc <= pc + imm_b; state <= FETCH_START;
                    end else if (is_jal) begin
                        if (rdId != 0) registers[rdId] <= pc + 4;
                        pc <= pc + imm_j; state <= FETCH_START;
                    end else if (is_jalr) begin
                        if (rdId != 0) registers[rdId] <= pc + 4;
                        pc <= (rs1_data + imm_i) & 32'hFFFFFFFE; state <= FETCH_START;
                    end else if (is_csr) begin
                        csr_rdata_reg <= csr_rdata; state <= CSR_WRITE;
                    end else if (is_mret) begin
                        pc <= mepc_reg; priv_mode <= mstatus_reg[12:11]; state <= FETCH_START;
                    end else if (is_sret) begin
                        pc <= sepc_reg; priv_mode <= sstatus_reg[9:8]; state <= FETCH_START;
                    end else if (is_sfence_vma) begin
                        // In this simple model, sfence.vma is a NOP as TLB is flushed implicitly
                        pc <= pc + 4; state <= FETCH_START;
                    end else begin // ALU, LUI, AUIPC
                        if (rd_wenable && rdId != 0) registers[rdId] <= rd_wdata;
                        pc <= pc + 4;
                        state <= FETCH_START;
                    end
                end

                MEM_SETUP: begin
                    state <= MEM_START;
                end

                MEM_START: begin
                    d_mmu_start <= 1'b1;
                    state <= MEM_WAIT;
                end

                MEM_WAIT: begin
                    if (d_mmu_done) begin
                        if (d_mmu_miss) begin
                            exception_cause_reg <= is_load ? 13 : 15; // Load or Store page fault
                            mtval_reg <= data_virt_addr;
                            state <= PAGE_FAULT_EXCEPTION;
                        end else begin
                            state <= is_load ? MEM_LOAD : MEM_STORE;
                        end
                    end
                end

                MEM_LOAD: begin
                    if (rdId != 0) registers[rdId] <= mem_rdata_in;
                    pc <= pc + 4;
                    state <= FETCH_START;
                end

                MEM_STORE: begin
                    pc <= pc + 4;
                    state <= FETCH_START;
                end

                CSR_WRITE: begin
                    if (rdId != 0) registers[rdId] <= csr_rdata_reg;
                    if (funct3 != 3'b000) begin // Don't write on read-only
                        case(csr_addr)
                            12'h300: mstatus_reg <= csr_wdata_calculated;
                            12'h341: mepc_reg <= csr_wdata_calculated;
                            12'h342: mcause_reg <= csr_wdata_calculated;
                            12'h305: mtvec_reg <= csr_wdata_calculated;
                            12'h343: mtval_reg <= csr_wdata_calculated;
                            12'h180: satp_reg <= csr_wdata_calculated;
                            12'h7C0: tlb_vpn_reg <= csr_wdata_calculated;
                            12'h7C1: tlb_ppn_perms_reg <= csr_wdata_calculated;
                            12'h7C2: tlb_write_index_reg <= csr_wdata_calculated;
                        endcase
                    end
                    state <= CSR_STALL;
                end
                
                CSR_STALL: begin
                    pc <= pc + 4;
                    state <= FETCH_START;
                end

                PAGE_FAULT_EXCEPTION: begin
                    mepc_reg <= pc;
                    mcause_reg <= {27'b0, exception_cause_reg};
                    mstatus_reg[12:11] <= priv_mode;
                    priv_mode <= 2'b11;
                    pc <= mtvec_reg;
                    state <= FETCH_START;
                end

                EXCEPTION: begin
                    mepc_reg <= pc;
                    mcause_reg <= {27'b0, exception_cause_reg};
                    mstatus_reg[12:11] <= priv_mode;
                    priv_mode <= 2'b11;
                    pc <= mtvec_reg;
                    state <= FETCH_START;
                end
            endcase
        end
    end

    // Inicialización para simulación
    integer i;
    initial begin
        for (i=0; i<32; i=i+1) registers[i] = 0;
    end

endmodule
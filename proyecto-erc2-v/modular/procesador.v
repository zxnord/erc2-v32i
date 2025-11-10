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
    output wire        mem_wenable_out
);

    // --- Banco de Registros ---
    reg [31:0] registers [0:31];

    // --- Registros Secuenciales y FSM ---
    reg [31:0] pc = 32'h00000000;
    reg [31:0] instruction_reg;

    reg [1:0] priv_mode; // 00: U-mode, 01: S-mode, 11: M-mode

    localparam FETCH     = 3'b000;
    localparam EXECUTE   = 3'b001;
    localparam MUL_WAIT  = 3'b010;
    localparam DIV_WAIT  = 3'b011;
    localparam ATOMIC    = 3'b100;
    localparam EXCEPTION = 3'b101;
    localparam CSR_WRITE = 3'b110;
    localparam CSR_STALL = 3'b111;
    reg [2:0] state = FETCH;
    reg [2:0] funct3_reg;

    // --- Reservation Station para LR/SC ---
    reg [31:0] reservation_addr;
    reg reservation_valid = 1'b0;

    // --- Atomic Operation Registers ---
    reg [31:0] atomic_mem_data; // Stores memory value read in ATOMIC state
    reg [31:0] atomic_addr;     // Stores address for atomic operation
    reg sc_write_enable;        // Indicates SC.W should write

    reg [31:0] csr_rdata_reg;
    reg [4:0] exception_cause_reg;
    reg exception_taken_reg;

    // --- CSR Registers ---
    reg [31:0] sstatus_reg;
    reg [31:0] sepc_reg;
    reg [31:0] scause_reg;
    reg [31:0] stvec_reg;
    reg [31:0] satp_reg;
    reg [31:0] stval_reg;

    // --- M-mode CSR Registers ---
    reg [31:0] mstatus_reg;
    reg [31:0] mepc_reg;
    reg [31:0] mcause_reg;
    reg [31:0] mtvec_reg;
    reg [31:0] mtval_reg;

    // --- Custom CSRs for TLB Management ---
    reg [31:0] tlb_vpn_reg;
    reg [31:0] tlb_ppn_perms_reg;
    reg [31:0] tlb_write_index_reg;

    // Connect CSR regs to the wires feeding the MMUs
    assign tlb_vpn_in = tlb_vpn_reg;
    assign tlb_ppn_perms = tlb_ppn_perms_reg;
    assign tlb_write_index = tlb_write_index_reg;

    wire csr_we;

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

    // --- Tipos de Instrucción (CORREGIDO) ---
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
    wire is_mul       = (opcode == 7'b0110011 && funct3 == 3'b000 && funct7 == 7'b0000001);
    wire is_div_rem   = (opcode == 7'b0110011 && funct7 == 7'b0000001 && funct3[2]);

    wire is_system = (opcode == 7'b1110011);
    wire [11:0] funct12 = instruction_reg[31:20];
    wire is_ecall = is_system && (funct12 == 12'h000) && (funct3 == 3'b000);
    wire is_ebreak = is_system && (funct12 == 12'h001) && (funct3 == 3'b000);
    wire is_sret = is_system && (funct12 == 12'h102) && (funct3 == 3'b000);
    wire is_mret = is_system && (funct12 == 12'h302) && (funct3 == 3'b000);
    wire is_csr = is_system && !is_ecall && !is_ebreak && !is_sret && !is_mret;

    wire is_illegal = !is_load && !is_store && !is_alu_imm && !is_alu_reg && !is_lui && !is_jal && !is_branch && !is_auipc && !is_jalr && !is_system && !is_atomic;

    // --- CSR Logic ---
    wire [11:0] csr_addr = instruction_reg[31:20];
    wire [31:0] csr_rdata = // S-mode CSRs
                          (csr_addr == 12'h100) ? sstatus_reg :
                          (csr_addr == 12'h141) ? sepc_reg :
                          (csr_addr == 12'h142) ? scause_reg :
                          (csr_addr == 12'h105) ? stvec_reg :
                          (csr_addr == 12'h180) ? satp_reg :
                          (csr_addr == 12'h143) ? stval_reg :
                          // M-mode CSRs
                          (csr_addr == 12'h300) ? mstatus_reg :
                          (csr_addr == 12'h341) ? mepc_reg :
                          (csr_addr == 12'h342) ? mcause_reg :
                          (csr_addr == 12'h305) ? mtvec_reg :
                          (csr_addr == 12'h343) ? mtval_reg :
                          // Custom TLB CSRs
                          (csr_addr == 12'h7C0) ? tlb_vpn_reg :
                          (csr_addr == 12'h7C1) ? tlb_ppn_perms_reg :
                          (csr_addr == 12'h7C2) ? tlb_write_index_reg :
                          32'h0;

    wire [31:0] csr_wdata_calculated = (funct3 == 3'b001) ? rs1_data :
                                     (funct3 == 3'b010) ? (csr_rdata_reg | rs1_data) :
                                     (funct3 == 3'b011) ? (csr_rdata_reg & ~rs1_data) :
                                     (funct3 == 3'b101) ? {27'b0, rs1Id} :
                                     (funct3 == 3'b110) ? (csr_rdata_reg | {27'b0, rs1Id}) :
                                     (funct3 == 3'b111) ? (csr_rdata_reg & ~{27'b0, rs1Id}) :
                                     32'h0;

    // --- A Extension (Atomic) Instructions ---
    wire is_atomic    = (opcode == 7'b0101111) && (funct3 == 3'b010); // AMO.W
    wire is_lr        = is_atomic && (funct7[6:2] == 5'b00010);
    wire is_sc        = is_atomic && (funct7[6:2] == 5'b00011);
    wire is_amoswap   = is_atomic && (funct7[6:2] == 5'b00001);
    wire is_amoadd    = is_atomic && (funct7[6:2] == 5'b00000);
    wire is_amoxor    = is_atomic && (funct7[6:2] == 5'b00100);
    wire is_amoand    = is_atomic && (funct7[6:2] == 5'b01100);
    wire is_amoor     = is_atomic && (funct7[6:2] == 5'b01000);
    wire is_amomin    = is_atomic && (funct7[6:2] == 5'b10000);
    wire is_amomax    = is_atomic && (funct7[6:2] == 5'b10100);
    wire is_amominu   = is_atomic && (funct7[6:2] == 5'b11000);
    wire is_amomaxu   = is_atomic && (funct7[6:2] == 5'b11100);

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

    // --- AMO Operation Computation ---
    reg [31:0] amo_result;
    always @(*) begin
        if (is_amoswap) amo_result = rs2_data;
        else if (is_amoadd) amo_result = atomic_mem_data + rs2_data;
        else if (is_amoxor) amo_result = atomic_mem_data ^ rs2_data;
        else if (is_amoand) amo_result = atomic_mem_data & rs2_data;
        else if (is_amoor) amo_result = atomic_mem_data | rs2_data;
        else if (is_amomin) amo_result = ($signed(atomic_mem_data) < $signed(rs2_data)) ? atomic_mem_data : rs2_data;
        else if (is_amomax) amo_result = ($signed(atomic_mem_data) > $signed(rs2_data)) ? atomic_mem_data : rs2_data;
        else if (is_amominu) amo_result = (atomic_mem_data < rs2_data) ? atomic_mem_data : rs2_data;
        else if (is_amomaxu) amo_result = (atomic_mem_data > rs2_data) ? atomic_mem_data : rs2_data;
        else amo_result = atomic_mem_data;
    end

    // --- Instancias de Multiplicador y Divisor ---
    wire mul_busy, div_busy;
    wire [31:0] mul_result, div_quotient, div_remainder;
    multiplier mul_unit ( .clk(clk), .reset(reset), .start(state == EXECUTE && is_mul), .rs1_data(rs1_data), .rs2_data(rs2_data), .result(mul_result), .busy(mul_busy) );
    divider div_unit ( .clk(clk), .reset(reset), .start(state == EXECUTE && is_div_rem), .dividend(rs1_data), .divisor(rs2_data), .quotient_out(div_quotient), .remainder_out(div_remainder), .busy(div_busy) );

    // --- MMU ---
    wire mmu_enable = ((satp_reg >> 31) & 1) && (priv_mode != 2'b11); // MMU enabled if satp.MODE is not bare AND we are in S-mode or U-mode

    wire [31:0] data_virt_addr = (state == ATOMIC) ? atomic_addr :
                                 (is_atomic ? rs1_data :
                                 (rs1_data + (is_load ? imm_i : imm_s)));

    wire [31:0] instr_phys_addr;
    wire [31:0] data_phys_addr;
    wire        instr_tlb_hit;
    wire        data_tlb_hit;
    wire        instr_tlb_miss;
    wire        data_tlb_miss;

    // TLB write ports - will be connected to CSRs later
    wire [31:0] tlb_vpn_in;
    wire [31:0] tlb_ppn_perms;
    wire [31:0] tlb_write_index;

    mmu i_mmu (
        .clk(clk),
        .reset(reset),
        .virt_addr(pc),
        .phys_addr(instr_phys_addr),
        .satp(satp_reg),
        .mmu_enable(mmu_enable),
        .tlb_hit(instr_tlb_hit),
        .tlb_miss(instr_tlb_miss),
        .tlb_vpn_in(tlb_vpn_in),
        .tlb_ppn_perms(tlb_ppn_perms),
        .tlb_write_index(tlb_write_index)
    );

    mmu d_mmu (
        .clk(clk),
        .reset(reset),
        .virt_addr(data_virt_addr),
        .phys_addr(data_phys_addr),
        .satp(satp_reg),
        .mmu_enable(mmu_enable),
        .tlb_hit(data_tlb_hit),
        .tlb_miss(data_tlb_miss),
        .tlb_vpn_in(tlb_vpn_in),
        .tlb_ppn_perms(tlb_ppn_perms),
        .tlb_write_index(tlb_write_index)
    );


    // --- Salidas a la Memoria/SoC ---
    assign instruction_address_out = instr_phys_addr;
    assign mem_address_out   = data_phys_addr;
    assign mem_wdata_out     = (state == ATOMIC) ? amo_result : rs2_data;
    assign mem_wenable_out   = ((is_store && (funct3 == 3'b010)) || (state == ATOMIC) || sc_write_enable); // SW, atomic write, or SC.W

    assign csr_we = (state == CSR_WRITE);

    // --- Lógica Secuencial Principal ---
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc <= 32'h00000000;
            state <= FETCH;
            instruction_reg <= 32'h00000013; // NOP
            sc_write_enable <= 1'b0;
            priv_mode <= 2'b11; // M-mode
            reservation_valid <= 1'b0;
            sstatus_reg <= 32'h0;
            sepc_reg <= 32'h0;
            scause_reg <= 32'h0;
            stvec_reg <= 32'h0;
            satp_reg <= 32'h0;
            stval_reg <= 32'h0;
            // M-mode CSRs
            mstatus_reg <= 32'h0;
            mepc_reg <= 32'h0;
            mcause_reg <= 32'h0;
            mtvec_reg <= 32'h0;
            mtval_reg <= 32'h0;
        end else begin
            // Invalidate reservation if a store occurs to the reserved address
            if (mem_wenable_out && !sc_write_enable && reservation_valid && (mem_address_out == reservation_addr)) begin
                reservation_valid <= 1'b0;
            end

            sc_write_enable <= 1'b0; // Default: no SC write
            case (state)
                FETCH: begin
                    instruction_reg <= instruction_in;
                    state <= EXECUTE;
                end
                EXECUTE: begin
                    if (mmu_enable && instr_tlb_miss) begin
                        exception_cause_reg <= 12; // Instruction page fault
                        exception_taken_reg <= 1'b1;
                        state <= EXCEPTION;
                    end else if (mmu_enable && is_load && data_tlb_miss) begin
                        exception_cause_reg <= 13; // Load page fault
                        exception_taken_reg <= 1'b1;
                        state <= EXCEPTION;
                    end else if (mmu_enable && is_store && data_tlb_miss) begin
                        exception_cause_reg <= 15; // Store page fault
                        exception_taken_reg <= 1'b1;
                        state <= EXCEPTION;
                    end else if (is_illegal) begin
                        exception_cause_reg <= 2;
                        exception_taken_reg <= 1'b1;
                        state <= EXCEPTION;
                    end else if (is_mul) state <= MUL_WAIT;
                    else if (is_div_rem) begin
                        funct3_reg <= funct3;
                        state <= DIV_WAIT;
                    end
                    else if (is_lr) begin
                        // LR.W: Load reserved - read memory and set reservation
                        if (rdId != 0) registers[rdId] <= mem_rdata_in;
                        reservation_addr <= rs1_data;
                        reservation_valid <= 1'b1;
                        pc <= pc + 4;
                        state <= FETCH;
                    end
                    else if (is_sc) begin
                        // SC.W: Store conditional - succeed if reservation matches
                        if (reservation_valid && (reservation_addr == rs1_data)) begin
                            // Success: write to memory, return 0 in rd
                            if (rdId != 0) registers[rdId] <= 32'h00000000;
                            sc_write_enable <= 1'b1; // Enable write for this cycle
                            reservation_valid <= 1'b0;
                        end else begin
                            // Failure: don't write, return 1 in rd
                            if (rdId != 0) registers[rdId] <= 32'h00000001;
                        end
                        pc <= pc + 4;
                        state <= FETCH;
                    end
                    else if (is_atomic && !is_lr && !is_sc) begin
                        // AMO operations: enter ATOMIC state for read-modify-write
                        atomic_addr <= rs1_data;
                        atomic_mem_data <= mem_rdata_in; // Read current memory value
                        state <= ATOMIC;
                    end
                    else begin
                        if (rd_wenable && rdId != 0) registers[rdId] <= rd_wdata;
                        if (is_branch && branch_taken) pc <= pc + imm_b;
                        else if (is_jal) pc <= pc + imm_j;
                        else if (is_jalr) pc <= (rs1_data + imm_i) & 32'hFFFFFFFE;
                        else if (is_system) begin
                            if (is_csr) begin
                                csr_rdata_reg <= csr_rdata;
                                state <= CSR_WRITE;
                            end else if (is_ecall) begin
                                exception_cause_reg <= 8;
                                exception_taken_reg <= 1'b1;
                                state <= EXCEPTION;
                            end else if (is_ebreak) begin
                                exception_cause_reg <= 3;
                                exception_taken_reg <= 1'b1;
                                state <= EXCEPTION;
                            end else if (is_mret) begin
                                // Return from M-mode trap
                                pc <= mepc_reg;
                                priv_mode <= mstatus_reg[12:11]; // Restore privilege from mstatus.MPP
                                state <= FETCH;
                            end else if (is_sret) begin
                                pc <= sepc_reg;
                                priv_mode <= sstatus_reg[9:8]; // sstatus.SPP
                                state <= FETCH;
                            end else begin
                                exception_cause_reg <= 2;
                                exception_taken_reg <= 1'b1;
                                state <= EXCEPTION;
                            end
                        end else begin
                            pc <= pc + 4;
                            state <= FETCH;
                        end
                    end
                end
                MUL_WAIT: begin
                    if (!mul_busy) begin
                        if (rdId != 0) registers[rdId] <= mul_result;
                        pc <= pc + 4;
                        state <= FETCH;
                    end
                end
                DIV_WAIT: begin
                    if (!div_busy) begin
                        if (rdId != 0) begin
                            if (funct3_reg[1]) registers[rdId] <= div_remainder; // REM, REMU
                            else registers[rdId] <= div_quotient; // DIV, DIVU
                        end
                        pc <= pc + 4;
                        state <= FETCH;
                    end
                end
                ATOMIC: begin
                    // AMO: Write computed value back, return original value to rd
                    if (rdId != 0) registers[rdId] <= atomic_mem_data;
                    // Clear reservation on any atomic operation to maintain SC semantics
                    reservation_valid <= 1'b0;
                    pc <= pc + 4;
                    state <= FETCH;
                end
                EXCEPTION: begin
                    // Trap to M-mode
                    mepc_reg <= pc;
                    mcause_reg <= {1'b0, 27'b0, exception_cause_reg}; // For now, no interrupts
                    mtval_reg <= pc; // For now, store faulting PC. A real implementation might store faulting address.

                    // Save current privilege in mstatus.MPP (bits 12:11)
                    // and prepare for M-mode
                    mstatus_reg[12:11] <= priv_mode;

                    priv_mode <= 2'b11; // Enter M-mode
                    pc <= mtvec_reg;    // Jump to M-mode trap handler
                    state <= FETCH;
                end
                CSR_WRITE: begin
                    if (rdId != 0) registers[rdId] <= csr_rdata_reg;
                    if (!((funct3 == 3'b010 || funct3 == 3'b011) && rs1Id == 5'b0)) begin
                        case (csr_addr)
                            // S-mode
                            12'h100: sstatus_reg <= csr_wdata_calculated;
                            12'h141: sepc_reg    <= csr_wdata_calculated;
                            12'h142: scause_reg  <= csr_wdata_calculated;
                            12'h105: stvec_reg   <= csr_wdata_calculated;
                            12'h180: satp_reg    <= csr_wdata_calculated;
                            12'h143: stval_reg   <= csr_wdata_calculated;
                            // M-mode
                            12'h300: mstatus_reg <= csr_wdata_calculated;
                            12'h341: mepc_reg    <= csr_wdata_calculated;
                            12'h342: mcause_reg  <= csr_wdata_calculated;
                            12'h305: mtvec_reg   <= csr_wdata_calculated;
                            12'h343: mtval_reg   <= csr_wdata_calculated;
                            // Custom TLB CSRs
                            12'h7C0: tlb_vpn_reg <= csr_wdata_calculated;
                            12'h7C1: tlb_ppn_perms_reg <= csr_wdata_calculated;
                            12'h7C2: tlb_write_index_reg <= csr_wdata_calculated;
                        endcase
                    end
                    state <= CSR_STALL;
                end
                CSR_STALL: begin
                    pc <= pc + 4;
                    state <= FETCH;
                end
                default: begin
                    state <= FETCH;
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
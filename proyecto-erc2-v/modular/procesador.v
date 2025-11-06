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

    localparam FETCH     = 3'b000;
    localparam EXECUTE   = 3'b001;
    localparam MUL_WAIT  = 3'b010;
    localparam DIV_WAIT  = 3'b011;
    localparam ATOMIC    = 3'b100;
    reg [2:0] state = FETCH;
    reg [2:0] funct3_reg;

    // --- Reservation Station para LR/SC ---
    reg [31:0] reservation_addr;
    reg reservation_valid = 1'b0;

    // --- Atomic Operation Registers ---
    reg [31:0] atomic_mem_data; // Stores memory value read in ATOMIC state
    reg [31:0] atomic_addr;     // Stores address for atomic operation
    reg sc_write_enable;        // Indicates SC.W should write

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

    // --- Salidas a la Memoria/SoC ---
    assign instruction_address_out = pc;
    assign mem_address_out   = (state == ATOMIC) ? atomic_addr : 
                                (is_atomic ? rs1_data : 
                                (rs1_data + (is_load ? imm_i : imm_s)));
    assign mem_wdata_out     = (state == ATOMIC) ? amo_result : rs2_data;
    assign mem_wenable_out   = ((is_store && (funct3 == 3'b010)) || (state == ATOMIC) || sc_write_enable); // SW, atomic write, or SC.W

    // --- Lógica Secuencial Principal ---
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc <= 32'h00000000;
            state <= FETCH;
            instruction_reg <= 32'h00000013; // NOP
            sc_write_enable <= 1'b0;
            reservation_valid <= 1'b0;
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
                    if (is_mul) state <= MUL_WAIT;
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
                        else if (is_system) pc <= pc; // Halt
                        else pc <= pc + 4;
                        state <= FETCH;
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
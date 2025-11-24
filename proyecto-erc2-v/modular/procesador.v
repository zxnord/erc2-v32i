`default_nettype none

// procesador.v - Version 2.1
// Arquitectura profundamente segmentada con logica LR/SC corregida.

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
    localparam EXECUTE_TRANSLATE  = 5'd4; // Placeholder para la MMU
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
    
    reg [4:0] state = FETCH_START;
    reg [4:0] next_state;

    // Registros de Pipeline
    reg [6:0] opcode_p;
    reg [2:0] funct3_p;
    reg [6:0] funct7_p;
    reg [4:0] inst_type_reg;

    // Registros para operaciones de memoria y atomicas
    reg [31:0] amo_read_data_reg;
    reg [31:0] amo_addr_reg;
    reg [31:0] amo_rs2_data_reg;
    reg is_load_reg, is_store_reg, is_atomic_reg, is_lr_reg, is_sc_reg;
    reg [4:0] rdId_reg;
    reg sc_success_reg;

    // Reservation Station para LR/SC
    reg [31:0] reservation_addr;
    reg reservation_valid = 1'b0;

    // --- Decodificacion ---
    // Etapa 0: Campos brutos desde instruction_reg (usados en DECODE_PREP)
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

    // Etapa 1: Banderas de decodificacion desde campos segmentados (opcode_p, etc.)
    wire is_load_p      = (opcode_p == 7'b0000011);
    wire is_store_p     = (opcode_p == 7'b0100011);
    wire is_alu_imm_p   = (opcode_p == 7'b0010011);
    wire is_alu_reg_p   = (opcode_p == 7'b0110011) && (funct7_p == 7'b0000000 || funct7_p == 7'b0100000);
    wire is_lui_p       = (opcode_p == 7'b0110111);
    wire is_jal_p       = (opcode_p == 7'b1101111);
    wire is_branch_p    = (opcode_p == 7'b1100011);
    wire is_auipc_p     = (opcode_p == 7'b0010111);
    wire is_jalr_p      = (opcode_p == 7'b1100111);
    wire is_m_extension_p = (opcode_p == 7'b0110011) && (funct7_p == 7'b0000001);
    wire is_mul_p       = is_m_extension_p && (funct3_p == 3'b000);
    wire is_div_rem_p   = is_m_extension_p && funct3_p[2];
    wire is_atomic_op_p = (opcode_p == 7'b0101111) && (funct3_p == 3'b010);
    wire is_lr_p        = is_atomic_op_p && (funct7_p[6:2] == 5'b00010);
    wire is_sc_p        = is_atomic_op_p && (funct7_p[6:2] == 5'b00011);
    wire is_amo_p       = is_atomic_op_p && !is_lr_p && !is_sc_p;
    wire is_illegal_p   = !(is_load_p || is_store_p || is_alu_imm_p || is_alu_reg_p || is_lui_p || is_jal_p || is_branch_p || is_auipc_p || is_jalr_p || is_m_extension_p || is_atomic_op_p);

    // --- Lectura de Registros y Logica de Salto ---
    wire [31:0] rs1_data = (rs1Id == 5'b0) ? 32'b0 : registers[rs1Id];
    wire [31:0] rs2_data = (rs2Id == 5'b0) ? 32'b0 : registers[rs2Id];
    
    reg branch_taken;
    always @(*) begin
        case (funct3) // Usa funct3 directo, se calcula antes de la decision de salto
            3'b000: branch_taken = (rs1_data == rs2_data); // BEQ
            3'b001: branch_taken = (rs1_data != rs2_data); // BNE
            3'b100: branch_taken = ($signed(rs1_data) < $signed(rs2_data)); // BLT
            3'b101: branch_taken = ($signed(rs1_data) >= $signed(rs2_data)); // BGE
            3'b110: branch_taken = (rs1_data < rs2_data);     // BLTU
            3'b111: branch_taken = (rs1_data >= rs2_data);    // BGEU
            default: branch_taken = 1'b0;
        endcase
    end

    // --- ALU y Logica de Escritura ---
    wire [31:0] alu_op2 = is_alu_reg_p ? rs2_data : imm_i;
    wire [4:0] shamt = alu_op2[4:0];
    reg [31:0] rd_wdata;
    wire rd_wenable = is_alu_reg_p || is_alu_imm_p || is_lui_p || is_jal_p || is_load_p || is_auipc_p || is_jalr_p || is_lr_p || is_sc_p || is_amo_p;

    // --- Computo de Operacion AMO ---
    reg [31:0] amo_result;
    always @(*) begin
        case (funct7_p[6:2]) 
            5'b00001: amo_result = rs2_data; // AMOSWAP
            5'b00000: amo_result = amo_read_data_reg + rs2_data; // AMOADD
            5'b00100: amo_result = amo_read_data_reg ^ rs2_data; // AMOXOR
            5'b01100: amo_result = amo_read_data_reg & rs2_data; // AMOAND
            5'b01000: amo_result = amo_read_data_reg | rs2_data; // AMOOR
            5'b10000: amo_result = ($signed(amo_read_data_reg) < $signed(rs2_data)) ? amo_read_data_reg : rs2_data; // AMOMIN
            5'b10100: amo_result = ($signed(amo_read_data_reg) > $signed(rs2_data)) ? amo_read_data_reg : rs2_data; // AMOMAX
            5'b11000: amo_result = (amo_read_data_reg < rs2_data) ? amo_read_data_reg : rs2_data; // AMOMINU
            5'b11100: amo_result = (amo_read_data_reg > rs2_data) ? amo_read_data_reg : rs2_data; // AMOMAXU
            default:  amo_result = amo_read_data_reg;
        endcase
    end

    always @(*) begin
        if (is_load_p) rd_wdata = mem_rdata_in;
        else if (is_lr_p) rd_wdata = mem_rdata_in;
        else if (is_sc_p) rd_wdata = sc_success_reg ? 32'h0 : 32'h1;
        else if (is_amo_p) rd_wdata = amo_read_data_reg;
        else if (is_lui_p) rd_wdata = imm_u;
        else if (is_auipc_p) rd_wdata = pc + imm_u;
        else if (is_jal_p || is_jalr_p) rd_wdata = pc + 4;
        else begin // ALU y M-extension
            case (funct3_p)
                3'b000: rd_wdata = is_m_extension_p ? mul_result : ((is_alu_reg_p && funct7_p[5]) ? (rs1_data - alu_op2) : (rs1_data + alu_op2));
                3'b001: rd_wdata = rs1_data << shamt;
                3'b010: rd_wdata = ($signed(rs1_data) < $signed(alu_op2)) ? 1 : 0;
                3'b011: rd_wdata = (rs1_data < alu_op2) ? 1 : 0;
                3'b100: rd_wdata = is_m_extension_p ? div_quotient : (rs1_data ^ alu_op2);
                3'b101: rd_wdata = is_m_extension_p ? div_quotient : (funct7_p[5] ? ($signed(rs1_data) >>> shamt) : (rs1_data >> shamt));
                3'b110: rd_wdata = is_m_extension_p ? div_remainder : (rs1_data | alu_op2);
                3'b111: rd_wdata = is_m_extension_p ? div_remainder : (rs1_data & alu_op2);
                default: rd_wdata = 32'b0;
            endcase
        end
    end

    // --- Instancias de Multiplicador y Divisor ---
    wire mul_busy, div_busy;
    wire [31:0] mul_result, div_quotient, div_remainder;
    multiplier mul_unit ( .clk(clk), .reset(reset), .start(state == MUL_START), .rs1_data(rs1_data), .rs2_data(rs2_data), .result(mul_result), .busy(mul_busy) );
    divider div_unit ( .clk(clk), .reset(reset), .start(state == DIV_START), .dividend(rs1_data), .divisor(rs2_data), .quotient_out(div_quotient), .remainder_out(div_remainder), .busy(div_busy) );

    // --- Salidas a la Memoria/SoC ---
    assign instruction_address_out = pc;
    assign mem_address_out = amo_addr_reg; // Unificado para todas las operaciones de memoria
    assign mem_wdata_out   = is_store_reg ? amo_rs2_data_reg : amo_result;
    assign mem_wenable_out = (is_store_reg && state == MEM_WAIT) || (state == AMO_WRITE && (!is_sc_reg || sc_success_reg));

    // --- Lógica Secuencial Principal ---
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc <= 32'h0;
            state <= FETCH_START;
            instruction_reg <= 32'h13; // NOP
            priv_mode <= 2'b11;
            reservation_valid <= 1'b0;
        end else begin
            state <= next_state;

            // Invalidate reservation on external write
            if (mem_wenable_out && !is_sc_reg && reservation_valid && (mem_address_out == reservation_addr)) begin
                reservation_valid <= 1'b0;
            end

            case (state)
                FETCH_WAIT: instruction_reg <= instruction_in;
                
                DECODE_PREP: begin
                    opcode_p <= opcode;
                    funct3_p <= funct3;
                    funct7_p <= funct7;
                end

                EXECUTE_DECODE: begin
                    inst_type_reg <= inst_type;
                end

                EXECUTE_TRANSLATE: begin
                    amo_addr_reg <= is_atomic_op_p ? rs1_data : (rs1_data + (is_load_p ? imm_i : imm_s));
                    amo_rs2_data_reg <= rs2_data;
                    rdId_reg <= rdId;
                    is_load_reg <= is_load_p;
                    is_store_reg <= is_store_p;
                    is_atomic_reg <= is_atomic_op_p;
                    is_lr_reg <= is_lr_p;
                    is_sc_reg <= is_sc_p;
                end

                EXECUTE_DISPATCH: begin
                    if (is_sc_reg) begin
                        sc_success_reg <= reservation_valid && (reservation_addr == amo_addr_reg);
                    end
                end

                ALU_EXEC: begin if (rd_wenable && rdId != 0) registers[rdId] <= rd_wdata; pc <= pc + 4; end
                BRANCH_EXEC: pc <= pc + imm_b;
                JAL_EXEC: begin if (rdId != 0) registers[rdId] <= pc + 4; pc <= pc + imm_j; end
                JALR_EXEC: begin if (rdId != 0) registers[rdId] <= pc + 4; pc <= (rs1_data + imm_i) & 32'hFFFFFFFE; end
                
                MEM_WAIT: begin
                    if (is_load_reg) begin
                        if (rdId_reg != 0) registers[rdId_reg] <= mem_rdata_in;
                        if (is_lr_reg) begin
                            reservation_addr <= amo_addr_reg;
                            reservation_valid <= 1'b1;
                        end
                        pc <= pc + 4;
                    end else if (is_store_reg) begin
                        if (is_sc_reg && sc_success_reg) begin
                            reservation_valid <= 1'b0; // Consume reservation on successful SC
                        end
                        pc <= pc + 4;
                    end else if (is_amo_p) begin // AMO, but not LR/SC
                        amo_read_data_reg <= mem_rdata_in;
                    end
                end

                AMO_WRITE: begin
                    // For AMO, write original value to rd. For SC, write 0/1.
                    if (is_sc_reg) begin
                        if (rdId_reg != 0) registers[rdId_reg] <= sc_success_reg ? 32'h0 : 32'h1;
                    end else begin // AMO
                        if (rdId_reg != 0) registers[rdId_reg] <= amo_read_data_reg;
                    end
                    pc <= pc + 4;
                end
        
                EXCEPTION: begin
                    pc <= pc; // Halt on exception
                end
        
                MUL_WAIT: begin if (!mul_busy) begin if (rdId != 0) registers[rdId] <= mul_result; pc <= pc + 4; end end
                DIV_WAIT: begin if (!div_busy) begin if (rdId != 0) registers[rdId] <= (funct3_p[1] ? div_remainder : div_quotient); pc <= pc + 4; end end
            endcase
        end
    end

    // --- FSM Combinatorial Logic ---
    localparam INST_TYPE_ILLEGAL = 5'd1;
    localparam INST_TYPE_MEM     = 5'd2;
    localparam INST_TYPE_M_EXT   = 5'd3;
    localparam INST_TYPE_BRANCH  = 5'd7;
    localparam INST_TYPE_JAL     = 5'd8;
    localparam INST_TYPE_JALR    = 5'd9;
    localparam INST_TYPE_ALU     = 5'd12;

    reg [4:0] inst_type;
    always @(*) begin
        if (is_illegal_p) inst_type = INST_TYPE_ILLEGAL;
        else if (is_load_p || is_store_p || is_atomic_op_p) inst_type = INST_TYPE_MEM;
        else if (is_m_extension_p) inst_type = INST_TYPE_M_EXT;
        else if (is_branch_p && branch_taken) inst_type = INST_TYPE_BRANCH;
        else if (is_jal_p) inst_type = INST_TYPE_JAL;
        else if (is_jalr_p) inst_type = INST_TYPE_JALR;
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
                case(inst_type_reg)
                    INST_TYPE_ILLEGAL: next_state = EXCEPTION;
                    INST_TYPE_MEM:     next_state = MEM_SETUP;
                    INST_TYPE_M_EXT:   if (is_mul_p) next_state = MUL_START; else next_state = DIV_START;
                    INST_TYPE_BRANCH:  next_state = BRANCH_EXEC;
                    INST_TYPE_JAL:     next_state = JAL_EXEC;
                    INST_TYPE_JALR:    next_state = JALR_EXEC;
                    INST_TYPE_ALU:     next_state = ALU_EXEC;
                    default:           next_state = EXCEPTION;
                endcase
            end

            ALU_EXEC, BRANCH_EXEC, JAL_EXEC, JALR_EXEC: next_state = FETCH_START;
            EXCEPTION: next_state = EXCEPTION;

            MEM_SETUP: next_state = MEM_WAIT;
            MEM_WAIT: begin
                if (is_load_reg || (is_store_reg && !is_sc_reg)) next_state = FETCH_START;
                else if (is_sc_reg) next_state = AMO_WRITE; // SC needs to write back result to rd
                else if (is_atomic_reg) next_state = AMO_WRITE; // AMO needs to write back
                else next_state = FETCH_START;
            end
            AMO_WRITE: next_state = FETCH_START;
            
            MUL_START: next_state = MUL_WAIT;
            MUL_WAIT: if (!mul_busy) next_state = FETCH_START;
            
            DIV_START: next_state = DIV_WAIT;
            DIV_WAIT: if (!div_busy) next_state = FETCH_START;
        endcase
    end

    // Inicialización para simulación
    integer i;
    initial begin
        for (i=0; i<32; i=i+1) registers[i] = 0;
    end

endmodule
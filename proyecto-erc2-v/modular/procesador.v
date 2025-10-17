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

    localparam FETCH     = 2'b00;
    localparam EXECUTE   = 2'b01;
    reg [1:0] state = FETCH;

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
    wire is_alu_reg   = (opcode == 7'b0110011);
    wire is_lui       = (opcode == 7'b0110111);
    wire is_jal       = (opcode == 7'b1101111);
    wire is_branch    = (opcode == 7'b1100011);
    wire is_auipc     = (opcode == 7'b0010111);
    wire is_jalr      = (opcode == 7'b1100111);
    wire is_system    = (opcode == 7'b1110011);

    // --- Lógica de Escritura en Registros ---
    wire rd_wenable = is_alu_reg || is_alu_imm || is_lui || is_jal || is_load || is_auipc || is_jalr;

    // --- Lógica de Forwarding ---
    reg [4:0] prev_rdId_reg;
    reg       prev_rd_wenable_reg;
    reg [31:0] prev_rd_wdata_reg;

    wire [31:0] rs1_data = (rs1Id == 5'b0) ? 32'b0 : registers[rs1Id];
    wire [31:0] rs2_data = (rs2Id == 5'b0) ? 32'b0 : registers[rs2Id];

    wire do_forward_rs1 = prev_rd_wenable_reg && (prev_rdId_reg != 0) && (prev_rdId_reg == rs1Id);
    wire do_forward_rs2 = prev_rd_wenable_reg && (prev_rdId_reg != 0) && (prev_rdId_reg == rs2Id);

    wire [31:0] forwarded_rs1_data = do_forward_rs1 ? prev_rd_wdata_reg : rs1_data;
    wire [31:0] forwarded_rs2_data = do_forward_rs2 ? prev_rd_wdata_reg : rs2_data;

    // --- ALU y Lógica de Escritura (Combinacional) ---
    wire [31:0] alu_op2 = is_alu_reg ? forwarded_rs2_data : imm_i;
    wire [4:0] shamt = alu_op2[4:0];
    reg [31:0] rd_wdata;

    always @(*) begin
        if (is_load) begin
            rd_wdata = mem_rdata_in;
        end else if (is_lui) begin
            rd_wdata = imm_u;
        end else if (is_auipc) begin
            rd_wdata = (pc - 4) + imm_u;
        end else if (is_jal || is_jalr) begin
            rd_wdata = pc;
        end else begin
            case (funct3)
                3'b000: rd_wdata = (is_alu_reg && funct7[5]) ? (forwarded_rs1_data - alu_op2) : (forwarded_rs1_data + alu_op2);
                3'b001: rd_wdata = forwarded_rs1_data << shamt;
                3'b010: rd_wdata = ($signed(forwarded_rs1_data) < $signed(alu_op2)) ? 1 : 0;
                3'b011: rd_wdata = (forwarded_rs1_data < alu_op2) ? 1 : 0;
                3'b100: rd_wdata = forwarded_rs1_data ^ alu_op2;
                3'b101: rd_wdata = funct7[5] ? ($signed(forwarded_rs1_data) >>> shamt) : (forwarded_rs1_data >> shamt);
                3'b110: rd_wdata = forwarded_rs1_data | alu_op2;
                3'b111: rd_wdata = forwarded_rs1_data & alu_op2;
                default: rd_wdata = 32'b0;
            endcase
        end
    end

    // --- Lógica de Salto Condicional (Branch) ---
    reg branch_taken;
    always @(*) begin
        case (funct3)
            3'b000: branch_taken = (forwarded_rs1_data == forwarded_rs2_data); // BEQ
            3'b001: branch_taken = (forwarded_rs1_data != forwarded_rs2_data); // BNE
            3'b100: branch_taken = ($signed(forwarded_rs1_data) < $signed(forwarded_rs2_data)); // BLT
            3'b101: branch_taken = ($signed(forwarded_rs1_data) >= $signed(forwarded_rs2_data)); // BGE
            3'b110: branch_taken = (forwarded_rs1_data < forwarded_rs2_data);     // BLTU
            3'b111: branch_taken = (forwarded_rs1_data >= forwarded_rs2_data);    // BGEU
            default: branch_taken = 1'b0;
        endcase
    end

    // --- Salidas a la Memoria/SoC ---
    assign instruction_address_out = pc;
    assign mem_address_out   = forwarded_rs1_data + (is_load ? imm_i : imm_s);
    assign mem_wdata_out     = forwarded_rs2_data;
    assign mem_wenable_out   = is_store && (funct3 == 3'b010); // SW

    // --- Lógica Secuencial Principal ---
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc <= 32'h00000000;
            state <= FETCH;
            instruction_reg <= 32'h00000013; // NOP
            prev_rd_wenable_reg <= 1'b0;
        end else begin
            case (state)
                FETCH: begin
                    instruction_reg <= instruction_in;
                    state <= EXECUTE;
                end

                EXECUTE: begin
                    // Escritura en el banco de registros
                    if (rd_wenable && rdId != 0) begin
                        registers[rdId] <= rd_wdata;
                    end

                    // Actualización de registros para forwarding
                    prev_rdId_reg <= rdId;
                    prev_rd_wenable_reg <= rd_wenable;
                    prev_rd_wdata_reg <= rd_wdata;

                    // Actualización del PC
                    if (is_branch && branch_taken) begin
                        pc <= pc + imm_b;
                    end else if (is_jal) begin
                        pc <= pc + imm_j;
                    end else if (is_jalr) begin
                        pc <= (forwarded_rs1_data + imm_i) & 32'hFFFFFFFE;
                    end else if (is_system) begin
                        pc <= pc; // Halt
                    end else begin
                        pc <= pc + 4;
                    end
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

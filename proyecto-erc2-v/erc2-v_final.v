/*
 * erc2-v: Versión Final Monolítica (Corregida)
 * ---------------------------------------------
 * Corregida la condición de carrera en la escritura de registros y LEDs.
 * El dato a escribir (`rd_wdata`) ahora se calcula de forma combinacional.
 */

`default_nettype none

module erc2_v_final (
    input  wire clk_25mhz,
    output wire [7:0] led
);

    // --- Reset y Reloj ---
    reg reset = 1'b1;
    reg [4:0] reset_counter = 5'b0;
    always @(posedge clk_25mhz) begin
        if (&reset_counter) reset <= 1'b0;
        else reset_counter <= reset_counter + 1;
    end

    // --- CPU y Memoria ---
    reg [31:0] pc = 0;
    reg [31:0] instruction;
    reg [31:0] memory [0:1023];    // Memoria de 1024 palabras (4KB), asíncrona

    // Carga del programa desde un fichero externo
    initial begin
        $readmemh("firmware/firmware.hex", memory);
    end

    // --- Decodificación ---
    wire [6:0] opcode = instruction[6:0];
    wire [4:0] rdId   = instruction[11:7];
    wire [4:0] rs1Id  = instruction[19:15];
    wire [4:0] rs2Id  = instruction[24:20];
    wire [2:0] funct3 = instruction[14:12];
    wire [6:0] funct7 = instruction[31:25];
    wire [31:0] imm_i = {{21{instruction[31]}}, instruction[30:20]};
    wire [31:0] imm_u = {instruction[31:12], 12'b0};
    wire [31:0] imm_j = {{12{instruction[31]}}, instruction[19:12], instruction[20], instruction[30:21], 1'b0};
    wire [31:0] imm_b = {{20{instruction[31]}}, instruction[7], instruction[30:25], instruction[11:8], 1'b0};
    wire [31:0] imm_s = {{21{instruction[31]}}, instruction[30:25], instruction[11:7]};

    // --- Lógica de Salto Condicional (Branch) ---
    reg branch_taken;
    always @(*) begin
        case (funct3)
            3'b000: branch_taken = (rs1_data == rs2_data); // BEQ
            3'b001: branch_taken = (rs1_data != rs2_data); // BNE
            3'b100: branch_taken = ($signed(rs1_data) < $signed(rs2_data)); // BLT
            3'b101: branch_taken = ($signed(rs1_data) >= $signed(rs2_data)); // BGE
            3'b110: branch_taken = (rs1_data < rs2_data); // BLTU
            3'b111: branch_taken = (rs1_data >= rs2_data); // BGEU
            default: branch_taken = 1'b0;
        endcase
    end

    // --- Banco de Registros y Lógica Combinacional de Escritura ---
    reg [31:0] registers [0:31];
    wire [31:0] rs1_data = (rs1Id == 5'b0) ? 32'b0 : registers[rs1Id];
    wire [31:0] rs2_data = (rs2Id == 5'b0) ? 32'b0 : registers[rs2Id];
    
    // --- ALU (Unidad Aritmético-Lógica) ---
    reg [31:0] alu_result;
    wire [31:0] alu_op2 = (opcode == 7'b0110011) ? rs2_data : imm_i; // R-Type vs I-Type
    wire [4:0] shamt = alu_op2[4:0];

    always @(*) begin
        case (funct3)
            3'b000: alu_result = (opcode == 7'b0110011 && funct7[5]) ? (rs1_data - alu_op2) : (rs1_data + alu_op2);
            3'b001: alu_result = rs1_data << shamt;
            3'b010: alu_result = ($signed(rs1_data) < $signed(alu_op2)) ? 1 : 0;
            3'b011: alu_result = (rs1_data < alu_op2) ? 1 : 0;
            3'b100: alu_result = rs1_data ^ alu_op2;
            3'b101: alu_result = funct7[5] ? ($signed(rs1_data) >>> shamt) : (rs1_data >> shamt);
            3'b110: alu_result = rs1_data | alu_op2;
            3'b111: alu_result = rs1_data & alu_op2;
            default: alu_result = 32'b0;
        endcase
    end

    // --- Lógica de Escritura en Registros ---
    wire is_load      = (opcode == 7'b0000011);
    wire is_alu_imm   = (opcode == 7'b0010011);
    wire is_alu_reg   = (opcode == 7'b0110011);
    wire is_lui       = (opcode == 7'b0110111);
    wire is_jal       = (opcode == 7'b1101111);
    wire is_branch    = (opcode == 7'b1100011);
    wire is_auipc     = (opcode == 7'b0010111);
    wire is_jalr      = (opcode == 7'b1100111);
    wire is_system    = (opcode == 7'b1110011);

    wire rd_wenable = is_alu_reg || is_alu_imm || is_lui || is_jal || is_load || is_auipc || is_jalr;
    wire [31:0] rd_wdata;
    wire [31:0] mem_access_addr = rs1_data + (is_load ? imm_i : imm_s);

    assign rd_wdata = is_load ? memory[mem_access_addr[11:2]] :
                      is_lui  ? imm_u :
                      is_auipc? pc + imm_u :
                      (is_jal || is_jalr) ? pc + 4 :
                      alu_result;

    // --- Máquina de Estados y Lógica Secuencial ---
    localparam FETCH   = 0;
    localparam EXECUTE = 1;
    reg state = FETCH;

    always @(posedge clk_25mhz) begin
        if (reset) begin
            pc <= 0;
            state <= FETCH;
        end else begin
            // La escritura en registros ocurre aquí, usando los valores combinacionales
            if (rd_wenable && rdId != 0) begin
                registers[rdId] <= rd_wdata;
            end

            case (state)
                FETCH: begin
                    instruction <= memory[pc[11:2]];
                    state <= EXECUTE;
                end

                EXECUTE: begin
                    // Lógica de escritura para STORE
                    if (opcode == 7'b0100011 && funct3 == 3'b010) begin // SW
                        memory[mem_access_addr[11:2]] <= rs2_data;
                    end

                    // Actualización del PC con lógica de control de flujo completa
                    if (is_branch && branch_taken) begin
                        pc <= pc + imm_b;
                    end else if (is_jal) begin
                        pc <= pc + imm_j;
                    end else if (is_jalr) begin
                        pc <= (rs1_data + imm_i) & 32'hFFFFFFFE; // LSB a 0
                    end else if (is_system) begin
                        pc <= pc; // Bucle infinito para ECALL/EBREAK
                    end else begin
                        pc <= pc + 4;
                    end
                    state <= FETCH;
                end
            endcase
        end
    end

    // --- Salida a LEDs ---
    reg [7:0] led_reg;
    assign led = led_reg;
    localparam LED_ADDR = 32'h80000000;

    // Lógica de I/O Mapeado en Memoria para los LEDs
    always @(posedge clk_25mhz) begin
        if (state == EXECUTE && opcode == 7'b0100011 && funct3 == 3'b010) begin // SW
            if (mem_access_addr == LED_ADDR) begin
                led_reg <= rs2_data[7:0];
            end
        end
    end

    // Inicialización para simulación
    integer i;
    initial begin
        for (i=0; i<32; i=i+1) registers[i] = 0;
    end

endmodule
`default_nettype none

module soc(
    input  wire clk_25mhz,
    output wire [7:0] led
);

    // --- Reset Logic ---
    reg reset = 1'b1;
    reg [4:0] reset_counter = 5'b0;
    always @(posedge clk_25mhz) begin
        if (&reset_counter) reset <= 1'b0;
        else reset_counter <= reset_counter + 1;
    end

    // --- Memoria RAM (Directamente en el SOC) ---
    // La memoria se inicializa desde un fichero externo.
    // La siguiente línea es un atributo específico para Yosys (síntesis).
    (* ram_init_file = "../firmware/test_atomic_full.hex" *)
    reg [31:0] memory [0:16383]; // Memoria de 64KB (16384 palabras de 32 bits)

    // La siguiente línea es para la simulación (ej. con Icarus Verilog).
    initial begin
        $readmemh("../firmware/test_atomic_full.hex", memory);
    end

    // --- Interconexiones CPU <-> Memoria/Periféricos ---
    wire [31:0] instruction_address; // Dirección de la instrucción (desde el PC)
    wire [31:0] data_address;        // Dirección para loads/stores (desde la ALU)
    wire [31:0] data_wdata;          // Dato a escribir por la CPU
    wire        data_wenable;        // Habilitación de escritura por la CPU

    reg [31:0] instruction_out;     // Salida del puerto de instrucción de la memoria (REGISTRADA)
    reg [31:0] data_out;            // Salida del puerto de datos de la memoria (REGISTRADA)

    // --- Instancia del Procesador ---
    wire [7:0] cpu_debug_signals;

    procesador_monolithic cpu (
        .clk(clk_25mhz),
        .reset(reset),
        .instruction_in(instruction_out),
        .mem_rdata_in(data_out),
        .instruction_address_out(instruction_address),
        .mem_address_out(data_address),
        .mem_wdata_out(data_wdata),
        .mem_wenable_out(data_wenable),
        .debug_out(cpu_debug_signals)
    );

    // --- Decodificador de Direcciones y Acceso a Memoria/Periféricos ---
    localparam RAM_START_ADDR = 32'h00000000;
    localparam RAM_END_ADDR   = 32'h0000FFFF; // 64KB
    localparam LED_ADDR       = 32'h80000000;

    // Lógica de Lectura SÍNCRONA
    always @(posedge clk_25mhz) begin
        // Puerto de Instrucción
        instruction_out <= memory[instruction_address[15:2]];

        // Puerto de Datos
        if (data_address >= RAM_START_ADDR && data_address <= RAM_END_ADDR) begin
            data_out <= memory[data_address[15:2]];
        end else begin
            data_out <= 32'h0;
        end
    end

    // Lógica de Escritura en RAM
    always @(posedge clk_25mhz) begin
        if (data_wenable && (data_address >= RAM_START_ADDR && data_address <= RAM_END_ADDR)) begin
            memory[data_address[15:2]] <= data_wdata;
        end
    end

    // --- Periférico: LEDs Mapeados en Memoria ---
    assign led = cpu_debug_signals; // DEBUG: Show all debug signals on LEDs

    // Original memory-mapped LED logic (now bypassed for debug)
    reg [7:0] led_reg = 8'h00;
    always @(posedge clk_25mhz) begin
        if (data_wenable && (data_address == LED_ADDR)) begin
            led_reg <= data_wdata[7:0];
        end
    end

endmodule
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
    (* ram_init_file = "../firmware/test_sw_ptw.hex" *)
    reg [31:0] memory [0:32767]; // Memoria de 128KB (32768 palabras de 32 bits)

    // La siguiente línea es para la simulación (ej. con Icarus Verilog).
    initial begin
        $readmemh("../firmware/test_sw_ptw.hex", memory);
    end

    // --- Interconexiones CPU <-> Memoria/Periféricos ---
    wire [31:0] instruction_address; // Dirección de la instrucción (desde el PC)
    wire [31:0] data_address;        // Dirección para loads/stores (desde la ALU)
    wire [31:0] data_wdata;          // Dato a escribir por la CPU
    wire        data_wenable;        // Habilitación de escritura por la CPU

    wire [31:0] instruction_out;     // Salida del puerto de instrucción de la memoria
    wire [31:0] data_out;            // Salida del puerto de datos de la memoria

    // --- Instancia del Procesador ---
    wire [3:0] cpu_state;

    procesador cpu (
        .debug_state_out(cpu_state),
        .clk(clk_25mhz),
        .reset(reset),
        .instruction_in(instruction_out),
        .mem_rdata_in(data_out),
        .instruction_address_out(instruction_address),
        .mem_address_out(data_address),
        .mem_wdata_out(data_wdata),
        .mem_wenable_out(data_wenable)
    );

    // --- Decodificador de Direcciones y Acceso a Memoria/Periféricos ---
    localparam RAM_START_ADDR = 32'h00000000;
    localparam RAM_END_ADDR   = 32'h0001FFFF;
    localparam LED_ADDR       = 32'h80000000;

    // Lógica de Lectura - Puerto de Instrucción (Combinacional)
    assign instruction_out = memory[instruction_address[11:2]];

    // Lógica de Lectura/Escritura - Puerto de Datos
    wire is_ram_access = (data_address >= RAM_START_ADDR && data_address <= RAM_END_ADDR);
    wire is_led_access = (data_address == LED_ADDR);

    // Lectura de datos (combinacional, ya que la CPU espera un ciclo)
    assign data_out = is_ram_access ? memory[data_address[11:2]] : 32'h00000000;

    // Escritura en RAM
    always @(posedge clk_25mhz) begin
        if (data_wenable && is_ram_access) begin
            memory[data_address[11:2]] <= data_wdata;
        end
    end

    // --- Periférico: LEDs Mapeados en Memoria ---
    reg [7:0] led_reg = 8'h00;  // Start with LEDs off
    assign led = led_reg;

    always @(posedge clk_25mhz) begin
        // --- DEBUG: Override LEDs to show CPU FSM state ---
        led_reg[3:0] <= cpu_state;
        led_reg[7:4] <= 4'b0; // Keep upper LEDs off for clarity

        // Original logic commented out for debugging
        // if (data_wenable && is_led_access) begin
        //     led_reg <= data_wdata[7:0];
        // end
    end

endmodule
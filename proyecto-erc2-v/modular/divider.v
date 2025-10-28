/*
 * Módulo Divisor Secuencial Pipelineado para RISC-V
 * --------------------------------------------------
 * Implementa DIVU y REMU usando un algoritmo de restauración.
 * Utiliza un pipeline interno de 2 etapas por bit para garantizar la estabilidad temporal.
 * Tarda ~64 ciclos por operación.
 */

`default_nettype none

module divider(
    input  wire clk,
    input  wire reset,

    input  wire start,
    input  wire [31:0] dividend,
    input  wire [31:0] divisor,

    output wire [31:0] quotient_out,
    output wire [31:0] remainder_out,
    output wire        busy
);

    // Estados de la FSM interna
    localparam S_IDLE      = 2'b00;
    localparam S_RUN_CALC  = 2'b01; // Ciclo 1: Calcula la resta
    localparam S_RUN_WRITE = 2'b10; // Ciclo 2: Actualiza el resultado
    localparam S_DONE      = 2'b11;
    reg [1:0] state_reg;

    // Registros para el algoritmo
    reg [63:0] rem_quot_reg; // [63:32] para el resto (A), [31:0] para el cociente (Q)
    reg [31:0] divisor_reg;
    reg [5:0]  count_reg;
    
    // Registros de pipeline para romper el camino crítico
    reg [63:0] shifted_val_pipe;
    reg [32:0] trial_rem_pipe;

    // Wire intermedio para el cálculo del desplazamiento
    wire [63:0] temp_shifted_val = rem_quot_reg << 1;

    // Asignación de salidas
    assign quotient_out  = rem_quot_reg[31:0];
    assign remainder_out = rem_quot_reg[63:32];
    assign busy          = (state_reg != S_IDLE);

    // Lógica del divisor (FSM)
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state_reg    <= S_IDLE;
            rem_quot_reg <= 64'd0;
            divisor_reg  <= 32'd0;
            count_reg    <= 6'd0;
        end else begin
            case (state_reg)
                S_IDLE: begin
                    if (start) begin
                        if (divisor == 32'd0) begin
                            rem_quot_reg <= {dividend, 32'hFFFFFFFF}; // Div by zero
                            state_reg    <= S_DONE;
                        end else begin
                            rem_quot_reg <= {32'd0, dividend};
                            divisor_reg  <= divisor;
                            count_reg    <= 32;
                            state_reg    <= S_RUN_CALC;
                        end
                    end
                end

                S_RUN_CALC: begin // Ciclo 1: Desplaza y calcula la resta
                    if (count_reg > 0) begin
                        shifted_val_pipe <= temp_shifted_val;
                        trial_rem_pipe   <= {1'b0, temp_shifted_val[63:32]} - {1'b0, divisor_reg};
                        state_reg        <= S_RUN_WRITE;
                    end else begin
                        state_reg <= S_DONE;
                    end
                end

                S_RUN_WRITE: begin // Ciclo 2: Actualiza el registro principal con los resultados calculados
                    if (trial_rem_pipe[32]) begin // Si la resta fue negativa
                        rem_quot_reg <= shifted_val_pipe; // Restaurar
                    end else begin
                        rem_quot_reg <= {trial_rem_pipe[31:0], shifted_val_pipe[31:1], 1'b1};
                    end
                    count_reg <= count_reg - 1;
                    state_reg <= S_RUN_CALC;
                end

                S_DONE: begin
                    state_reg <= S_IDLE;
                end

                default: begin
                    state_reg <= S_IDLE;
                end
            endcase
        end
    end

endmodule
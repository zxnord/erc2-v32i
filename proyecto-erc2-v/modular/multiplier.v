`default_nettype none

/*
 * Módulo Multiplicador Secuencial Robusto para RISC-V
 * -----------------------------------------------------
 * Implementa la instrucción MUL usando una FSM interna para un control preciso.
 */

module multiplier(
    input  wire clk,
    input  wire reset,

    input  wire start,
    input  wire [31:0] rs1_data,
    input  wire [31:0] rs2_data,

    output wire [31:0] result,
    output wire        busy
);

    // Estados de la FSM interna del multiplicador
    localparam S_IDLE = 2'b00;
    localparam S_RUN  = 2'b01;
    localparam S_DONE = 2'b10;
    reg [1:0] state_reg;

    // Registros para el algoritmo
    reg [63:0] product_reg;
    reg [31:0] multiplicand_reg;
    reg [5:0]  count_reg;

    // --- Lógica Combinacional para la Multiplicación ---
    // Se usa un wire intermedio para calcular el valor del producto tras la suma condicional.
    // Esto corrige el bug donde la suma se perdía por la sobrescritura del desplazamiento.
    wire [63:0] added_product = product_reg[0] ? {product_reg[63:32] + multiplicand_reg, product_reg[31:0]} : product_reg;

    // Asignación de salidas
    assign result = product_reg[31:0];
    assign busy = (state_reg != S_IDLE);

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state_reg <= S_IDLE;
            product_reg <= 64'd0;
            multiplicand_reg <= 32'd0;
            count_reg <= 6'd0;
        end else begin
            case (state_reg)
                S_IDLE: begin
                    if (start) begin
                        product_reg[31:0]  <= rs2_data; // Carga multiplicador
                        product_reg[63:32] <= 32'd0;
                        multiplicand_reg <= rs1_data; // Carga multiplicando
                        count_reg        <= 32;
                        state_reg        <= S_RUN;
                    end
                end

                S_RUN: begin
                    if (count_reg > 0) begin
                        // El producto se actualiza con el valor pre-calculado y desplazado.
                        product_reg <= added_product >> 1;
                        count_reg   <= count_reg - 1;
                    end else begin
                        // Termina los ciclos de cálculo, pasa a DONE
                        state_reg <= S_DONE;
                    end
                end

                S_DONE: begin
                    // El resultado es estable. En el próximo ciclo, vuelve a IDLE.
                    state_reg <= S_IDLE;
                end

                default: begin
                    state_reg <= S_IDLE;
                end
            endcase
        end
    end

endmodule
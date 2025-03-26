/**
 * Paso 21: Creando un procesador RISC-V
 *         Ejecutando codigo en C compilado con GCC
 * Listo*
 */

module Processor (
    input             clk,          // reloj de sistema
    input             resetn,       // boton de reset
    output     [31:0] mem_addr,     // direccion de memoria a utilizar (LOAD/STORE)
    input      [31:0] mem_rdata,    // Data leida (LOAD)
    output            mem_rstrb,    // Bit de "procesador quiere leer" de forma verificada
    output [31:0]     mem_wdata,    // Data a escribir (STORE)
    output [3:0]      mem_wmask     // Mascara para determinar tamano de palabra a escribir SW, SB, SH
);

    reg [31:0] PC=0;        // program counter
    reg [31:0] instr;       // current instruction

    // See the table P. 105 in RISC-V manual

    // The 10 RISC-V instructions
    wire isALUreg  =  (instr[6:0] == 7'b0110011); // rd <- rs1 OP rs2   
    wire isALUimm  =  (instr[6:0] == 7'b0010011); // rd <- rs1 OP Iimm
    wire isBranch  =  (instr[6:0] == 7'b1100011); // if(rs1 OP rs2) PC<-PC+Bimm
    wire isJALR    =  (instr[6:0] == 7'b1100111); // rd <- PC+4; PC<-rs1+Iimm
    wire isJAL     =  (instr[6:0] == 7'b1101111); // rd <- PC+4; PC<-PC+Jimm
    wire isAUIPC   =  (instr[6:0] == 7'b0010111); // rd <- PC + Uimm
    wire isLUI     =  (instr[6:0] == 7'b0110111); // rd <- Uimm   
    wire isLoad    =  (instr[6:0] == 7'b0000011); // rd <- mem[rs1+Iimm]
    wire isStore   =  (instr[6:0] == 7'b0100011); // mem[rs1+Simm] <- rs2
    wire isSYSTEM  =  (instr[6:0] == 7'b1110011); // special
    wire isLR      =  (instr[6:0] == 7'b0101111) && (instr[31:27] == 5'b00010); // LR.W
    wire isSC      =  (instr[6:0] == 7'b0101111) && (instr[31:27] == 5'b00011); // SC.W
    wire isMUL     =  (instr[6:0] == 7'b0110011) && (instr[31:25] == 7'b0000001) && (instr[14:12] == 3'b000); // MUL
    wire isMULH    =  (instr[6:0] == 7'b0110011) && (instr[31:25] == 7'b0000001) && (instr[14:12] == 3'b001); // MULH

    // The 5 immediate formats
    wire [31:0] Uimm={    instr[31],   instr[30:12], {12{1'b0}}};
    wire [31:0] Iimm={{21{instr[31]}}, instr[30:20]};
    wire [31:0] Simm={{21{instr[31]}}, instr[30:25],instr[11:7]};
    wire [31:0] Bimm={{20{instr[31]}}, instr[7],instr[30:25],instr[11:8],1'b0};
    wire [31:0] Jimm={{12{instr[31]}}, instr[19:12],instr[20],instr[30:21],1'b0};

    // Source and destination registers
    wire [4:0] rs1Id = instr[19:15];
    wire [4:0] rs2Id = instr[24:20];
    wire [4:0] rdId  = instr[11:7];

    // function codes
    wire [2:0] funct3 = instr[14:12];
    wire [6:0] funct7 = instr[31:25];

    // The registers bank
    reg [31:0] RegisterBank [0:31];
    reg [31:0] rs1; // value of source
    reg [31:0] rs2; //  registers.
    wire [31:0] writeBackData; // data to be written to rd
    wire        writeBackEn;   // asserted if data should be written to rd

`ifdef BENCH
    integer     i;
    initial begin
        for(i=0; i<32; ++i) begin
            RegisterBank[i] = 0;
        end
    end
`endif

    // The ALU
    wire [31:0] aluIn1 = rs1;
    wire [31:0] aluIn2 = isALUreg | isBranch ? rs2 : Iimm;

    wire [4:0] shamt = isALUreg ? rs2[4:0] : instr[24:20]; // shift amount

    // The adder is used by both arithmetic instructions and JALR.
    wire [31:0] aluPlus = aluIn1 + aluIn2;

    // Use a single 33 bits subtract to do subtraction and all comparisons
    // (trick borrowed from swapforth/J1)
    wire [32:0] aluMinus = {1'b1, ~aluIn2} + {1'b0,aluIn1} + 33'b1;
    wire        LT  = (aluIn1[31] ^ aluIn2[31]) ? aluIn1[31] : aluMinus[32];
    wire        LTU = aluMinus[32];
    wire        EQ  = (aluMinus[31:0] == 0);

    // Flip a 32 bit word. Used by the shifter (a single shifter for
    // left and right shifts, saves silicium !)
    function [31:0] flip32;
        input [31:0] x;
        flip32 = {x[ 0], x[ 1], x[ 2], x[ 3], x[ 4], x[ 5], x[ 6], x[ 7], 
                 x[ 8], x[ 9], x[10], x[11], x[12], x[13], x[14], x[15], 
                 x[16], x[17], x[18], x[19], x[20], x[21], x[22], x[23],
                 x[24], x[25], x[26], x[27], x[28], x[29], x[30], x[31]};
    endfunction

    wire [31:0] shifter_in = (funct3 == 3'b001) ? flip32(aluIn1) : aluIn1;

    /* verilator lint_off WIDTH */
    wire [31:0] shifter = 
                $signed({instr[30] & aluIn1[31], shifter_in}) >>> aluIn2[4:0];
    /* verilator lint_on WIDTH */

    wire [31:0] leftshift = flip32(shifter);

    // ADD/SUB/ADDI: 
    // funct7[5] is 1 for SUB and 0 for ADD. We need also to test instr[5]
    // to make the difference with ADDI
    //
    // SRLI/SRAI/SRL/SRA: 
    // funct7[5] is 1 for arithmetic shift (SRA/SRAI) and 
    // 0 for logical shift (SRL/SRLI)
    reg [31:0]  aluOut;
    always @(*) begin
        case(funct3)
        3'b000: aluOut = (funct7[5] & instr[5]) ? aluMinus[31:0] : aluPlus;
        3'b001: aluOut = leftshift;
        3'b010: aluOut = {31'b0, LT};
        3'b011: aluOut = {31'b0, LTU};
        3'b100: aluOut = (aluIn1 ^ aluIn2);
        3'b101: aluOut = shifter;
        3'b110: aluOut = (aluIn1 | aluIn2);
        3'b111: aluOut = (aluIn1 & aluIn2);	
        endcase
    end

    // The predicate for branch instructions
    reg takeBranch;
    always @(*) begin
        case(funct3)
        3'b000: takeBranch = EQ;
        3'b001: takeBranch = !EQ;
        3'b100: takeBranch = LT;
        3'b101: takeBranch = !LT;
        3'b110: takeBranch = LTU;
        3'b111: takeBranch = !LTU;
        default: takeBranch = 1'b0;
        endcase
    end

    // Address computation
    // An adder used to compute branch address, JAL address and AUIPC.
    // branch->PC+Bimm    AUIPC->PC+Uimm    JAL->PC+Jimm
    // Equivalent to PCplusImm = PC + (isJAL ? Jimm : isAUIPC ? Uimm : Bimm)
    wire [31:0] PCplusImm = PC + ( instr[3] ? Jimm[31:0] :
                                   instr[4] ? Uimm[31:0] :
                                              Bimm[31:0] );

    // Registros para manejar las instrucciones atómicas
    reg [31:0] reservation; // Dirección reservada para LR/SC
    reg reservation_valid;  // Indica si la reserva es válida

    // Estados del procesador
    localparam FETCH_INSTR = 0;
    localparam WAIT_INSTR  = 1;
    localparam FETCH_REGS  = 2;
    localparam EXECUTE     = 3;
    localparam LOAD        = 4;
    localparam WAIT_DATA   = 5;
    localparam STORE       = 6;
    reg [2:0] state = FETCH_INSTR;

    always @(posedge clk) begin
        if(!resetn) begin
            PC    <= 0;
            state <= FETCH_INSTR;
            reservation <= 32'b0;
            reservation_valid <= 1'b0;
        end else begin
            if(writeBackEn && rdId != 0) begin
                RegisterBank[rdId] <= writeBackData;
                // $display("r%0d <= %b (%d) (%d)",rdId,writeBackData,writeBackData,$signed(writeBackData));
                // For displaying what happens.
            end
            case(state)
            FETCH_INSTR: begin
                state <= WAIT_INSTR;
                end
            WAIT_INSTR: begin
                instr <= mem_rdata;
                state <= FETCH_REGS;
                end
            FETCH_REGS: begin
                rs1 <= RegisterBank[rs1Id];
                rs2 <= RegisterBank[rs2Id];
                state <= EXECUTE;
                end
            EXECUTE: begin
                if(!isSYSTEM) begin
                    PC <= nextPC;
                end
                if (isLR) begin
                    reservation <= rs1 + Iimm; // Guardar la dirección reservada
                    reservation_valid <= 1'b1;
                    state <= WAIT_DATA;
                end else if (isSC) begin
                    if (reservation_valid && (reservation == (rs1 + Iimm))) begin
                        reservation_valid <= 1'b0; // Invalidar la reserva
                    end
                    state <= FETCH_INSTR;
                end else begin
                    state <= isLoad  ? LOAD  :
                             isStore ? STORE :
                             FETCH_INSTR;
                end
`ifdef BENCH
                if(isSYSTEM) $finish();
`endif      
                end
                LOAD: begin
                    state <= WAIT_DATA;
                    end
                WAIT_DATA: begin
                    state <= FETCH_INSTR;
                    end
                STORE: begin
                    state <= FETCH_INSTR;
                    end
            endcase
        end
    end

    assign writeBackEn = (state==EXECUTE && !isBranch && !isStore) ||
                         (state==WAIT_DATA) || (state==FETCH_INSTR && isSC);

    assign mem_addr = (state == WAIT_INSTR || state == FETCH_INSTR) ?
                      PC : loadstore_addr ;
    assign mem_rstrb = (isLR || state == FETCH_INSTR || state == LOAD);
    assign mem_wmask = (isSC && reservation_valid && (reservation == (rs1 + Iimm))) ? 4'b1111 :
                        {4{(state == STORE)}} & STORE_wmask;

    // Multiplicación
    wire [63:0] mul_result = rs1 * rs2;
    wire [31:0] mul_lo = mul_result[31:0];
    wire [31:0] mul_hi = mul_result[63:32];

    wire [31:0] PCplus4 = PC+4;

    // register write back
    assign writeBackData = (isJAL || isJALR) ? PCplus4   :
                               isLUI         ? Uimm      :
                               isAUIPC       ? PCplusImm :
                               isLoad        ? LOAD_data :
                               isMUL         ? mul_lo    :
                               isMULH        ? mul_hi    :
                               isSC          ? ((reservation_valid && (reservation == (rs1 + Iimm))) ? 32'b0 : 32'b1) :
                                               aluOut;

    wire [31:0] nextPC = ((isBranch && takeBranch) || isJAL) ? PCplusImm   :
                                           isJALR   ? {aluPlus[31:1],1'b0} :
                                                      PCplus4;

    wire [31:0] loadstore_addr = rs1 + (isStore ? Simm : Iimm);

    // Load
    // All memory accesses are aligned on 32 bits boundary. For this
    // reason, we need some circuitry that does unaligned halfword
    // and byte load/store, based on:
    // - funct3[1:0]:  00->byte 01->halfword 10->word
    // - mem_addr[1:0]: indicates which byte/halfword is accessed

    wire mem_byteAccess     = funct3[1:0] == 2'b00;
    wire mem_halfwordAccess = funct3[1:0] == 2'b01;


    wire [15:0] LOAD_halfword =
                loadstore_addr[1] ? mem_rdata[31:16] : mem_rdata[15:0];

    wire  [7:0] LOAD_byte =
                loadstore_addr[0] ? LOAD_halfword[15:8] : LOAD_halfword[7:0];

    // LOAD, in addition to funct3[1:0], LOAD depends on:
    // - funct3[2] (instr[14]): 0->do sign expansion   1->no sign expansion
    wire LOAD_sign =
         !funct3[2] & (mem_byteAccess ? LOAD_byte[7] : LOAD_halfword[15]);

    wire [31:0] LOAD_data =
          mem_byteAccess ? {{24{LOAD_sign}},     LOAD_byte} :
      mem_halfwordAccess ? {{16{LOAD_sign}}, LOAD_halfword} :
                           mem_rdata ;

    // Store
    // ------------------------------------------------------------------------
    assign mem_wdata = (reservation_valid && (reservation == (rs1 + Iimm))) ? rs2 : 
                       (mem_byteAccess ? {4{rs2[7:0]}} :
                       (mem_halfwordAccess ? {2{rs2[15:0]}} : rs2));

    // The memory write mask:
    //    1111                     if writing a word
    //    0011 or 1100             if writing a halfword
    //                                (depending on loadstore_addr[1])
    //    0001, 0010, 0100 or 1000 if writing a byte
    //                                (depending on loadstore_addr[1:0])

    wire [3:0] STORE_wmask =
               mem_byteAccess      ?
                     (loadstore_addr[1] ?
                           (loadstore_addr[0] ? 4'b1000 : 4'b0100) :
                           (loadstore_addr[0] ? 4'b0010 : 4'b0001)
                     ) :
               mem_halfwordAccess ?
                     (loadstore_addr[1] ? 4'b1100 : 4'b0011) :
               4'b1111;

endmodule
# Makefile para sintetizar el diseño modular del erc2-v

# --- Configuración del Proyecto y Placa ---
PROJECT = soc
FPGA_TYPE = 85k
FPGA_PKG  = CABGA381
BOARD     = ulx3s

# --- Ficheros Fuente ---
# Ahora incluimos los 3 módulos de nuestro diseño
VERILOG_FILES = soc.v procesador.v multiplier.v divider.v mmu.v
LPF_FILE      = ulx3s.lpf

# --- Ficheros de Salida ---
JSON_FILE = $(PROJECT).json
ASC_FILE  = $(PROJECT).asc
BIT_FILE  = $(PROJECT).bit

# --- Herramientas ---
YOSYS     = yosys
NEXTPNR   = nextpnr-ecp5
ECPPACK   = ecppack
UPLOADER  = openFPGALoader

# --- Objetivos del Makefile ---

.PHONY: all
all: $(BIT_FILE)

# 1. Síntesis (Yosys)
$(JSON_FILE): $(VERILOG_FILES)
	@echo "--- 1. Sintetizando con Yosys ---"
	$(YOSYS) -p "synth_ecp5 -top $(PROJECT) -json $@" $(VERILOG_FILES)

# 2. Place & Route (nextpnr)
$(ASC_FILE): $(JSON_FILE) $(LPF_FILE)
	@echo "--- 2. Place & Route con nextpnr-ecp5 ---"
	$(NEXTPNR) --$(FPGA_TYPE) --package $(FPGA_PKG) --json $(JSON_FILE) --lpf $(LPF_FILE) --textcfg $@

# 3. Empaquetado (ecppack)
$(BIT_FILE): $(ASC_FILE)
	@echo "--- 3. Empaquetando a bitstream con ecppack ---"
	$(ECPPACK) $^ $@

# 4. Carga (openFPGALoader)
.PHONY: upload
upload: $(BIT_FILE)
	@echo "--- 4. Cargando bitstream en la ULX3S ---"
	$(UPLOADER) -b $(BOARD) $(BIT_FILE)

# Limpieza de ficheros generados
.PHONY: clean
clean:
	@echo "--- Limpiando ficheros generados ---"
	rm -f $(JSON_FILE) $(ASC_FILE) $(BIT_FILE)

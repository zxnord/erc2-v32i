all: paso2/paso2.bit paso3/paso3.bit paso7/paso7.bit paso9/paso9.bit paso11/paso11.bit


# Definimos una variable que contiene los nombres de los pasos
PASOS = paso1 paso2 paso3 paso4 paso5 paso6 paso7 paso8 paso9 paso10 paso11 paso12

# Definimos una variable que contiene las extensiones de los archivos
EXTENSIONES = json config bit svf

# Creamos una regla para cada paso y extensión
$(foreach paso,$(PASOS),$(foreach ext,$(EXTENSIONES),$(eval $(paso)/$(paso).$(ext): $(paso)/$(paso).v)))


IDCODE ?= 0x41113043 # 85f

PNR ?= nextpnr-ecp5
LOADER ?= openFPGALoader

%.bit: %.config
	ecppack --idcode $(IDCODE) $< $@

%.svf: %.config
	ecppack --idcode $(IDCODE) --input $< --svf $@

# Generate a desired MHz pll
pll_%.v:
	ecppll \
		-i 25 \
		-o $(subst pll_,,$(basename $@)) \
		-n $(basename $@) \
		-f $@

%.json: %.v
	$(ICEPATH)yosys \
		-q \
		-p 'read_verilog $<' \
		-p 'synth_ecp5 -top SOC -json $@' \
		-E .$(notdir $(basename $@)).d

%.upload: %.bit
	$(LOADER) \
		--board ulx3s \
		$(basename $@).bit \

%.flash: %.bit
	$(LOADER) \
		--write-flash \
		--unprotect-flash \
		--board ulx3s \
		$(basename $@).bit \

%.config: %.json
	$(PNR) \
		--json $< \
		--textcfg $@ \
		--lpf ulx3s_v3.lpf \
		--85k \
		--package CABGA381


clean:
	$(RM) */*.svf */*.config */*.bit */*.json .*.d */*/*.o */*/*.hex */*/*.elf
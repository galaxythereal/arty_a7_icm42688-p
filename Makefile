# ==============================================================================
# Makefile — ICM-42688-P IMU Streaming on Arty A7-100T
# Open-source toolchain: Yosys + nextpnr-xilinx + prjxray + openFPGALoader
# ==============================================================================

# --- Project ---
TOP        := top
DEVICE     := xc7a100tcsg324-1
FAMILY     := artix7

# --- Directories ---
RTL_DIR    := rtl
TB_DIR     := tb
XDC_DIR    := constraints
BUILD_DIR  := build
CHIPDB_DIR := chipdb
SCRIPT_DIR := scripts

# --- Tool paths (openxc7 snap) ---
YOSYS          := yosys
NEXTPNR        := nextpnr-xilinx
FASM2FRAMES    := fasm2frames
XC7FRAMES2BIT  := xc7frames2bit
BBASM          := bbasm
LOADER         := openFPGALoader
IVERILOG       := iverilog
VVP            := vvp

# --- prjxray database (from openxc7 snap) ---
PRJXRAY_DB := /snap/openxc7/x1/opt/nextpnr-xilinx/external/prjxray-db/$(FAMILY)
BBAEXPORT  := /snap/openxc7/x1/opt/nextpnr-xilinx/python/bbaexport.py
PYPY3      := /snap/openxc7/x1/usr/bin/python3.8

# --- Chipdb ---
CHIPDB_BBA := $(CHIPDB_DIR)/$(DEVICE).bba
CHIPDB_BIN := $(CHIPDB_DIR)/$(DEVICE).bin

# --- Source files ---
RTL_SRCS := $(RTL_DIR)/icm42688_regs.vh \
            $(RTL_DIR)/spi_master.v      \
            $(RTL_DIR)/icm42688_ctrl.v   \
            $(RTL_DIR)/uart_tx.v         \
            $(RTL_DIR)/imu_data_stream.v \
            $(RTL_DIR)/top.v

VERILOG_SRCS := $(filter %.v,$(RTL_SRCS))

# --- Build artifacts ---
SYNTH_JSON := $(BUILD_DIR)/$(TOP).json
FASM_FILE  := $(BUILD_DIR)/$(TOP).fasm
FRAMES     := $(BUILD_DIR)/$(TOP).frames
BITSTREAM  := $(BUILD_DIR)/$(TOP).bit

# --- XDC ---
XDC_FILE := $(XDC_DIR)/arty_a7_100t.xdc

# ==============================================================================
# Default target
# ==============================================================================
.PHONY: all clean program flash sim chipdb detect help

all: $(BITSTREAM)
	@echo ""
	@echo "============================================"
	@echo " BUILD COMPLETE: $(BITSTREAM)"
	@echo " Run 'make program' to load onto FPGA"
	@echo "============================================"

# ==============================================================================
# Step 0: Generate chipdb (only needed once, takes ~10-20 min)
# ==============================================================================
$(CHIPDB_BBA):
	@echo ">>> Generating chipdb BBA for $(DEVICE) (this takes a while)..."
	@mkdir -p $(CHIPDB_DIR)
	$(PYPY3) $(BBAEXPORT) --device $(DEVICE) --bba $@

$(CHIPDB_BIN): $(CHIPDB_BBA)
	@echo ">>> Compiling chipdb BBA → BIN..."
	$(BBASM) -l $< $@

chipdb: $(CHIPDB_BIN)
	@echo ">>> Chipdb ready: $(CHIPDB_BIN)"

# --- Or copy an existing pre-built chipdb ---
chipdb-copy:
	@echo ">>> Copying pre-built chipdb..."
	@mkdir -p $(CHIPDB_DIR)
	@if [ -f "/home/galaxy/icm42688_arty/chipdb/xc7a100tcsg324.bin" ]; then \
		cp /home/galaxy/icm42688_arty/chipdb/xc7a100tcsg324.bin $(CHIPDB_BIN); \
		echo "    Copied from /home/galaxy/icm42688_arty/chipdb/"; \
	elif [ -f "/home/galaxy/Downloads/files(3)/xc7a100t.bin" ]; then \
		cp "/home/galaxy/Downloads/files(3)/xc7a100t.bin" $(CHIPDB_BIN); \
		echo "    Copied from /home/galaxy/Downloads/files(3)/"; \
	else \
		echo "ERROR: No pre-built chipdb found. Run 'make chipdb' to generate."; \
		exit 1; \
	fi

# ==============================================================================
# Step 1: Synthesis (Yosys)
# ==============================================================================
$(SYNTH_JSON): $(VERILOG_SRCS) $(RTL_DIR)/icm42688_regs.vh
	@echo ">>> [1/4] Synthesizing with Yosys..."
	@mkdir -p $(BUILD_DIR)
	$(YOSYS) -p " \
		read_verilog -I$(RTL_DIR) $(VERILOG_SRCS); \
		synth_xilinx -flatten -abc9 -arch xc7 -top $(TOP); \
		write_json $@" \
		2>&1 | tee $(BUILD_DIR)/yosys.log
	@echo ">>> Synthesis done."

# ==============================================================================
# Step 2: Place & Route (nextpnr-xilinx)
# ==============================================================================
$(FASM_FILE): $(SYNTH_JSON) $(XDC_FILE) $(CHIPDB_BIN)
	@echo ">>> [2/4] Place & Route with nextpnr-xilinx..."
	$(NEXTPNR) \
		--chipdb $(CHIPDB_BIN) \
		--xdc $(XDC_FILE) \
		--json $(SYNTH_JSON) \
		--write $(BUILD_DIR)/$(TOP)_routed.json \
		--fasm $@ \
		--verbose \
		2>&1 | tee $(BUILD_DIR)/nextpnr.log
	@echo ">>> Place & Route done."

# ==============================================================================
# Step 3: FASM → Frames
# ==============================================================================
$(FRAMES): $(FASM_FILE)
	@echo ">>> [3/4] FASM → Frames..."
	$(FASM2FRAMES) \
		--part $(DEVICE) \
		--db-root $(PRJXRAY_DB) \
		$< > $@
	@echo ">>> Frames generated."

# ==============================================================================
# Step 4: Frames → Bitstream
# ==============================================================================
$(BITSTREAM): $(FRAMES)
	@echo ">>> [4/4] Frames → Bitstream..."
	$(XC7FRAMES2BIT) \
		--part_file $(PRJXRAY_DB)/$(DEVICE)/part.yaml \
		--part_name $(DEVICE) \
		--frm_file $< \
		--output_file $@
	@echo ">>> Bitstream generated: $@"

# ==============================================================================
# Program FPGA (volatile — lost on power cycle)
# ==============================================================================
program: $(BITSTREAM)
	@echo ">>> Programming FPGA (SRAM)..."
	$(LOADER) --board arty_a7_100t $<

# ==============================================================================
# Flash FPGA (non-volatile — persists across power cycles)
# ==============================================================================
flash: $(BITSTREAM)
	@echo ">>> Writing to SPI flash (non-volatile)..."
	$(LOADER) --board arty_a7_100t --write-flash $<

# ==============================================================================
# Detect JTAG
# ==============================================================================
detect:
	$(LOADER) --detect

# ==============================================================================
# Simulation (Icarus Verilog)
# ==============================================================================
sim:
	@$(SCRIPT_DIR)/run_sim.sh

sim-spi:
	@echo ">>> Running SPI Master testbench..."
	cd $(BUILD_DIR) && $(IVERILOG) -g2012 -I../$(RTL_DIR) \
		-o tb_spi_master.vvp ../$(RTL_DIR)/spi_master.v ../$(TB_DIR)/tb_spi_master.v
	cd $(BUILD_DIR) && $(VVP) tb_spi_master.vvp

sim-uart:
	@echo ">>> Running UART TX testbench..."
	cd $(BUILD_DIR) && $(IVERILOG) -g2012 -I../$(RTL_DIR) \
		-o tb_uart_tx.vvp ../$(RTL_DIR)/uart_tx.v ../$(TB_DIR)/tb_uart_tx.v
	cd $(BUILD_DIR) && $(VVP) tb_uart_tx.vvp

sim-ctrl:
	@echo ">>> Running ICM42688 Controller testbench..."
	cd $(BUILD_DIR) && $(IVERILOG) -g2012 -I../$(RTL_DIR) \
		-o tb_icm42688_ctrl.vvp \
		../$(RTL_DIR)/spi_master.v ../$(RTL_DIR)/icm42688_ctrl.v \
		../$(TB_DIR)/tb_icm42688_ctrl.v
	cd $(BUILD_DIR) && $(VVP) tb_icm42688_ctrl.vvp

sim-top:
	@echo ">>> Running integration testbench..."
	cd $(BUILD_DIR) && $(IVERILOG) -g2012 -I../$(RTL_DIR) \
		-o tb_top.vvp \
		../$(RTL_DIR)/spi_master.v ../$(RTL_DIR)/icm42688_ctrl.v \
		../$(RTL_DIR)/uart_tx.v ../$(RTL_DIR)/imu_data_stream.v \
		../$(TB_DIR)/tb_top.v
	cd $(BUILD_DIR) && $(VVP) tb_top.vvp

# ==============================================================================
# Clean
# ==============================================================================
clean:
	rm -rf $(BUILD_DIR)/*.json $(BUILD_DIR)/*.fasm $(BUILD_DIR)/*.frames \
	       $(BUILD_DIR)/*.bit $(BUILD_DIR)/*.log $(BUILD_DIR)/*.vvp \
	       $(BUILD_DIR)/*.vcd

clean-all: clean
	rm -rf $(CHIPDB_DIR)/*.bba $(CHIPDB_DIR)/*.bin

# ==============================================================================
# Help
# ==============================================================================
help:
	@echo ""
	@echo "  ICM-42688 IMU → Arty A7-100T (Open-Source Toolchain)"
	@echo "  ====================================================="
	@echo ""
	@echo "  First-time setup:"
	@echo "    make chipdb-copy   Copy pre-built chipdb (fast)"
	@echo "    make chipdb        Generate chipdb from scratch (~20 min)"
	@echo ""
	@echo "  Build flow:"
	@echo "    make               Full build: synth → PnR → bitstream"
	@echo "    make program       Program FPGA via JTAG (volatile)"
	@echo "    make flash         Write to SPI flash (non-volatile)"
	@echo "    make detect        Detect JTAG chain"
	@echo ""
	@echo "  Simulation:"
	@echo "    make sim           Run all testbenches"
	@echo "    make sim-spi       SPI master testbench only"
	@echo "    make sim-uart      UART TX testbench only"
	@echo "    make sim-ctrl      IMU controller testbench only"
	@echo "    make sim-top       Integration testbench only"
	@echo ""
	@echo "  Cleanup:"
	@echo "    make clean         Remove build artifacts"
	@echo "    make clean-all     Also remove chipdb"
	@echo ""

#!/usr/bin/env bash
# ==============================================================================
# run_sim.sh — Run all testbenches with Icarus Verilog
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
RTL_DIR="$PROJECT_DIR/rtl"
TB_DIR="$PROJECT_DIR/tb"
BUILD_DIR="$PROJECT_DIR/build"

mkdir -p "$BUILD_DIR"

TOTAL_PASS=0
TOTAL_FAIL=0
TOTAL_TESTS=0

run_tb() {
    local name="$1"
    shift
    local srcs=("$@")

    echo ""
    echo "================================================================"
    echo " Running: $name"
    echo "================================================================"

    iverilog -g2012 -I"$RTL_DIR" -o "$BUILD_DIR/${name}.vvp" "${srcs[@]}"
    output=$(vvp "$BUILD_DIR/${name}.vvp" 2>&1) || true
    echo "$output"

    # Extract pass/fail counts
    local p=$(echo "$output" | grep -oP 'PASS=\K[0-9]+' | tail -1)
    local f=$(echo "$output" | grep -oP 'FAIL=\K[0-9]+' | tail -1)
    p=${p:-0}
    f=${f:-0}

    TOTAL_PASS=$((TOTAL_PASS + p))
    TOTAL_FAIL=$((TOTAL_FAIL + f))
    TOTAL_TESTS=$((TOTAL_TESTS + p + f))
}

# --- Top Extended integration ---
run_tb "tb_top_extended" \
    "$RTL_DIR/spi_master.v" \
    "$RTL_DIR/icm42688_ctrl.v" \
    "$RTL_DIR/uart_tx.v" \
    "$RTL_DIR/imu_data_stream.v" \
    "$TB_DIR/tb_top_extended.v"

# --- SPI Master ---
run_tb "tb_spi_master" \
    "$RTL_DIR/spi_master.v" \
    "$TB_DIR/tb_spi_master.v"

# --- UART TX ---
run_tb "tb_uart_tx" \
    "$RTL_DIR/uart_tx.v" \
    "$TB_DIR/tb_uart_tx.v"

# --- ICM42688 Controller ---
run_tb "tb_icm42688_ctrl" \
    "$RTL_DIR/spi_master.v" \
    "$RTL_DIR/icm42688_ctrl.v" \
    "$TB_DIR/tb_icm42688_ctrl.v"

# --- Top integration ---
run_tb "tb_top" \
    "$RTL_DIR/spi_master.v" \
    "$RTL_DIR/icm42688_ctrl.v" \
    "$RTL_DIR/uart_tx.v" \
    "$RTL_DIR/imu_data_stream.v" \
    "$TB_DIR/tb_top.v"

# --- Summary ---
echo ""
echo "================================================================"
echo " ALL TESTBENCHES COMPLETE"
echo " Total: $TOTAL_TESTS  |  Pass: $TOTAL_PASS  |  Fail: $TOTAL_FAIL"
echo "================================================================"

if [ "$TOTAL_FAIL" -eq 0 ]; then
    echo " ✅ ALL $TOTAL_PASS TESTS PASSED"
else
    echo " ❌ $TOTAL_FAIL TEST(S) FAILED"
    exit 1
fi

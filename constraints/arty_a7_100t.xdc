## =============================================================================
## Arty A7-100T XDC — nextpnr-xilinx compatible
## ICM-42688-P IMU Streaming Project
## =============================================================================

## --- System Clock (100 MHz) ---
set_property PACKAGE_PIN E3 [get_ports {clk}]
set_property IOSTANDARD LVCMOS33 [get_ports {clk}]

## --- Reset Button (BTN0, active-low) ---
set_property PACKAGE_PIN C2 [get_ports {resetn}]
set_property IOSTANDARD LVCMOS33 [get_ports {resetn}]

## --- Pmod JA: SPI to ICM-42688-P ---
set_property PACKAGE_PIN G13 [get_ports {ja_pin1}]
set_property IOSTANDARD LVCMOS33 [get_ports {ja_pin1}]

set_property PACKAGE_PIN B11 [get_ports {ja_pin2}]
set_property IOSTANDARD LVCMOS33 [get_ports {ja_pin2}]

set_property PACKAGE_PIN A11 [get_ports {ja_pin3}]
set_property IOSTANDARD LVCMOS33 [get_ports {ja_pin3}]

set_property PACKAGE_PIN D12 [get_ports {ja_pin4}]
set_property IOSTANDARD LVCMOS33 [get_ports {ja_pin4}]

## --- UART TX (USB-UART bridge) ---
set_property PACKAGE_PIN D10 [get_ports {uart_rxd_out}]
set_property IOSTANDARD LVCMOS33 [get_ports {uart_rxd_out}]

## --- LEDs ---
set_property PACKAGE_PIN H17 [get_ports {led[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[0]}]

set_property PACKAGE_PIN K15 [get_ports {led[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[1]}]

set_property PACKAGE_PIN J13 [get_ports {led[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[2]}]

set_property PACKAGE_PIN J14 [get_ports {led[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[3]}]

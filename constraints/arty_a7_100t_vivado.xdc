## =============================================================================
## Arty A7-100T XDC Constraints
## ICM-42688-P IMU Streaming Project
## =============================================================================

## --- System Clock (100 MHz) ---
set_property -dict { PACKAGE_PIN E3 IOSTANDARD LVCMOS33 } [get_ports { clk }]
create_clock -add -name sys_clk_pin -period 10.00 -waveform {0 5} [get_ports { clk }]

## --- Reset Button (active-low) ---
set_property -dict { PACKAGE_PIN C2 IOSTANDARD LVCMOS33 } [get_ports { resetn }]

## =============================================================================
## Pmod JA - SPI to ICM-42688-P
## Standard Pmod (200Ω series resistors on board, limit to ~1MHz SPI)
##
## Pin mapping (Pmod JA, row 1):
##   JA1  (G13) -> IMU MOSI
##   JA2  (B11) -> IMU MISO
##   JA3  (A11) -> IMU SCK
##   JA4  (D12) -> IMU CS_N
## =============================================================================
set_property -dict { PACKAGE_PIN G13 IOSTANDARD LVCMOS33 } [get_ports { ja_pin1 }]
set_property -dict { PACKAGE_PIN B11 IOSTANDARD LVCMOS33 } [get_ports { ja_pin2 }]
set_property -dict { PACKAGE_PIN A11 IOSTANDARD LVCMOS33 } [get_ports { ja_pin3 }]
set_property -dict { PACKAGE_PIN D12 IOSTANDARD LVCMOS33 } [get_ports { ja_pin4 }]

## =============================================================================
## UART - USB-UART bridge (FTDI FT2232HQ)
## uart_rxd_out -> IO4 (D10) -> goes to FTDI RXD
## =============================================================================
set_property -dict { PACKAGE_PIN D10 IOSTANDARD LVCMOS33 } [get_ports { uart_rxd_out }]

## =============================================================================
## LEDs
## LD0 = init done (green)
## LD1 = data streaming (blue)
## LD2 = error (red)
## LD3 = unused
## =============================================================================
set_property -dict { PACKAGE_PIN H17 IOSTANDARD LVCMOS33 } [get_ports { led[0] }]
set_property -dict { PACKAGE_PIN K15 IOSTANDARD LVCMOS33 } [get_ports { led[1] }]
set_property -dict { PACKAGE_PIN J13 IOSTANDARD LVCMOS33 } [get_ports { led[2] }]
set_property -dict { PACKAGE_PIN J14 IOSTANDARD LVCMOS33 } [get_ports { led[3] }]

## =============================================================================
## Timing Constraints
## =============================================================================
## SPI clock (derived, 1MHz)
## No timing critical since SPI is slow relative to 100MHz
set_false_path -from [get_ports ja_pin2]  ;# SPI MISO - async input
set_false_path -to   [get_ports ja_pin1]  ;# SPI MOSI
set_false_path -to   [get_ports ja_pin3]  ;# SPI SCK
set_false_path -to   [get_ports ja_pin4]  ;# SPI CS_N
set_false_path -to   [get_ports uart_rxd_out]
set_false_path -to   [get_ports led[*]]

## =============================================================================
## I/O Drive Strength
## =============================================================================
set_property DRIVE 8 [get_ports ja_pin1]
set_property DRIVE 8 [get_ports ja_pin3]
set_property DRIVE 8 [get_ports ja_pin4]
set_property DRIVE 4 [get_ports uart_rxd_out]
set_property DRIVE 4 [get_ports led[*]]

## =============================================================================
## Misc
## =============================================================================
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]

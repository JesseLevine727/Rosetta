# PYNQ-Z1 (rev C) constraints for the Rosetta harness.
# Pin locations from Digilent's PYNQ-Z1_C.xdc master file.
# This file is read with PROCESSING_ORDER LATE so the PS7 / MMCM clocks exist.

## 125 MHz board clock (independent oscillator, used only as a frequency reference)
set_property -dict { PACKAGE_PIN H16 IOSTANDARD LVCMOS33 } [get_ports sysclk]
create_clock -name sysclk -period 8.000 [get_ports sysclk]

## LEDs LD0..LD3
set_property -dict { PACKAGE_PIN R14 IOSTANDARD LVCMOS33 } [get_ports {led[0]}]
set_property -dict { PACKAGE_PIN P14 IOSTANDARD LVCMOS33 } [get_ports {led[1]}]
set_property -dict { PACKAGE_PIN N16 IOSTANDARD LVCMOS33 } [get_ports {led[2]}]
set_property -dict { PACKAGE_PIN M14 IOSTANDARD LVCMOS33 } [get_ports {led[3]}]

## PMOD JA = uo_out[0..7]   (pins 1-4 = ja[0..3], pins 7-10 = ja[4..7])
set_property -dict { PACKAGE_PIN Y18 IOSTANDARD LVCMOS33 } [get_ports {ja[0]}]
set_property -dict { PACKAGE_PIN Y19 IOSTANDARD LVCMOS33 } [get_ports {ja[1]}]
set_property -dict { PACKAGE_PIN Y16 IOSTANDARD LVCMOS33 } [get_ports {ja[2]}]
set_property -dict { PACKAGE_PIN Y17 IOSTANDARD LVCMOS33 } [get_ports {ja[3]}]
set_property -dict { PACKAGE_PIN U18 IOSTANDARD LVCMOS33 } [get_ports {ja[4]}]
set_property -dict { PACKAGE_PIN U19 IOSTANDARD LVCMOS33 } [get_ports {ja[5]}]
set_property -dict { PACKAGE_PIN W18 IOSTANDARD LVCMOS33 } [get_ports {ja[6]}]
set_property -dict { PACKAGE_PIN W19 IOSTANDARD LVCMOS33 } [get_ports {ja[7]}]

## PMOD JB = uio[0..7]      (pins 1-4 = jb[0..3], pins 7-10 = jb[4..7])
set_property -dict { PACKAGE_PIN W14 IOSTANDARD LVCMOS33 } [get_ports {jb[0]}]
set_property -dict { PACKAGE_PIN Y14 IOSTANDARD LVCMOS33 } [get_ports {jb[1]}]
set_property -dict { PACKAGE_PIN T11 IOSTANDARD LVCMOS33 } [get_ports {jb[2]}]
set_property -dict { PACKAGE_PIN T10 IOSTANDARD LVCMOS33 } [get_ports {jb[3]}]
set_property -dict { PACKAGE_PIN V16 IOSTANDARD LVCMOS33 } [get_ports {jb[4]}]
set_property -dict { PACKAGE_PIN W16 IOSTANDARD LVCMOS33 } [get_ports {jb[5]}]
set_property -dict { PACKAGE_PIN V12 IOSTANDARD LVCMOS33 } [get_ports {jb[6]}]
set_property -dict { PACKAGE_PIN W13 IOSTANDARD LVCMOS33 } [get_ports {jb[7]}]

## Arduino/chipKIT digital IO0..IO7 = ui_in[0..7]
set_property -dict { PACKAGE_PIN T14 IOSTANDARD LVCMOS33 } [get_ports {ck_io[0]}]
set_property -dict { PACKAGE_PIN U12 IOSTANDARD LVCMOS33 } [get_ports {ck_io[1]}]
set_property -dict { PACKAGE_PIN U13 IOSTANDARD LVCMOS33 } [get_ports {ck_io[2]}]
set_property -dict { PACKAGE_PIN V13 IOSTANDARD LVCMOS33 } [get_ports {ck_io[3]}]
set_property -dict { PACKAGE_PIN V15 IOSTANDARD LVCMOS33 } [get_ports {ck_io[4]}]
set_property -dict { PACKAGE_PIN T15 IOSTANDARD LVCMOS33 } [get_ports {ck_io[5]}]
set_property -dict { PACKAGE_PIN R16 IOSTANDARD LVCMOS33 } [get_ports {ck_io[6]}]
set_property -dict { PACKAGE_PIN U17 IOSTANDARD LVCMOS33 } [get_ports {ck_io[7]}]

## Weak pulls so undriven pins read a defined level:
## ui inputs and uo outputs pull down; uio pulls up (open-drain idle high).
set_property PULLDOWN true [get_ports {ck_io[*]}]
set_property PULLDOWN true [get_ports {ja[*]}]
set_property PULLUP   true [get_ports {jb[*]}]
set_property DRIVE 8      [get_ports {ck_io[*] ja[*] jb[*]}]
set_property SLEW SLOW    [get_ports {ck_io[*] ja[*] jb[*]}]

## Header pins are asynchronous to the FPGA (the chip has its own input
## synchronisers); no board-level IO timing is claimed.
set_false_path -from [get_ports {ck_io[*] ja[*] jb[*]}]
set_false_path -to   [get_ports {ck_io[*] ja[*] jb[*] led[*]}]

## Frequency-meter references are only crossed through 2-flop synchronisers
## (rh_freqmeter, ASYNC_REG); treat them as asynchronous to everything else.
## The async far ends (rh_async_uart on sysclk, rh_line on the 200 MHz MMCM
## clock made from sysclk) are asynchronous to the chip by design: their
## outputs enter the chip's synchronisers like a real remote device's.
set_clock_groups -asynchronous -group [get_clocks -include_generated_clocks sysclk]
set_clock_groups -asynchronous -group [get_clocks clk_fpga_0]

## AXI Quad SPI marks its pin registers IOB=TRUE; here they drive fabric
## logic (the chip's host port), not pads.
set_property IOB FALSE [get_cells -quiet -hierarchical -filter {IOB == TRUE && NAME =~ "*/spi/U0/*"}]

## Smaller bitstream, faster download
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]

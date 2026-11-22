# Rosetta FPGA harness: Vivado project-mode build (block design + module reference).
# Invoked by build.sh:
#   vivado -mode batch -source tcl/build.tcl -tclargs <chip_filelist> <out_dir> <jobs> <sck_ratio> <shim.v> [defines]
# defines: space-separated NAME or NAME=VALUE Verilog defines for every source (e.g. ROSETTA_SRAM_BEHAV)
# Produces <out_dir>/rosetta.bit, <out_dir>/rosetta.hwh and reports in <out_dir>/reports/.

set script_dir [file dirname [file normalize [info script]]]
set root       [file dirname $script_dir]
lassign $argv filelist out_dir jobs sck_ratio shim defines
if {![info exists defines]} { set defines "" }
set out_dir  [file normalize $out_dir]
set part     xc7z020clg400-1
set proj_dir [file join $out_dir vivado]
set rpt_dir  [file join $out_dir reports]
file mkdir $out_dir $rpt_dir

proc ipvlnv {name} {
    set defs [get_ipdefs -all xilinx.com:ip:${name}:*]
    if {[llength $defs] == 0} { error "IP $name not found in catalog" }
    return [lindex [lsort -dictionary $defs] end]
}

# ------------------------------------------------------------------ project
create_project -force rosetta $proj_dir -part $part
set_property target_language Verilog [current_project]

set chip_files {}
set fh [open $filelist r]
foreach line [split [read $fh] "\n"] {
    set l [string trim $line]
    if {$l ne ""} { lappend chip_files [file normalize $l] }
}
close $fh
puts "ROSETTA: chip sources: $chip_files"
add_files -norecurse -fileset sources_1 $chip_files
foreach f $chip_files {
    switch -glob -- $f {
        *.sv  { set_property file_type SystemVerilog [get_files $f] }
        *.svh { set_property file_type {Verilog Header} [get_files $f] }
        *.vh  { set_property file_type {Verilog Header} [get_files $f] }
    }
}
set inc_dirs {}
foreach f $chip_files { lappend inc_dirs [file dirname $f] }
set_property include_dirs [lsort -unique $inc_dirs] [get_filesets sources_1]
if {[string trim $defines] ne ""} {
    puts "ROSETTA: verilog defines: $defines"
    set_property verilog_define [string trim $defines] [get_filesets sources_1]
}
add_files -norecurse -fileset sources_1 [list $shim {*}[glob [file join $root hdl *.v]]]
add_files -norecurse -fileset constrs_1 [file join $root xdc pynq_z1_rosetta.xdc]
set_property PROCESSING_ORDER LATE [get_files pynq_z1_rosetta.xdc]
update_compile_order -fileset sources_1

# ------------------------------------------------------------------ block design
create_bd_design rosetta_bd

# Zynq PS7 with the PYNQ-Z1 board settings; FCLK0 = 100 MHz, M_AXI_GP0 only
set ps7 [create_bd_cell -type ip -vlnv [ipvlnv processing_system7] ps7]
source [file join $script_dir ps7_pynq_z1.tcl]
if {[catch {set_property -dict $PYNQ_Z1_PS7_PRESET $ps7} err]} {
    puts "ROSETTA: bulk PS7 preset failed ($err); applying parameters one by one"
    foreach {k v} $PYNQ_Z1_PS7_PRESET {
        if {[catch {set_property $k $v $ps7} e]} { puts "ROSETTA:   skipped $k" }
    }
}
set_property -dict [list \
    CONFIG.PCW_USE_M_AXI_GP0            {1} \
    CONFIG.PCW_USE_M_AXI_GP1            {0} \
    CONFIG.PCW_USE_S_AXI_HP0            {0} \
    CONFIG.PCW_USE_FABRIC_INTERRUPT     {0} \
    CONFIG.PCW_EN_CLK0_PORT             {1} \
    CONFIG.PCW_EN_RST0_PORT             {1} \
    CONFIG.PCW_FPGA_FCLK0_ENABLE        {1} \
    CONFIG.PCW_FCLK0_PERIPHERAL_CLKSRC  {IO PLL} \
    CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ {100} \
    CONFIG.PCW_EN_CLK1_PORT {0} CONFIG.PCW_EN_CLK2_PORT {0} CONFIG.PCW_EN_CLK3_PORT {0} \
    CONFIG.PCW_FPGA_FCLK1_ENABLE {0} CONFIG.PCW_FPGA_FCLK2_ENABLE {0} CONFIG.PCW_FPGA_FCLK3_ENABLE {0} \
] $ps7
apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 \
    -config {make_external "FIXED_IO, DDR" apply_board_preset "0" Master "Disable" Slave "Disable"} $ps7

# MMCM: 100 MHz FCLK0 -> 60 MHz (clk_out1 via BUFG for harness/AXI, clk_out2
# unbuffered into the harness BUFGCE that makes the gateable chip clock)
set clkw [create_bd_cell -type ip -vlnv [ipvlnv clk_wiz] clk_wiz]
connect_bd_net [get_bd_pins ps7/FCLK_CLK0] [get_bd_pins clk_wiz/clk_in1]
set_property -dict [list \
    CONFIG.PRIM_SOURCE                {No_buffer} \
    CONFIG.PRIM_IN_FREQ               {100.000} \
    CONFIG.CLKOUT1_REQUESTED_OUT_FREQ {60.000} \
    CONFIG.CLKOUT2_USED               {true} \
    CONFIG.CLKOUT2_REQUESTED_OUT_FREQ {60.000} \
    CONFIG.CLKOUT2_DRIVES             {No_buffer} \
    CONFIG.USE_RESET                  {false} \
    CONFIG.USE_LOCKED                 {true} \
] $clkw

set rst [create_bd_cell -type ip -vlnv [ipvlnv proc_sys_reset] rst60]
connect_bd_net [get_bd_pins clk_wiz/clk_out1]  [get_bd_pins rst60/slowest_sync_clk]
connect_bd_net [get_bd_pins ps7/FCLK_RESET0_N] [get_bd_pins rst60/ext_reset_in]
connect_bd_net [get_bd_pins clk_wiz/locked]    [get_bd_pins rst60/dcm_locked]

# AXI interconnect: 1 master (PS7 GP0) -> 28 slaves, all on the 60 MHz clock
# (21 AXI GPIOs, AXI Quad SPI, capture BRAM controller, 3 trace BRAM
# controllers, the waveform-player and async-line BRAM controllers)
set nslaves 28
set ic [create_bd_cell -type ip -vlnv [ipvlnv axi_interconnect] axi_ic]
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI $nslaves] $ic
connect_bd_intf_net [get_bd_intf_pins ps7/M_AXI_GP0] [get_bd_intf_pins axi_ic/S00_AXI]
set clk60_pins [list ps7/M_AXI_GP0_ACLK axi_ic/ACLK axi_ic/S00_ACLK]
set rstn_pins  [list axi_ic/S00_ARESETN]
for {set i 0} {$i < $nslaves} {incr i} {
    lappend clk60_pins [format axi_ic/M%02d_ACLK $i]
    lappend rstn_pins  [format axi_ic/M%02d_ARESETN $i]
}
connect_bd_net [get_bd_pins rst60/interconnect_aresetn] [get_bd_pins axi_ic/ARESETN]

# AXI GPIOs: name ch1_is_output ch2_is_output ch1_default ch2_default base_address
# gpio_ctrl's default leaves the chip clock stopped (rst_n=0, ena=1) until
# load() or board_server.py takes over, so the chip gets no edges before them.
set gpios {
    gpio_ctrl  1 0 0x00000002 0x00000000 0x41200000
    gpio_pins  1 0 0x00000000 0x00000000 0x41210000
    gpio_route 1 1 0x00000000 0x00000007 0x41220000
    gpio_loop  1 1 0x00000000 0x00000000 0x41230000
    gpio_trig  1 1 0x00000000 0x00000000 0x41240000
    gpio_cap   1 0 0x00000000 0x00000000 0x41250000
    gpio_mon   0 0 0x00000000 0x00000000 0x41260000
    gpio_freq  0 0 0x00000000 0x00000000 0x41270000
    gpio_ls    1 0 0x00000000 0x00000000 0x41280000
    gpio_run   1 1 0x00000000 0x00000000 0x41290000
    gpio_host  1 0 0x00000000 0x00000000 0x412A0000
    gpio_net   1 1 0x00000000 0x00000000 0x412B0000
    gpio_skew  1 1 0x00000000 0x00000000 0x412C0000
    gpio_fault 1 1 0x00000000 0x00000000 0x412D0000
    gpio_cyc   0 0 0x00000000 0x00000000 0x412E0000
    gpio_async 1 1 0x00000000 0x00000000 0x412F0000
    gpio_wave  1 0 0x00000000 0x00000000 0x41300000
    gpio_chmap 1 1 0x00000000 0x00000000 0x41310000
    gpio_pull  1 0 0x00000000 0x00000000 0x41320000
    gpio_can   1 1 0x00000000 0x00000000 0x41330000
    gpio_line  1 0 0x000007FE 0x00000000 0x41340000
}
set mi 0
foreach {name o1 o2 d1 d2 base} $gpios {
    set g [create_bd_cell -type ip -vlnv [ipvlnv axi_gpio] $name]
    set_property -dict [list \
        CONFIG.C_GPIO_WIDTH {32} CONFIG.C_IS_DUAL {1} CONFIG.C_GPIO2_WIDTH {32} \
        CONFIG.C_ALL_OUTPUTS $o1 CONFIG.C_ALL_INPUTS [expr {!$o1}] \
        CONFIG.C_ALL_OUTPUTS_2 $o2 CONFIG.C_ALL_INPUTS_2 [expr {!$o2}] \
        CONFIG.C_DOUT_DEFAULT $d1 CONFIG.C_DOUT_DEFAULT_2 $d2 \
    ] $g
    connect_bd_intf_net [get_bd_intf_pins [format axi_ic/M%02d_AXI $mi]] [get_bd_intf_pins $name/S_AXI]
    lappend clk60_pins $name/s_axi_aclk
    lappend rstn_pins  $name/s_axi_aresetn
    set addr($name) [list $name/S_AXI/Reg $base 64K]
    incr mi
}

# AXI Quad SPI, standard SPI mode, master; SCK = 60 MHz / sck_ratio
set spi [create_bd_cell -type ip -vlnv [ipvlnv axi_quad_spi] spi]
set_property -dict [list \
    CONFIG.C_SPI_MODE {0} CONFIG.C_SCK_RATIO $sck_ratio CONFIG.C_NUM_SS_BITS {1} \
    CONFIG.C_FIFO_DEPTH {256} CONFIG.C_USE_STARTUP {0} CONFIG.C_USE_STARTUP_INT {0} \
    CONFIG.C_TYPE_OF_AXI4_INTERFACE {0} CONFIG.C_XIP_MODE {0} \
] $spi
connect_bd_intf_net [get_bd_intf_pins [format axi_ic/M%02d_AXI $mi]] [get_bd_intf_pins spi/AXI_LITE]
lappend clk60_pins spi/s_axi_aclk spi/ext_spi_clk
lappend rstn_pins  spi/s_axi_aresetn
set addr(spi) [list spi/AXI_LITE/Reg 0x41E00000 64K]
incr mi
set one [create_bd_cell -type ip -vlnv [ipvlnv xlconstant] const_one]
set_property -dict [list CONFIG.CONST_VAL {1} CONFIG.CONST_WIDTH {1}] $one
connect_bd_net [get_bd_pins const_one/dout] [get_bd_pins spi/ss_i]

# Capture buffer: AXI BRAM controller (port A) + true dual-port BRAM (port B = harness)
set bc [create_bd_cell -type ip -vlnv [ipvlnv axi_bram_ctrl] cap_bram_ctrl]
set_property -dict [list CONFIG.PROTOCOL {AXI4LITE} CONFIG.SINGLE_PORT_BRAM {1} CONFIG.DATA_WIDTH {32}] $bc
connect_bd_intf_net [get_bd_intf_pins [format axi_ic/M%02d_AXI $mi]] [get_bd_intf_pins cap_bram_ctrl/S_AXI]
lappend clk60_pins cap_bram_ctrl/s_axi_aclk
lappend rstn_pins  cap_bram_ctrl/s_axi_aresetn
set addr(cap_bram_ctrl) [list cap_bram_ctrl/S_AXI/Mem0 0x40000000 64K]
incr mi
set bm [create_bd_cell -type ip -vlnv [ipvlnv blk_mem_gen] cap_bram]
set_property -dict [list CONFIG.Memory_Type {True_Dual_Port_RAM} CONFIG.use_bram_block {BRAM_Controller} \
    CONFIG.Enable_B {Use_ENB_Pin} CONFIG.Use_RSTB_Pin {true}] $bm
connect_bd_intf_net [get_bd_intf_pins cap_bram_ctrl/BRAM_PORTA] [get_bd_intf_pins cap_bram/BRAM_PORTA]

# Change-trace buffers: three 32-bit words per entry, each an AXI BRAM
# controller + true dual-port BRAM like the capture buffer
set trc_base {0x42000000 0x42010000 0x42020000}
for {set t 0} {$t < 3} {incr t} {
    set bc [create_bd_cell -type ip -vlnv [ipvlnv axi_bram_ctrl] trc${t}_bram_ctrl]
    set_property -dict [list CONFIG.PROTOCOL {AXI4LITE} CONFIG.SINGLE_PORT_BRAM {1} CONFIG.DATA_WIDTH {32}] $bc
    connect_bd_intf_net [get_bd_intf_pins [format axi_ic/M%02d_AXI $mi]] [get_bd_intf_pins trc${t}_bram_ctrl/S_AXI]
    lappend clk60_pins trc${t}_bram_ctrl/s_axi_aclk
    lappend rstn_pins  trc${t}_bram_ctrl/s_axi_aresetn
    set addr(trc${t}_bram_ctrl) [list trc${t}_bram_ctrl/S_AXI/Mem0 [lindex $trc_base $t] 64K]
    incr mi
    set bm [create_bd_cell -type ip -vlnv [ipvlnv blk_mem_gen] trc${t}_bram]
    set_property -dict [list CONFIG.Memory_Type {True_Dual_Port_RAM} CONFIG.use_bram_block {BRAM_Controller} \
        CONFIG.Enable_B {Use_ENB_Pin} CONFIG.Use_RSTB_Pin {true}] $bm
    connect_bd_intf_net [get_bd_intf_pins trc${t}_bram_ctrl/BRAM_PORTA] [get_bd_intf_pins trc${t}_bram/BRAM_PORTA]
}

# Waveform-player queue (lockstep stimulus) and async-line event buffer: the
# processor writes through AXI BRAM controllers, the harness reads port B
# (the line buffer's port B runs on the harness's 200 MHz line clock)
foreach {nm base} {wave 0x42030000 line 0x42040000} {
    set bc [create_bd_cell -type ip -vlnv [ipvlnv axi_bram_ctrl] ${nm}_bram_ctrl]
    set_property -dict [list CONFIG.PROTOCOL {AXI4LITE} CONFIG.SINGLE_PORT_BRAM {1} CONFIG.DATA_WIDTH {32}] $bc
    connect_bd_intf_net [get_bd_intf_pins [format axi_ic/M%02d_AXI $mi]] [get_bd_intf_pins ${nm}_bram_ctrl/S_AXI]
    lappend clk60_pins ${nm}_bram_ctrl/s_axi_aclk
    lappend rstn_pins  ${nm}_bram_ctrl/s_axi_aresetn
    set addr(${nm}_bram_ctrl) [list ${nm}_bram_ctrl/S_AXI/Mem0 $base 64K]
    incr mi
    set bm [create_bd_cell -type ip -vlnv [ipvlnv blk_mem_gen] ${nm}_bram]
    set_property -dict [list CONFIG.Memory_Type {True_Dual_Port_RAM} CONFIG.use_bram_block {BRAM_Controller} \
        CONFIG.Enable_B {Use_ENB_Pin} CONFIG.Use_RSTB_Pin {true}] $bm
    connect_bd_intf_net [get_bd_intf_pins ${nm}_bram_ctrl/BRAM_PORTA] [get_bd_intf_pins ${nm}_bram/BRAM_PORTA]
}

# The harness (module reference; instantiates rosetta_chip -> the TT top)
set h [create_bd_cell -type module -reference rosetta_harness harness]
connect_bd_intf_net [get_bd_intf_pins harness/CAP_BRAM] [get_bd_intf_pins cap_bram/BRAM_PORTB]
for {set t 0} {$t < 3} {incr t} {
    connect_bd_intf_net [get_bd_intf_pins harness/TRC${t}_BRAM] [get_bd_intf_pins trc${t}_bram/BRAM_PORTB]
}
connect_bd_intf_net [get_bd_intf_pins harness/WAVE_BRAM] [get_bd_intf_pins wave_bram/BRAM_PORTB]
connect_bd_intf_net [get_bd_intf_pins harness/LINE_BRAM] [get_bd_intf_pins line_bram/BRAM_PORTB]
lappend clk60_pins harness/clk60
lappend rstn_pins  harness/aresetn
connect_bd_net [get_bd_pins clk_wiz/clk_out2] [get_bd_pins harness/clk60_raw]
connect_bd_net [get_bd_pins ps7/FCLK_CLK0]    [get_bd_pins harness/ref_fclk]
connect_bd_net [get_bd_pins clk_wiz/locked]   [get_bd_pins harness/mmcm_locked]

foreach {gpio pin harness_port} {
    gpio_ctrl  gpio_io_o   ctrl
    gpio_ctrl  gpio2_io_i  status
    gpio_pins  gpio_io_o   drive
    gpio_pins  gpio2_io_i  chip_out
    gpio_route gpio_io_o   src_sel
    gpio_route gpio2_io_o  route2
    gpio_loop  gpio_io_o   loop_lo
    gpio_loop  gpio2_io_o  loop_hi
    gpio_trig  gpio_io_o   trig_mask
    gpio_trig  gpio2_io_o  trig_value
    gpio_cap   gpio_io_o   cap_cfg
    gpio_cap   gpio2_io_i  cap_status
    gpio_mon   gpio_io_i   pads
    gpio_mon   gpio2_io_i  chip_cycles
    gpio_freq  gpio_io_i   freq_sys
    gpio_freq  gpio2_io_i  freq_fclk
    gpio_ls    gpio_io_o   ls_ctrl
    gpio_ls    gpio2_io_i  ls_status
    gpio_run   gpio_io_o   run_n
    gpio_run   gpio2_io_o  host_cmd
    gpio_host  gpio_io_o   host_data
    gpio_host  gpio2_io_i  host_rdata
    gpio_net   gpio_io_o   nets
    gpio_net   gpio2_io_o  trace_rp
    gpio_skew  gpio_io_o   skew_lo
    gpio_skew  gpio2_io_o  skew_hi
    gpio_fault gpio_io_o   fault_off
    gpio_fault gpio2_io_o  fault_len
    gpio_cyc   gpio_io_i   cyc_lo
    gpio_cyc   gpio2_io_i  cyc_hi
    gpio_async gpio_io_o   async_inc
    gpio_async gpio2_io_o  async_cmd
    gpio_wave  gpio_io_o   wave_wp
    gpio_wave  gpio2_io_i  wave_status
    gpio_chmap gpio_io_o   chmap_lo
    gpio_chmap gpio2_io_o  chmap_hi
    gpio_pull  gpio_io_o   pulls
    gpio_pull  gpio2_io_i  contention
    gpio_can   gpio_io_o   can_cfg
    gpio_can   gpio2_io_o  can_masks
    gpio_line  gpio_io_o   line_ctrl
    gpio_line  gpio2_io_i  line_status
} {
    connect_bd_net [get_bd_pins $gpio/$pin] [get_bd_pins harness/$harness_port]
}
connect_bd_net [get_bd_pins spi/sck_o]  [get_bd_pins harness/spi_sck]
connect_bd_net [get_bd_pins spi/io0_o]  [get_bd_pins harness/spi_mosi]
connect_bd_net [get_bd_pins spi/ss_o]   [get_bd_pins harness/spi_ss_n]
connect_bd_net [get_bd_pins harness/spi_miso] [get_bd_pins spi/io1_i]

connect_bd_net [get_bd_pins clk_wiz/clk_out1] {*}[lmap p $clk60_pins {get_bd_pins $p}]
connect_bd_net [get_bd_pins rst60/peripheral_aresetn] {*}[lmap p $rstn_pins {get_bd_pins $p}]

# Board ports
create_bd_port -dir I -type clk -freq_hz 125000000 sysclk
connect_bd_net [get_bd_ports sysclk] [get_bd_pins harness/sysclk]
create_bd_port -dir IO -from 7 -to 0 ck_io
create_bd_port -dir IO -from 7 -to 0 ja
create_bd_port -dir IO -from 7 -to 0 jb
create_bd_port -dir O  -from 3 -to 0 led
connect_bd_net [get_bd_ports ck_io] [get_bd_pins harness/ck_io]
connect_bd_net [get_bd_ports ja]    [get_bd_pins harness/ja]
connect_bd_net [get_bd_ports jb]    [get_bd_pins harness/jb]
connect_bd_net [get_bd_ports led]   [get_bd_pins harness/led]

# Address map (fixed, documented in README)
foreach k [array names addr] {
    lassign $addr($k) seg base range
    assign_bd_address -offset $base -range $range \
        -target_address_space [get_bd_addr_spaces ps7/Data] [get_bd_addr_segs $seg] -force
}

validate_bd_design
save_bd_design
if {[info exists ::env(ROSETTA_BD_ONLY)]} { puts "ROSETTA: BD-only run done"; return }
set bd_file [get_files rosetta_bd.bd]
set_property synth_checkpoint_mode None $bd_file
generate_target all $bd_file
add_files -norecurse [make_wrapper -files $bd_file -top]
set_property top rosetta_bd_wrapper [current_fileset]
update_compile_order -fileset sources_1

# Record the MMCM configuration chosen by the clocking wizard
set fh [open [file join $rpt_dir mmcm_settings.txt] w]
foreach p {PRIM_IN_FREQ CLKOUT1_REQUESTED_OUT_FREQ CLKOUT1_JITTER CLKOUT2_REQUESTED_OUT_FREQ \
           MMCM_DIVCLK_DIVIDE MMCM_CLKFBOUT_MULT_F MMCM_CLKOUT0_DIVIDE_F MMCM_CLKOUT1_DIVIDE \
           MMCM_CLKIN1_PERIOD PRIMITIVE} {
    puts $fh [format "%-28s %s" $p [get_property CONFIG.$p $clkw]]
}
close $fh

# ------------------------------------------------------------------ implementation
launch_runs synth_1 -jobs $jobs
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] ne "100%"} { error "ROSETTA: synthesis failed" }
launch_runs impl_1 -to_step write_bitstream -jobs $jobs
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} { error "ROSETTA: implementation failed" }

open_run impl_1
report_timing_summary -max_paths 10 -report_unconstrained -file [file join $rpt_dir timing_summary.rpt]
report_clocks       -file [file join $rpt_dir clocks.rpt]
report_clock_interaction -file [file join $rpt_dir clock_interaction.rpt]
report_utilization  -file [file join $rpt_dir utilization.rpt]
report_utilization -hierarchical -hierarchical_depth 2 -file [file join $rpt_dir utilization_hier.rpt]
report_utilization -hierarchical -hierarchical_depth 6 -file [file join $rpt_dir utilization_hier6.rpt]
report_drc          -file [file join $rpt_dir drc.rpt]
report_cdc          -file [file join $rpt_dir cdc.rpt]

# Per-clock setup/hold slack, machine readable
set fh [open [file join $rpt_dir clock_slack.txt] w]
foreach clk [get_clocks] {
    set sp [lindex [get_timing_paths -to $clk -setup -max_paths 1 -nworst 1] 0]
    set hp [lindex [get_timing_paths -to $clk -hold  -max_paths 1 -nworst 1] 0]
    set ws [expr {$sp eq "" ? "n/a" : [get_property SLACK $sp]}]
    set wh [expr {$hp eq "" ? "n/a" : [get_property SLACK $hp]}]
    puts $fh [format "%-40s period=%-8s WNS=%-8s WHS=%s" $clk [get_property PERIOD $clk] $ws $wh]
}
close $fh

set bit [lindex [glob [file join $proj_dir rosetta.runs impl_1 *.bit]] 0]
set hwh [lindex [glob [file join $proj_dir rosetta.gen sources_1 bd rosetta_bd hw_handoff rosetta_bd.hwh]] 0]
file copy -force $bit [file join $out_dir rosetta.bit]
file copy -force $hwh [file join $out_dir rosetta.hwh]
puts "ROSETTA: wrote [file join $out_dir rosetta.bit] and rosetta.hwh"

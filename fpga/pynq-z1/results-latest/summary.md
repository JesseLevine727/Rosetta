# Rosetta FPGA regression 2026-11-22T03:56:48: PASS

Repo /home/elfo/Documents/Rosetta at a2a3acbbd014 (src dirty); bitstream d6343fa5d22eb848; total 480.8 s.

Timing: design WNS 0.359 ns, WHS 0.014 ns; chip clock {'period_ns': '16.667', 'wns_ns': '0.932', 'whs_ns': '0.026'}.
Chip clock: 60000226 Hz vs the 125 MHz oscillator, 60000000 Hz vs FCLK0, 60000007.3 Hz vs Linux time.

| Test | Verilator | Board | s (board) |
|---|---|---|---|
| test_blocks_rtl.CheckUnit.test_crc16_ccitt_round_trip | pass | pass | 15.484 |
| test_blocks_rtl.CheckUnit.test_crc_known_answers | pass | pass | 1.065 |
| test_blocks_rtl.Events.test_event_triggers | pass | pass | 1.254 |
| test_blocks_rtl.Events.test_wait_timeout_in_period_units | pass | pass | 0.657 |
| test_blocks_rtl.SharedLogic.test_alu_and_memory | pass | pass | 0.695 |
| test_blocks_rtl.SharedLogic.test_channels_between_lanes | pass | pass | 0.377 |
| test_blocks_rtl.SharedLogic.test_crossbar_internal_loopback | pass | pass | 1.237 |
| test_blocks_rtl.SharedLogic.test_event_abandons_a_field | pass | pass | 0.406 |
| test_blocks_rtl.SharedLogic.test_host_queues_full_and_empty | pass | pass | 0.897 |
| test_blocks_rtl.SharedLogic.test_memories_full_range | pass | pass | 2.128 |
| test_blocks_rtl.SharedLogic.test_raw_drive_and_sample | pass | pass | 1.213 |
| test_blocks_rtl.SharedLogic.test_step_pause_halt_restart | pass | pass | 0.73 |
| test_blocks_rtl.StuffingAndCoding.test_differential_and_invert | pass | pass | 1.186 |
| test_blocks_rtl.StuffingAndCoding.test_expect_seek_finds_pattern | pass | pass | 1.178 |
| test_blocks_rtl.StuffingAndCoding.test_manchester_round_trip | pass | pass | 1.189 |
| test_blocks_rtl.StuffingAndCoding.test_nrzi_round_trip | pass | pass | 1.199 |
| test_blocks_rtl.StuffingAndCoding.test_pulse_width_and_symbol | pass | pass | 1.201 |
| test_blocks_rtl.StuffingAndCoding.test_pulse_width_ends_at_capture_tick | pass | pass | 0.648 |
| test_blocks_rtl.StuffingAndCoding.test_stuff_violation_raises_event_3 | pass | pass | 1.157 |
| test_blocks_rtl.StuffingAndCoding.test_stuffing_equal_round_trip | pass | pass | 1.155 |
| test_blocks_rtl.StuffingAndCoding.test_stuffing_ones_round_trip | pass | pass | 1.187 |
| test_blocks_rtl.TimingUnits.test_launch_delay_on_clock | pass | pass | 1.309 |
| test_blocks_rtl.TimingUnits.test_launch_delay_on_data | pass | pass | 1.273 |
| test_blocks_rtl.TimingUnits.test_launch_delay_prescaled | pass | pass | 1.281 |
| test_blocks_rtl.TimingUnits.test_prescale_64_and_512 | pass | pass | 1.294 |
| test_can_rtl.CanOnRtl.test_ack_only_node | pass | pass | 2.195 |
| test_can_rtl.CanOnRtl.test_arbitration_1000k | pass | pass | 2.436 |
| test_can_rtl.CanOnRtl.test_arbitration_250k | pass | pass | 2.476 |
| test_can_rtl.CanOnRtl.test_arbitration_500k | pass | pass | 2.398 |
| test_can_rtl.CanOnRtl.test_listen_only_node | pass | pass | 2.255 |
| test_eth_rtl.EthOnRtl.test_receive_beyond_margin | pass | pass | 1.337 |
| test_eth_rtl.EthOnRtl.test_receive_frame_lengths | pass | pass | 0.515 |
| test_eth_rtl.EthOnRtl.test_receive_inside_margin | pass | pass | 0.629 |
| test_eth_rtl.EthOnRtl.test_transmit_loopback | pass | pass | 1.4 |
| test_i2c_rtl.I2cHoldOnRtl.test_100khz | pass | pass | 3.822 |
| test_i2c_rtl.I2cHoldOnRtl.test_400khz | pass | pass | 3.893 |
| test_i2c_rtl.I2cHoldOnRtl.test_fastest | pass | pass | 3.928 |
| test_i2c_rtl.I2cHoldOnRtl.test_no_confirm_window | pass | pass | 3.861 |
| test_i2c_rtl.I2cOnRtl.test_100khz | pass | pass | 3.841 |
| test_i2c_rtl.I2cOnRtl.test_400khz | pass | pass | 3.806 |
| test_i2c_rtl.I2cOnRtl.test_fastest | pass | pass | 3.922 |
| test_spi_rtl.SpiOnRtl.test_against_hand_written | pass | pass | 3.262 |
| test_spi_rtl.SpiOnRtl.test_mode0 | pass | pass | 3.945 |
| test_spi_rtl.SpiOnRtl.test_mode1 | pass | pass | 3.972 |
| test_spi_rtl.SpiOnRtl.test_mode2 | pass | pass | 3.961 |
| test_spi_rtl.SpiOnRtl.test_mode3 | pass | pass | 3.953 |
| test_uart_rtl.UartOnRtl.test_parity_even_and_wrong_parity | pass | pass | 1.107 |
| test_uart_rtl.UartOnRtl.test_parity_none_and_framing_error | pass | pass | 1.075 |
| test_uart_rtl.UartOnRtl.test_parity_odd_and_wrong_parity | pass | pass | 1.071 |
| test_uart_rtl.UartOnRtl.test_same_behaviour_as_hand_written | pass | pass | 1.103 |
| test_usb_rtl.UsbOnRtl.test_host_clock_fast_1p5pct | pass | pass | 2.545 |
| test_usb_rtl.UsbOnRtl.test_host_clock_nominal | pass | pass | 2.548 |
| test_usb_rtl.UsbOnRtl.test_host_clock_slow_1p5pct | pass | pass | 2.498 |

Transcripts: 62 of 62 sessions identical (1956496 pin changes compared).
Async UART sweep: pass; decodes [-5.2, 5.6] % (bit [493.219, 549.404] cycles), compiler window [-5.25, 5.2] %.
Ethernet receive margin (async source, 78.125 ps grid, 100 frames per point): pass
  dcd: board last pass 16.5 ns, first fail 17 ns; study 16.5 / 17.0 ns
  dcd_neg: board last pass 16 ns, first fail 16.5 ns; study 16.5 / 17.0 ns
  alt: board last pass 8.2 ns, first fail 8.5 ns; study 8.2 / 8.5 ns
  uniform: board last pass 8 ns, first fail 9 ns; study 8.0 / 9.0 ns
USB async host: pass; turnarounds (cycles) +0 ppm [134, 136, 83]; +15000 ppm [135, 134, 85]; -15000 ppm [132, 136, 83]
CAN async node: pass; +5000 ppm [['0x700', True, 0, 1], ['0xf0', True, 0, 1], ['0x123', True, 0, 1]]; -5000 ppm [['0x700', True, 0, 1], ['0xf0', True, 0, 1], ['0x123', True, 0, 1]]

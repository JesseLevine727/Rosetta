# Rosetta FPGA regression 2026-11-21T22:26:43: PASS

Repo /home/elfo/Documents/Rosetta at 8fc3800b3049; bitstream 63dfda4ddf786f8e; total 480.0 s.

Timing: design WNS 0.659 ns, WHS 0.017 ns; chip clock {'period_ns': '16.667', 'wns_ns': '1.884', 'whs_ns': '0.06'}.
Chip clock: 60000227 Hz vs the 125 MHz oscillator, 60000000 Hz vs FCLK0, 59999707.1 Hz vs Linux time.

| Test | Verilator | Board | s (board) |
|---|---|---|---|
| test_blocks_rtl.CheckUnit.test_crc16_ccitt_round_trip | pass | pass | 15.435 |
| test_blocks_rtl.CheckUnit.test_crc_known_answers | pass | pass | 1.08 |
| test_blocks_rtl.Events.test_event_triggers | pass | pass | 1.239 |
| test_blocks_rtl.Events.test_wait_timeout_in_period_units | pass | pass | 0.656 |
| test_blocks_rtl.SharedLogic.test_alu_and_memory | pass | pass | 0.674 |
| test_blocks_rtl.SharedLogic.test_channels_between_lanes | pass | pass | 0.371 |
| test_blocks_rtl.SharedLogic.test_crossbar_internal_loopback | pass | pass | 1.245 |
| test_blocks_rtl.SharedLogic.test_event_abandons_a_field | pass | pass | 0.37 |
| test_blocks_rtl.SharedLogic.test_host_queues_full_and_empty | pass | pass | 0.904 |
| test_blocks_rtl.SharedLogic.test_memories_full_range | pass | pass | 2.142 |
| test_blocks_rtl.SharedLogic.test_raw_drive_and_sample | pass | pass | 1.199 |
| test_blocks_rtl.SharedLogic.test_step_pause_halt_restart | pass | pass | 0.736 |
| test_blocks_rtl.StuffingAndCoding.test_differential_and_invert | pass | pass | 1.21 |
| test_blocks_rtl.StuffingAndCoding.test_expect_seek_finds_pattern | pass | pass | 1.169 |
| test_blocks_rtl.StuffingAndCoding.test_manchester_round_trip | pass | pass | 1.216 |
| test_blocks_rtl.StuffingAndCoding.test_nrzi_round_trip | pass | pass | 1.186 |
| test_blocks_rtl.StuffingAndCoding.test_pulse_width_and_symbol | pass | pass | 1.208 |
| test_blocks_rtl.StuffingAndCoding.test_pulse_width_ends_at_capture_tick | pass | pass | 0.658 |
| test_blocks_rtl.StuffingAndCoding.test_stuff_violation_raises_event_3 | pass | pass | 1.201 |
| test_blocks_rtl.StuffingAndCoding.test_stuffing_equal_round_trip | pass | pass | 1.165 |
| test_blocks_rtl.StuffingAndCoding.test_stuffing_ones_round_trip | pass | pass | 1.213 |
| test_blocks_rtl.TimingUnits.test_launch_delay_on_clock | pass | pass | 1.287 |
| test_blocks_rtl.TimingUnits.test_launch_delay_on_data | pass | pass | 1.279 |
| test_blocks_rtl.TimingUnits.test_launch_delay_prescaled | pass | pass | 1.303 |
| test_blocks_rtl.TimingUnits.test_prescale_64_and_512 | pass | pass | 1.304 |
| test_can_rtl.CanOnRtl.test_ack_only_node | pass | pass | 2.199 |
| test_can_rtl.CanOnRtl.test_arbitration_1000k | pass | pass | 2.424 |
| test_can_rtl.CanOnRtl.test_arbitration_250k | pass | pass | 2.444 |
| test_can_rtl.CanOnRtl.test_arbitration_500k | pass | pass | 2.428 |
| test_can_rtl.CanOnRtl.test_listen_only_node | pass | pass | 2.229 |
| test_eth_rtl.EthOnRtl.test_receive_beyond_margin | pass | pass | 1.334 |
| test_eth_rtl.EthOnRtl.test_receive_frame_lengths | pass | pass | 0.509 |
| test_eth_rtl.EthOnRtl.test_receive_inside_margin | pass | pass | 0.624 |
| test_eth_rtl.EthOnRtl.test_transmit_loopback | pass | pass | 1.401 |
| test_i2c_rtl.I2cHoldOnRtl.test_100khz | pass | pass | 3.839 |
| test_i2c_rtl.I2cHoldOnRtl.test_400khz | pass | pass | 3.846 |
| test_i2c_rtl.I2cHoldOnRtl.test_fastest | pass | pass | 3.945 |
| test_i2c_rtl.I2cHoldOnRtl.test_no_confirm_window | pass | pass | 3.885 |
| test_i2c_rtl.I2cOnRtl.test_100khz | pass | pass | 3.86 |
| test_i2c_rtl.I2cOnRtl.test_400khz | pass | pass | 3.824 |
| test_i2c_rtl.I2cOnRtl.test_fastest | pass | pass | 3.964 |
| test_spi_rtl.SpiOnRtl.test_against_hand_written | pass | pass | 3.331 |
| test_spi_rtl.SpiOnRtl.test_mode0 | pass | pass | 3.942 |
| test_spi_rtl.SpiOnRtl.test_mode1 | pass | pass | 3.955 |
| test_spi_rtl.SpiOnRtl.test_mode2 | pass | pass | 3.948 |
| test_spi_rtl.SpiOnRtl.test_mode3 | pass | pass | 3.951 |
| test_uart_rtl.UartOnRtl.test_parity_even_and_wrong_parity | pass | pass | 1.105 |
| test_uart_rtl.UartOnRtl.test_parity_none_and_framing_error | pass | pass | 1.084 |
| test_uart_rtl.UartOnRtl.test_parity_odd_and_wrong_parity | pass | pass | 1.078 |
| test_uart_rtl.UartOnRtl.test_same_behaviour_as_hand_written | pass | pass | 1.059 |
| test_usb_rtl.UsbOnRtl.test_host_clock_fast_1p5pct | pass | pass | 2.51 |
| test_usb_rtl.UsbOnRtl.test_host_clock_nominal | pass | pass | 2.514 |
| test_usb_rtl.UsbOnRtl.test_host_clock_slow_1p5pct | pass | pass | 2.525 |

Transcripts: 62 of 62 sessions identical (1956496 pin changes compared).
Async UART sweep: pass; decodes [-5.2, 5.6] % (bit [493.212, 549.404] cycles), compiler window [-5.25, 5.2] %.
Ethernet receive margin (async source, 78.125 ps grid, 100 frames per point): pass
  dcd: board last pass 16.5 ns, first fail 17 ns; study 16.5 / 17.0 ns
  dcd_neg: board last pass 16.5 ns, first fail 17 ns; study 16.5 / 17.0 ns
  alt: board last pass 8.2 ns, first fail 8.5 ns; study 8.2 / 8.5 ns
  uniform: board last pass 8 ns, first fail 9 ns; study 8.0 / 9.0 ns
USB async host: pass; turnarounds (cycles) +0 ppm [133, 135, 84]; +15000 ppm [134, 134, 83]; -15000 ppm [132, 132, 85]
CAN async node: pass; +5000 ppm [['0x700', True, 0, 1], ['0xf0', True, 0, 1], ['0x123', True, 0, 1]]; -5000 ppm [['0x700', True, 0, 1], ['0xf0', True, 0, 1], ['0x123', True, 0, 1]]

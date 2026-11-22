# Rosetta FPGA regression 2026-11-21T21:13:48: PASS

Repo /home/elfo/Documents/Rosetta at fba6aa46869b; bitstream ec85fcc700dc11e4; total 210.3 s.

Timing: design WNS 1.619 ns, WHS 0.027 ns; chip clock {'period_ns': '16.667', 'wns_ns': '2.287', 'whs_ns': '0.102'}.
Chip clock: 60000219 Hz vs the 125 MHz oscillator, 60000000 Hz vs FCLK0, 59999497.5 Hz vs Linux time.

| Test | Verilator | Board | s (board) |
|---|---|---|---|
| test_blocks_rtl.CheckUnit.test_crc16_ccitt_round_trip | pass | pass | 15.535 |
| test_blocks_rtl.CheckUnit.test_crc_known_answers | pass | pass | 1.051 |
| test_blocks_rtl.Events.test_event_triggers | pass | pass | 1.233 |
| test_blocks_rtl.Events.test_wait_timeout_in_period_units | pass | pass | 0.637 |
| test_blocks_rtl.SharedLogic.test_alu_and_memory | pass | pass | 0.667 |
| test_blocks_rtl.SharedLogic.test_channels_between_lanes | pass | pass | 0.366 |
| test_blocks_rtl.SharedLogic.test_crossbar_internal_loopback | pass | pass | 1.229 |
| test_blocks_rtl.SharedLogic.test_event_abandons_a_field | pass | pass | 0.365 |
| test_blocks_rtl.SharedLogic.test_host_queues_full_and_empty | pass | pass | 0.852 |
| test_blocks_rtl.SharedLogic.test_memories_full_range | pass | pass | 2.085 |
| test_blocks_rtl.SharedLogic.test_raw_drive_and_sample | pass | pass | 1.242 |
| test_blocks_rtl.SharedLogic.test_step_pause_halt_restart | pass | pass | 0.74 |
| test_blocks_rtl.StuffingAndCoding.test_differential_and_invert | pass | pass | 1.202 |
| test_blocks_rtl.StuffingAndCoding.test_expect_seek_finds_pattern | pass | pass | 1.245 |
| test_blocks_rtl.StuffingAndCoding.test_manchester_round_trip | pass | pass | 1.241 |
| test_blocks_rtl.StuffingAndCoding.test_nrzi_round_trip | pass | pass | 1.203 |
| test_blocks_rtl.StuffingAndCoding.test_pulse_width_and_symbol | pass | pass | 1.216 |
| test_blocks_rtl.StuffingAndCoding.test_pulse_width_ends_at_capture_tick | pass | pass | 0.668 |
| test_blocks_rtl.StuffingAndCoding.test_stuff_violation_raises_event_3 | pass | pass | 1.199 |
| test_blocks_rtl.StuffingAndCoding.test_stuffing_equal_round_trip | pass | pass | 1.207 |
| test_blocks_rtl.StuffingAndCoding.test_stuffing_ones_round_trip | pass | pass | 1.19 |
| test_blocks_rtl.TimingUnits.test_launch_delay_on_clock | pass | pass | 1.293 |
| test_blocks_rtl.TimingUnits.test_launch_delay_on_data | pass | pass | 1.285 |
| test_blocks_rtl.TimingUnits.test_launch_delay_prescaled | pass | pass | 1.308 |
| test_blocks_rtl.TimingUnits.test_prescale_64_and_512 | pass | pass | 1.298 |
| test_i2c_rtl.I2cHoldOnRtl.test_100khz | pass | pass | 3.912 |
| test_i2c_rtl.I2cHoldOnRtl.test_400khz | pass | pass | 3.978 |
| test_i2c_rtl.I2cHoldOnRtl.test_fastest | pass | pass | 4.032 |
| test_i2c_rtl.I2cHoldOnRtl.test_no_confirm_window | pass | pass | 3.937 |
| test_i2c_rtl.I2cOnRtl.test_100khz | pass | pass | 4.01 |
| test_i2c_rtl.I2cOnRtl.test_400khz | pass | pass | 3.93 |
| test_i2c_rtl.I2cOnRtl.test_fastest | pass | pass | 4.231 |
| test_spi_rtl.SpiOnRtl.test_against_hand_written | pass | pass | 3.284 |
| test_spi_rtl.SpiOnRtl.test_mode0 | pass | pass | 3.865 |
| test_spi_rtl.SpiOnRtl.test_mode1 | pass | pass | 3.9 |
| test_spi_rtl.SpiOnRtl.test_mode2 | pass | pass | 3.887 |
| test_spi_rtl.SpiOnRtl.test_mode3 | pass | pass | 3.917 |
| test_uart_rtl.UartOnRtl.test_parity_even_and_wrong_parity | pass | pass | 1.105 |
| test_uart_rtl.UartOnRtl.test_parity_none_and_framing_error | pass | pass | 1.106 |
| test_uart_rtl.UartOnRtl.test_parity_odd_and_wrong_parity | pass | pass | 1.094 |
| test_uart_rtl.UartOnRtl.test_same_behaviour_as_hand_written | pass | pass | 1.082 |

Transcripts: 50 of 50 sessions identical (1521248 pin changes compared).
Async UART sweep: pass; decodes [-5.2, 5.6] % (bit [493.212, 549.404] cycles), compiler window [-5.25, 5.2] %.

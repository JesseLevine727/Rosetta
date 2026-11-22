# Hardware coverage (PYNQ-Z1, lockstep board runs)

`checked by`: passing board tests whose checks rely on the feature (test_blocks_rtl COVERS, hw_coverage.SCENARIOS). `configured in`: other passing board tests whose loaded programs or host operations use it (decoded from the session transcripts).

| Block | Feature | What | On hardware | Checked by | Configured in |
|---|---|---|---|---|---|
| rosetta_cfg | `cfg.setm` | mode registers written by SETM (immediate) | yes |  | CheckUnit.crc16_ccitt_round_trip, CheckUnit.crc_known_answers, Events.event_triggers +46 |
| rosetta_cfg | `cfg.setm_hi` | upper half of a 32-bit register (SETM .hi, CRCPOLY) | yes | CheckUnit.crc_known_answers, EthOnRtl.transmit_loopback | EthOnRtl.receive_beyond_margin, EthOnRtl.receive_frame_lengths, EthOnRtl.receive_inside_margin |
| rosetta_cfg | `cfg.setm_reg` | SETM value from a register | yes | CheckUnit.crc_known_answers |  |
| rosetta_cfg | `cfg.pulse` | PULSE register (pulse-width high times) | yes | StuffingAndCoding.pulse_width_and_symbol, StuffingAndCoding.pulse_width_ends_at_capture_tick |  |
| rosetta_tick | `tick.BAUD` | BAUD: launch, capture after PERIOD, next launch after PERIOD2 | yes | CheckUnit.crc16_ccitt_round_trip, CanOnRtl.arbitration_1000k, CanOnRtl.arbitration_250k, CanOnRtl.arbitration_500k +20 | CheckUnit.crc_known_answers, SharedLogic.alu_and_memory, SharedLogic.channels_between_lanes +14 |
| rosetta_tick | `tick.EDGE` | EDGE: launch and capture on the clock input's edges | yes | CheckUnit.crc16_ccitt_round_trip, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +9 | SharedLogic.channels_between_lanes, SharedLogic.crossbar_internal_loopback, SharedLogic.event_abandons_a_field +6 |
| rosetta_tick | `tick.RECOVER` | RECOVER: capture half a PERIOD after each data edge | yes | board:uart_async_sweep, CanOnRtl.ack_only_node, CanOnRtl.arbitration_1000k, CanOnRtl.arbitration_250k +9 |  |
| rosetta_tick | `tick.PULSE` | PULSE: a capture at each input edge, symbol from THR0 | yes | board:eth_rx_sweep, StuffingAndCoding.manchester_round_trip, StuffingAndCoding.pulse_width_and_symbol, EthOnRtl.receive_beyond_margin +3 |  |
| rosetta_tick | `tick.period2` | unequal BAUD halves (PERIOD2 != PERIOD) | yes | TimingUnits.prescale_64_and_512, CanOnRtl.arbitration_1000k, CanOnRtl.arbitration_250k, CanOnRtl.arbitration_500k +7 |  |
| rosetta_tick | `tick.prescale1` | prescaler 1 | yes | TimingUnits.launch_delay_on_data | CheckUnit.crc16_ccitt_round_trip, CheckUnit.crc_known_answers, SharedLogic.alu_and_memory +41 |
| rosetta_tick | `tick.prescale8` | prescaler 8 | yes | TimingUnits.launch_delay_prescaled |  |
| rosetta_tick | `tick.prescale64` | prescaler 64 | yes | TimingUnits.prescale_64_and_512 |  |
| rosetta_tick | `tick.prescale512` | prescaler 512 | yes | TimingUnits.prescale_64_and_512 |  |
| rosetta_tick | `tick.clock_out` | clock output on out3 | yes | CheckUnit.crc16_ccitt_round_trip, TimingUnits.launch_delay_on_clock, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz +10 | CheckUnit.crc_known_answers, SharedLogic.alu_and_memory, SharedLogic.channels_between_lanes +7 |
| rosetta_tick | `tick.clock_idle_high` | clock idle level 1 | yes | I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest, I2cHoldOnRtl.no_confirm_window +5 |  |
| rosetta_tick | `tick.clock_od` | clock output open-drain | yes | I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest, I2cHoldOnRtl.no_confirm_window +3 |  |
| rosetta_tick | `tick.rest` | rest: clock stays at the launch level after a field | yes | I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest, I2cHoldOnRtl.no_confirm_window +3 |  |
| rosetta_tick | `tick.capture_rising` | capture on the rising clock edge | yes | CheckUnit.crc16_ccitt_round_trip, SpiOnRtl.against_hand_written, SpiOnRtl.mode0, SpiOnRtl.mode3 | SharedLogic.channels_between_lanes, SharedLogic.crossbar_internal_loopback, SharedLogic.event_abandons_a_field +13 |
| rosetta_tick | `tick.capture_falling` | capture on the falling clock edge | yes | SpiOnRtl.mode1, SpiOnRtl.mode2 |  |
| rosetta_tick | `tick.stretch_honour` | BAUD clock owner honours a stretched clock | yes | I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest, I2cHoldOnRtl.no_confirm_window +3 |  |
| rosetta_tick | `tick.stretch_hold` | EDGE lane holds the clock low while it waits | yes | I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest, I2cHoldOnRtl.no_confirm_window +3 |  |
| rosetta_tick | `tick.thr0` | THR0 threshold (Manchester / pulse symbol) | yes | StuffingAndCoding.manchester_round_trip, StuffingAndCoding.pulse_width_and_symbol, EthOnRtl.receive_beyond_margin, EthOnRtl.receive_frame_lengths +2 |  |
| rosetta_tick | `tick.edge_interval` | edge-interval counter (RAW SAMPLE) | yes | SharedLogic.raw_drive_and_sample |  |
| rosetta_delay | `delay.data` | launch delay on the data output | yes | TimingUnits.launch_delay_on_data, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +1 | TimingUnits.launch_delay_prescaled |
| rosetta_delay | `delay.clock` | launch delay on the clock output | yes | TimingUnits.launch_delay_on_clock |  |
| rosetta_delay | `delay.prescaled` | launch delay in prescaled cycles | yes | TimingUnits.launch_delay_prescaled |  |
| rosetta_coding | `coding.level` | level coding | yes | CheckUnit.crc16_ccitt_round_trip, UartOnRtl.parity_even_and_wrong_parity, UartOnRtl.parity_none_and_framing_error, UartOnRtl.parity_odd_and_wrong_parity +1 | SharedLogic.crossbar_internal_loopback, StuffingAndCoding.differential_and_invert, StuffingAndCoding.manchester_round_trip +28 |
| rosetta_coding | `coding.tx.nrzi` | NRZI transmit | yes | StuffingAndCoding.nrzi_round_trip, UsbOnRtl.host_clock_fast_1p5pct, UsbOnRtl.host_clock_nominal, UsbOnRtl.host_clock_slow_1p5pct |  |
| rosetta_coding | `coding.rx.nrzi` | NRZI receive | yes | StuffingAndCoding.nrzi_round_trip, UsbOnRtl.host_clock_fast_1p5pct, UsbOnRtl.host_clock_nominal, UsbOnRtl.host_clock_slow_1p5pct |  |
| rosetta_coding | `coding.tx.manchester` | Manchester transmit | yes | StuffingAndCoding.manchester_round_trip, EthOnRtl.transmit_loopback |  |
| rosetta_coding | `coding.rx.manchester` | Manchester receive (PULSE mode) | yes | board:eth_rx_sweep, StuffingAndCoding.manchester_round_trip, EthOnRtl.receive_beyond_margin, EthOnRtl.receive_frame_lengths +2 |  |
| rosetta_coding | `coding.tx.pulse_width` | pulse-width transmit | yes | StuffingAndCoding.pulse_width_and_symbol, StuffingAndCoding.pulse_width_ends_at_capture_tick |  |
| rosetta_coding | `coding.rx.pulse_symbol` | pulse-symbol receive | yes | StuffingAndCoding.pulse_width_and_symbol |  |
| rosetta_coding | `coding.invert` | invert (both directions) | yes | StuffingAndCoding.differential_and_invert |  |
| rosetta_coding | `coding.data_od` | data output open-drain | yes | SharedLogic.crossbar_internal_loopback, CanOnRtl.ack_only_node, CanOnRtl.arbitration_1000k, CanOnRtl.arbitration_250k +9 |  |
| rosetta_coding | `coding.diff_out` | differential output (out1 = complement) | yes | StuffingAndCoding.differential_and_invert, EthOnRtl.transmit_loopback, UsbOnRtl.host_clock_fast_1p5pct, UsbOnRtl.host_clock_nominal +1 |  |
| rosetta_coding | `coding.diff_in` | differential input | yes | StuffingAndCoding.differential_and_invert, UsbOnRtl.host_clock_fast_1p5pct, UsbOnRtl.host_clock_nominal, UsbOnRtl.host_clock_slow_1p5pct |  |
| rosetta_coding | `coding.data_in` | data input select other than in0 | yes | StuffingAndCoding.differential_and_invert |  |
| rosetta_stuff | `stuff.ones` | ONES rule (insert 0 after N ones) | yes | StuffingAndCoding.stuffing_ones_round_trip, UsbOnRtl.host_clock_fast_1p5pct, UsbOnRtl.host_clock_nominal, UsbOnRtl.host_clock_slow_1p5pct | StuffingAndCoding.stuff_violation_raises_event_3 |
| rosetta_stuff | `stuff.equal` | EQUAL rule (insert complement after N equal bits) | yes | StuffingAndCoding.stuffing_equal_round_trip, CanOnRtl.ack_only_node, CanOnRtl.arbitration_1000k, CanOnRtl.arbitration_250k +2 |  |
| rosetta_stuff | `stuff.n5` | N = 5 | yes | StuffingAndCoding.stuffing_equal_round_trip, CanOnRtl.ack_only_node, CanOnRtl.arbitration_1000k, CanOnRtl.arbitration_250k +2 |  |
| rosetta_stuff | `stuff.n6` | N = 6 | yes | StuffingAndCoding.stuffing_ones_round_trip, UsbOnRtl.host_clock_fast_1p5pct, UsbOnRtl.host_clock_nominal, UsbOnRtl.host_clock_slow_1p5pct | StuffingAndCoding.stuff_violation_raises_event_3 |
| rosetta_stuff | `stuff.violation_event` | violation raises event 3 | yes | StuffingAndCoding.stuff_violation_raises_event_3, CanOnRtl.arbitration_1000k, CanOnRtl.arbitration_250k, CanOnRtl.arbitration_500k | StuffingAndCoding.stuffing_ones_round_trip, CanOnRtl.ack_only_node, CanOnRtl.listen_only_node |
| rosetta_crc | `crc.w1` | width 1 (parity) | yes | CheckUnit.crc_known_answers, UartOnRtl.parity_even_and_wrong_parity, UartOnRtl.parity_odd_and_wrong_parity |  |
| rosetta_crc | `crc.w5` | width 5 | yes | CheckUnit.crc_known_answers, UsbOnRtl.host_clock_fast_1p5pct, UsbOnRtl.host_clock_nominal, UsbOnRtl.host_clock_slow_1p5pct |  |
| rosetta_crc | `crc.w8` | width 8 | yes | CheckUnit.crc_known_answers |  |
| rosetta_crc | `crc.w15` | width 15 | yes | CheckUnit.crc_known_answers, CanOnRtl.ack_only_node, CanOnRtl.arbitration_1000k, CanOnRtl.arbitration_250k +2 |  |
| rosetta_crc | `crc.w16` | width 16 | yes | CheckUnit.crc16_ccitt_round_trip, CheckUnit.crc_known_answers, UsbOnRtl.host_clock_fast_1p5pct, UsbOnRtl.host_clock_nominal +1 | SharedLogic.crossbar_internal_loopback |
| rosetta_crc | `crc.w32` | width 32 | yes | board:eth_rx_sweep, CheckUnit.crc_known_answers, EthOnRtl.receive_beyond_margin, EthOnRtl.receive_frame_lengths +2 |  |
| rosetta_crc | `crc.init_zero` | initial value 0 | yes | CheckUnit.crc_known_answers, CanOnRtl.arbitration_1000k, CanOnRtl.arbitration_250k, CanOnRtl.arbitration_500k | CanOnRtl.ack_only_node, CanOnRtl.listen_only_node, UartOnRtl.parity_even_and_wrong_parity +1 |
| rosetta_crc | `crc.init_ones` | initial value all ones | yes | CheckUnit.crc16_ccitt_round_trip, CheckUnit.crc_known_answers, EthOnRtl.transmit_loopback, UsbOnRtl.host_clock_fast_1p5pct +2 | SharedLogic.crossbar_internal_loopback, EthOnRtl.receive_beyond_margin, EthOnRtl.receive_frame_lengths +1 |
| rosetta_crc | `crc.xorout` | final XOR all ones | yes | CheckUnit.crc_known_answers, EthOnRtl.transmit_loopback, UartOnRtl.parity_odd_and_wrong_parity, UsbOnRtl.host_clock_fast_1p5pct +2 | EthOnRtl.receive_beyond_margin, EthOnRtl.receive_frame_lengths, EthOnRtl.receive_inside_margin |
| rosetta_crc | `crc.msb_first` | data MSB first | yes | CheckUnit.crc16_ccitt_round_trip | CheckUnit.crc_known_answers, SharedLogic.crossbar_internal_loopback, CanOnRtl.ack_only_node +8 |
| rosetta_crc | `crc.lsb_first` | data LSB first (reflected CRC) | yes | CheckUnit.crc_known_answers | EthOnRtl.transmit_loopback, UartOnRtl.parity_even_and_wrong_parity, UartOnRtl.parity_odd_and_wrong_parity +3 |
| rosetta_crc | `crc.expect` | EXPECT against the CRC | yes | CheckUnit.crc16_ccitt_round_trip, CanOnRtl.ack_only_node, CanOnRtl.arbitration_1000k, CanOnRtl.arbitration_250k +5 | SharedLogic.crossbar_internal_loopback, UartOnRtl.parity_even_and_wrong_parity, UartOnRtl.parity_odd_and_wrong_parity |
| rosetta_crc | `crc.pushcrc` | PUSHCRC to the host | yes | CheckUnit.crc_known_answers |  |
| rosetta_crc | `crc.crcchk` | CRCCHK against {R1, R0} | yes | CheckUnit.crc_known_answers, EthOnRtl.receive_beyond_margin, EthOnRtl.receive_frame_lengths, EthOnRtl.receive_inside_margin +4 |  |
| rosetta_field | `field.drive` | FIELD DRIVE | yes | CheckUnit.crc16_ccitt_round_trip, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +13 | CheckUnit.crc_known_answers, SharedLogic.alu_and_memory, SharedLogic.channels_between_lanes +22 |
| rosetta_field | `field.sample` | FIELD SAMPLE | yes | CheckUnit.crc16_ccitt_round_trip, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +13 | SharedLogic.channels_between_lanes, SharedLogic.crossbar_internal_loopback, SharedLogic.event_abandons_a_field +20 |
| rosetta_field | `field.exchange` | FIELD EXCHANGE | yes | CanOnRtl.arbitration_1000k, CanOnRtl.arbitration_250k, CanOnRtl.arbitration_500k, SpiOnRtl.against_hand_written +4 |  |
| rosetta_field | `field.expect` | FIELD EXPECT | yes | CheckUnit.crc16_ccitt_round_trip, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +8 | SharedLogic.crossbar_internal_loopback, StuffingAndCoding.expect_seek_finds_pattern, CanOnRtl.ack_only_node +11 |
| rosetta_field | `field.early` | early first bit | yes | StuffingAndCoding.expect_seek_finds_pattern, SpiOnRtl.against_hand_written, SpiOnRtl.mode0, SpiOnRtl.mode1 +2 |  |
| rosetta_field | `field.lsb` | LSB first | yes | CheckUnit.crc_known_answers, EthOnRtl.transmit_loopback, UartOnRtl.parity_even_and_wrong_parity, UartOnRtl.parity_none_and_framing_error +5 | EthOnRtl.receive_beyond_margin, EthOnRtl.receive_frame_lengths, EthOnRtl.receive_inside_margin |
| rosetta_field | `field.tag` | record tags | yes | I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest, I2cHoldOnRtl.no_confirm_window +12 | CanOnRtl.ack_only_node, CanOnRtl.arbitration_1000k, CanOnRtl.arbitration_250k +5 |
| rosetta_field | `field.seek` | EXPECT seek | yes | StuffingAndCoding.expect_seek_finds_pattern, EthOnRtl.receive_beyond_margin, EthOnRtl.receive_frame_lengths, EthOnRtl.receive_inside_margin +4 |  |
| rosetta_field | `field.crc_flag` | CRC update flag | yes | CheckUnit.crc16_ccitt_round_trip, CanOnRtl.ack_only_node, CanOnRtl.arbitration_1000k, CanOnRtl.arbitration_250k +6 | CheckUnit.crc_known_answers, SharedLogic.crossbar_internal_loopback, EthOnRtl.receive_beyond_margin +4 |
| rosetta_field | `field.stuff_flag` | stuffing flag | yes | StuffingAndCoding.stuffing_equal_round_trip, StuffingAndCoding.stuffing_ones_round_trip, CanOnRtl.ack_only_node, CanOnRtl.arbitration_1000k +6 | StuffingAndCoding.stuff_violation_raises_event_3 |
| rosetta_field | `field.src_mem` | MEM as a field source | yes | SharedLogic.alu_and_memory, CanOnRtl.arbitration_1000k, CanOnRtl.arbitration_250k, CanOnRtl.arbitration_500k +4 |  |
| rosetta_field | `field.dst_mem` | MEM as a field destination (R3 post-increment) | yes | EthOnRtl.receive_beyond_margin, EthOnRtl.receive_frame_lengths, EthOnRtl.receive_inside_margin, EthOnRtl.transmit_loopback +3 |  |
| rosetta_field | `field.src_imm_marker` | IMM source as a length marker (partial last word) | yes | EthOnRtl.receive_beyond_margin, EthOnRtl.receive_frame_lengths, EthOnRtl.receive_inside_margin |  |
| rosetta_field | `field.raw_drive` | RAW DRIVE | yes | SharedLogic.raw_drive_and_sample |  |
| rosetta_field | `field.raw_sample` | RAW SAMPLE | yes | SharedLogic.raw_drive_and_sample |  |
| rosetta_field | `field.abandon` | a field abandoned by an event | yes | SharedLogic.event_abandons_a_field |  |
| rosetta_events | `events.level` | trigger: A at level L | yes | Events.event_triggers, CanOnRtl.arbitration_1000k, CanOnRtl.arbitration_250k, CanOnRtl.arbitration_500k +14 |  |
| rosetta_events | `events.edge` | trigger: A edge to L | yes | board:uart_async_sweep, Events.event_triggers, SharedLogic.event_abandons_a_field, I2cHoldOnRtl.100khz +15 |  |
| rosetta_events | `events.any` | trigger: A any edge | yes | Events.event_triggers |  |
| rosetta_events | `events.bothlow` | trigger: A and B both low | yes | Events.event_triggers, UsbOnRtl.host_clock_fast_1p5pct, UsbOnRtl.host_clock_nominal, UsbOnRtl.host_clock_slow_1p5pct |  |
| rosetta_events | `events.b_high` | B must be high | yes | Events.event_triggers, CanOnRtl.arbitration_1000k, CanOnRtl.arbitration_250k, CanOnRtl.arbitration_500k +7 |  |
| rosetta_events | `events.b_low` | B must be low | yes | Events.event_triggers, CanOnRtl.arbitration_1000k, CanOnRtl.arbitration_250k, CanOnRtl.arbitration_500k +3 |  |
| rosetta_events | `events.k_window` | confirm window K | yes | Events.event_triggers, Events.wait_timeout_in_period_units, CanOnRtl.arbitration_1000k, CanOnRtl.arbitration_250k +15 |  |
| rosetta_events | `events.clear` | EVT clear of a pending bit | yes | Events.event_triggers, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +8 | SharedLogic.event_abandons_a_field, CanOnRtl.ack_only_node, CanOnRtl.arbitration_1000k +15 |
| rosetta_events | `events.vector` | an armed event vectors the lane | yes | SharedLogic.event_abandons_a_field, StuffingAndCoding.stuff_violation_raises_event_3, CanOnRtl.arbitration_1000k, CanOnRtl.arbitration_250k +16 |  |
| rosetta_pins | `pins.pin` | PIN sets value and enable | yes | StuffingAndCoding.nrzi_round_trip, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +16 | Events.event_triggers, Events.wait_timeout_in_period_units, SharedLogic.crossbar_internal_loopback +16 |
| rosetta_pins | `pins.pin_wait` | PIN with a wait | yes | EthOnRtl.transmit_loopback, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +12 | Events.event_triggers, CanOnRtl.ack_only_node, CanOnRtl.arbitration_1000k +3 |
| rosetta_pins | `pins.release_on_halt` | halt releases every output | yes | SharedLogic.step_pause_halt_restart, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +4 |  |
| rosetta_lane | `lane.0` | lane 0 runs a program | yes | board:can_async, board:eth_rx_sweep, board:usb_async, CanOnRtl.arbitration_1000k +25 | CheckUnit.crc16_ccitt_round_trip, CheckUnit.crc_known_answers, Events.event_triggers +21 |
| rosetta_lane | `lane.1` | lane 1 runs a program | yes | board:can_async, board:uart_async_sweep, board:usb_async, CanOnRtl.ack_only_node +23 | CheckUnit.crc16_ccitt_round_trip, Events.event_triggers, SharedLogic.channels_between_lanes +11 |
| rosetta_lane | `lane.2` | lane 2 runs a program | yes | board:can_async, board:uart_async_sweep, CanOnRtl.arbitration_1000k, CanOnRtl.arbitration_250k +18 |  |
| rosetta_lane | `lane.3` | lane 3 runs a program | yes | board:can_async, CanOnRtl.arbitration_1000k, CanOnRtl.arbitration_250k, CanOnRtl.arbitration_500k +12 |  |
| rosetta_seq | `seq.alu` | ALU operations | yes | SharedLogic.alu_and_memory, SharedLogic.step_pause_halt_restart, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz +5 | CheckUnit.crc16_ccitt_round_trip, CheckUnit.crc_known_answers, SharedLogic.channels_between_lanes +23 |
| rosetta_seq | `seq.alu_flags` | Z and N flags | yes | SharedLogic.alu_and_memory, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +4 |  |
| rosetta_seq | `seq.mem_ld_st` | LD / ST in the lane's data region | yes | SharedLogic.alu_and_memory, UsbOnRtl.host_clock_fast_1p5pct, UsbOnRtl.host_clock_nominal, UsbOnRtl.host_clock_slow_1p5pct | CheckUnit.crc_known_answers, CanOnRtl.arbitration_1000k, CanOnRtl.arbitration_250k +1 |
| rosetta_seq | `seq.movsr` | MOVSR / MOVSRH | yes | SharedLogic.alu_and_memory, EthOnRtl.receive_beyond_margin, EthOnRtl.receive_frame_lengths, EthOnRtl.receive_inside_margin | SharedLogic.event_abandons_a_field, EthOnRtl.transmit_loopback, I2cHoldOnRtl.100khz +8 |
| rosetta_seq | `seq.loop` | LOOP | yes | CheckUnit.crc16_ccitt_round_trip, SharedLogic.alu_and_memory, EthOnRtl.transmit_loopback | CheckUnit.crc_known_answers, SharedLogic.crossbar_internal_loopback, SharedLogic.host_queues_full_and_empty +20 |
| rosetta_seq | `seq.br_queue` | BR on QE / QNE / QF / QNF | yes | SharedLogic.host_queues_full_and_empty, EthOnRtl.transmit_loopback | SpiOnRtl.against_hand_written, SpiOnRtl.mode0, SpiOnRtl.mode1 +2 |
| rosetta_seq | `seq.br_event` | BR on an event pending bit | yes | Events.event_triggers |  |
| rosetta_seq | `seq.wait_event` | WAIT on events | yes | Events.event_triggers, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +16 |  |
| rosetta_seq | `seq.wait_timeout` | WAIT timeout | yes | Events.event_triggers, Events.wait_timeout_in_period_units, EthOnRtl.transmit_loopback | CheckUnit.crc16_ccitt_round_trip, SharedLogic.channels_between_lanes, SharedLogic.crossbar_internal_loopback +12 |
| rosetta_seq | `seq.pushts` | PUSHTS (timestamp record) | yes | Events.event_triggers | I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +13 |
| rosetta_seq | `seq.cap_records` | CAP records (listener timestamp + value) | yes | board:uart_async_sweep, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +16 |  |
| rosetta_seq | `seq.halt` | HALT (with attention) | yes | CheckUnit.crc16_ccitt_round_trip, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +4 | CheckUnit.crc_known_answers, Events.event_triggers, Events.wait_timeout_in_period_units +19 |
| rosetta_seq | `seq.reserved_halt` | a reserved opcode halts the lane | yes | SharedLogic.step_pause_halt_restart |  |
| rosetta_seq | `seq.host_halt` | host CTRL.run = 0 halts a lane | yes | SharedLogic.step_pause_halt_restart |  |
| rosetta_seq | `seq.restart` | restart after a halt (PC, CTRL.run) | yes | SharedLogic.step_pause_halt_restart |  |
| rosetta_seq | `seq.pause` | pause | yes | SharedLogic.step_pause_halt_restart |  |
| rosetta_seq | `seq.single_step` | single step | yes | SharedLogic.step_pause_halt_restart |  |
| rosetta_seq | `seq.abort` | abort: an event abandons the current operation | yes | SharedLogic.event_abandons_a_field, StuffingAndCoding.stuff_violation_raises_event_3, CanOnRtl.arbitration_1000k, CanOnRtl.arbitration_250k +8 |  |
| rosetta_chan | `chan.put_get` | PUT / GET between lanes | yes | SharedLogic.channels_between_lanes, CanOnRtl.arbitration_1000k, CanOnRtl.arbitration_250k, CanOnRtl.arbitration_500k | CanOnRtl.ack_only_node, CanOnRtl.listen_only_node |
| rosetta_chan | `chan.get_waits` | GET waits for a channel | yes | SharedLogic.channels_between_lanes, CanOnRtl.arbitration_1000k, CanOnRtl.arbitration_250k, CanOnRtl.arbitration_500k |  |
| rosetta_chan | `chan.putsr` | PUTSR (32 bits) | yes | SharedLogic.channels_between_lanes |  |
| rosetta_chan | `chan.br_valid` | BR on channel valid | yes | SharedLogic.channels_between_lanes, CanOnRtl.ack_only_node, CanOnRtl.arbitration_1000k, CanOnRtl.arbitration_250k +2 |  |
| rosetta_chan | `chan.host_chstat` | host CHSTAT | yes | SharedLogic.channels_between_lanes |  |
| rosetta_chan | `chan.host_chdata` | host CHDATA | yes | SharedLogic.channels_between_lanes |  |
| rosetta_chan | `chan.host_clear` | host clears a channel (W1C) | yes | SharedLogic.channels_between_lanes |  |
| rosetta_chan | `chan.timestamp` | timestamp counter (records, TIME) | yes | Events.event_triggers, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +13 |  |
| rosetta_xbar | `xbar.insel` | lane input selects (XIN) | yes | SharedLogic.crossbar_internal_loopback, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +13 |  |
| rosetta_xbar | `xbar.outsel` | output selects (XOUT) to uo | yes | SharedLogic.crossbar_internal_loopback, SpiOnRtl.against_hand_written, SpiOnRtl.mode0, SpiOnRtl.mode1 +6 |  |
| rosetta_xbar | `xbar.uio_out` | outputs to uio pins | yes | SharedLogic.crossbar_internal_loopback, CanOnRtl.arbitration_1000k, CanOnRtl.arbitration_250k, CanOnRtl.arbitration_500k +10 | CanOnRtl.ack_only_node, CanOnRtl.listen_only_node |
| rosetta_xbar | `xbar.uio_oe` | uio enables follow the lanes | yes | SharedLogic.crossbar_internal_loopback, CanOnRtl.arbitration_1000k, CanOnRtl.arbitration_250k, CanOnRtl.arbitration_500k +10 |  |
| rosetta_xbar | `xbar.uio_in` | lanes read uio pins (open-drain buses) | yes | CanOnRtl.arbitration_1000k, CanOnRtl.arbitration_250k, CanOnRtl.arbitration_500k, I2cHoldOnRtl.100khz +9 |  |
| rosetta_xbar | `xbar.loopback` | internal loopback (XCTL) | yes | SharedLogic.crossbar_internal_loopback |  |
| rosetta_host | `host.write` | register write frames | yes | I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest, I2cHoldOnRtl.no_confirm_window +12 |  |
| rosetta_host | `host.read` | register read frames | yes | I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest, I2cHoldOnRtl.no_confirm_window +12 |  |
| rosetta_host | `host.id` | ID register | yes | SharedLogic.host_queues_full_and_empty |  |
| rosetta_host | `host.maddr` | MADDR read back | yes | SharedLogic.host_queues_full_and_empty |  |
| rosetta_host | `host.mdata_autoinc` | MDATA auto-increment | yes | SharedLogic.memories_full_range |  |
| rosetta_host | `host.time` | TIME register | yes | SharedLogic.host_queues_full_and_empty |  |
| rosetta_host | `host.irqmask` | IRQMASK | yes | SharedLogic.host_queues_full_and_empty | SharedLogic.step_pause_halt_restart |
| rosetta_host | `host.irqstat` | IRQSTAT | yes | SharedLogic.host_queues_full_and_empty | SharedLogic.step_pause_halt_restart |
| rosetta_host | `host.irqstat_clear` | IRQSTAT write-1-to-clear | yes | SharedLogic.step_pause_halt_restart |  |
| rosetta_host | `host.irq_queue` | IRQ pin from a non-empty out queue | yes | SharedLogic.host_queues_full_and_empty |  |
| rosetta_host | `host.irq_attn` | IRQ pin from attention | yes | SharedLogic.step_pause_halt_restart |  |
| rosetta_mem_prog | `mem.prog_load` | program load and read back through the host | yes | I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest, I2cHoldOnRtl.no_confirm_window +12 | CheckUnit.crc16_ccitt_round_trip, CheckUnit.crc_known_answers, Events.event_triggers +34 |
| rosetta_mem_prog | `mem.prog_full` | all 512 program words | yes | SharedLogic.memories_full_range |  |
| rosetta_mem_data | `mem.data_full` | all 512 data words through the host | yes | SharedLogic.memories_full_range |  |
| rosetta_mem_data | `mem.queues` | host-in / host-out queues | yes | I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest, I2cHoldOnRtl.no_confirm_window +12 |  |
| rosetta_mem_data | `mem.queue_full_waits` | a push waits while the host-out queue is full | yes | SharedLogic.host_queues_full_and_empty |  |
| rosetta_mem_data | `mem.queue_empty_waits` | a POP waits while the host-in queue is empty | yes | SharedLogic.host_queues_full_and_empty |  |
| rosetta_mem_data | `mem.qcfg_sizes` | queue sizes other than 64 (QCFG) | yes | SharedLogic.host_queues_full_and_empty, EthOnRtl.transmit_loopback, UsbOnRtl.host_clock_fast_1p5pct, UsbOnRtl.host_clock_nominal +1 |  |
| rosetta_mem_data | `mem.data_lane` | lane data memory (LD/ST, MEM) | yes | SharedLogic.alu_and_memory, UsbOnRtl.host_clock_fast_1p5pct, UsbOnRtl.host_clock_nominal, UsbOnRtl.host_clock_slow_1p5pct | CheckUnit.crc_known_answers, CanOnRtl.arbitration_1000k, CanOnRtl.arbitration_250k +1 |
| top | `top.reset` | reset and reset synchroniser | yes | I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest, I2cHoldOnRtl.no_confirm_window +12 |  |
| top | `top.four_lanes` | four lanes at once with the host | yes | CanOnRtl.arbitration_1000k, CanOnRtl.arbitration_250k, CanOnRtl.arbitration_500k, I2cHoldOnRtl.100khz +11 |  |
| top | `top.async_input` | an asynchronous far end on a pin (board only) | yes | board:can_async, board:eth_rx_sweep, board:uart_async_sweep, board:usb_async |  |
| protocols | `proto.usb_device` | low-speed USB device: NAK, DATA1 + ACK, SETUP/DATA0 + ACK, address filter | yes | board:usb_async, UsbOnRtl.host_clock_fast_1p5pct, UsbOnRtl.host_clock_nominal, UsbOnRtl.host_clock_slow_1p5pct |  |
| protocols | `proto.usb_listener` | low-speed USB listener: every packet both ways | yes | UsbOnRtl.host_clock_fast_1p5pct, UsbOnRtl.host_clock_nominal, UsbOnRtl.host_clock_slow_1p5pct |  |
| protocols | `proto.usb_turnaround` | USB turnaround within 2..7.5 bit times at host clock 0, +-1.5 % | yes | board:usb_async, UsbOnRtl.host_clock_fast_1p5pct, UsbOnRtl.host_clock_nominal, UsbOnRtl.host_clock_slow_1p5pct |  |
| protocols | `proto.eth_tx` | 10BASE-T transmit: Manchester, CRC-32 FCS, TP_IDL, link pulses | yes | EthOnRtl.transmit_loopback |  |
| protocols | `proto.eth_rx` | 10BASE-T receive: SFD seek, payload to memory, CRC-32 residue | yes | board:eth_rx_sweep, EthOnRtl.receive_beyond_margin, EthOnRtl.receive_frame_lengths, EthOnRtl.receive_inside_margin +1 |  |
| protocols | `proto.eth_rx_margin` | 10BASE-T receive margin against an asynchronous source (board only) | yes | board:eth_rx_sweep |  |
| protocols | `proto.can_arbitration` | CAN: two-lane nodes, arbitration, loser backs off, ACK, retransmit | yes | board:can_async, CanOnRtl.arbitration_1000k, CanOnRtl.arbitration_250k, CanOnRtl.arbitration_500k |  |
| protocols | `proto.can_ack_only` | CAN ACK-only node | yes | CanOnRtl.ack_only_node |  |
| protocols | `proto.can_listen_only` | CAN listen-only node | yes | CanOnRtl.listen_only_node |  |
| protocols | `proto.async_far_ends` | USB host and CAN node on their own clock (board only) | yes | board:can_async, board:usb_async |  |

Features without a passing board test: 0

## Not exercisable on this board

- **Pad and Tiny Tapeout mux timing.** The chip's pins are FPGA nets: no pads, no TT mux, no board wiring (the headers are not connected). Output delay, slew, input thresholds and the host.md 2.1 margins for pad delay cannot be measured.
- **The IHP SRAM macro.** The FPGA uses the behavioural memory (ROSETTA_SRAM_BEHAV, block RAM); the macro, its BIST pins and its 0.92 ns hold need are ASIC-only.
- **ASIC timing at 60 MHz.** FPGA timing closure says nothing about the IHP gate-level paths; that is the layout flow's STA.
- **Asynchronous reset release.** The harness releases rst_n between chip edges (clock stopped); the reset synchroniser's behaviour with a release near a clock edge is not exercised.
- **Synchroniser metastability.** Every lockstep input changes between chip edges; only the async UART input (one ui pin) is truly asynchronous, and metastability resolution is not observable.
- **ena input.** Tiny Tapeout's ena is unused by the design (top.md).
- **Reactive asynchronous far ends.** The board's async lines play scripted event lists: the USB host's handshake follows the device's packet milliseconds later (the device waits for it), and the async CAN node sends one frame without arbitrating or acknowledging. A far end that answers in real time needs an HDL model on its own clock or a real device.
- **Edges closer than 20 ns from an async line.** rh_line.v needs 4 of its 5 ns cycles between events; DCD above about 30 ns on 10BASE-T (low pulses under 20 ns) needs a deeper event prefetch, or two IDELAYs per line used alternately.
- **Host SPI at other rates or with an asynchronous SCLK.** The lockstep host engine plays exactly the Verilator harness's f_clk/16 mode-0 frames, synchronous to the chip clock.

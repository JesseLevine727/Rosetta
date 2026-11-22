# Hardware coverage (PYNQ-Z1, lockstep board runs)

`checked by`: passing board tests whose checks rely on the feature (test_blocks_rtl COVERS, hw_coverage.SCENARIOS). `configured in`: other passing board tests whose loaded programs or host operations use it (decoded from the session transcripts).

| Block | Feature | What | On hardware | Checked by | Configured in |
|---|---|---|---|---|---|
| rosetta_cfg | `cfg.setm` | mode registers written by SETM (immediate) | yes |  | CheckUnit.crc16_ccitt_round_trip, CheckUnit.crc_known_answers, Events.event_triggers +34 |
| rosetta_cfg | `cfg.setm_hi` | upper half of a 32-bit register (SETM .hi, CRCPOLY) | yes | CheckUnit.crc_known_answers |  |
| rosetta_cfg | `cfg.setm_reg` | SETM value from a register | yes | CheckUnit.crc_known_answers |  |
| rosetta_cfg | `cfg.pulse` | PULSE register (pulse-width high times) | yes | StuffingAndCoding.pulse_width_and_symbol, StuffingAndCoding.pulse_width_ends_at_capture_tick |  |
| rosetta_tick | `tick.BAUD` | BAUD: launch, capture after PERIOD, next launch after PERIOD2 | yes | CheckUnit.crc16_ccitt_round_trip, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +13 | CheckUnit.crc_known_answers, SharedLogic.alu_and_memory, SharedLogic.channels_between_lanes +14 |
| rosetta_tick | `tick.EDGE` | EDGE: launch and capture on the clock input's edges | yes | CheckUnit.crc16_ccitt_round_trip, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +9 | SharedLogic.channels_between_lanes, SharedLogic.crossbar_internal_loopback, SharedLogic.event_abandons_a_field +6 |
| rosetta_tick | `tick.RECOVER` | RECOVER: capture half a PERIOD after each data edge | yes | board:uart_async_sweep, UartOnRtl.parity_even_and_wrong_parity, UartOnRtl.parity_none_and_framing_error, UartOnRtl.parity_odd_and_wrong_parity +1 |  |
| rosetta_tick | `tick.PULSE` | PULSE: a capture at each input edge, symbol from THR0 | yes | StuffingAndCoding.manchester_round_trip, StuffingAndCoding.pulse_width_and_symbol |  |
| rosetta_tick | `tick.period2` | unequal BAUD halves (PERIOD2 != PERIOD) | yes | TimingUnits.prescale_64_and_512, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +4 |  |
| rosetta_tick | `tick.prescale1` | prescaler 1 | yes | TimingUnits.launch_delay_on_data | CheckUnit.crc16_ccitt_round_trip, CheckUnit.crc_known_answers, SharedLogic.alu_and_memory +29 |
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
| rosetta_tick | `tick.thr0` | THR0 threshold (Manchester / pulse symbol) | yes | StuffingAndCoding.manchester_round_trip, StuffingAndCoding.pulse_width_and_symbol |  |
| rosetta_tick | `tick.edge_interval` | edge-interval counter (RAW SAMPLE) | yes | SharedLogic.raw_drive_and_sample |  |
| rosetta_delay | `delay.data` | launch delay on the data output | yes | TimingUnits.launch_delay_on_data, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +1 | TimingUnits.launch_delay_prescaled |
| rosetta_delay | `delay.clock` | launch delay on the clock output | yes | TimingUnits.launch_delay_on_clock |  |
| rosetta_delay | `delay.prescaled` | launch delay in prescaled cycles | yes | TimingUnits.launch_delay_prescaled |  |
| rosetta_coding | `coding.level` | level coding | yes | CheckUnit.crc16_ccitt_round_trip, UartOnRtl.parity_even_and_wrong_parity, UartOnRtl.parity_none_and_framing_error, UartOnRtl.parity_odd_and_wrong_parity +1 | SharedLogic.crossbar_internal_loopback, StuffingAndCoding.differential_and_invert, StuffingAndCoding.manchester_round_trip +16 |
| rosetta_coding | `coding.tx.nrzi` | NRZI transmit | yes | StuffingAndCoding.nrzi_round_trip |  |
| rosetta_coding | `coding.rx.nrzi` | NRZI receive | yes | StuffingAndCoding.nrzi_round_trip |  |
| rosetta_coding | `coding.tx.manchester` | Manchester transmit | yes | StuffingAndCoding.manchester_round_trip |  |
| rosetta_coding | `coding.rx.manchester` | Manchester receive (PULSE mode) | yes | StuffingAndCoding.manchester_round_trip |  |
| rosetta_coding | `coding.tx.pulse_width` | pulse-width transmit | yes | StuffingAndCoding.pulse_width_and_symbol, StuffingAndCoding.pulse_width_ends_at_capture_tick |  |
| rosetta_coding | `coding.rx.pulse_symbol` | pulse-symbol receive | yes | StuffingAndCoding.pulse_width_and_symbol |  |
| rosetta_coding | `coding.invert` | invert (both directions) | yes | StuffingAndCoding.differential_and_invert |  |
| rosetta_coding | `coding.data_od` | data output open-drain | yes | SharedLogic.crossbar_internal_loopback, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +4 |  |
| rosetta_coding | `coding.diff_out` | differential output (out1 = complement) | yes | StuffingAndCoding.differential_and_invert |  |
| rosetta_coding | `coding.diff_in` | differential input | yes | StuffingAndCoding.differential_and_invert |  |
| rosetta_coding | `coding.data_in` | data input select other than in0 | yes | StuffingAndCoding.differential_and_invert |  |
| rosetta_stuff | `stuff.ones` | ONES rule (insert 0 after N ones) | yes | StuffingAndCoding.stuffing_ones_round_trip | StuffingAndCoding.stuff_violation_raises_event_3 |
| rosetta_stuff | `stuff.equal` | EQUAL rule (insert complement after N equal bits) | yes | StuffingAndCoding.stuffing_equal_round_trip |  |
| rosetta_stuff | `stuff.n5` | N = 5 | yes | StuffingAndCoding.stuffing_equal_round_trip |  |
| rosetta_stuff | `stuff.n6` | N = 6 | yes | StuffingAndCoding.stuffing_ones_round_trip | StuffingAndCoding.stuff_violation_raises_event_3 |
| rosetta_stuff | `stuff.violation_event` | violation raises event 3 | yes | StuffingAndCoding.stuff_violation_raises_event_3 | StuffingAndCoding.stuffing_ones_round_trip |
| rosetta_crc | `crc.w1` | width 1 (parity) | yes | CheckUnit.crc_known_answers, UartOnRtl.parity_even_and_wrong_parity, UartOnRtl.parity_odd_and_wrong_parity |  |
| rosetta_crc | `crc.w5` | width 5 | yes | CheckUnit.crc_known_answers |  |
| rosetta_crc | `crc.w8` | width 8 | yes | CheckUnit.crc_known_answers |  |
| rosetta_crc | `crc.w15` | width 15 | yes | CheckUnit.crc_known_answers |  |
| rosetta_crc | `crc.w16` | width 16 | yes | CheckUnit.crc16_ccitt_round_trip, CheckUnit.crc_known_answers | SharedLogic.crossbar_internal_loopback |
| rosetta_crc | `crc.w32` | width 32 | yes | CheckUnit.crc_known_answers |  |
| rosetta_crc | `crc.init_zero` | initial value 0 | yes | CheckUnit.crc_known_answers | UartOnRtl.parity_even_and_wrong_parity, UartOnRtl.parity_odd_and_wrong_parity |
| rosetta_crc | `crc.init_ones` | initial value all ones | yes | CheckUnit.crc16_ccitt_round_trip, CheckUnit.crc_known_answers | SharedLogic.crossbar_internal_loopback |
| rosetta_crc | `crc.xorout` | final XOR all ones | yes | CheckUnit.crc_known_answers, UartOnRtl.parity_odd_and_wrong_parity |  |
| rosetta_crc | `crc.msb_first` | data MSB first | yes | CheckUnit.crc16_ccitt_round_trip | CheckUnit.crc_known_answers, SharedLogic.crossbar_internal_loopback |
| rosetta_crc | `crc.lsb_first` | data LSB first (reflected CRC) | yes | CheckUnit.crc_known_answers | UartOnRtl.parity_even_and_wrong_parity, UartOnRtl.parity_odd_and_wrong_parity |
| rosetta_crc | `crc.expect` | EXPECT against the CRC | yes | CheckUnit.crc16_ccitt_round_trip | SharedLogic.crossbar_internal_loopback, UartOnRtl.parity_even_and_wrong_parity, UartOnRtl.parity_odd_and_wrong_parity |
| rosetta_crc | `crc.pushcrc` | PUSHCRC to the host | yes | CheckUnit.crc_known_answers |  |
| rosetta_crc | `crc.crcchk` | CRCCHK against {R1, R0} | yes | CheckUnit.crc_known_answers |  |
| rosetta_field | `field.drive` | FIELD DRIVE | yes | CheckUnit.crc16_ccitt_round_trip, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +13 | CheckUnit.crc_known_answers, SharedLogic.alu_and_memory, SharedLogic.channels_between_lanes +14 |
| rosetta_field | `field.sample` | FIELD SAMPLE | yes | CheckUnit.crc16_ccitt_round_trip, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +13 | SharedLogic.channels_between_lanes, SharedLogic.crossbar_internal_loopback, SharedLogic.event_abandons_a_field +8 |
| rosetta_field | `field.exchange` | FIELD EXCHANGE | yes | SpiOnRtl.against_hand_written, SpiOnRtl.mode0, SpiOnRtl.mode1, SpiOnRtl.mode2 +1 |  |
| rosetta_field | `field.expect` | FIELD EXPECT | yes | CheckUnit.crc16_ccitt_round_trip, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +8 | SharedLogic.crossbar_internal_loopback, StuffingAndCoding.expect_seek_finds_pattern |
| rosetta_field | `field.early` | early first bit | yes | StuffingAndCoding.expect_seek_finds_pattern, SpiOnRtl.against_hand_written, SpiOnRtl.mode0, SpiOnRtl.mode1 +2 |  |
| rosetta_field | `field.lsb` | LSB first | yes | CheckUnit.crc_known_answers, UartOnRtl.parity_even_and_wrong_parity, UartOnRtl.parity_none_and_framing_error, UartOnRtl.parity_odd_and_wrong_parity +1 |  |
| rosetta_field | `field.tag` | record tags | yes | I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest, I2cHoldOnRtl.no_confirm_window +12 |  |
| rosetta_field | `field.seek` | EXPECT seek | yes | StuffingAndCoding.expect_seek_finds_pattern |  |
| rosetta_field | `field.crc_flag` | CRC update flag | yes | CheckUnit.crc16_ccitt_round_trip | CheckUnit.crc_known_answers, SharedLogic.crossbar_internal_loopback, UartOnRtl.parity_even_and_wrong_parity +1 |
| rosetta_field | `field.stuff_flag` | stuffing flag | yes | StuffingAndCoding.stuffing_equal_round_trip, StuffingAndCoding.stuffing_ones_round_trip | StuffingAndCoding.stuff_violation_raises_event_3 |
| rosetta_field | `field.src_mem` | MEM as a field source | yes | SharedLogic.alu_and_memory |  |
| rosetta_field | `field.raw_drive` | RAW DRIVE | yes | SharedLogic.raw_drive_and_sample |  |
| rosetta_field | `field.raw_sample` | RAW SAMPLE | yes | SharedLogic.raw_drive_and_sample |  |
| rosetta_field | `field.abandon` | a field abandoned by an event | yes | SharedLogic.event_abandons_a_field |  |
| rosetta_events | `events.level` | trigger: A at level L | yes | Events.event_triggers, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +4 |  |
| rosetta_events | `events.edge` | trigger: A edge to L | yes | board:uart_async_sweep, Events.event_triggers, SharedLogic.event_abandons_a_field, I2cHoldOnRtl.100khz +15 |  |
| rosetta_events | `events.any` | trigger: A any edge | yes | Events.event_triggers |  |
| rosetta_events | `events.bothlow` | trigger: A and B both low | yes | Events.event_triggers |  |
| rosetta_events | `events.b_high` | B must be high | yes | Events.event_triggers, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +4 |  |
| rosetta_events | `events.b_low` | B must be low | yes | Events.event_triggers |  |
| rosetta_events | `events.k_window` | confirm window K | yes | Events.event_triggers, Events.wait_timeout_in_period_units, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz +5 |  |
| rosetta_events | `events.clear` | EVT clear of a pending bit | yes | Events.event_triggers, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +8 | SharedLogic.event_abandons_a_field, SpiOnRtl.against_hand_written, SpiOnRtl.mode0 +3 |
| rosetta_events | `events.vector` | an armed event vectors the lane | yes | SharedLogic.event_abandons_a_field, StuffingAndCoding.stuff_violation_raises_event_3, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz +10 |  |
| rosetta_pins | `pins.pin` | PIN sets value and enable | yes | StuffingAndCoding.nrzi_round_trip, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +13 | Events.event_triggers, Events.wait_timeout_in_period_units, SharedLogic.crossbar_internal_loopback +10 |
| rosetta_pins | `pins.pin_wait` | PIN with a wait | yes | I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest, I2cHoldOnRtl.no_confirm_window +8 | Events.event_triggers |
| rosetta_pins | `pins.release_on_halt` | halt releases every output | yes | SharedLogic.step_pause_halt_restart, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +4 |  |
| rosetta_lane | `lane.0` | lane 0 runs a program | yes | I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest, I2cHoldOnRtl.no_confirm_window +12 | CheckUnit.crc16_ccitt_round_trip, CheckUnit.crc_known_answers, Events.event_triggers +21 |
| rosetta_lane | `lane.1` | lane 1 runs a program | yes | board:uart_async_sweep, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +13 | CheckUnit.crc16_ccitt_round_trip, Events.event_triggers, SharedLogic.channels_between_lanes +11 |
| rosetta_lane | `lane.2` | lane 2 runs a program | yes | board:uart_async_sweep, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +13 |  |
| rosetta_lane | `lane.3` | lane 3 runs a program | yes | I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest, I2cHoldOnRtl.no_confirm_window +8 |  |
| rosetta_seq | `seq.alu` | ALU operations | yes | SharedLogic.alu_and_memory, SharedLogic.step_pause_halt_restart, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz +5 | CheckUnit.crc16_ccitt_round_trip, CheckUnit.crc_known_answers, SharedLogic.channels_between_lanes +11 |
| rosetta_seq | `seq.alu_flags` | Z and N flags | yes | SharedLogic.alu_and_memory, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +4 |  |
| rosetta_seq | `seq.mem_ld_st` | LD / ST in the lane's data region | yes | SharedLogic.alu_and_memory | CheckUnit.crc_known_answers |
| rosetta_seq | `seq.movsr` | MOVSR / MOVSRH | yes | SharedLogic.alu_and_memory | SharedLogic.event_abandons_a_field, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz +7 |
| rosetta_seq | `seq.loop` | LOOP | yes | CheckUnit.crc16_ccitt_round_trip, SharedLogic.alu_and_memory | CheckUnit.crc_known_answers, SharedLogic.crossbar_internal_loopback, SharedLogic.host_queues_full_and_empty +12 |
| rosetta_seq | `seq.br_queue` | BR on QE / QNE / QF / QNF | yes | SharedLogic.host_queues_full_and_empty | SpiOnRtl.against_hand_written, SpiOnRtl.mode0, SpiOnRtl.mode1 +2 |
| rosetta_seq | `seq.br_event` | BR on an event pending bit | yes | Events.event_triggers |  |
| rosetta_seq | `seq.wait_event` | WAIT on events | yes | Events.event_triggers, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +13 |  |
| rosetta_seq | `seq.wait_timeout` | WAIT timeout | yes | Events.event_triggers, Events.wait_timeout_in_period_units | CheckUnit.crc16_ccitt_round_trip, SharedLogic.channels_between_lanes, SharedLogic.crossbar_internal_loopback +12 |
| rosetta_seq | `seq.pushts` | PUSHTS (timestamp record) | yes | Events.event_triggers | I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +13 |
| rosetta_seq | `seq.cap_records` | CAP records (listener timestamp + value) | yes | board:uart_async_sweep, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +13 |  |
| rosetta_seq | `seq.halt` | HALT (with attention) | yes | CheckUnit.crc16_ccitt_round_trip, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +4 | CheckUnit.crc_known_answers, Events.event_triggers, Events.wait_timeout_in_period_units +19 |
| rosetta_seq | `seq.reserved_halt` | a reserved opcode halts the lane | yes | SharedLogic.step_pause_halt_restart |  |
| rosetta_seq | `seq.host_halt` | host CTRL.run = 0 halts a lane | yes | SharedLogic.step_pause_halt_restart |  |
| rosetta_seq | `seq.restart` | restart after a halt (PC, CTRL.run) | yes | SharedLogic.step_pause_halt_restart |  |
| rosetta_seq | `seq.pause` | pause | yes | SharedLogic.step_pause_halt_restart |  |
| rosetta_seq | `seq.single_step` | single step | yes | SharedLogic.step_pause_halt_restart |  |
| rosetta_seq | `seq.abort` | abort: an event abandons the current operation | yes | SharedLogic.event_abandons_a_field, StuffingAndCoding.stuff_violation_raises_event_3, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz +5 |  |
| rosetta_chan | `chan.put_get` | PUT / GET between lanes | yes | SharedLogic.channels_between_lanes |  |
| rosetta_chan | `chan.get_waits` | GET waits for a channel | yes | SharedLogic.channels_between_lanes |  |
| rosetta_chan | `chan.putsr` | PUTSR (32 bits) | yes | SharedLogic.channels_between_lanes |  |
| rosetta_chan | `chan.br_valid` | BR on channel valid | yes | SharedLogic.channels_between_lanes |  |
| rosetta_chan | `chan.host_chstat` | host CHSTAT | yes | SharedLogic.channels_between_lanes |  |
| rosetta_chan | `chan.host_chdata` | host CHDATA | yes | SharedLogic.channels_between_lanes |  |
| rosetta_chan | `chan.host_clear` | host clears a channel (W1C) | yes | SharedLogic.channels_between_lanes |  |
| rosetta_chan | `chan.timestamp` | timestamp counter (records, TIME) | yes | Events.event_triggers, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +13 |  |
| rosetta_xbar | `xbar.insel` | lane input selects (XIN) | yes | SharedLogic.crossbar_internal_loopback, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +13 |  |
| rosetta_xbar | `xbar.outsel` | output selects (XOUT) to uo | yes | SharedLogic.crossbar_internal_loopback, SpiOnRtl.against_hand_written, SpiOnRtl.mode0, SpiOnRtl.mode1 +6 |  |
| rosetta_xbar | `xbar.uio_out` | outputs to uio pins | yes | SharedLogic.crossbar_internal_loopback, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +4 |  |
| rosetta_xbar | `xbar.uio_oe` | uio enables follow the lanes | yes | SharedLogic.crossbar_internal_loopback, I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest +4 |  |
| rosetta_xbar | `xbar.uio_in` | lanes read uio pins (open-drain buses) | yes | I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest, I2cHoldOnRtl.no_confirm_window +3 |  |
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
| rosetta_mem_prog | `mem.prog_load` | program load and read back through the host | yes | I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest, I2cHoldOnRtl.no_confirm_window +12 | CheckUnit.crc16_ccitt_round_trip, CheckUnit.crc_known_answers, Events.event_triggers +22 |
| rosetta_mem_prog | `mem.prog_full` | all 512 program words | yes | SharedLogic.memories_full_range |  |
| rosetta_mem_data | `mem.data_full` | all 512 data words through the host | yes | SharedLogic.memories_full_range |  |
| rosetta_mem_data | `mem.queues` | host-in / host-out queues | yes | I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest, I2cHoldOnRtl.no_confirm_window +12 |  |
| rosetta_mem_data | `mem.queue_full_waits` | a push waits while the host-out queue is full | yes | SharedLogic.host_queues_full_and_empty |  |
| rosetta_mem_data | `mem.queue_empty_waits` | a POP waits while the host-in queue is empty | yes | SharedLogic.host_queues_full_and_empty |  |
| rosetta_mem_data | `mem.qcfg_sizes` | queue sizes other than 64 (QCFG) | yes | SharedLogic.host_queues_full_and_empty |  |
| rosetta_mem_data | `mem.data_lane` | lane data memory (LD/ST, MEM) | yes | SharedLogic.alu_and_memory | CheckUnit.crc_known_answers |
| top | `top.reset` | reset and reset synchroniser | yes | I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest, I2cHoldOnRtl.no_confirm_window +12 |  |
| top | `top.four_lanes` | four lanes at once with the host | yes | I2cHoldOnRtl.100khz, I2cHoldOnRtl.400khz, I2cHoldOnRtl.fastest, I2cHoldOnRtl.no_confirm_window +8 |  |
| top | `top.async_input` | an asynchronous far end on a pin (board only) | yes | board:uart_async_sweep |  |

Features without a passing board test: 0

## Not exercisable on this board

- **Pad and Tiny Tapeout mux timing.** The chip's pins are FPGA nets: no pads, no TT mux, no board wiring (the headers are not connected). Output delay, slew, input thresholds and the host.md 2.1 margins for pad delay cannot be measured.
- **The IHP SRAM macro.** The FPGA uses the behavioural memory (ROSETTA_SRAM_BEHAV, block RAM); the macro, its BIST pins and its 0.92 ns hold need are ASIC-only.
- **ASIC timing at 60 MHz.** FPGA timing closure says nothing about the IHP gate-level paths; that is the layout flow's STA.
- **Asynchronous reset release.** The harness releases rst_n between chip edges (clock stopped); the reset synchroniser's behaviour with a release near a clock edge is not exercised.
- **Synchroniser metastability.** Every lockstep input changes between chip edges; only the async UART input (one ui pin) is truly asynchronous, and metastability resolution is not observable.
- **ena input.** Tiny Tapeout's ena is unused by the design (top.md).
- **Host SPI at other rates or with an asynchronous SCLK.** The lockstep host engine plays exactly the Verilator harness's f_clk/16 mode-0 frames, synchronous to the chip clock.

# TODO

Lower-priority issues found in code review. Nothing here blocks the current firmware from running.

## Robustness
- [ ] No CAN-silence timeout. If the ECM stops transmitting, the tach, speedo, and warning flags freeze at their last values. Zero `rpm` and `mph` and clear the flags if no frame arrives within a timeout.
- [ ] Only one CAN frame is read per main-loop tick. Drain the RX buffers with `while (CAN_MSGAVAIL == CAN.checkReceive())` so a burst can't overflow the MCP2515's two buffers.

## Unfinished features
- [ ] Tach / speedo scaling: `RPM2HZ` assumes 1 pulse/rev (the NB tach is probably 2 pulses/rev), and `MPH2HZ = 1` is a placeholder. The `static_assert`s fail the build if the scaling makes the interval at `MIN_RPM` / `MIN_MPH` overflow 16 bits.
- [ ] `COOL_PWM_HIGHHIGH` (255) and the other coolant PWM breakpoints are uncalibrated.
- [ ] **AC compressor (do last):** fill the two empty slots in `loop()`. Send 0x1F1 Platform General Status (mode request Engage/Disengage from `AC_req`, power mode Run, VDA available) and 0x4E9 Platform Configuration Data (compressor type). Add an `AC_REQ_ACTIVE` define for the active level of `AC_req`. Don't receive 0x3D1. The reference implementation is `test_scripts/ac_compressor_cycle.py`.

## Open design questions
- [ ] `flag_cool_hot` isn't gated on `RPM_ENGINE_RUNNING`, so after a hot shutdown the SRS blinks at key-on until the coolant drops below `COOL_T_HIGH`. Oil and voltage are gated. Gate coolant too?
- [ ] `flag_cool_hot` has no hysteresis. A reading sitting at `COOL_T_HIGH` will toggle the warning on and off. Add a deadband?

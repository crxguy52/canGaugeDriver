# TODO

Lower-priority issues found in code review. Nothing here blocks the current firmware from running.

##TODO
- [ ] Port to FD board
- [ ] Calibrate coolant / fuel gauge outputs

## Robustness
- [ ] No CAN-silence timeout. If the ECM stops transmitting, the tach, speedo, and warning flags freeze at their last values. Zero `rpm` and `mph` and clear the flags if no frame arrives within a timeout.


## Open design questions
- [ ] `flag_cool_hot` isn't gated on `RPM_ENGINE_RUNNING`, so after a hot shutdown the SRS blinks at key-on until the coolant drops below `COOL_T_HIGH`. Oil and voltage are gated. Gate coolant too?
- [ ] `flag_cool_hot` has no hysteresis. A reading sitting at `COOL_T_HIGH` will toggle the warning on and off. Add a deadband?

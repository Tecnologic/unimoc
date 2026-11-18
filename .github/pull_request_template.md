<!--
Title = squash commit = change-log entry (docs/release_plan.md):
  type(scope)!: subject

  type   feat (minor) | fix, perf, refactor (patch) |
         docs, test, ci, build, chore, style (no release, not in the change log)
  scope  optional: motor_control, units, settings, cyphal, dsdl, drive, vdc,
         vehicle, startup, trace, current_control, jacobi, h7, sim, hardware,
         ci, docs, release
  !      breaking change (raises the minor version while < 1.0). Always for a
         DSDL, register or subject-ID change.

  e.g.  fix(jacobi): devicetree to the PCB layout rev 0.3
        feat(dsdl)!: readiness subject per drive
-->

## Summary

<!-- What changes and why. -->

## Breaking changes

<!-- Bus protocol, registers, settings, behavior. Delete if none. -->

## Hardware

<!-- Pins, current limits, PWM, dead time, power-stage enable, startup:
     what changes and on which board it was validated. Delete if none. -->

## Testing

- [ ] `scripts/check_format.sh`
- [ ] `scripts/twister_sca.sh`
- [ ] Docs updated (`README.md`, `CONTEXT.md`, `docs/`) where behavior changed

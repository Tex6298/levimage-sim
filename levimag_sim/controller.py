from levimag_sim.model import Mode, State, PlantParams

def next_mode(mode: Mode, s: State, rpm_target: float, flags: dict) -> Mode:
    """State machine transitions based on commands and conditions."""
    if not flags.get('interlocks_ok', True):
        return Mode.FAULT

    rpm = s.omega * 60.0 / (2*3.141592653589793)
    hold_band = flags.get('hold_band', 50.0)
    rpm_min = flags.get('rpm_min', 100.0)

    if mode == Mode.IDLE and flags.get('cmd_start', False):
        return Mode.SPINUP

    if mode == Mode.SPINUP and rpm >= max(0.0, rpm_target - hold_band):
        return Mode.HOLD

    if mode in (Mode.SPINUP, Mode.HOLD) and flags.get('cmd_brake', False):
        return Mode.BRAKE

    if mode == Mode.BRAKE and (rpm <= rpm_min or flags.get('cmd_stop', False)):
        return Mode.IDLE

    return mode

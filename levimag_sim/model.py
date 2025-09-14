from dataclasses import dataclass
import enum
import numpy as np

@dataclass
class PlantParams:
    I: float                # rotor inertia [kg·m^2]
    Kt: float               # torque constant [N·m/A]
    R: float                # coil resistance [ohm] at T_ref
    alpha_R: float          # tempco of R [/K]
    T_ref: float            # reference coil temp [K]
    C_th: float             # coil thermal capacity [J/K]
    h_th: float             # thermal loss coeff to ambient [W/K]
    Tamb: float             # ambient temp [K]
    ke: float               # eddy drag torque coeff -> tau_e = ke*ω   [N·m·s]
    kg: float               # gas drag torque coeff  -> tau_g = kg*ω   [N·m·s]
    kv: float               # viscous-like torque    -> tau_v = kv*ω^2 [N·m·s^2]
    c_mech: float           # speed-independent torque [N·m]
    rpm_limit: float        # overspeed threshold
    T_limit: float          # coil temp limit
    i_max: float            # driver current limit [A]
    duty_max: float         # max pulse duty per rev (0..1)
    pulses_per_rev: int     # pulses per revolution

@dataclass
class State:
    theta: float            # [rad]
    omega: float            # [rad/s]
    Tcoil: float            # [K]
    t: float                # [s]

class Mode(enum.Enum):
    IDLE  = 0
    SPINUP= 1
    HOLD  = 2
    BRAKE = 3
    FAULT = 4


def pulse_torque_command(theta, omega, mode, set_rpm, p: PlantParams):
    """Compute commanded current and duty based on position and mode (simple window).
    Includes a **start-from-rest** helper: when |ω| is tiny, always allow a pulse so
    the rotor receives an initial kick instead of waiting for a phase window it will
    never reach.
    """
    rpm = omega * 60.0 / (2*np.pi)

    # ---- Start-from-rest helper ----
    omega_min = 1e-3  # rad/s threshold considered "stopped"
    force_window = abs(omega) < omega_min

    # Sector/window logic
    sector_width = 2*np.pi / max(1, p.pulses_per_rev)
    th_in_sector = (theta % sector_width)
    center = 0.5 * sector_width
    window_half = 0.05 * sector_width  # 10% of sector
    in_window = abs(th_in_sector - center) < window_half or force_window

    i_cmd = 0.0
    if mode == Mode.SPINUP and in_window:
        err = (set_rpm - rpm)
        i_cmd = np.clip(0.01*err, 0, p.i_max)  # simple proportional
    elif mode == Mode.HOLD and in_window:
        err = (set_rpm - rpm)
        i_cmd = np.clip(0.02*err, 0, p.i_max)
    elif mode == Mode.BRAKE and in_window:
        i_cmd = -min(p.i_max, 0.02 * rpm)

    # Duty: ensure a tiny minimum during SPINUP from rest so a pulse happens
    duty = 2*window_half/sector_width
    if mode == Mode.SPINUP and force_window:
        duty = max(duty, min(0.02, p.duty_max))  # up to 2% per rev at start
    duty = min(duty, p.duty_max)
    return i_cmd, duty
def plant_derivatives(s: State, mode: Mode, set_rpm: float, p: PlantParams):
    # Commanded current & duty
    i_cmd, duty = pulse_torque_command(s.theta, s.omega, mode, set_rpm, p)

    # Resistance vs temperature
    R_T = p.R * (1 + p.alpha_R * (s.Tcoil - p.T_ref))
    i = np.clip(i_cmd, -p.i_max, p.i_max)

    # Average drive torque over dt
    tau_drive = p.Kt * i * duty

    # Parasitic torques
    tau_par = (p.ke + p.kg) * s.omega + p.kv * s.omega * abs(s.omega) + np.sign(s.omega)*p.c_mech

    # Dynamics
    domega = (tau_drive - tau_par) / p.I
    dtheta = s.omega
    # Electrical heating (average): I^2 R * duty; minus cooling
    P_joule = (i**2) * R_T * duty
    dT = (P_joule - p.h_th * (s.Tcoil - p.Tamb)) / p.C_th

    return dtheta, domega, dT

def step(s: State, mode: Mode, set_rpm: float, p: PlantParams, dt: float):
    dtheta, domega, dT = plant_derivatives(s, mode, set_rpm, p)
    s.theta = (s.theta + dtheta * dt) % (2*np.pi)
    s.omega = s.omega + domega * dt
    s.Tcoil = s.Tcoil + dT * dt
    s.t     = s.t + dt

    rpm = s.omega * 60/(2*np.pi)
    if rpm > p.rpm_limit or s.Tcoil > p.T_limit or np.isnan(s.omega):
        return s, Mode.FAULT
    return s, mode

from dataclasses import dataclass
import enum, numpy as np

@dataclass
class PlantParams:
    I: float
    Kt: float
    R: float
    alpha_R: float
    T_ref: float
    C_th: float
    h_th: float
    Tamb: float
    ke: float
    kg: float
    kv: float
    c_mech: float
    rpm_limit: float
    T_limit: float
    i_max: float
    duty_max: float
    pulses_per_rev: int

@dataclass
class State:
    theta: float
    omega: float
    Tcoil: float
    t: float

class Mode(enum.Enum):
    IDLE=0; SPINUP=1; HOLD=2; BRAKE=3; FAULT=4

def pulse_torque_command(theta, omega, mode, set_rpm, p: PlantParams):
    rpm = omega * 60.0 / (2*np.pi)
    # start-from-rest helper
    omega_min = 1e-3
    force_window = abs(omega) < omega_min
    sector_width = 2*np.pi / max(1, p.pulses_per_rev)
    th_in_sector = (theta % sector_width)
    center = 0.5 * sector_width
    window_half = 0.05 * sector_width
    in_window = abs(th_in_sector - center) < window_half or force_window

    i_cmd = 0.0
    if mode == Mode.SPINUP and in_window:
        err = (set_rpm - rpm)
        i_cmd = float(np.clip(0.01*err, 0, p.i_max))
    elif mode == Mode.HOLD and in_window:
        err = (set_rpm - rpm)
        i_cmd = float(np.clip(0.02*err, 0, p.i_max))
    elif mode == Mode.BRAKE and in_window:
        i_cmd = -min(p.i_max, 0.02 * rpm)

    duty = 2*window_half/sector_width
    if mode == Mode.SPINUP and force_window:
        duty = max(duty, min(0.02, p.duty_max))
    duty = min(duty, p.duty_max)
    return i_cmd, duty

def plant_derivatives(s: State, mode: Mode, set_rpm: float, p: PlantParams):
    i_cmd, duty = pulse_torque_command(s.theta, s.omega, mode, set_rpm, p)
    R_T = p.R * (1 + p.alpha_R * (s.Tcoil - p.T_ref))
    i = float(np.clip(i_cmd, -p.i_max, p.i_max))
    tau_drive = p.Kt * i * duty
    tau_par = (p.ke + p.kg) * s.omega + p.kv * s.omega * abs(s.omega) + np.sign(s.omega)*p.c_mech
    domega = (tau_drive - tau_par) / p.I
    dtheta = s.omega
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

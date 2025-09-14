# Levitated Magnet — Pulsed Drive Simulator

A runnable **Dash** web app for exploring a levitating permanent‑magnet rotor with
**pulsed acceleration/braking**, **loss models** (eddy, gas, viscous, mechanical), a simple
**thermal model**, and a **state machine** (IDLE → SPINUP → HOLD → BRAKE → FAULT).

> **Why this exists:** to prototype control strategies and loss budgets without heavyweight FE tools.
> It’s ideal for early design, controller tuning, and “what‑if” studies. Bring in FE (COMSOL/Maxwell)
> later only to validate geometry‑sensitive constants and containment stresses.

---

## ✨ Features
- Pulsed, phase‑windowed drive with adjustable **pulses per revolution** and **max duty**.
- Loss terms scaling with speed: ω, ω², ω³ (gas, eddy, viscous) + a constant torque term.
- Lumped coil thermal model (I²R heating, ambient cooling, temperature‑dependent resistance).
- State machine with interlocks and guardrails (overspeed/overtemp ⇒ **FAULT** with reset).
- Live plots: **RPM**, **loss power**, **coil temperature**.
- Clean separation of concerns: `model.py` (ODE), `controller.py` (state machine), `app.py` (UI).

---

## 🚀 Quickstart

```bash
git clone <your-repo-url>
cd levimag-sim

python -m venv .venv
# Windows: .venv\Scripts\activate
# macOS/Linux:
source .venv/bin/activate

pip install -r requirements.txt
python app.py
# open http://127.0.0.1:8050
```

---

## 🧠 Modeling overview

The rotor is modeled as:
- Angle θ, speed ω, and coil temperature T.
- Drive torque is **pulsed** once (or N times) per revolution within a narrow phase window.
- Average torque in each small timestep is `τ_drive = Kt · i_cmd · duty`.
- Parasitic torques: `τ_par = (ke + kg)·ω + kv·ω·|ω| + sign(ω)·c_mech`.
- Coil heating: `P_joule = i² R(T) · duty`, with `R(T) = R·(1 + α·ΔT)` and linear cooling.

This captures the dominant speed dependencies: **eddy (ω² power)** and **gas (ω² in free‑molecular)**,
optionally **viscous (ω³ power)** if the vacuum is poor.

---

## 🧰 UI controls

- **Targets**: `rpm_target` + buttons *(Start, Brake, Stop, Reset Fault)*.
- **Rotor & Losses**: `I, ke, kg, kv, c_mech`.
- **Coil & Driver**: `Kt, R, α_R, I_max, duty_max`.
- **Thermal**: `C_th, h_th, T_amb, T_ref, T_limit`.
- **Safety**: `rpm_limit`, `pulses_per_rev`.
- **Interlocks**: simple checkboxes for vacuum/containment.

---

## 🧪 Suggested presets

You can type these directly in the UI to explore extremes:

### Glass chamber @ 1e‑7 torr (near‑ideal)
- `ke = 1e-6`, `kg = 1e-6`, `kv = 0`, `c_mech = 0`  
- Expect **very long** coast‑down; tiny duty needed to hold speed.

### Slotted steel wall at 10 cm (eddy‑lossy)
- `ke = 1e-3`, `kg = 1e-6`, `kv = 0`, `c_mech = 0`  
- Expect steep **ω²** loss; more coil heating and much shorter coast.

*(Tune from experiments: perform coast‑down fits for `ke`,`kg`; pressure sweeps help separate them.)*

---

## 📦 Project layout

```
levimag-sim/
├─ app.py                 # Dash UI and simulation loop
├─ requirements.txt
├─ LICENSE                # Proprietary (non‑commercial without permission)
├─ README.md
└─ levimag_sim/
   ├─ __init__.py
   ├─ model.py            # ODEs, parameters, integrator step
   └─ controller.py       # State machine & transitions
```

---

## 🔒 License (summary)

This repository is **not open source**. It is provided for personal review and non‑commercial
evaluation. **Commercial use, redistribution, or modification requires written permission** from
the copyright holder. See [LICENSE](./LICENSE) for the full text.

If you’d like a commercial or academic license, open an issue or email the maintainer.

---

## 🗺️ Roadmap

- Preset loader/saver (JSON) and multi‑scenario compare.
- Separate plots for **eddy vs gas vs viscous** power.
- Optional hardware‑in‑the‑loop via `pyserial`/`pyvisa`.
- Import loss constants from FEMM/COMSOL tables.
- CSV/Parquet export of traces with metadata.

---

## 🙌 Citation

If this simulator informs your research or design, please cite the repo URL and version tag.
A short acknowledgment in papers/talks is appreciated.

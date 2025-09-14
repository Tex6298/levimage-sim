from dash import Dash, dcc, html, Input, Output, State as DashState, ctx
import plotly.graph_objects as go
import numpy as np, os, json
from levimag_sim.model import PlantParams, State as SimState, Mode, step
from levimag_sim.controller import next_mode

def list_presets():
    preset_dir = os.path.join(os.path.dirname(__file__), "presets")
    try:
        files = sorted([f for f in os.listdir(preset_dir) if f.endswith(".json")])
    except FileNotFoundError:
        files = []
    return [{"label": f, "value": f} for f in files]

def load_preset_params(filename):
    preset_dir = os.path.join(os.path.dirname(__file__), "presets")
    path = os.path.join(preset_dir, filename)
    with open(path, "r") as fp:
        data = json.load(fp)
    return data.get("params", {}), data.get("name", filename)

app = Dash(__name__)
server = app.server

def label_input(label, id, value, step=None, placeholder=None):
    return html.Div([
        html.Label(label, htmlFor=id, style={"fontSize":"0.9rem"}),
        dcc.Input(id=id, type="number", value=value, step=step, placeholder=placeholder,
                  style={"width":"100%","marginBottom":"6px"})
    ])

app.layout = html.Div([
    html.H2("Levitated Magnet – Pulsed Drive Simulator"),
    html.Div([
        html.Div([
            html.H4("Targets"),
            html.Div("Preset"),
            dcc.Dropdown(id="preset_select", options=list_presets(), value=None, placeholder="Select a preset..."),
            html.Button("Load Preset", id="btn_load_preset", n_clicks=0, className="btn", style={"marginTop":"6px"}),
            html.Div("Target RPM"),
            dcc.Slider(id="rpm_target", min=0, max=60000, step=50, value=3000, tooltip={"placement":"bottom"}),
            html.Div([
                html.Button("Start", id="btn_start", n_clicks=0, className="btn"),
                html.Button("Brake", id="btn_brake", n_clicks=0, className="btn"),
                html.Button("Pause", id="btn_pause", n_clicks=0, className="btn"),
                html.Button("Reset Plots", id="btn_reset_plots", n_clicks=0, className="btn"),
                html.Button("Stop",  id="btn_stop",  n_clicks=0, className="btn"),
                html.Button("Reset Fault", id="btn_reset", n_clicks=0, className="btn"),
            ], style={"display":"flex","gap":"8px","marginTop":"8px","flexWrap":"wrap"}),

            html.H4("Rotor & Losses"),
            label_input("Inertia I [kg·m²]", "I", 0.02, 0.001),
            label_input("Eddy coeff ke [N·m·s]", "ke", 1e-4, 1e-5),
            label_input("Gas coeff kg [N·m·s]", "kg", 5e-5, 1e-5),
            label_input("Viscous kv [N·m·s²]", "kv", 0.0, 1e-6),
            label_input("Speed-indep. torque c_mech [N·m]", "cmech", 0.0, 1e-5),

            html.H4("Coil & Driver"),
            label_input("Torque const Kt [N·m/A]", "Kt", 0.05, 0.001),
            label_input("Coil resistance R @ Tref [Ω]", "R", 2.0, 0.01),
            label_input("Tempco alpha_R [/K]", "alphaR", 0.0039, 1e-4),
            label_input("Max current Imax [A]", "imax", 5.0, 0.1),
            html.Div("Max duty per rev"),
            dcc.Slider(id="duty_max", min=0.0, max=0.2, step=0.0025, value=0.02),

            html.H4("Thermal"),
            label_input("Coil thermal capacity C_th [J/K]", "Cth", 200.0, 1.0),
            label_input("Thermal loss coeff h_th [W/K]", "hth", 0.8, 0.05),
            label_input("Ambient T [K]", "Tamb", 293.15, 0.1),
            label_input("T_ref [K]", "Tref", 293.15, 0.1),
            label_input("Temp limit [K]", "Tlim", 363.15, 0.1),

            html.H4("Safety"),
            label_input("RPM limit", "rpm_limit", 12000, 10),
            label_input("Pulses per rev", "pulses_per_rev", 1, 1),

            html.H4("Interlocks"),
            dcc.Checklist(
                id="interlocks",
                options=[{"label":"Vacuum OK","value":"vac_ok"},
                         {"label":"Containment OK","value":"cont_ok"}],
                value=["vac_ok","cont_ok"]
            ),
        ], style={"width":"32%","display":"inline-block","verticalAlign":"top","padding":"10px","borderRight":"1px solid #eee"}),

        html.Div([
            html.Div(id="mode_badge", style={"fontWeight":"600","marginBottom":"6px"}),
            html.Div(id="advisor_panel", style={"background":"#fff7e6","border":"1px solid #ffd591","padding":"8px","borderRadius":"6px","marginBottom":"8px"}),
            dcc.Graph(id="rpm_plot", config={"displaylogo": False}),
            dcc.Graph(id="power_plot", config={"displaylogo": False}),
            dcc.Graph(id="temp_plot", config={"displaylogo": False}),
        ], style={"width":"66%","display":"inline-block","padding":"10px"}),
    ]),

    dcc.Interval(id="tick", interval=50, n_intervals=0),
    dcc.Store(id="sim_state"),
    dcc.Store(id="sim_params"),
    dcc.Store(id="sim_flags")
])

# Params builder
@app.callback(
    Output("sim_params","data"),
    Input("I","value"), Input("Kt","value"), Input("R","value"),
    Input("alphaR","value"),
    Input("imax","value"), Input("duty_max","value"),
    Input("ke","value"), Input("kg","value"), Input("kv","value"), Input("cmech","value"),
    Input("Cth","value"), Input("hth","value"), Input("Tamb","value"), Input("Tref","value"),
    Input("Tlim","value"), Input("rpm_limit","value"), Input("pulses_per_rev","value"),
)
def update_params(I,Kt,R,alphaR,imax,duty_max,ke,kg,kv,cmech,Cth,hth,Tamb,Tref,Tlim,rpm_limit,ppr):
    p = PlantParams(I=I,Kt=Kt,R=R,alpha_R=alphaR,T_ref=Tref,
                    C_th=Cth,h_th=hth,Tamb=Tamb,ke=ke,kg=kg,kv=kv,
                    c_mech=cmech,rpm_limit=rpm_limit,T_limit=Tlim,
                    i_max=imax,duty_max=duty_max,pulses_per_rev=int(ppr))
    return p.__dict__

# Load preset
@app.callback(
    Output("I","value"),
    Output("Kt","value"),
    Output("R","value"),
    Output("alphaR","value"),
    Output("imax","value"),
    Output("duty_max","value"),
    Output("ke","value"),
    Output("kg","value"),
    Output("kv","value"),
    Output("cmech","value"),
    Output("Cth","value"),
    Output("hth","value"),
    Output("Tamb","value"),
    Output("Tref","value"),
    Output("Tlim","value"),
    Output("rpm_limit","value"),
    Output("pulses_per_rev","value"),
    Output("rpm_target","value"),
    Input("btn_load_preset","n_clicks"),
    DashState("preset_select","value"),
    prevent_initial_call=True
)
def load_preset(n_clicks, filename):
    if not filename:
        raise Exception("No preset selected")
    params, _ = load_preset_params(filename)
    rpm_limit = params.get("rpm_limit", 12000)
    rpm_target = int(0.8 * rpm_limit)
    return (
        params.get("I", 0.02),
        params.get("Kt", 0.05),
        params.get("R", 2.0),
        params.get("alpha_R", 0.0039),
        params.get("i_max", 5.0),
        params.get("duty_max", 0.02),
        params.get("ke", 1e-4),
        params.get("kg", 5e-5),
        params.get("kv", 0.0),
        params.get("c_mech", 0.0),
        params.get("C_th", 200.0),
        params.get("h_th", 0.8),
        params.get("Tamb", 293.15),
        params.get("T_ref", 293.15),
        params.get("T_limit", 363.15),
        rpm_limit,
        params.get("pulses_per_rev", 1),
        rpm_target
    )

# Commands & interlocks
@app.callback(
    Output("sim_flags","data"),
    Input("btn_start","n_clicks"), Input("btn_brake","n_clicks"),
    Input("btn_pause","n_clicks"), Input("btn_stop","n_clicks"), Input("btn_reset_plots","n_clicks"), Input("btn_reset","n_clicks"),
    DashState("interlocks","value"), DashState("rpm_target","value"),
    prevent_initial_call=False
)
def commands(n1,n2,n3,n4,locks,target):
    trig = ctx.triggered_id if ctx.triggered_id else None
    running = trig in ("btn_start","btn_brake")
    reset_hist = trig in ("btn_start","btn_reset_plots")
    return {
        "cmd_start": trig=="btn_start",
        "cmd_brake": trig=="btn_brake",
        "cmd_pause": trig=="btn_pause",
        "cmd_stop":  trig=="btn_stop",
        "cmd_reset": trig=="btn_reset",
        "interlocks_ok": ("vac_ok" in (locks or []) and "cont_ok" in (locks or [])),
        "hold_band": 50,
        "rpm_min": 100,
        "rpm_target": target or 0,
        "running": running,
        "reset_hist": reset_hist
    }

# Simulation step
@app.callback(
    Output("sim_state","data"),
    Output("rpm_plot","figure"),
    Output("power_plot","figure"),
    Output("temp_plot","figure"),
    Output("mode_badge","children"),
    Input("tick","n_intervals"),
    DashState("sim_state","data"), DashState("sim_params","data"), DashState("sim_flags","data")
)
def sim_step(n, sim_state, sim_params, flags):
    running = bool(flags.get("running")) if isinstance(flags, dict) else False
    reset_hist = bool(flags.get("reset_hist")) if isinstance(flags, dict) else False
    # init
    if sim_state is None:
        s = SimState(theta=0.0, omega=0.0, Tcoil=293.15, t=0.0)
        mode = Mode.IDLE
        hist = {"t":[], "rpm":[], "Ploss":[], "T":[]}
    else:
        s = SimState(**sim_state["state"])
        mode = Mode(sim_state["mode"])
        hist = sim_state["hist"]
        if reset_hist:
            hist = {"t":[], "rpm":[], "Ploss":[], "T":[]}
            s.t = 0.0

    # load params
    if sim_params is None:
        p = PlantParams(I=0.02,Kt=0.05,R=2.0,alpha_R=0.0039,T_ref=293.15,
                        C_th=200.0,h_th=0.8,Tamb=293.15,ke=1e-4,kg=5e-5,kv=0.0,
                        c_mech=0.0,rpm_limit=12000,T_limit=363.15,
                        i_max=5.0,duty_max=0.02,pulses_per_rev=1)
    else:
        p = PlantParams(**sim_params)

    # safe flags
    if flags is None or not isinstance(flags, dict):
        flags = {}
    rpm_target = flags.get("rpm_target", 3000)
    hold_band  = flags.get("hold_band", 50)
    rpm_min    = flags.get("rpm_min", 100)
    interlocks_ok = flags.get("interlocks_ok", True)
    flags = {
        "cmd_start": flags.get("cmd_start", False),
        "cmd_brake": flags.get("cmd_brake", False),
        "cmd_stop":  flags.get("cmd_stop", False),
        "cmd_reset": flags.get("cmd_reset", False),
        "interlocks_ok": interlocks_ok,
        "hold_band": hold_band,
        "rpm_min": rpm_min,
        "rpm_target": rpm_target,
    }

    if not running:
        rpm = s.omega * 60/(2*np.pi)
        P_eddy_gas = (p.ke + p.kg) * s.omega**2
        P_visc = p.kv * abs(s.omega**3)
        P_mech = abs((1 if s.omega>=0 else -1)*p.c_mech * s.omega)
        P_loss = P_eddy_gas + P_visc + P_mech
        fig_rpm = go.Figure().add_scatter(x=hist.get("t",[]), y=hist.get("rpm",[]), name="RPM")
        fig_rpm.update_layout(margin=dict(l=30,r=10,t=20,b=30), yaxis_title="RPM", xaxis_title="Time [s]")
        fig_power = go.Figure().add_scatter(x=hist.get("t",[]), y=hist.get("Ploss",[]), name="Loss Power")
        fig_power.update_layout(margin=dict(l=30,r=10,t=20,b=30), yaxis_title="W (approx)", xaxis_title="Time [s]")
        fig_temp = go.Figure().add_scatter(x=hist.get("t",[]), y=hist.get("T",[]), name="Coil T")
        fig_temp.update_layout(margin=dict(l=30,r=10,t=20,b=30), yaxis_title="K", xaxis_title="Time [s]")
        badge = f"Mode: {mode.name} (paused)"
        sim_state = {"state": s.__dict__, "mode": mode.value, "hist": hist}
        return sim_state, fig_rpm, fig_power, fig_temp, badge
    # mode transitions
    if mode != Mode.FAULT:
        mode = next_mode(mode, s, rpm_target, flags)

    # integrate
    dt = 0.001; steps = 50
    for _ in range(steps):
        s, mode = step(s, mode, rpm_target, p, dt)
        if mode == Mode.FAULT and not flags.get("cmd_reset"):
            break
        if mode == Mode.FAULT and flags.get("cmd_reset"):
            s = SimState(theta=0.0, omega=0.0, Tcoil=p.Tamb, t=0.0); mode = Mode.IDLE

    rpm = s.omega * 60/(2*np.pi)
    P_eddy_gas = (p.ke + p.kg) * s.omega**2
    P_visc = p.kv * abs(s.omega**3)
    P_mech = abs(np.sign(s.omega)*p.c_mech * s.omega)
    P_loss = P_eddy_gas + P_visc + P_mech

    hist["t"].append(s.t); hist["rpm"].append(rpm); hist["Ploss"].append(P_loss); hist["T"].append(s.Tcoil)
    if len(hist["t"]) > 5000:
        for k in list(hist.keys()):
            hist[k] = hist[k][-4000:]

    fig_rpm = go.Figure().add_scatter(x=hist["t"], y=hist["rpm"], name="RPM")
    fig_rpm.update_layout(margin=dict(l=30,r=10,t=20,b=30), yaxis_title="RPM", xaxis_title="Time [s]")
    fig_power = go.Figure().add_scatter(x=hist["t"], y=hist["Ploss"], name="Loss Power")
    fig_power.update_layout(margin=dict(l=30,r=10,t=20,b=30), yaxis_title="W (approx)", xaxis_title="Time [s]")
    fig_temp = go.Figure().add_scatter(x=hist["t"], y=hist["T"], name="Coil T")
    fig_temp.update_layout(margin=dict(l=30,r=10,t=20,b=30), yaxis_title="K", xaxis_title="Time [s]")
    badge = f"Mode: {mode.name}"

    sim_state = {"state": s.__dict__, "mode": mode.value, "hist": hist}
    return sim_state, fig_rpm, fig_power, fig_temp, badge


# ---- Limit Advisor: compute quick feasibility and warnings ----
@app.callback(
    Output("advisor_panel","children"),
    Input("sim_state","data"),
    Input("sim_params","data"),
    Input("rpm_target","value"),
)
def limit_advisor(sim_state, sim_params, rpm_target):
    import math
    if sim_params is None:
        return "Adjust parameters or load a preset to see advisory hints."
    # unpack params
    p = sim_params
    I = float(p.get("I", 0.02))
    Kt = float(p.get("Kt", 0.05))
    R  = float(p.get("R", 2.0))
    alphaR = float(p.get("alpha_R", 0.0039))
    Tref = float(p.get("T_ref", 293.15))
    Cth = float(p.get("C_th", 200.0))
    hth = float(p.get("h_th", 0.8))
    Tamb = float(p.get("Tamb", 293.15))
    ke = float(p.get("ke", 1e-4))
    kg = float(p.get("kg", 5e-5))
    kv = float(p.get("kv", 0.0))
    c_mech = float(p.get("c_mech", 0.0))
    rpm_limit = float(p.get("rpm_limit", 12000))
    Tlim = float(p.get("T_limit", 363.15))
    i_max = float(p.get("i_max", 5.0))
    duty_max = float(p.get("duty_max", 0.02))

    # current state
    if sim_state is None:
        omega_now = 0.0; Tnow = Tamb
    else:
        try:
            s = sim_state["state"]
            omega_now = float(s["omega"]); Tnow = float(s["Tcoil"])
        except Exception:
            omega_now = 0.0; Tnow = Tamb

    # basic derived
    tau_max = Kt * i_max * duty_max  # average torque per rev during windowed drive
    alpha0 = tau_max / max(I, 1e-12)
    omega_target = (rpm_target or 0) * 2*math.pi/60.0
    # Loss torque at target
    tau_loss_target = (ke + kg) * omega_target + kv * omega_target * abs(omega_target) + math.copysign(1.0, omega_target if omega_target!=0 else 1.0) * c_mech
    # Time to target ignoring losses (if accel > 0)
    time_to_target = math.inf
    if alpha0 > 1e-9 and omega_target > omega_now:
        time_to_target = (omega_target - omega_now)/alpha0

    # Thermal check at max drive (steady-state estimate)
    R_T = R * (1 + alphaR * (Tnow - Tref))
    P_joule = (i_max**2) * R_T * duty_max
    if hth > 1e-9:
        Teq = Tamb + P_joule / hth
    else:
        Teq = float('inf')
    # approximate time to Tlim if Teq > Tlim (first-order RC)
    time_to_Tlim = None
    if Teq > Tlim and hth > 1e-9:
        tau_th = Cth / hth
        # T(t) = Teq + (T0-Teq) * exp(-t/tau)
        # solve for t where T(t)=Tlim
        if Tnow < Tlim:
            try:
                time_to_Tlim = -tau_th * math.log((Tlim - Teq) / max(Tnow - Teq, -1e-9))
            except Exception:
                time_to_Tlim = None

    # Overspeed margin
    overspeed = rpm_target > 0.9 * rpm_limit

    msgs = []
    # feasibility
    if tau_max <= 0:
        msgs.append("❌ Max drive torque is zero. Increase Kt, Imax, or duty.")
    else:
        # can it hold target against losses?
        if tau_max < tau_loss_target:
            msgs.append("⚠️ Max torque is **below loss torque at target** — it will never reach/hold the target RPM.")
        # time to target
        if math.isfinite(time_to_target):
            if time_to_target > 300:
                msgs.append(f"⚠️ Very slow spin‑up: est. **{time_to_target:,.0f} s** to target (ignoring losses).")
            elif time_to_target > 60:
                msgs.append(f"ℹ️ Slow spin‑up: est. **{time_to_target:,.0f} s** to target (ignoring losses).")
        else:
            msgs.append("⚠️ Cannot estimate time to target (target ≤ current speed or accel ~ 0).")

    # thermal
    if Teq > Tlim:
        if time_to_Tlim is not None and time_to_Tlim > 0:
            msgs.append(f"⚠️ Thermal limit breach at max drive: **Teq≈{Teq:.1f} K**, time to T_limit ≈ **{time_to_Tlim:,.0f} s**.")
        else:
            msgs.append(f"⚠️ Thermal limit breach at max drive: **Teq≈{Teq:.1f} K**.")
    else:
        msgs.append(f"✅ Thermal OK at max drive (Teq≈{Teq:.1f} K).")

    # overspeed margin
    if overspeed:
        msgs.append("⚠️ Target RPM is within 10% of RPM limit. Reduce target or raise limit with care.")

    # summaries
    msgs.insert(0, f"τ_max≈{tau_max:.3g} N·m, α≈{alpha0:.3g} rad/s², ω_now≈{omega_now:.3g} rad/s.")
    return [html.Div(m) for m in msgs] if msgs else "No issues detected."


if __name__ == "__main__":
    app.run(debug=True, use_reloader=False, host="127.0.0.1", port=8050)

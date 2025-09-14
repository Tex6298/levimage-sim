from dash import Dash, dcc, html, Input, Output, State, ctx
import plotly.graph_objects as go
import numpy as np
from levimag_sim.model import PlantParams, State, Mode, step
from levimag_sim.controller import next_mode

app = Dash(__name__)
server = app.server  # for gunicorn if deployed

def label_input(label, id, value, step=None, placeholder=None):
    return html.Div([
        html.Label(label, htmlFor=id, style={"fontSize":"0.9rem"}),
        dcc.Input(id=id, type="number", value=value, step=step, placeholder=placeholder,
                  style={"width":"100%","marginBottom":"6px"})
    ])

app.layout = html.Div([
    html.H2("Levitated Magnet – Pulsed Drive Simulator"),
    html.Div([
        # ==== Controls column ====
        html.Div([
            html.H4("Targets"),
            html.Div("Target RPM"),
            dcc.Slider(id="rpm_target", min=0, max=20000, step=50, value=3000,
                       tooltip={"placement":"bottom"}),
            html.Div([
                html.Button("Start", id="btn_start", n_clicks=0, className="btn"),
                html.Button("Brake", id="btn_brake", n_clicks=0, className="btn"),
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

        # ==== Plots & status ====
        html.Div([
            html.Div(id="mode_badge", style={"fontWeight":"600","marginBottom":"6px"}),
            dcc.Graph(id="rpm_plot", config={"displaylogo": False}),
            dcc.Graph(id="power_plot", config={"displaylogo": False}),
            dcc.Graph(id="temp_plot", config={"displaylogo": False}),
        ], style={"width":"66%","display":"inline-block","padding":"10px"}),
    ]),

    dcc.Interval(id="tick", interval=50, n_intervals=0),  # 20 Hz UI update
    dcc.Store(id="sim_state"),
    dcc.Store(id="sim_params"),
    dcc.Store(id="sim_flags")
])

# Assemble PlantParams from UI
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

# Commands & interlocks
@app.callback(
    Output("sim_flags","data"),
    Input("btn_start","n_clicks"), Input("btn_brake","n_clicks"),
    Input("btn_stop","n_clicks"), Input("btn_reset","n_clicks"),
    Input("interlocks","value"), State("rpm_target","value"),
    prevent_initial_call=False
)
def commands(n1,n2,n3,n4,locks,target):
    trig = ctx.triggered_id if ctx.triggered_id else None
    return {
        "cmd_start": trig=="btn_start",
        "cmd_brake": trig=="btn_brake",
        "cmd_stop":  trig=="btn_stop",
        "cmd_reset": trig=="btn_reset",
        "interlocks_ok": ("vac_ok" in (locks or []) and "cont_ok" in (locks or [])),
        "hold_band": 50,
        "rpm_min": 100,
        "rpm_target": target or 0
    }

# Simulation step
@app.callback(
    Output("sim_state","data"),
    Output("rpm_plot","figure"),
    Output("power_plot","figure"),
    Output("temp_plot","figure"),
    Output("mode_badge","children"),
    Input("tick","n_intervals"),
    State("sim_state","data"), State("sim_params","data"), State("sim_flags","data")
)
def sim_step(n, sim_state, sim_params, flags):
    # init
    if sim_state is None:
        s = State(theta=0.0, omega=0.0, Tcoil=293.15, t=0.0)
        mode = Mode.IDLE
        hist = {"t":[], "rpm":[], "Ploss":[], "T":[]}
    else:
        s = State(**sim_state["state"])
        mode = Mode(sim_state["mode"])
        hist = sim_state["hist"]

    # load params
    if sim_params is None:
        # default params
        p = PlantParams(I=0.02,Kt=0.05,R=2.0,alpha_R=0.0039,T_ref=293.15,
                        C_th=200.0,h_th=0.8,Tamb=293.15,ke=1e-4,kg=5e-5,kv=0.0,
                        c_mech=0.0,rpm_limit=12000,T_limit=363.15,
                        i_max=5.0,duty_max=0.02,pulses_per_rev=1)
    else:
        p = PlantParams(**sim_params)

    # mode transitions unless fault
    if flags is None:
        flags = {"interlocks_ok": True, "rpm_target": 3000, "cmd_start": False, "cmd_brake": False, "cmd_stop": False, "cmd_reset": False, "hold_band":50, "rpm_min":100}
    if mode != Mode.FAULT:
        mode = next_mode(mode, s, flags["rpm_target"], flags)

    # integrate several 1 ms steps per UI tick
    dt = 0.001
    steps = 50
    for _ in range(steps):
        s, mode = step(s, mode, flags["rpm_target"], p, dt)
        if mode == Mode.FAULT and not flags.get("cmd_reset"): 
            break
        if mode == Mode.FAULT and flags.get("cmd_reset"):
            s = State(theta=0.0, omega=0.0, Tcoil=p.Tamb, t=0.0); mode = Mode.IDLE

    rpm = s.omega * 60/(2*np.pi)

    # approximate loss power breakdown for plotting
    P_eddy_gas = (p.ke + p.kg) * s.omega**2
    P_visc = p.kv * abs(s.omega**3)
    P_mech = abs(np.sign(s.omega)*p.c_mech * s.omega)
    P_loss = P_eddy_gas + P_visc + P_mech

    hist["t"].append(s.t); hist["rpm"].append(rpm); hist["Ploss"].append(P_loss); hist["T"].append(s.Tcoil)

    fig_rpm = go.Figure().add_scatter(x=hist["t"], y=hist["rpm"], name="RPM")
    fig_rpm.update_layout(margin=dict(l=30,r=10,t=20,b=30), yaxis_title="RPM", xaxis_title="Time [s]")

    fig_power = go.Figure().add_scatter(x=hist["t"], y=hist["Ploss"], name="Loss Power")
    fig_power.update_layout(margin=dict(l=30,r=10,t=20,b=30), yaxis_title="W (approx)", xaxis_title="Time [s]")

    fig_temp = go.Figure().add_scatter(x=hist["t"], y=hist["T"], name="Coil T")
    fig_temp.update_layout(margin=dict(l=30,r=10,t=20,b=30), yaxis_title="K", xaxis_title="Time [s]")

    badge = f"Mode: {mode.name}"

    sim_state = {"state": s.__dict__, "mode": mode.value, "hist": hist}
    return sim_state, fig_rpm, fig_power, fig_temp, badge

if __name__ == "__main__":
    app.run_server(debug=True)

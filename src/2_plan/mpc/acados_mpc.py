# %%
import os
import numpy as np
import casadi as cs

from casadi import MX, vertcat
from acados_template import (
    AcadosOcp, 
    AcadosOcpSolver, 
    AcadosModel,
    builders
)

# %%
def define_bicycle_model() -> AcadosModel:
    s = MX.sym('s')                 # Path progress
    e_y = MX.sym('e_y')             # lateral error
    e_theta = MX.sym('e_theta')     # heading error
    v = MX.sym('v')                 # velocity
    delta = MX.sym('delta')         # steering angle
    x = cs.MX.sym('x')              # Global x position (m)
    y = cs.MX.sym('y')              # Global x position (m)
    states = vertcat(s, e_y, e_theta, v, delta, x, y)
    
    a = MX.sym('a')                 # acceleration
    omega = MX.sym('omega')         # steering rate
    controls = vertcat(a, omega)    

    # Reference trajectory parameters
    p = MX.sym('p', 8)
    chord = cs.sqrt((p[6]-p[0])**2 + (p[7]-p[1])**2)
    cp1   = cs.sqrt((p[2]-p[0])**2 + (p[3]-p[1])**2)
    cp2   = cs.sqrt((p[4]-p[2])**2 + (p[5]-p[3])**2)
    cp3   = cs.sqrt((p[6]-p[4])**2 + (p[7]-p[5])**2)
    L_total_expr = 0.5 * (chord + cp1 + cp2 + cp3)
    t_raw = s / L_total_expr
    t = cs.fmin(1.0, cs.fmax(0.0, t_raw))
    
    # Define the 2D Bézier curve for the lane geometry:
    x_lane = (1 - t)**3 * p[0] + 3*(1 - t)**2 * t * p[2] \
           + 3*(1 - t) * t**2 * p[4] + t**3 * p[6]
    y_lane = (1 - t)**3 * p[1] + 3*(1 - t)**2 * t * p[3] \
           + 3*(1 - t) * t**2 * p[5] + t**3 * p[7]
    
    # Compute derivatives with respect to s
    dx_ds   = cs.jacobian(x_lane, s)   # first derivative of x
    d2x_ds2 = cs.jacobian(dx_ds, s)      # second derivative of x
    dy_ds   = cs.jacobian(y_lane, s)     # first derivative of y
    d2y_ds2 = cs.jacobian(dy_ds, s)       # second derivative of y
    
    # Compute curvature κ(s):
    #   κ(s) = (dx/ds * d2y/ds2 - dy/ds * d2x/ds2) / ((dx/ds)^2 + (dy/ds)^2)^(3/2)
    numerator   = dx_ds * d2y_ds2 - dy_ds * d2x_ds2
    denominator = (dx_ds**2 + dy_ds**2)**(3/2)
    kappa = numerator / denominator

    theta_lane = cs.atan2(dy_ds, dx_ds)
    theta_vehicle = theta_lane + e_theta

    # Dynamics
    # (Note: the term 1 - kappa * e_y in the denominator “projects” the speed along the path)
    L = 0.258  # Wheelbase of the bicycle model
    s_dot = v * MX.cos(e_theta) / (1 - kappa * e_y)
    e_y_dot = v * MX.sin(e_theta)
    e_theta_dot = v / L * MX.tan(delta) - kappa * s_dot
    v_dot = a
    delta_dot = omega
    x_dot_global = v * cs.cos(theta_vehicle)
    y_dot_global = v * cs.sin(theta_vehicle)
    
    f = vertcat(s_dot, e_y_dot, e_theta_dot, v_dot, delta_dot, x_dot_global, y_dot_global)

    # Discs of Model for nonlinear constraints
    # r = 0.05
    # x_r = 0.5
    # y_r = 0.1
    # h_expr = - r + cs.sqrt((x - x_r)**2 + (y - y_r)**2)

    model = AcadosModel()
    model.f_expl_expr = f
    model.x = states
    model.u = controls
    model.p = p
    model.name = "bicycle_model" 
    # model.con_h_expr = h_expr
    return model


# Setup acados OCP
def setup_acados_ocp_solver(path_name: str) -> AcadosOcpSolver:
    # Define OCP
    ocp = AcadosOcp()
    ocp.model = define_bicycle_model()

    v_max = 3.0
    delta_max = 0.314159 # 18 degrees in radians
    NX = 7
    NU = 2
    NY = 7
    NP = 8
    NY_E = 5
    N_VX = 5
    N_VU = 2

    # Constraints
    ocp.constraints.x0 = np.zeros(NX)
    ocp.parameter_values = np.zeros(NP)
    # ocp.dims.nh = 1
    # ocp.dims.nh_0 = 1
    # ocp.dims.nh_e = 1

    ocp.constraints.lbx = np.array([0, -1e3, -1e2, 0, -delta_max, -1e2, -1e2])
    ocp.constraints.ubx = np.array([5, 1e3, 1e2, v_max, delta_max, 1e2, 1e2])
    ocp.constraints.idxbx = np.arange(NX)

    ocp.constraints.lbx_e = np.array([0, -1e2, -1e2, 0, -delta_max, -1e2, -1e2])
    ocp.constraints.ubx_e = np.array([5, 1e2, 1e2, v_max, delta_max, 1e2, 1e2])
    ocp.constraints.idxbx_e = np.arange(NX)

    ocp.constraints.lbu = np.array([0.0, -1e2])
    ocp.constraints.ubu = np.array([6.0, 1e2])
    ocp.constraints.idxbu = np.arange(NU)

    # ocp.constraints.lh = np.array([-1e2])
    # ocp.constraints.uh = np.array([1e2])

    # Cost weights
    Q_s = 0.1
    Q_e_y = 20.0  # Lateral error weight
    Q_e_theta = 20.0 # Yaw error weight
    Q_v = 0.0 
    Q_delta = 5.0
    R_a = 1.0   # Control effort weight (a)
    R_omega = 0.0   # Control effort weight (omega)

    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.W = np.diag([Q_s, Q_e_y, Q_e_theta, Q_v, Q_delta, R_a, R_omega])
    ocp.cost.Vx = np.zeros((NY, NX))
    ocp.cost.Vx[:N_VX, :N_VX] = np.eye(N_VX)
    ocp.cost.Vu = np.zeros((NY, NU))
    ocp.cost.Vu[N_VX:, :] = np.eye(N_VU)
    ocp.cost.yref = np.zeros(NY)
    ocp.cost.yref[0] = 1e2
    ocp.cost.yref[3] = v_max

    ocp.cost.cost_type_e = "LINEAR_LS"
    ocp.cost.W_e = np.diag([Q_s, Q_e_y, Q_e_theta, Q_v, Q_delta])
    ocp.cost.Vx_e = np.zeros((NY_E, NX))
    ocp.cost.Vx_e[:N_VX, :N_VX] = np.eye(N_VX)
    ocp.cost.yref_e = np.zeros(N_VX)
    ocp.cost.yref_e[0] = 1e2
    ocp.cost.yref[3] = v_max

    # Solver Options
    ocp.solver_options.N_horizon = 15
    ocp.solver_options.tf = 0.021 * ocp.solver_options.N_horizon
    ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.hpipm_mode = "SPEED"
    # ocp.solver_options.as_rti_level = 3
    # ocp.solver_options.as_rti_iter = 3
    # ocp.solver_options.qp_solver_warm_start = True
    # ocp.solver_options.levenberg_marquardt = 0.2
    # ocp.solver_options.rti_phase = 1
    ocp.solver_options.nlp_solver_tol_stat = 1e-4
    ocp.solver_options.nlp_solver_tol_comp = 1e-4
    ocp.solver_options.nlp_solver_tol_eq = 1e-3
    ocp.solver_options.nlp_solver_tol_ineq = 1e-2
    ocp.code_export_directory = os.path.join(path_name, 'c_generated_code')

    cm_builder = builders.ocp_get_default_cmake_builder()
    cm_builder.options_on = [
        'BUILD_ACADOS_OCP_SOLVER_LIB',
        'BUILD_ACADOS_SOLVER_LIB'
    ]
    return AcadosOcpSolver(ocp, json_file=os.path.join(path_name, 'bicycle_model.json'), cmake_builder=cm_builder)


def solve_ocp(control_points: np.ndarray, ocp_solver: AcadosOcpSolver):
    N_horizon = ocp_solver.acados_ocp.solver_options.N_horizon

    state = np.array([0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0]) # [s, e_y, e_theta, v, delta, x_global, y_global]

    # Solve OCP
    ocp_solver.options_set("rti_phase", 1)
    status = ocp_solver.solve()
    ocp_solver.print_statistics()
    if status != 5:
        print(f"Solver failed with status {status}")
        return

    bezier_params = control_points.flatten()
    for k in range(N_horizon):
        ocp_solver.set(k, "p", bezier_params)
    ocp_solver.set(0, 'lbx', state)
    ocp_solver.set(0, 'ubx', state)

    ocp_solver.options_set("rti_phase", 2)
    status = ocp_solver.solve()
    ocp_solver.print_statistics()

    print("Solving time: ", ocp_solver.get_stats("time_tot"))
    if status != 0:
        print(f"Solver failed with status {status}")

    x_out = []
    u_out = []
    for k in range(N_horizon + 1):
        xk = ocp_solver.get(k, "x")
        x_out.append(xk)
    for k in range(N_horizon):
        uk = ocp_solver.get(k, "u")
        u_out.append(uk)

    print("States:")
    for i, xk in enumerate(x_out):
        print(f"Stage {i}: {xk}")

    print("Inputs:")
    for i, uk in enumerate(u_out):
        print(f"Stage {i}: {uk}")


if __name__ == "__main__":
    path_name = os.path.dirname(os.path.abspath(__file__))
    ocp_solver = setup_acados_ocp_solver(path_name)
    
    control_points = np.array([
        [0.0, 0.0],
        [0.6, 0.0],
        [1., 0.1],
        [1.6, -0.1]
    ])
    solve_ocp(control_points, ocp_solver)
    solve_ocp(control_points, ocp_solver)
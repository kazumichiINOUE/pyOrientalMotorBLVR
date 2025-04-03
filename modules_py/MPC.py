import casadi
import numpy as np

class MPCController:
    def __init__(self, x_ref, u_ref, K=20, T=1, obstacles = []):
        self.Q   = casadi.diag([0.5, 0.5, 0.8])
        self.Q_f = casadi.diag([5.0, 5.0, 0.1])
        self.R   = casadi.diag([1.0, 1.0])
        self.R_start = 3.0  # 初期段階の制御入力の重み
        self.R_end = 1.0    # 最終段階の制御入力の重み
        self.R_weights = np.linspace(self.R_start, self.R_end, K)  # 時間依存の制御入力の重み
        
        self.K = K
        self.T = T
        self.dt = T / K
        
        self.x_lb = [-np.inf, -np.inf, -2*np.pi]
        self.x_ub = [np.inf, np.inf, 2*np.pi]
        self.u_lb = [0.0, -0.1 * np.pi]
        self.u_ub = [0.4, 0.1 * np.pi]
        
        self.x_ref = x_ref
        self.u_ref = u_ref
        
        self.obstacles = obstacles
        
        self.S = self.make_nlp()
        self.I = self.make_integrator()
        self.total_vars = 3 * (K + 1) + 2 * K

    def angle_difference(self, theta1, theta2):
        diff = theta1 - theta2
        return diff - 2 * np.pi * casadi.floor((diff + np.pi) / (2 * np.pi))
    
    def make_f(self):
        states = casadi.SX.sym("states", 3)
        ctrls = casadi.SX.sym("ctrls", 2)
        x, y, theta = states[0], states[1], states[2]
        v, w = ctrls[0], ctrls[1]
        x_dot = v * casadi.cos(theta)
        y_dot = v * casadi.sin(theta)
        theta_dot = w
        states_dot = casadi.vertcat(x_dot, y_dot, theta_dot)
        return casadi.Function("f", [states, ctrls], [states_dot], ["x", "u"], ["x_dot"])
    
    def make_F_RK4(self):
        states = casadi.SX.sym("states", 3)
        ctrls = casadi.SX.sym("ctrls", 2)
        f = self.make_f()
        r1 = f(x=states, u=ctrls)["x_dot"]
        r2 = f(x=states + self.dt * r1 / 2, u=ctrls)["x_dot"]
        r3 = f(x=states + self.dt * r2 / 2, u=ctrls)["x_dot"]
        r4 = f(x=states + self.dt * r3, u=ctrls)["x_dot"]
        states_next = states + self.dt * (r1 + 2 * r2 + 2 * r3 + r4) / 6
        return casadi.Function("F_RK4", [states, ctrls], [states_next], ["x", "u"], ["x_next"])
    
    def make_integrator(self):
        states = casadi.SX.sym("states", 3)
        ctrls = casadi.SX.sym("ctrls", 2)
        f = self.make_f()
        ode = f(x=states, u=ctrls)["x_dot"]
        dae = {"x": states, "p": ctrls, "ode": ode}
        return casadi.integrator("I", "cvodes", dae, {"tf": self.dt})

    def compute_obstacle_cost(self, state):
        cost = 0
        for obs_x, obs_y in self.obstacles:
            distance = casadi.sqrt((state[0] - obs_x)**2 + (state[1] - obs_y)**2)
            cost += casadi.if_else(distance > 0.5, 1 / distance, 1 / 0.5)
        return cost
    
    def compute_stage_cost(self, x, u, R_k):
        #x_diff = x - self.x_ref
        u_diff = u - self.u_ref
        obstacle_cost = self.compute_obstacle_cost(x)
        theta_diff = self.angle_difference(x[2], self.x_ref[2])
        x_diff = casadi.vertcat(x[0] - self.x_ref[0], x[1] - self.x_ref[1], theta_diff)
        return (casadi.dot(self.Q @ x_diff, x_diff) + casadi.dot(R_k @ u_diff, u_diff)) / 2 + obstacle_cost
    
    #def compute_terminal_cost(self, x):
    #    x_diff = x - self.x_ref
    #    return casadi.dot(self.Q_f @ x_diff, x_diff) / 2
    def compute_terminal_cost(self, x):
        theta_diff = self.angle_difference(x[2], self.x_ref[2])
        x_diff = casadi.vertcat(x[0] - self.x_ref[0], x[1] - self.x_ref[1], theta_diff)
        return casadi.dot(self.Q_f @ x_diff, x_diff) / 2
    
    def make_nlp(self):
        F_RK4 = self.make_F_RK4()
        U = [casadi.SX.sym(f"u_{k}", 2) for k in range(self.K)]
        X = [casadi.SX.sym(f"x_{k}", 3) for k in range(self.K + 1)]
        G = []
        J = 0
        for k in range(self.K):
            R_k = casadi.diag([self.R_weights[k], self.R_weights[k]])
            J += self.compute_stage_cost(X[k], U[k], R_k) * self.dt
            eq = X[k + 1] - F_RK4(x=X[k], u=U[k])["x_next"]
            G.append(eq)
        J += self.compute_terminal_cost(X[-1])
        option = {'print_time': False, 'ipopt': {'max_iter': 10, 'print_level': 0}}
        return casadi.nlpsol("S", "ipopt", {"x": casadi.vertcat(*X, *U), "f": J, "g": casadi.vertcat(*G)}, option)
    
    def compute_optimal_control(self, x_init, x0):
        x_init = x_init.full().ravel().tolist()
        lbx = x_init + self.x_lb * self.K + self.u_lb * self.K
        ubx = x_init + self.x_ub * self.K + self.u_ub * self.K
        lbg = [0] * 3 * self.K
        ubg = [0] * 3 * self.K
        res = self.S(lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg, x0=x0)
        offset = 3 * (self.K + 1)
        x0 = res["x"]
        u_opt = x0[offset:offset + 2]
        return u_opt, x0


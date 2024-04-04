import numpy as np
from copy import deepcopy
import math

from numpy import *

V_MAX = 2           # maximum velocity [m/s]
ACC_MAX = 2         # maximum acceleration [m/ss]
K_MAX = 4           # maximum curvature [1/m]

TARGET_SPEED = 1    # target speed [m/s] #횡 방향 속도를 최대 몇까지 할래
LANE_WIDTH = 0.2   # lane width [m] 
                    #차선 넓이 만큼, or 차량 넓이 만큼

COL_CHECK = 0.5    # collision check distance [m] #충돌 판단

MIN_T = 1           # minimum terminal time [s]
MAX_T = 2           # maximum terminal time [s]
DT_T = 0.5          # dt for terminal time [s] : MIN_T 에서 MAX_T 로 어떤 dt 로 늘려갈지를 나타냄
DT = 0.1            # timestep for update

K_J = 0.1           # weight for jerk
K_T = 0.1           # weight for terminal time
K_D = 1.0           # weight for consistency
K_V = 1.0           # weight for getting to target speed
K_LAT = 1.0         # weight for lateral direction
K_LON = 1.0         # weight for longitudinal direction

DF_SET = np.array([LANE_WIDTH/2, 0, -LANE_WIDTH/2])

FAIL = -1
def calc_frenet_paths(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, opt_d):
    frenet_paths = []

    # generate path to each offset goal
    for df in DF_SET:

        # Lateral motion planning
        for T in np.arange(MIN_T, MAX_T+DT_T, DT_T):
            fp = FrenetPath()
            lat_traj = QuinticPolynomial(di, di_d, di_dd, df, df_d, df_dd, T)

            fp.t = [t for t in np.arange(0.0, T, DT)]
            fp.d = [lat_traj.calc_pos(t) for t in fp.t]
            fp.d_d = [lat_traj.calc_vel(t) for t in fp.t]
            fp.d_dd = [lat_traj.calc_acc(t) for t in fp.t]
            fp.d_ddd = [lat_traj.calc_jerk(t) for t in fp.t]

            # Longitudinal motion planning (velocity keeping)
            tfp = deepcopy(fp)
            lon_traj = QuarticPolynomial(si, si_d, si_dd, sf_d, sf_dd, T)

            tfp.s = [lon_traj.calc_pos(t) for t in fp.t]
            tfp.s_d = [lon_traj.calc_vel(t) for t in fp.t]
            tfp.s_dd = [lon_traj.calc_acc(t) for t in fp.t]
            tfp.s_ddd = [lon_traj.calc_jerk(t) for t in fp.t]

            # 경로 늘려주기 (In case T < MAX_T)
            for _t in np.arange(T, MAX_T, DT):
                tfp.t.append(_t)
                tfp.d.append(tfp.d[-1])
                _s = tfp.s[-1] + tfp.s_d[-1] * DT
                tfp.s.append(_s)

                tfp.s_d.append(tfp.s_d[-1])
                tfp.s_dd.append(tfp.s_dd[-1])
                tfp.s_ddd.append(tfp.s_ddd[-1])

                tfp.d_d.append(tfp.d_d[-1])
                tfp.d_dd.append(tfp.d_dd[-1])
                tfp.d_ddd.append(tfp.d_ddd[-1])

            J_lat = sum(np.power(tfp.d_ddd, 2))  # lateral jerk
            J_lon = sum(np.power(tfp.s_ddd, 2))  # longitudinal jerk

            # cost for consistency
            d_diff = (tfp.d[-1] - opt_d) ** 2
            # cost for target speed
            v_diff = (TARGET_SPEED - tfp.s_d[-1]) ** 2

            # lateral cost
            tfp.c_lat = K_J * J_lat + K_T * T + K_D * d_diff
            # logitudinal cost
            tfp.c_lon = K_J * J_lon + K_T * T + K_V * v_diff

            # total cost combined
            tfp.c_tot = K_LAT * tfp.c_lat + K_LON * tfp.c_lon

            frenet_paths.append(tfp)
    return frenet_paths

class QuinticPolynomial:

    def __init__(self, xi, vi, ai, xf, vf, af, T):
        # calculate coefficient of quintic polynomial
        # used for lateral trajectory
        self.a0 = xi
        self.a1 = vi
        self.a2 = 0.5*ai

        A = np.array([[T**3, T**4, T**5],
                      [3*T**2, 4*T**3, 5*T** 4],
                      [6*T, 12*T**2, 20*T**3]])
        b = np.array([xf - self.a0 - self.a1*T - self.a2*T**2,
                      vf - self.a1 - 2*self.a2*T,
                      af - 2*self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]
        self.a5 = x[2]

    # calculate postition info.
    def calc_pos(self, t):
        x = self.a0 + self.a1*t + self.a2*t**2 + self.a3*t**3 + self.a4*t**4 + self.a5 * t ** 5
        return x

    # calculate velocity info.
    def calc_vel(self, t):
        v = self.a1 + 2*self.a2*t + 3*self.a3*t**2 + 4*self.a4*t**3 + 5*self.a5*t**4
        return v

    # calculate acceleration info.
    def calc_acc(self, t):
        a = 2*self.a2 + 6*self.a3*t + 12*self.a4*t**2 + 20*self.a5*t**3
        return a

    # calculate jerk info.
    def calc_jerk(self, t):
        j = 6*self.a3 + 24*self.a4*t + 60*self.a5*t**2
        return j

class QuarticPolynomial:

    def __init__(self, xi, vi, ai, vf, af, T):
        # calculate coefficient of quartic polynomial
        # used for longitudinal trajectory
        self.a0 = xi
        self.a1 = vi
        self.a2 = 0.5*ai

        A = np.array([[3*T**2, 4*T**3],
                             [6*T, 12*T**2]])
        b = np.array([vf - self.a1 - 2*self.a2*T,
                             af - 2*self.a2])

        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]

    # calculate postition info.
    def calc_pos(self, t):
        x = self.a0 + self.a1*t + self.a2*t**2 + self.a3*t**3 + self.a4*t**4
        return x

    # calculate velocity info.
    def calc_vel(self, t):
        v = self.a1 + 2*self.a2*t + 3*self.a3*t**2 + 4*self.a4*t**3
        return v

    # calculate acceleration info.
    def calc_acc(self, t):
        a = 2*self.a2 + 6*self.a3*t + 12*self.a4*t**2
        return a

    # calculate jerk info.
    def calc_jerk(self, t):
        j = 6*self.a3 + 24*self.a4*t
        return j

class FrenetPath:

    def __init__(self):
        # time
        self.t = []

        # lateral traj in Frenet frame
        self.d = []
        self.d_d = []
        self.d_dd = []
        self.d_ddd = []

        # longitudinal traj in Frenet frame
        self.s = []
        self.s_d = []
        self.s_dd = []
        self.s_ddd = []

        # cost
        self.c_lat = 0.0
        self.c_lon = 0.0
        self.c_tot = 0.0

        # combined traj in global frame
        
def collision_check(fp, obs):
    for i in range(len(obs[:, 0])):

        d = [((_x - obs[i, 0]) ** 2 + (_y - obs[i, 1]) ** 2)
             for (_x, _y) in zip(fp.s, fp.d)]
        collision = any([di <= COL_CHECK ** 2 for di in d])

        if collision:
            return True

    return False

def check_path(fplist, obs):
    ok_ind = []
    for i, _path in enumerate(fplist):
        acc_squared = [(abs(a_s**2 + a_d**2)) for (a_s, a_d) in zip(_path.s_dd, _path.d_dd)]
        if any([v > V_MAX for v in _path.s_d]):  # Max speed check
            continue
        elif any([acc > ACC_MAX**2 for acc in acc_squared]):
            continue
        elif collision_check(_path, obs):
            continue

        ok_ind.append(i)
    return [fplist[i] for i in ok_ind]


def frenet_optimal_planning(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, obs,opt_d):
    fplist = calc_frenet_paths(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, opt_d)

    fplist = check_path(fplist, obs)
    # find minimum cost path
    min_cost = float("inf")
    opt_traj = None
    opt_ind = 0
    for fp in fplist:
        if min_cost >= fp.c_tot:
            min_cost = fp.c_tot
            opt_traj = fp
            _opt_ind = opt_ind
        opt_ind += 1

    try:
        _opt_ind
    except NameError:
        return fplist, FAIL

    return fplist, _opt_ind

def test(obs,opt_d):
    s = 0
    d = 0
    v = 1 #현재 내 차량 속도
    a = 0 #등속도 운동
    si = s
    si_d = v
    si_dd = a
    sf_d = TARGET_SPEED
    sf_dd = 0

    # d 방향 초기조건
    di = d
    di_d = 0
    di_dd = 0
    df_d = 0
    df_dd = 0
    path, opt_ind = frenet_optimal_planning(si, si_d, si_dd,
                sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, obs, opt_d)
    if opt_ind == FAIL:
        print("can't find")
        return path, FAIL
    else:
        opt_d = path[opt_ind].d[-1]
        return path[opt_ind], opt_d
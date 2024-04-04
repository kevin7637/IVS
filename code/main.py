from control.PID import *
from trejectory.trejectory import *
FAIL = -1
if __name__ == "__main__":
    WIDTH = 0
    measure = 0
    step_time = 0.05 
    controller = PD_Controller(measure, step_time)
    yaw_controll(controller.u)
    obs = np.array([[0.6, WIDTH],
                [5, -WIDTH],
                [7, WIDTH],
                [8.5, -WIDTH]
                ])
    opt = 0
    path, opt= test(obs,opt)
    if opt == FAIL:
        print("FAIL")
    else:
        print(path.d)
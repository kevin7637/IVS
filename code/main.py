from control.PID import *
from trejectory.trejectory import *
from move.move import*
from move.sonic import*
from Adafruit_PCA9685 import PCA9685

HERTZ = 50
FAIL = -1
WIDTH = 0
pwm = PCA9685()
pwm.set_pwm_freq(HERTZ)

if __name__ == "__main__":

    measure = 0
    step_time = 0.05 
    #controller = PD_Controller(measure, step_time)
    setup()
    speed_set = 50
    tick = 300
    opt = 0
    try:
        move(speed_set, 'forward')
        pwm.set_pwm(0, 0, tick)
        '''
        obs = np.array([[0.6, WIDTH]])
        path, opt= Trejectory(obs,opt)
        if opt == FAIL:
            print("FAIL")
        else:
            print("path",path.d)
            for measure in path.d:
                controller.ControllerInput(measure)
                print("error",controller.u)
                tick = yaw_controll(controller.u)
                print("tick",tick)
                #pwm.set_pwm(0, 0, tick)
        '''
    except KeyboardInterrupt:
        destroy()
        
    
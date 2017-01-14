SENSING = Trueimport threading

from BrickPi import *


def compare(a, b):
    if a > b:
        return 1
    elif a < b:
        return -1
    else:
        return 0


def MotorRotateDegree(power, deg, port, relative=True, sampling_time=0, delay_when_stopping=0.05):
    number_of_motors = len(power)
    current = [0]*number_of_motors
    final = [0]*number_of_motors
    rotate = [0]*number_of_motors
    finished = [False]*number_of_motors

    for i in range(number_of_motors):
        current[i] = BrickPi.Encoder[port[i]]
        final[i] = int(relative)*current[i] + deg[i]*2
        power[i] = abs(power[i])
        rotate[i] = compare(final[i], current[i])
        BrickPi.MotorSpeed[port[i]] = rotate[i]*power[i]
    
    while True:
        for i in range(number_of_motors):
            if finished[i]:
                continue
            if (rotate[i] > 0 and final[i] > current[i]) or (rotate[i] < 0 and final[i] < current[i]):
                current[i] = BrickPi.Encoder[port[i]]
            else:
                finished[i] = True
                BrickPi.MotorSpeed[port[i]] = -rotate[i]*power[i]
                time.sleep(delay_when_stopping)
                BrickPi.MotorSpeed[port[i]] = 0
        time.sleep(sampling_time)
        if(all(finished)):
            break
    return 0

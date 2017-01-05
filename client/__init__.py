import BrickPi
from config import SENSING


def motorRotateDegree(power,deg,port,relative=True,sampling_time=.1,delay_when_stopping=.05):
    """Rotate the selected motors by specified degre

    Args:
      power    : an array of the power values at which to rotate the motors (0-255)
      deg    : an array of the angle's (in degrees) by which to rotate each of the motor
      port    : an array of the port's on which the motor is connected
      sampling_time  : (optional) the rate(in seconds) at which to read the data in the encoders
      delay_when_stopping:  (optional) the delay (in seconds) for which the motors are run in the opposite direction before stopping

    Returns:
      0 on success

    Usage:
      Pass the arguments in a list. if a single motor has to be controlled then the arguments should be
      passed like elements of an array,e.g, motorRotateDegree([255],[360],[PORT_A]) or
      motorRotateDegree([255,255],[360,360],[PORT_A,PORT_B])
    """

    num_motor=len(power)    #Number of motors being used
    init_val=[0]*num_motor
    final_val=[0]*num_motor
    BrickPiUpdateValues()
    for i in range(num_motor):
        BrickPi.MotorEnable[port[i]] = 1        #Enable the Motors
        power[i]=abs(power[i])
        # Updated for compatibility with Python3
        # BrickPi.MotorSpeed[port[i]] = power[i] if deg[i]>0 else -power[i]  #For running clockwise and anticlockwise
        # init_val[i]=BrickPi.Encoder[port[i]]        #Initial reading of the encoder
        #For running clockwise and anticlockwise
        if deg[i]>0:
           BrickPi.MotorSpeed[port[i]] = power[i]
        elif deg[i]<0:
           BrickPi.MotorSpeed[port[i]] = -power[i]
        else:
           BrickPi.MotorSpeed[port[i]] = 0
        init_val[i]=BrickPi.Encoder[port[i]]        #Initial reading of the encoder     
        final_val[i]=int(relative)*init_val[i]+(deg[i]*2)        #Final value when the motor has to be stopped;One encoder value counts for 0.5 degrees
    run_stat=[0]*num_motor
    while SENSING:
        result = BrickPiUpdateValues()          #Ask BrickPi to update values for sensors/motors
        if not result :
            for i in range(num_motor):        #Do for each of the motors
                if run_stat[i]==1:
                    continue
                # Check if final value reached for each of the motors
                if(deg[i]>0 and final_val[i]>init_val[i]) or (deg[i]<0 and final_val[i]<init_val[i]) :
                    # Read the encoder degrees
                    init_val[i]=BrickPi.Encoder[port[i]]
                else:
                    run_stat[i]=1
                    
                    # Updated for compatibility with Python3
                    # BrickPi.MotorSpeed[port[i]]=-power[i] if deg[i]>0 else power[i]  #Run the motors in reverse direction to stop instantly
                    
                    if deg[i]>0:
                        BrickPi.MotorSpeed[port[i]] = -power[i]
                    elif deg[i]<0:
                        BrickPi.MotorSpeed[port[i]] = power[i]
                    else:
                        BrickPi.MotorSpeed[port[i]] = 0            
                    BrickPiUpdateValues()
                    time.sleep(delay_when_stopping)
                    BrickPi.MotorEnable[port[i]] = 0
                    BrickPiUpdateValues()
        time.sleep(sampling_time)          #sleep for the sampling time given (default:100 ms)
        if(all(e==1 for e in run_stat)):        #If all the motors have already completed their rotation, then stop
          break
    return 0

BrickPi.motorRotateDegree = motorRotateDegree
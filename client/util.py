from BrickPi import *
import time

# Port variables
RIGHT_WHEEL = PORT_A
LEFT_WHEEL = PORT_B
SENSOR = PORT_C
FRONT_SENSOR = PORT_2
REAR_SENSOR = PORT_3

# Perform BrickPi initialisation
BrickPiSetup()

# Enable the motors
BrickPi.MotorEnable[LEFT_WHEEL] = 1
BrickPi.MotorEnable[RIGHT_WHEEL] = 1
BrickPi.MotorEnable[SENSOR] = 1

# Enable the ultrasonic sensors
BrickPi.SensorType[FRONT_SENSOR] = TYPE_SENSOR_ULTRASONIC_CONT
BrickPi.SensorType[REAR_SENSOR] = TYPE_SENSOR_ULTRASONIC_CONT

# Setup the sensors
BrickPiSetupSensors()
BrickPiUpdateValues()

# Set the encoder offset to the encoder value (so they are initialised at 0)
BrickPi.EncoderOffset[LEFT_WHEEL] = BrickPi.Encoder[LEFT_WHEEL]
BrickPi.EncoderOffset[RIGHT_WHEEL] = BrickPi.Encoder[RIGHT_WHEEL]
BrickPi.EncoderOffset[SENSOR] = BrickPi.Encoder[SENSOR]
BrickPiUpdateValues()


def update():
    BrickPiUpdateValues()


def motors():
    sensor, left, right = BrickPi.Encoder[SENSOR], BrickPi.Encoder[LEFT_WHEEL], BrickPi.Encoder[RIGHT_WHEEL]
    return sensor, left, right


def wheels():
    left, right = BrickPi.Encoder[LEFT_WHEEL], BrickPi.Encoder[RIGHT_WHEEL]
    return left, right


def sensors():
    front, rear = BrickPi.Sensor[FRONT_SENSOR], BrickPi.Sensor[REAR_SENSOR]
    return rear, front


def rotate(degree, power, sampling_time=0.05):
    finished = False
    current = BrickPi.Encoder[SENSOR]
    final = degree * 2
    rotate = (final > current) - (final < current)
    start_time = time.time()

    while not finished:
        delta_time = time.time() - start_time
        if (delta_time > 5) or not ((rotate > 0 and final > current) or (rotate < 0 and final < current)):
            finished = True
        else:
            current = BrickPi.Encoder[SENSOR]
            BrickPi.MotorSpeed[SENSOR] = rotate * min(int(power + 30 * delta_time), 255)
        time.sleep(sampling_time)
    BrickPi.MotorSpeed[SENSOR] = 0

    return 0


def move(power, rotate=False):
    right_power = power
    left_power = (2 * (not rotate) - 1) * power
    BrickPi.MotorSpeed[RIGHT_WHEEL] = right_power
    BrickPi.MotorSpeed[LEFT_WHEEL] = left_power

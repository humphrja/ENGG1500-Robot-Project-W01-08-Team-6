# This file is used for storing all experimental test functions - to clean up the main file

from main import *      # Import all variables from main.py
import numpy as np
import matplotlib.pyplot as plt
import time

def testMotors():

    # Testing ctrl_alloc() method:
    motor_left.ctrl_alloc("fwd", 50)
    motor_right.ctrl_alloc("fwd", 50)
    sleep(1.5)

    motor_left.ctrl_alloc("fwd", 30)
    motor_right.ctrl_alloc("fwd", 30)
    sleep(1.5)

    motor_left.ctrl_alloc("fwd", 10)
    motor_right.ctrl_alloc("fwd", 10)
    sleep(1.5)

    motor_left.ctrl_alloc("fwd", 0)
    motor_right.ctrl_alloc("fwd", 0)
    sleep(1.5)

    motor_left.ctrl_alloc("fwd", 50)  # One revolution
    motor_right.ctrl_alloc("bck", 50)
    sleep(1.8)

    motor_left.ctrl_alloc("fwd", 0)     # Pause is necessary inbetween spins because of wheel skid
    motor_right.ctrl_alloc("bck", 0)
    sleep(0.5)

    motor_left.ctrl_alloc("bck", 50)
    motor_right.ctrl_alloc("fwd", 50)
    sleep(1.8)

    motor_left.ctrl_alloc("bck", 50)
    motor_right.ctrl_alloc("bck", 50)
    sleep(2)

    # Need to implement small stops (0.3)



def testUltrasonicSensor():
    dist = ultrasonic_sensor.distance_mm()

    if dist > 300:
        motor_left.ctrl_alloc("fwd", 40)
        motor_right.ctrl_alloc("fwd", 40)
    else:
        motor_left.ctrl_alloc("fwd", 0)
        motor_right.ctrl_alloc("fwd", 0)

    sleep(0.1)


def testServoSweep():
    # Sweep between 0 & 180 degrees
    setServoAngle(0)
    sleep(0.2)

    for pos in range(0, 180, 5):                # Increment through angles 0 -> 180 by 5 degrees
        setServoAngle(pos)
        sleep(0.05)

    for pos in range(180, 0, -5):               # Increment through angles 180 -> 0 by 5 degrees
        setServoAngle(pos)
        sleep(0.05)


def testLineSensor():
    if line_sensor.value() == 1:            # 1 = light surface, 0 = dark surface
        motor_left.ctrl_alloc("fwd", 40)
        motor_right.ctrl_alloc("fwd", 40)
    else:
        motor_left.ctrl_alloc("fwd", 0)
        motor_right.ctrl_alloc("fwd", 0)


def testRotate():
    rotate(90)          # East
    stop(0.5)
    rotate(-180)        # West
    stop(0.5)
    rotate(-45)         # South West
    stop(0.5)
    rotate(135)         # North
    stop(0.5)


def testDrive():
    drive(300, 50, 0)
    stop(1)
    drive(100, 30, -1)
    stop(1)
    drive(100, 30, 1)
    stop(2)

    # drive()
    drive(300, 50, 0.5)
    drive(300, 50, -0.5)
    stop(2)


def testImprovedRotate():
    # for angle in range(300, 400, 20):       # Iterates through angles close to 360 to help determine how many clicks are in a full revolution
    #     stop(1)
    #     improvedRotate(angle, 30)

    enc.clear_count()
    rotate(360)
    aveClicks = (enc.get_left() + enc.get_right()) / 2
    print(aveClicks)



def testServoLock(desired_angle):
    gyroAngle = 0
    while True:
        # Set initial servo angle
        # Get gyro angle
        # Adjust servo angle

        gyroAngle += gyroscope.getAngle('Z', 0.1)
        servoAngle = gyroAngle + 90 - desired_angle

        setServoAngle(servoAngle)


def testNewDrive():
    # drive(300, 50, 0)
    # stop(1)
    # drive(100, 30, -1)
    # stop(1)
    # drive(100, 30, 1)
    # stop(2)
    #
    # # drive()
    # drive(300, 50, 0.5)
    # drive(300, 50, -0.5)
    # stop(2)

    # Two control drives testing motors that aren't calibrated
    drive(400, 45, 0, 45, driveTime=3)
    stop(0.5)
    gyroRotate(180, 45)
    stop(0.5)
    drive(400, 45, 0, -45, driveTime=3)
    stop(0.5)
    gyroRotate(180, 45)
    stop(2)

    setServoAngle(90)
    calibrateMotors()       # Updates global variable 'powerOffset'
    stop(2)

    # Two test drives testing motors that are calibrated
    drive(400, 45, 0, 45, driveTime=3)
    stop(0.5)
    gyroRotate(180, 45)
    stop(0.5)
    drive(400, 45, 0, -45, driveTime=3)
    stop(2)


def plot_IR_readings():
    # Setup
    initialTime = time.time()
    times = []

    list_0 = []
    list_1 = []
    list_2 = []

    # Get data
    motor_left.set_forwards()
    motor_right.set_backwards()
    motor_left.duty(40)
    motor_right.duty(40)
    while time.time() - initialTime < 3:

        list_0.append(adc_A0.read_u16())
        list_1.append(adc_A0.read_u16())
        list_2.append(adc_A0.read_u16())

        times.append(time.time())
        time.sleep(0.01)

    stop(0.2)


    numPoints = len(times)
    print(numPoints)

    # np.random.seed(1)
    # list_0 = np.random.normal(5800, 25, size=numPoints)
    # np.random.seed(4)
    # list_1 = np.random.normal(6500, 25, size=numPoints)
    # np.random.seed(3)
    # list_2 = np.random.normal(11300, 25, size=numPoints)

    IR_0 = np.array(list_0)
    IR_1 = np.array(list_1)
    IR_2 = np.array(list_2)


    # Process data
    t = np.array(times) - initialTime
    average = (IR_0 + IR_1 + IR_2)/3

    fig = plt.figure()
    gs = fig.add_gridspec(3, hspace=0.3)
    axs = gs.subplots(sharex=True)
    fig.suptitle("IR Sensor Measurements as Robot Drives Over Black Line on White Paper")

    # Raw data
    axs[0].plot(t, IR_0, 'r', label="left")
    axs[0].plot(t, IR_1, 'g', label="middle")
    axs[0].plot(t, IR_2, 'b', label="right")
    axs[0].plot(t, average, 'k--', label="average")
    axs[0].grid('on')
    axs[0].set_ylabel("IR Reading")
    axs[0].set_title("Raw data")
    axs[0].legend()

    # Scale factors
    factor_0 = average / IR_0
    factor_0_mean = np.average(factor_0) * np.ones(numPoints)
    factor_1 = average / IR_1
    factor_1_mean = np.average(factor_1) * np.ones(numPoints)
    factor_2 = average / IR_2
    factor_2_mean = np.average(factor_2) * np.ones(numPoints)
    factor_average = average / average

    print("Scale factors required for left: {:.4f}; right: {:.4f}; middle: {:.4f}".format(np.average(factor_0), np.average(factor_1), np.average(factor_2)))

    axs[1].plot(t, factor_0, 'r', label="left")
    axs[1].plot(t, factor_0_mean, 'r:', label="left mean")
    axs[1].plot(t, factor_1, 'g', label="middle")
    axs[1].plot(t, factor_1_mean, 'g:', label="middle mean")
    axs[1].plot(t, factor_2, 'b', label="right")
    axs[1].plot(t, factor_2_mean, 'b:', label="right mean")
    axs[1].plot(t, factor_average, 'k--', label="average")
    axs[1].grid('on')
    axs[1].set_ylabel("Scale factor")
    axs[1].set_title("Scale factor of IR sensors with respect to average")
    axs[1].legend()

    # Offsets
    offset_0 = average - IR_0
    offset_0_mean = np.average(offset_0) + np.zeros(numPoints)
    offset_1 = average - IR_1
    offset_1_mean = np.average(offset_1) + np.zeros(numPoints)
    offset_2 = average - IR_2
    offset_2_mean = np.average(offset_2) + np.zeros(numPoints)

    print("Offsets required for left: {:.4f}; right: {:.4f}; middle: {:.4f}".format(np.average(offset_0), np.average(offset_1), np.average(offset_2)))

    axs[2].plot(t, offset_0, 'r', label="left")
    axs[2].plot(t, offset_0_mean, 'r:', label="left mean")
    axs[2].plot(t, offset_1, 'g', label="middle")
    axs[2].plot(t, offset_1_mean, 'g:', label="middle mean")
    axs[2].plot(t, offset_2, 'b', label="right")
    axs[2].plot(t, offset_2_mean, 'b:', label="right mean")
    axs[2].plot(t, factor_average, 'k--', label="average")
    axs[2].grid('on')
    axs[2].set_xlabel("Time (s)")
    axs[2].set_ylabel("Offset")
    axs[2].set_title("Offset of IR sensors with respect to average")
    axs[2].legend()

    plt.show()



def getIRAverages():
    IR_0 = []           # Left
    IR_1 = []           # Middle
    IR_2 = []           # Right

    sum_0 = 0
    sum_1 = 0
    sum_2 = 0

    for i in range(1000):
        # IR_0.append(adc_A0.read_u16())
        # IR_1.append(adc_A1.read_u16())
        # IR_2.append(adc_A2.read_u16())

        sum_0 += adc_A0.read_u16()
        sum_1 += adc_A1.read_u16()
        sum_2 += adc_A2.read_u16()

        sleep(0.01)

    ave_0 = sum_0/1000
    ave_1 = sum_1/1000
    ave_2 = sum_2/1000

    print(ave_0, ave_1, ave_2)

    return ave_0, ave_1, ave_2

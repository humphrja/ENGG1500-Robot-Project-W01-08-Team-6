# This file is used for storing all experimental test functions - to clean up the main file

from main import *      # Import all variables from main.py

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



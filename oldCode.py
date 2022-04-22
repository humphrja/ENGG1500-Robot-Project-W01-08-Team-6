from main import *

# Rotates using time
def rotate(angle):
    # 360 degrees = 1.8 seconds at 50% power

    if angle > 0:       # Rotate clockwise (positive angles)
        motor_left.ctrl_alloc("fwd", 50)
        motor_right.ctrl_alloc("bck", 50)
    else:               # Rotate anticlockwise (negative angles)
        motor_left.ctrl_alloc("bck", 50)
        motor_right.ctrl_alloc("fwd", 50)

    duration = (abs(angle)/360)*1.8         # Without abs(), this was leading to the error involving negative angles
    sleep(duration)


# Rotates using encoders
def improvedRotate(angle, power):
    # Find number of wheel clicks for one revolution of robot

    # One robot revolution of 360 degrees = n clicks
    n = 30          # Adjust.
    numClicks = n*abs(angle)/360

    if angle > 0:
        motor_left.set_forwards()
        motor_right.set_backwards()
    else:
        motor_left.set_backwards()
        motor_right.set_forwards()

    enc.clear_count()
    while (enc.get_left() + enc.get_right()) / 2 < numClicks:
        motor_left.duty(power)
        motor_right.duty(power)

    stop(0.8)



def driveParallelToWall():
    # Check no walls are straight ahead
    # Measure initial distance to left wall
    # Drive straight ahead for 0.5s
    # Measure final distance to left wall
    # Compare initial distance to left wall and final distance to left wall
    # Turn robot accordingly
    # Repeat

    # Loop terminates upon reaching a wall straight ahead

    setServoAngle(90)
    sleep(0.5)
    distAhead = ultrasonic_sensor.distance_mm()
    while distAhead > 200:
    # while True:             #
        # testServoSweep()    #
        # sleep(2.0)          #

        setServoAngle(165)
        sleep(0.5)

        # setServoAngle(90)   #
        # sleep(2.0)          #
        # setServoAngle(1)    #                              # Wall is on the left of the robot
        # sleep(2.0)          #
        #setServoAngle(180)

        initDist = ultrasonic_sensor.distance_mm()

        motor_left.ctrl_alloc("fwd", 50)
        motor_right.ctrl_alloc("fwd", 50)
        sleep(0.8)
        stop(0.1)

        setServoAngle(180)
        sleep(0.5)
        finalDist = ultrasonic_sensor.distance_mm()

        changeInDist = finalDist - initDist

        if changeInDist > 0:    # If robot has moved away from wall
            rotate(-30)         # Rotate left
            stop(0.1)
        else:                   # If robot has moved toward wall
            rotate(30)          # Rotate right
            stop(0.1)

        # The above logic could be replaced with a formula converting changeInDist to a more accurate turning angle

        setServoAngle(90)
        sleep(0.5)
        distAhead = ultrasonic_sensor.distance_mm()



def driveForDistance(dist, power):
    # Every revolution of each wheel = 20 clicks
    # Diameter = 65mm

    D = 65      # Diameter
    pi = 3.1415926535
    circumference = pi*D

    numRevs = abs(dist) / circumference
    numClicks = 20*numRevs

    enc.clear_count()

    if dist > 0:
        motor_left.set_forwards()
        motor_right.set_forwards()
    else:
        motor_left.set_backwards()
        motor_right.set_backwards()

    while (enc.get_left() + enc.get_right()) / 2 < numClicks:
        motor_left.duty(power)
        motor_right.duty(power)


# Competency vv

# if __name__ == '__main__':
#     sleep(3)
#
#     # Three flashes of the green_LED indicate the start of the sequence
#     flashLED()
#
#     driveTowardsNearestWall()
#     while line_sensor.value() == 1:
#         rotate(90)
#         driveParallelToWall()
#     stop(5)
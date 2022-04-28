from time import sleep, time
from motor import Motor
from machine import Pin, PWM, ADC
from ultrasonic import sonic
from encoder import Encoder
import gyroscope
import testing
# from numpy import cos, pi

print("Program starting...")

# For motors
motor_left = Motor("left", 8, 9, 6)
motor_right = Motor("right", 10, 11, 7)   # These parameters need to be checked

# # For IR sensor
# line_sensor = Pin(26, Pin.IN)

# For ultrasonic sensor
TRIG = 3
ECHO = 2
ultrasonic_sensor = sonic(TRIG, ECHO)

# For fixed, front-facing ultrasonic sensor
F_TRIG = 13
F_ECHO = 12
ultrasonic_sensor_front = sonic(F_TRIG, F_ECHO)

# For Servo
pwm = PWM(Pin(15))
pwm.freq(50)

# Create encoder object
ENC_L = 18
ENC_R = 19
enc = Encoder(ENC_L, ENC_R)


# For IR sensors - distance from centreline
x0 = -15
x1 = 0
x2 = 15

adc_A0 = ADC(Pin(26))
adc_A1 = ADC(Pin(27))
adc_A2 = ADC(Pin(28))

# global powerOffset
powerOffset = 0.0
motors_calibrated = False

green_LED = Pin(25, Pin.OUT)

# Example change


def flashLED(numberFlashes):
    for i in range(numberFlashes):
        green_LED.value(1)
        sleep(0.1)
        green_LED.value(0)
        sleep(0.1)


def setServoAngle(angle):
    position = int(8000*(angle/180) + 1000)     # Convert angle into [1000,9000]
    pwm.duty_u16(position)                      # Set duty cycle



def getDistances():
    # Performs a sweep of the servo, and at each angle increment, a distance measurement is taken and stored in a list

    distances = []

    # Sweep between 0 & 180 degrees
    setServoAngle(0)
    sleep(0.2)

    minDist = 10000
    minDistAngle = 0

    for pos in range(0, 180, 5):  # Increment through angles 0 -> 180 by 5 degrees
        setServoAngle(pos)
        sleep(0.05)
        dist = ultrasonic_sensor.distance_mm()
        if dist > 0:
            distances.append(dist)          # Redundancy for invalid measurements

        if dist < minDist:     # Finds the minimum distance and corresponding angle
            minDist = dist
            minDistAngle = pos

        sleep(0.05)

    sleep(1)
    setServoAngle(minDistAngle)     # Sets the servo to point in the direction of the nearest obstacle
    sleep(1)

    # Performing a backwards sweep could be useful to take an average and minimise impact of noise

    # for pos in range(180, 0, -5):  # Increment through angles 180 -> 0 by 5 degrees
    #     setServoAngle(pos)
    #     sleep(0.05)

    return distances


def stop(duration):
    motor_left.ctrl_alloc("fwd", 0)
    motor_right.ctrl_alloc("bck", 0)
    sleep(duration)



def driveTowardsNearestWall():
    repeat = True

    while repeat:
        distances = getDistances()      # Perform a sweep of the ultrasonic sensor
        minDist = min(distances)
        minDistAngle = distances.index(minDist) * 5     # Calculate angle towards nearest wall

        print(minDist, minDistAngle)

        setServoAngle(minDistAngle)  # Sets the servo to point in the direction of the nearest obstacle
        sleep(1)                     # For visual reference only


        gyroRotate(90 - minDistAngle, 45)   # Rotate robot towards nearest wall
        stop(0.1)

        # Rotate function angle is 0 straight ahead and positive for clockwise
        # Servo angle is 0 directly right and positive anticlockwise


        setServoAngle(90)
        if ultrasonic_sensor.distance_mm() > 200:    # If wall is further than 80mm, drive ahead, otherwise the wall has been reached
            motor_left.ctrl_alloc("fwd", 40)
            motor_right.ctrl_alloc("fwd", 40)       # Forward for 0.5s
            sleep(1.0)

            stop(0.1)

        else:
            repeat = False





def drive(distance, power, powerBalance, desired_servo_angle, driveTime=100.0, servoLock=True, dist_front_threshold=200):
    # distance (mm) is a float whereby positive represents forwards and negative backwards
    # power is a float in range [0,100] (percentage), power represents the average power of both the motors
    # powerBalance is a float in range [-1,1], whereby 0 represents both motors at same power, +1 represents turning right where the right motor is at 0 power, -1 represents turning left where left motor is at 0 power

    powerBalance += powerOffset

    left_power = power * (1 + powerBalance)
    right_power = power * (1 - powerBalance)

    if left_power > 100:
        left_power = 100
    if right_power > 100:
        right_power = 100  # This ensures that power can't be greater than 100


    # Every revolution of each wheel = 20 clicks
    # Diameter = 65mm

    D = 65              # Diameter of wheel
    circumference = 3.1415926 * D

    enc.clear_count()
    leftDist = 0
    rightDist = 0

    gyroAngle = 0

    dist_front = ultrasonic_sensor_front.distance_mm()

    initialTime = time()
    elapsedTime = 0

    while (leftDist + rightDist) / 2 < abs(distance) and elapsedTime < driveTime and dist_front > dist_front_threshold:       # Take an average of distance from each wheel, also prevents drive from running longer than driveTime seconds
        leftDist = circumference * enc.get_left() / 20
        rightDist = circumference * enc.get_right() / 20    # 1 revolution = 20 clicks, wheel travels distance of 1 circumference in 1 revolution

        # print("Encoders:", enc.get_left(), enc.get_right())

        if distance > 0:
            motor_left.set_forwards()
            motor_right.set_forwards()          # Forwards
        else:
            motor_left.set_backwards()
            motor_right.set_backwards()         # Backwards

        motor_left.duty(int(left_power))
        motor_right.duty(int(right_power))

        if servoLock:                                           # This condition is used if the servo should not be locked to a specified bearing
            gyroAngle += gyroscope.getAngle('Z', 0.1)           # Sets the servo to the desired angle
            servoAngle = gyroAngle + 90 - desired_servo_angle

            setServoAngle(servoAngle)

        dist_front = ultrasonic_sensor_front.distance_mm()
        elapsedTime = time() - initialTime

    return gyroAngle



def followWall(distanceFromWall):
    # Map distance from wall to powerBalance variable:
    # [0, 400] -> [1.0, -1.0]   (turn right when close to wall, turn left when far from wall)


    desired_servo_angle = -75
    setServoAngle(90 - desired_servo_angle)
    sleep(0.5)

    robotAngle = 0

    IR = getAveIRValue()
    while IR < 100000:    # 6000                                    # Loop terminates after IR sensors detect an average value higher than 6000

        dist = ultrasonic_sensor.distance_mm()

        # The below code would try to compensate for the fact that the servo cannot reach angles beyond 180 degrees -> when the robot is heading away from the wall, the ultrasonic is not perpendicular to the wall
        # However this calculation would reduce the magnitude of dist
        # Thus robot would be even less inclined to turn towards the wall when far away

        # if robotAngle > 0:                                          # Robot is heading away from the wall
        #     theta = robotAngle * pi / 180                           # Converts robotAngle from degrees to radians
        #     dist *= cos(theta)

        if dist > distanceFromWall*2:                               # Constrains distance to a range of [0, 400] because powerBalance must be within [-1,1]
            dist = distanceFromWall*2
        elif dist < 0:                                              # This should never happen, but just in case
            dist = 0

        factor = 3                                                  # Used to limit the output powerBalance - larger values result in reduced turning sensitivity

        powerBalance = -1 * (dist - distanceFromWall) / (factor*distanceFromWall)
        # https://www.desmos.com/calculator/dnddxest7c - converts distance to appropriate power balance
            # b := powerBalance
            # d := dist
            # f := factor
            # D := distanceFromWall


        robotAngle += drive(50, 40, powerBalance, desired_servo_angle - robotAngle, driveTime=0.5)                            # Update powerBalance every 50mm or 0.5s

        # The above line of code both calls drive() - robot drives forward - and also adds the robot's change in angle to robotAngle
        # robotAngle can then be combined with desired_servo_angle because drive() resets the '0' heading everytime it is called

        IR = getAveIRValue()

    stop(1)


    # This function assumes the robot starts exaclty parallel with the wall
    # This function is working well when the robot starts 200mm away from the wall.
    # When the robot starts close to the wall it tends to move away from the wall in a straight line, sometimes turning back towards the wall, sometimes turning in a circle
    # When the robot starts far from the wall it tends to move in a straight line but stay far from the wall. It is less reliable and sometimes starts turning in a circle





def getDistanceFromLine():
    w0 = adc_A0.read_u16()
    w1 = adc_A1.read_u16()
    w2 = adc_A2.read_u16()

    print(w0, w1, w2)

    numerator = w0*x0 + w1*x1 + w2*x2
    denominator = w0 + w1 + w2

    line_dist = numerator/denominator
    print("Distance from line = {:3.2f}".format(line_dist))
    sleep(0.1)

    return line_dist

def getAveIRValue():
    w0 = adc_A0.read_u16()
    w1 = adc_A1.read_u16()
    w2 = adc_A2.read_u16()

    ave = (w0+w1+w2)/3
    print("Average IR value:", ave)
    return ave


def followLine():
    setServoAngle(90)       # Facing straight ahead
    sleep(0.5)
    distAhead = ultrasonic_sensor.distance_mm()

    while distAhead > 200:
        line_dist = getDistanceFromLine()
        powerBalance = line_dist / 30

        # Maps distance from line to powerBalance variable:
        # [-30, 30] -> [-1.0, 1.0]   (turn left when line is left of robot, turn right when line is right of robot)

        drive(50, 40, powerBalance, 0, driveTime=0.1)     # Drives for 50mm inrements without changing powerBalance

        distAhead = ultrasonic_sensor.distance_mm()

    # A thought: Maybe we can use the straight line track segment to calibrate the robot's motors to drive in a straight line
    # This could be done by adding some 'offset' to the powerBalance variable


def calibrateMotors():
    global powerOffset
    # Calibrates the motors by finding an appropriate powerBalance offset
    # Uses gyro to calibrate motors

    offset = 0

    green_LED.value(1)
    while True:
        changeInAngle = drive(400, 45, offset, 0, driveTime=2)
        stop(0.5)
        if changeInAngle > 10:      # Robot has turned right
            offset -= 0.1
        elif changeInAngle < -10:   # Robot has turned left
            offset += 0.1
        else:
            break
        gyroRotate(180, 45)
        stop(0.5)

    green_LED.value(0)

    print(offset)
    powerOffset = offset

    return offset

    # This function works reliably well
    # Returning a value of -0.1 on Wednesday 20/4 in L320

def updatePowerOffset(value):
    global powerOffset, motors_calibrated
    powerOffset = value
    motors_calibrated = True


def gyroRotate(angle, power):
    # power motors at given power in appropriate directions until gyroAngle reaches angle
    if angle > 0:
        motor_left.set_forwards()
        motor_right.set_backwards()
    else:
        motor_left.set_backwards()
        motor_right.set_forwards()


    motor_left.duty(power)
    motor_right.duty(power)

    gyroAngle = 0
    while abs(gyroAngle) < abs(angle):
        gyroAngle += gyroscope.getAngle('Z', 0.1)

    motor_left.duty(0)
    motor_right.duty(0)



def alignWithWall():        # This function serves to align the robot exactly parallel with the wall. It assumes the robot is within close proximity of the wall
    # Make robot do a slow 360 degree rotation with servo fixed facing straight ahead relative to the robot
    # During rotation record distances for every degree
    # After rotation, find the minimum distance and corresponding angle, theta
    # Rotate robot to theta + 90 (wall on left of robot)

    minDist = 100000
    theta = 0

    setServoAngle(90)

    motor_left.set_forwards()
    motor_right.set_backwards()
    motor_left.duty(45)
    motor_right.duty(45)

    gyroAngle = 0
    while gyroAngle < 360:
        dist = ultrasonic_sensor.distance_mm()
        if dist < minDist:
            minDist = dist
            theta = gyroAngle

        gyroAngle += gyroscope.getAngle('Z', 0.1)

    stop(0.5)

    angle = (theta + 90) % 360      # Theta is angle towards wall. 'angle' is angle by which robot should rotate
    if angle > 180:
        angle -= 360

    if minDist != 100000:
        gyroRotate(angle, 45)
    else:
        flashLED(4)




while True:
    updatePowerOffset(-0.1)         # Comment this out if the motors should be calibrated before running
    if not motors_calibrated:
        sleep(1)
        calibrateMotors()

    sleep(1)
    flashLED(3)     # Three flashes of the green_LED indicate the start of the loop

    # driveTowardsNearestWall()
    # getDistanceFromLine()                     # Working alright. Random noise value is about -0.3
    followWall(200)                             # Working reasonably well. Assumes robot starts parallel to wall
    # followLine()

    # dist_front = ultrasonic_sensor_front.distance_mm()
    # print(dist_front)

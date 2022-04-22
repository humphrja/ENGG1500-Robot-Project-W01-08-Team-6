from time import sleep, time
from machine import Pin, I2C

address = 104
dataXH = bytearray(8)           # bytearray(8) creates an array of length 8, whereby all elements are initialised to null
dataXL = bytearray(8)
dataYH = bytearray(8)
dataYL = bytearray(8)
dataZH = bytearray(8)
dataZL = bytearray(8)
gyroScale = 131

i2c = I2C(0, scl=Pin(5), sda=Pin(4))

sleep(0.1)

# create buffer for and configre PWR_MGMT
PWR_MGMT_1 = bytearray(8)
i2c.writeto_mem(104, 107, PWR_MGMT_1)

# create buffer for and configre GYRO_CONFIG
GYRO_CONFIG = bytearray(8)
i2c.writeto_mem(104, 27, GYRO_CONFIG)

# create buffer for and configure DLPF_CFG to 8kHz
DLPF = bytearray(8)
DLPF[0] = 0
i2c.writeto_mem(address, 26, DLPF)

# create buffer for SMPLRT_DIV and configure to
SMPLRT_DIV = bytearray(1)
SMPLRT_DIV[0] = 63


def read_i2c_combined(register):
    high = bytearray(1)
    low = bytearray(1)
    i2c.readfrom_mem_into(104, register, high)
    i2c.readfrom_mem_into(104, (register + 1), low)
    # sleep(0.1)
    highB = high[0]
    lowB = low[0]
    value = (highB << 8) + lowB         # value = highB * (2**8) + lowB
    if value >= 0x8000:
        return -((65535 - value) + 1)
    else:
        return value


def getAngle(axis, sleepTime):
    # X adjustment: -0.505, Y adjustment: -1.373, Z adjustment: +2.545
    axisOffsets = [-0.505, -1.373, 2.545]

    i = -1
    if axis == 'X':
        i = 0
    elif axis == 'Y':
        i = 1
    elif axis == 'Z':
        i = 2
    else:
        print("Invalid axis - (only 'X', 'Y' or 'Z'.")

    gyro = [read_i2c_combined(67), read_i2c_combined(69), read_i2c_combined(71)]

    vel = (gyro[i] / 131.0) + axisOffsets[i]
    rot = vel * sleepTime
    # print(axis + " displacement: {}".format(rot))

    sleep(sleepTime)

    return rot



def testGyro():
    sleep(0.5)

    rotZ = 0
    t0 = time()
    dt = 0.1    # change in time
    while True:
        gyroZ = read_i2c_combined(71)
        # X adjustment: -0.505, Y adjustment: -1.373, Z adjustment: +2.545
        velZ = (gyroZ / 131.0) + 2.545
        rotZ += velZ * dt
        print("Z displacement: {}".format(rotZ))
        sleep(dt)
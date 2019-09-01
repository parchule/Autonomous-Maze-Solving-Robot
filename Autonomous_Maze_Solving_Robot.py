# Proj3_Code_Team48
# File: Proj3_Code_Team48.py
# Date: 26 April 2018
# By: Phillip Archuleta
# parchule
# Jasmine Hughes
# hughe225
# Camille Hamilton 
# hamil140
# Raul Poma
# rpoma
# Section: 3
# Team: 48
#
# ELECTRONIC SIGNATURE
# Phillip Archuleta
# Jasmine Hughes
# Camille Hamilton
# Raul Poma
#
# The electronic signatures above indicate that the program
# submitted for evaluation is the combined effort of all
# team members and that each member of the team was an
# equal participant in its creation. In addition, each
# member of the team has a general understanding of
# all aspects of the program development and execution.
#
# THIS PROGRAM WILL CARRY OUT ALL TASKS REQUIRED OF THE PROJECT THREE ROBOT

import time                         # import the time library for the sleep function
import csv                          # import the csv library for mapping input
import math                         # import the math library
import grovepi                      # import the GrovePi drivers
import brickpi3                     # import the BrickPi3 drivers
from IR_Functions import *          # import IR functions
from MPU9250 import MPU9250         # import IMU capabilities
from IMUFilters import *            # import IMU capabilities

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

# --------------------------------
# Hardware initialization
# --------------------------------
# GrovePi Port Locations
ultrasonicRight = 2
ultrasonicLeft = 3

# BrickPi3 Port Locations
leftMotor = BP.PORT_D
rightMotor = BP.PORT_C
cargoMotor = BP.PORT_A
gyroscope = BP.PORT_4
ultrasonicFront = BP.PORT_1

#Initialize GrovePi Hardware
irSensor1 = 14
irSensor2 = 15
grovepi.pinMode(ultrasonicRight, "INPUT")
grovepi.pinMode(ultrasonicLeft, "INPUT")
grovepi.pinMode(irSensor1, "INPUT")
grovepi.pinMode(irSensor2, "INPUT")
myIMU = MPU9250()

#Initialize BrickPi Hardware
BP.offset_motor_encoder(cargoMotor, BP.get_motor_encoder(cargoMotor))
BP.offset_motor_encoder(rightMotor, BP.get_motor_encoder(rightMotor))
BP.offset_motor_encoder(leftMotor, BP.get_motor_encoder(leftMotor))
BP.set_sensor_type(ultrasonicFront, BP.SENSOR_TYPE.EV3_ULTRASONIC_CM)
BP.set_sensor_type(gyroscope, BP.SENSOR_TYPE.EV3_GYRO_ABS_DPS)
time.sleep(4)
# --------------------------------
# ---------------------------------------------------------
# Functions & Global Variables
# ---------------------------------------------------------
timeStep = 0.02     # sleep constant
magValue = 0        # Boolean value, indicates magnet obstacle
irValue = 0         # Boolean value, indicates IR obstacle
maxSideUltra = 45   # Threshold value for turning behavior
minFrontUltra = 16
maxFrontUltra = 8
maxSingleSide = 35
solved = 0          # Boolean value, indicates maze complete
initGyro = 0
mapSize = 40
mapGEARS = [[0]*mapSize for i in range(mapSize)]
startDirection = 0
XPos = 0
YPos = 0
tileSize = 40
magnetLocations = []
irLocations = []

def centerPointTurning(targetAngle, direction):
    # direction: 0 is left/CCW, 1 is right/CW
    BP.set_motor_power(rightMotor, 0)
    BP.set_motor_power(leftMotor, 0)
    
    if (direction):
        targetAngle = targetAngle + BP.get_sensor(gyroscope)[0]
        while (BP.get_sensor(gyroscope)[0] < targetAngle):
            BP.set_motor_power(rightMotor, -13)
            BP.set_motor_power(leftMotor, -16)
            time.sleep(timeStep * 2)
    else:
        targetAngle = BP.get_sensor(gyroscope)[0] - targetAngle
        while (BP.get_sensor(gyroscope)[0] > targetAngle):
            BP.set_motor_power(rightMotor, 13)
            BP.set_motor_power(leftMotor, 16)
            time.sleep(timeStep * 2)

    BP.set_motor_power(rightMotor, 0)
    BP.set_motor_power(leftMotor, 0)


def turnToPoint(angle):
    while (BP.get_sensor(gyroscope)[0] < angle):
        BP.set_motor_power(rightMotor, -13)
        BP.set_motor_power(leftMotor, -16)
        time.sleep(timeStep * 3)


def identifyMagnet():
    # sets magValue to 1 if a magnet is present, 0 if not
    global magValue
    magValue = 0
    maxMag = 115
    mag = myIMU.readMagnet()
    total_mag = math.sqrt(mag['x']*mag['x'] + mag['y']*mag['y'] + mag['z']*mag['z'])
    print("Total mag: " + str(total_mag))
    if(total_mag > maxMag and total_mag < 126):
        magValue = 1        


def identifyIR():
    # sets irValue to 1 if a IR beacon is present, 0 if not
    global irValue
    irValue = 0
    maxIR = 80
    ir1 = grovepi.analogRead(irSensor1)
    ir2 = grovepi.analogRead(irSensor2)
    print ("One = " + str(ir1) + "\n" "Two = " + str(ir2))
    if(ir1 > maxIR or ir2 > maxIR):
        irValue = 1


def cargo(direction):
    #0 is retract, 1 is deploy
    if(direction):
        BP.set_motor_power(cargoMotor, -55)
        time.sleep(1.3)
        BP.set_motor_power(cargoMotor, 0)
        time.sleep(2)
    else:
        BP.set_motor_power(cargoMotor, 55)
        time.sleep(1.3)
        BP.set_motor_power(cargoMotor, 0)


def mazeTurn():
    # turning algorithm for GEARS turning within the maze
    global magValue
    global irValue
    global initGyro
    global solved
    global maxSideUltra
    global maxSingleSide
    global minFrontUltra
    global maxFrontUltra
    global tileSize
    
    BP.set_motor_power(rightMotor, 13)
    BP.set_motor_power(leftMotor,-13)
    time.sleep(.85)
    BP.set_motor_power(rightMotor, 0)
    BP.set_motor_power(leftMotor,-0)
    
    # and rather than or allows 4-way intersection support
    if (magValue == 0 and irValue == 0 and grovepi.ultrasonicRead(ultrasonicLeft) + grovepi.ultrasonicRead(ultrasonicRight) < maxSideUltra and BP.get_sensor(ultrasonicFront) > minFrontUltra):
        time.sleep(timeStep)
    elif (grovepi.ultrasonicRead(ultrasonicLeft) > maxSingleSide):
        mappingTrack(0, -1, tileSize)
        turnToPoint(initGyro)
        initGyro -= 90
        # if there isn't a wall 40 cm or less away from the left of GEARS
        time.sleep(timeStep * 4)
        if(BP.get_sensor(ultrasonicFront) < 40):
            # 3-way intersection 
            time.sleep(timeStep * 4)
            while(BP.get_sensor(ultrasonicFront) > maxFrontUltra):    
                time.sleep(timeStep * 4)
                BP.set_motor_power(rightMotor, 12)
                BP.set_motor_power(leftMotor, -15)
                time.sleep(timeStep * 10)
                BP.set_motor_power(rightMotor, 0)
                BP.set_motor_power(leftMotor, 0)                
            BP.set_motor_power(rightMotor, 0)
            BP.set_motor_power(leftMotor, 0)
            centerPointTurning(90, 0)
            BP.set_motor_power(rightMotor, 0)
            BP.set_motor_power(leftMotor, 0)
            elapsed = 0
            start = time.time()
            while(grovepi.ultrasonicRead(ultrasonicLeft) > maxSingleSide and grovepi.ultrasonicRead(ultrasonicRight) > maxSingleSide and elapsed < 5):
                BP.set_motor_power(rightMotor, 12)
                BP.set_motor_power(leftMotor, -15)
                time.sleep(timeStep * 4)
                end = time.time()
                elapsed = end - start;
            BP.set_motor_power(rightMotor, 0)
            BP.set_motor_power(leftMotor, 0)
            # complete maze condition
            if(elapsed >= 5):
                BP.set_motor_power(rightMotor, -12)
                BP.set_motor_power(leftMotor, 15)
                time.sleep(5)
                BP.set_motor_power(rightMotor, 0)
                BP.set_motor_power(leftMotor, 0)
                solved = 1
        else:
            # 4-way intersection
            BP.offset_motor_encoder(leftMotor, BP.get_motor_encoder(leftMotor))
            while(BP.get_motor_encoder(leftMotor) > -185):
                BP.set_motor_power(rightMotor, 9)
                BP.set_motor_power(leftMotor, -11)
                time.sleep(timeStep * 4)
            BP.set_motor_power(rightMotor, 0)
            BP.set_motor_power(leftMotor, 0)
            centerPointTurning(90, 0)
            BP.set_motor_power(rightMotor, 0)
            BP.set_motor_power(leftMotor, 0)
            elapsed = 0
            start = time.time()
            while(grovepi.ultrasonicRead(ultrasonicLeft) > maxSingleSide and grovepi.ultrasonicRead(ultrasonicRight) > maxSingleSide and elapsed < 5):
                BP.set_motor_power(rightMotor, 9)
                BP.set_motor_power(leftMotor, -11)
                time.sleep(timeStep * 4)
                end = time.time()
                elapsed = end - start;
            BP.set_motor_power(rightMotor, 0)
            BP.set_motor_power(leftMotor, 0)
            # complete maze condition
            if(elapsed >= 5):
                BP.set_motor_power(rightMotor, -9)
                BP.set_motor_power(leftMotor, 11)
                time.sleep(5)
                BP.set_motor_power(rightMotor, 0)
                BP.set_motor_power(leftMotor, 0)
                solved = 1
    elif(grovepi.ultrasonicRead(ultrasonicRight) > maxSingleSide and BP.get_sensor(ultrasonicFront) < 40):
        mappingTrack(0, -1, tileSize)
        turnToPoint(initGyro)
        initGyro += 90
        # if there isn't a wall 40 cm or less away from the right of GEARS
        time.sleep(timeStep * 4)
        if(BP.get_sensor(ultrasonicFront) < 40):
            # 3-way intersection
            time.sleep(timeStep * 4)
            while(BP.get_sensor(ultrasonicFront) > maxFrontUltra):    
                BP.set_motor_power(rightMotor, 11)
                BP.set_motor_power(leftMotor, -9)
                time.sleep(timeStep * 10)
                BP.set_motor_power(rightMotor, 0)
                BP.set_motor_power(leftMotor, 0)
            BP.set_motor_power(rightMotor, 0)
            BP.set_motor_power(leftMotor, 0)
            centerPointTurning(90, 1)
            BP.set_motor_power(rightMotor, 0)
            BP.set_motor_power(leftMotor, 0)
            while (grovepi.ultrasonicRead(ultrasonicLeft) > maxSingleSide and grovepi.ultrasonicRead(ultrasonicRight) > maxSingleSide):
                BP.set_motor_power(rightMotor, 9)
                BP.set_motor_power(leftMotor, -11)
            BP.set_motor_power(rightMotor, 0)
            BP.set_motor_power(leftMotor, 0)
        else:
            # 4-way intersection
            BP.offset_motor_encoder(leftMotor, BP.get_motor_encoder(leftMotor))
            while(BP.get_motor_encoder(leftMotor) > -185):
                BP.set_motor_power(rightMotor, 9)
                BP.set_motor_power(leftMotor, -11)
                time.sleep(timeStep * 4)
            BP.set_motor_power(rightMotor, 0)
            BP.set_motor_power(leftMotor, 0)
            centerPointTurning(90, 1)
            BP.set_motor_power(rightMotor, 0)
            BP.set_motor_power(leftMotor, 0)
            while (grovepi.ultrasonicRead(ultrasonicLeft) > maxSingleSide and grovepi.ultrasonicRead(ultrasonicRight) > maxSingleSide):
                time.sleep(timeStep * 4)
                BP.set_motor_power(rightMotor, 9)
                BP.set_motor_power(leftMotor, -11)
                time.sleep(timeStep * 10)
                BP.set_motor_power(rightMotor, 0)
                BP.set_motor_power(leftMotor, 0)
            BP.set_motor_power(rightMotor, 0)
            BP.set_motor_power(leftMotor, 0)
    elif(BP.get_sensor(ultrasonicFront) > minFrontUltra):
        time.sleep(timeStep)
    else:
        # walls are present both to the left and right of GEARS
        turnToPoint(initGyro)
        initGyro += 180
        centerPointTurning(180, 1)

             
def mappingInit(originX, originY, startingDirection):
    # tileSize in cm always
    # startingDirection: 0 is N, 1 is W, 2 is S, 3 is E
    global mapGEARS
    global startDirection
    global XPos
    global YPos
    
    mapGEARS[originX][originY] = 5 
    startDirection = startingDirection
    XPos = originX
    YPos = originY
    
    
def mappingTrack(encoder, direction, tileSize):
    # Updates every mapping condition other than '4', 'exit point'
    global startDirection
    global XPos
    global YPos
    global magValue
    global irValue
    global mapGEARS
    global magnetLocations
    global irLocations

    print("encoder: " + str(encoder))
    degreesPerCm = 21.5
    if (encoder == 0 and direction == -1):
        if (startDirection == 0):
            XPos += 0
            YPos += 1
        elif (startDirection == 1):
            XPos += -1
            YPos += 0
        elif (startDirection == 2):
            XPos += 0
            YPos += -1
        elif (startDirection == 3):
            XPos += 1
            YPos += 0
        startDirection += 1
        if (startDirection == 4):
            startDirection == 0
        identifyMagnet()
        identifyIR()
        if (magValue == 1):
            mapGEARS[XPos][YPos] = 3
            magnetLocations.append(XPos)
            magnetLocations.append(YPos)
        elif (irValue == 1):
            mapGEARS[XPos][YPos] = 2
            irLocations.append(XPos)
            irLocations.append(YPos)
        else:
            mapGEARS[XPos][YPos] = 1
    elif (encoder == 0 and direction == 1):
        if (startDirection == 0):
            XPos += 0
            YPos += 1
        elif (startDirection == 1):
            XPos += -1
            YPos += 0
        elif (startDirection == 2):
            XPos += 0
            YPos += -1
        elif (startDirection == 3):
            XPos += 1
            YPos += 0
        startDirection -= 1        
        if (startDirection == -1):
            startDirection == 3
        identifyMagnet()
        identifyIR()
        if (magValue == 1):
            mapGEARS[XPos][YPos] = 3
            magnetLocations.append(XPos)
            magnetLocations.append(YPos)
        elif (irValue == 1):
            mapGEARS[XPos][YPos] = 2
            irLocations.append(XPos)
            irLocations.append(YPos)
        else:
            mapGEARS[XPos][YPos] = 1
    elif (abs(encoder) > (tileSize * degreesPerCm)):
        BP.offset_motor_encoder(leftMotor, BP.get_motor_encoder(leftMotor))
        if (startDirection == 0):
            XPos += 0
            YPos += 1
        elif (startDirection == 1):
            XPos += -1
            YPos += 0
        elif (startDirection == 2):
            XPos += 0
            YPos += -1
        elif (startDirection == 3):
            XPos += 1
            YPos += 0
        
        identifyMagnet()
        identifyIR()
        if (magValue == 1):
            mapGEARS[XPos][YPos] = 3
            magnetLocations.append(XPos)
            magnetLocations.append(YPos)
        elif (irValue == 1):
            mapGEARS[XPos][YPos] = 2
            irLocations.append(XPos)
            irLocations.append(YPos)
        else:
            mapGEARS[XPos][YPos] = 1        


def mappingFinal(originX, originY, mapNum, tileSize, unit):
    # Unit Info: 0 is mm, 1 is cm, 2 is m
    global mapGEARS
    global mapSize
    global magnetLocations
    global irLocations

    mapGEARS[XPos][YPos] = 4 
    row = []
    with open('team48_map.csv', mode='w') as csv_file_data:
        writer = csv.writer(csv_file_data, delimiter=',', quotechar='"')
        j = 1
        
        row = ["Team: 48"]
        writer.writerow(row)
        del row[:]
        
        row = ["Map: " + str(mapNum)]
        writer.writerow(row)
        del row[:]

        row = ["Unit Length: " + str(tileSize)]
        writer.writerow(row)
        del row[:]         
        
        if (unit == 0):
            row = ["Unit: mm"]
        elif (unit == 1):
            row = ["Unit: cm"]
        elif (unit == 2):
            row = ["Unit: m"]
        writer.writerow(row)
        del row[:]
        
        row = ["Origin: (" + str(originX) + "," + str(originY) + ")"]
        writer.writerow(row)
        del row[:]        
        
        while (j <= mapSize):
            for i in range(mapSize):
                row.append(mapGEARS[mapSize - j][i])
            writer.writerow(row)
            del row[:]
            j = j + 1        

    with open('team48_hazards.csv', mode='w') as csv_file_hazards:
        hazards = csv.writer(csv_file_hazards, delimiter=',', quotechar='"')
        # hard-coded magnet and IR locations for testing purposes only 
        #magnetLocations.append(1)
        #magnetLocations.append(1)
        #irLocations.append(2)
        #irLocations.append(2)

        row = ["Team: 48"]
        hazards.writerow(row)
        del row[:]

        row = ["Map: " + str(mapNum)]
        hazards.writerow(row)
        del row[:]

        row = ["Hazard Type", "Parameter of Interest", "Parameter Value", "Hazard X Coordinate", "Hazard Y Coordinate"]
        hazards.writerow(row)
        del row[:]

        k = 0
        while (k < len(magnetLocations)):
            row.append("Electrical Activity Source/Magnet")
            row.append("Field Strength (uT)")
            row.append("2600")
            row.append(str(magnetLocations[k] * tileSize) + " cm")
            row.append(str(magnetLocations[k + 1] * tileSize) + " cm")
            hazards.writerow(row)
            del row[:]
            k = k + 2

        l = 0
        while (l < len(irLocations)):
            row.append("High Temperature Heat Source/IR Beacon")
            row.append("Intensity of Radiation (unitless)")
            row.append("125")
            row.append(str(irLocations[l] * tileSize) + " cm")
            row.append(str(irLocations[l + 1] * tileSize) + " cm")
            hazards.writerow(row)
            del row[:]
            l = l + 2
        
        
def navigate():
    # algorithm information 
    # uses PID control to maintain in the center of the lane
    # when it approaches a wall in front of it, it first checks if there is a wall to the left of it
    # if there is not, it turns left and moves forward without PID control until the left ultrasonic sensor reads values indicating the presence of a wall
    # if there is, it checks if there is a wall to the right of it
    # if there is not, it turns right and moves forward without PID control until the right ultrasonic sensor reads values indicating the presence of a wall
    # if there is, it turns 180 degrees
    # GEARS then resumes using PID control to maintain its position in the center of the lane

    global magValue
    global irValue
    global initGyro
    global solved
    global maxSideUltra
    global maxSingleSide
    global minFrontUltra
    global maxFrontUltra
    global tileSize
        
    KP = 0.25
    KI = 0.05
    KD = 0.02

    P = 0.0
    I = 0.0
    D = 0.0
    e = 0.0

    #e_prev = 0

    # check magnetic and IR values
    # Set origin and direction in mappingInit
    # Enable magnet and IR checking
    # Update tileSize global variable
    # Update origin x, origin y in mappingInit and BOTH mappingFinals    
    
    mappingInit(3, 0, 0)
    # mappingInit(x-origin, y-origin, 0 N; 1 W; 2 S; 3 E)
    BP.offset_motor_encoder(leftMotor, BP.get_motor_encoder(leftMotor))
    while (solved == 0):
        while (grovepi.ultrasonicRead(ultrasonicLeft) + grovepi.ultrasonicRead(ultrasonicRight) > maxSideUltra):
            BP.set_motor_power(rightMotor, 13)
            BP.set_motor_power(leftMotor, -14)
        BP.set_motor_power(rightMotor, 0)
        BP.set_motor_power(leftMotor, 0)
        initGyro = BP.get_sensor(gyroscope)[0]
        e_prev = grovepi.ultrasonicRead(ultrasonicLeft) - grovepi.ultrasonicRead(ultrasonicRight)
        # keep and rather than or to support 4-way turning
        while (magValue == 0 and irValue == 0 and grovepi.ultrasonicRead(ultrasonicLeft) + grovepi.ultrasonicRead(ultrasonicRight) < maxSideUltra and BP.get_sensor(ultrasonicFront) > minFrontUltra):
            time.sleep(timeStep)
            e = (grovepi.ultrasonicRead(ultrasonicLeft) - grovepi.ultrasonicRead(ultrasonicRight)) + (0.6 * (BP.get_sensor(gyroscope)[0] - initGyro))
            #print("e: " + str(e))
            if (abs(e) > 20 or grovepi.ultrasonicRead(ultrasonicLeft) > maxSingleSide or grovepi.ultrasonicRead(ultrasonicRight) > maxSingleSide):
                e = 0
            #print("Front: ")
            #print(BP.get_sensor(ultrasonicFront))
            #print("Left: ")
            #print(grovepi.ultrasonicRead(ultrasonicLeft))
            #print("Right: ")
            #print(grovepi.ultrasonicRead(ultrasonicRight))               # positive e means GEARS is too far right, negative e means GEARS is too far left
            P = KP * e
            I += KI * e * (timeStep * 20) / 2
            D = KD * (e - e_prev) / (timeStep * 20)
            powerModifier = P + I + D
            if (e > 0 and powerModifier < 0):
                powerModifier *= -1
            elif (e < 0 and powerModifier > 0):
                powerModifier *= -1
            #print("Power: " + str(powerModifier))
            BP.set_motor_power(rightMotor, 20 + powerModifier)
            BP.set_motor_power(leftMotor, -20 + powerModifier)
            e_prev = e
            identifyMagnet()
            identifyIR()
            time.sleep(timeStep * 55)
            BP.set_motor_power(rightMotor, 0)
            BP.set_motor_power(leftMotor, 0)
            mappingTrack(BP.get_motor_encoder(leftMotor), 0, tileSize)
        BP.set_motor_power(rightMotor, 0)
        BP.set_motor_power(leftMotor, 0)
        mazeTurn()
    cargo(1)
    cargo(0)
    mappingFinal(3, 0, 6, tileSize, 1)
    #mappingFinal(originX, originY, mapNum, tileSize, unit)
# --------------------------------
# ---------------------------------------------------------
# Control loop -- run infinitely until a keyboard interrupt
# ---------------------------------------------------------    
def main():
    try:
        #while True:
        #    identifyMagnet()
        #    identifyIR()
        #    time.sleep(timeStep * 40)
        navigate()
    except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
        BP.reset_all()
        cargo(1)
        cargo(0)
        mappingFinal(3, 0, 6, tileSize, 1)
        BP.reset_all()

main()

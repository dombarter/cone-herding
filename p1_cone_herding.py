"""__CONFIG__
{"version":20,"widgetInfos":[{"hwid":"1","name":"LeftDrive","typeName":"motor","extraConfig":null,"bufferIndex":0},{"hwid":"2","name":"RightDrive","typeName":"motor_rp","extraConfig":null,"bufferIndex":1},{"hwid":"3","name":"TouchLed","typeName":"touch_led","extraConfig":null,"bufferIndex":2},{"hwid":"4","name":"Gyro1","typeName":"gyro","extraConfig":null,"bufferIndex":3},{"hwid":"5","name":"Gyro2","typeName":"gyro","extraConfig":null,"bufferIndex":4},{"hwid":"6","name":"MiddleUltra","typeName":"distance_cm","extraConfig":null,"bufferIndex":5},{"hwid":"7","name":"LeftColor","typeName":"color_hue","extraConfig":null,"bufferIndex":6},{"hwid":"8","name":"RightColor","typeName":"color_hue","extraConfig":null,"bufferIndex":7},{"hwid":"drivetrain","name":"dt","typeName":"drivetrain","extraConfig":{"leftMotorHwId":"1","rightMotorHwId":"2","wheelTravel":200,"trackWidth":176},"bufferIndex":8},{"hwid":"lcd","name":"lcd","typeName":"lcd","extraConfig":null,"bufferIndex":9},{"hwid":"sound","name":"sound","typeName":"sound","extraConfig":null,"bufferIndex":10},{"hwid":"btn_chk","name":"button_check","typeName":"face_button","extraConfig":null,"bufferIndex":11},{"hwid":"btn_up","name":"button_up","typeName":"face_button","extraConfig":null,"bufferIndex":12},{"hwid":"btn_down","name":"button_down","typeName":"face_button","extraConfig":null,"bufferIndex":13}]}"""

# external library imports ------------------

import sys
import vexiq
import drivetrain
import math

# -------------------------------------------

# objects and classes -----------------------

# XYCoordinates class
class XYCoordinates:
    def __init__(self):
        self.x = 0
        self.y = 0

# Robot class
class Robot():

    # public properties----------------------

    x = 0
    y = 0
    visitingHerdPoint = True
    carryingCone = False
    coneToCollect = False
    distanceTravelled = 0
    robotRadius = 15
    wheelCircumference = 20
    distanceSensorSpacing = 5

    # ---------------------------------------

    # object instantiation ------------------

    def __init__(self,leftDrive,rightDrive,gyro1,gyro2,dt,colorLeft,colorRight,distanceMiddle):
        self.leftDrive = leftDrive
        self.rightDrive = rightDrive
        self.gyro1 = gyro1
        self.gyro2 = gyro2
        self.drivetrain = dt
        self.colorRight = colorRight
        self.colorLeft = colorLeft
        self.distanceMiddle = distanceMiddle

    # ---------------------------------------

    # basic mathematical functions ----------

    def intify(self,number): #used to convert from float to int
        num = round(number)
        iterate = 0
        while iterate != num:
            iterate+=1
        return iterate

    # ---------------------------------------

    # NON-DEV robot specific functions ------

    def returnToHerdPoint(self,herdpoint):
        return None

    def moveToXYR(self,x,y,r):
        return None

    def collectCone(self):
        return None

    def moveBy(self,distance): #move the robot forwards by a certain distance

        self.cm = self.intify(distance)

        if self.checkCone() == True:
            return False #robot has been unable to reach the final destination
        else:
            if self.cm <= 15:
                self.drivetrain.drive_until(30,self.cm*10)
                return True #robot has been able to reach destination
            else:
                self.numberOfIterations = self.intify(math.floor(self.cm / 15))
                self.remainder = self.intify(self.cm % 15)

                if self.remainder == 0:
                    for i in range(0,self.numberOfIterations):
                        if self.checkCone() == True:
                            return False #robot has been unable to reach the final destination
                        self.drivetrain.drive_until(30,150) #move robot by 15cm

                    return True #robot has been able to reach destination

                else:
                    for i in range(0,self.numberOfIterations):
                        self.drivetrain.drive_until(30,150)
                        if self.checkCone() == True:
                            return False #robot has been unable to reach the final destination

                    self.drivetrain.drive_until(30,self.remainder*10)
                    return True #robot has been able to reach destination

    def rotateTo(self,degrees):
        return None

    def rotateBy(self,degrees,direction):
        return None

    def liftArm(self):
        return None

    def lowerArm(self):
        return None

    def closeClaw(self):
        return None

    def openClaw(self):
        return None

    def resolveReadings(self):
        return None

    def resolveXY(self,xCoord,yCoord,distance,rotation): #function to update distaplacement of the robot by calculating new coordinates

        self.radians = math.radians(rotation) #turns the gyro reading into radians

        self.gyro = rotation #keeps the gyro reading in degrees

        self.coordinates = XYCoordinates() #makes a new set of coordinates

        self.ninety = math.radians(90) # 90 in radians

        if self.gyro == 0:
            self.coordinates.y = yCoord + distance
        elif self.gyro == 180 or self.gyro == -180:
            self.coordinates.y = yCoord - distance
        elif self.gyro == 90:
            self.coordinates.x = xCoord + distance
        elif self.gyro == -90:
            self.coordinates.x = xCoord - distance
        else:
            if self.gyro > 0 and self.gyro < 90:
                self.coordinates.x = xCoord + (math.sin(self.radians) * distance)
                self.coordinates.y = yCoord + (math.cos(self.radians) * distance)
            elif self.gyro > 0 and self.gyro > 90:
                self.coordinates.x = xCoord + (math.cos(self.radians - self.ninety) * distance)
                self.coordinates.y = yCoord - (math.sin(self.radians - self.ninety) * distance)
            elif self.gyro < 0 and self.gyro > -90:
                self.coordinates.x = xCoord - (math.sin(math.fabs(self.radians)) * distance)
                self.coordinates.y = yCoord + (math.cos(math.fabs(self.radians)) * distance)
            elif self.gyro < 0 and self.gyro < -90:
                self.coordinates.x = xCoord - (math.cos(math.fabs(self.radians + self.ninety)) * distance)
                self.coordinates.y = yCoord - (math.sin(math.fabs(self.radians + self.ninety)) * distance)

        self.coordinates.x = round(self.coordinates.x)
        self.coordinates.y = round(self.coordinates.y)

        return self.coordinates

    def startGyro(self): #calibrates both gyros
        self.gyro1.calibrate()
        self.gyro2.calibrate()
        while self.gyro1.is_calibrating() and self.gyro2.is_calibrating(): #doesnt return from function until calibration complete
            continue
        self.gyro1.angle(0)
        self.gyro2.angle(0)

    def angle(self): #gets both gyro readings and returns an average
        self.angle_1 = -1 * round((self.gyro1.angle() + self.gyro2.angle()) / 2)
        return self.angle_1

    # ---------------------------------------

    # DEV robot specific functions ----------

    def checkCone(self): # a simple test function to check to see if there is anything in front of the robot

        if self.distanceMiddle.distance() < 30:
            return True
        else:
            return False

# -------------------------------------------

# Hardware setup and port selection ---------

#region config
LeftDrive   = vexiq.Motor(1)
RightDrive  = vexiq.Motor(2, True) # Reverse Polarity
TouchLed    = vexiq.TouchLed(3)
Gyro1       = vexiq.Gyro(4)
Gyro2       = vexiq.Gyro(5)
MiddleUltra = vexiq.DistanceSensor(6, vexiq.UNIT_CM)
LeftColor   = vexiq.ColorSensor(7) # hue
RightColor  = vexiq.ColorSensor(8) # hue
dt          = drivetrain.Drivetrain(LeftDrive, RightDrive, 200, 176)
#endregion config

# -------------------------------------------

# Pre-program object creation ---------------

robot = Robot(LeftDrive,RightDrive,Gyro1,Gyro2,dt,LeftColor,RightColor,MiddleUltra) #create robot class

# -------------------------------------------

# Pre-program calibration and startup -------
TouchLed.named_color(3) #orange
vexiq.lcd_write("Gyro Calibrating..") #output to lcd screen
robot.startGyro() #calibrate both gyros

# -------------------------------------------

# Main Program ------------------------------

while True:
    TouchLed.named_color(9) #blue

    vexiq.lcd_write("Gyro: " + str(robot.angle()),1)
    vexiq.lcd_write("X Coord: " + str(robot.x),2)
    vexiq.lcd_write("Y Coord: " + str(robot.y),3)


    if TouchLed.is_touch():

        TouchLed.named_color(3) #orange

        result = robot.moveBy(50)

        if result == True:
            TouchLed.named_color(7) #green
        else:
            TouchLed.named_color(1) #red
        sys.sleep(1.5)

# -------------------------------------------

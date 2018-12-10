"""__CONFIG__
{"version":20,"widgetInfos":[{"hwid":"1","name":"LeftDrive","typeName":"motor","extraConfig":null,"bufferIndex":0},{"hwid":"2","name":"RightDrive","typeName":"motor_rp","extraConfig":null,"bufferIndex":1},{"hwid":"3","name":"TouchLed","typeName":"touch_led","extraConfig":null,"bufferIndex":2},{"hwid":"6","name":"MiddleUltra","typeName":"distance_cm","extraConfig":null,"bufferIndex":3},{"hwid":"7","name":"LeftColor","typeName":"color_hue","extraConfig":null,"bufferIndex":4},{"hwid":"8","name":"RightColor","typeName":"color_hue","extraConfig":null,"bufferIndex":5},{"hwid":"drivetrain","name":"dt","typeName":"drivetrain","extraConfig":{"leftMotorHwId":"1","rightMotorHwId":"2","wheelTravel":200,"trackWidth":254},"bufferIndex":6},{"hwid":"lcd","name":"lcd","typeName":"lcd","extraConfig":null,"bufferIndex":7},{"hwid":"sound","name":"sound","typeName":"sound","extraConfig":null,"bufferIndex":8},{"hwid":"btn_chk","name":"button_check","typeName":"face_button","extraConfig":null,"bufferIndex":9},{"hwid":"btn_up","name":"button_up","typeName":"face_button","extraConfig":null,"bufferIndex":10},{"hwid":"btn_down","name":"button_down","typeName":"face_button","extraConfig":null,"bufferIndex":11}]}"""

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
    angle = 0
    visitingHerdPoint = True
    carryingCone = False
    coneToCollect = False
    distanceTravelled = 0
    robotRadius = 15
    wheelCircumference = 20
    distanceSensorSpacing = 5

    # ---------------------------------------

    # object instantiation ------------------

    def __init__(self,leftDrive,rightDrive,dt,colorLeft,colorRight,distanceMiddle):
        self.leftDrive = leftDrive
        self.rightDrive = rightDrive
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

        self.currentAngle = self.angle #gets current gyro reading

        self.cm = self.intify(distance) #gets the distance to travel in cm (0 dp)

        if self.checkCone() == True: #if cone is in the way
            return False #robot has been unable to reach the final destination

        else: #if no cone in the way
            self.numberOfIterations = self.intify(math.floor(self.cm / 30))
            self.remainder = self.intify(self.cm % 30)

            if self.remainder == 0: #if distance is a multiple of 15

                for i in range(0,self.numberOfIterations):

                    if self.checkCone() == True:
                        return False #robot has been unable to reach the final destination

                    self.drivetrain.drive_until(30,300) #move robot by 15cm
                    self.resolveResult = self.resolveXY(self.x,self.y,30,self.currentAngle)
                    self.x , self.y = self.resolveResult.x , self.resolveResult.y

                return True #robot has been able to reach destination

            else: #if distance is not a multiple of 15

                for i in range(0,self.numberOfIterations):

                    self.drivetrain.drive_until(30,300)
                    self.resolveResult = self.resolveXY(self.x,self.y,30,self.currentAngle)
                    self.x , self.y = self.resolveResult.x , self.resolveResult.y

                    if self.checkCone() == True:
                        return False #robot has been unable to reach the final destination

                self.drivetrain.drive_until(30,self.remainder*10)
                self.resolveResult = self.resolveXY(self.x,self.y,self.remainder,self.currentAngle)
                self.x , self.y = self.resolveResult.x , self.resolveResult.y

                return True #robot has been able to reach destination

    def rotateTo(self,degrees): #rotate the robot to a certain angle

        if degrees < -180 or degrees > 180:
            return False #invalid goal angle
        else:
            self.currentAngle = self.angle #grabs the current angle of the robot
            self.goalAngle = degrees #sets the goal degrees to a new variable

            if self.currentAngle < 0 and self.goalAngle == 180: #couteract 180/-180 clash
                self.goalAngle = -180
            elif self.currentAngle > 0 and self.goalAngle == -180:
                self.goalAngle = 180

            self.deltaR = self.goalAngle - self.currentAngle #calculates how far the robot needs to rotate

            #changes the deltaR value to the desired value (avoids 180/-180 cutoff)
            if self.deltaR > 180:
                self.deltaR = (self.deltaR - 180) * -1
            elif self.deltaR < -180:
                self.deltaR = (self.deltaR + 180) * -1

            self.drivetrain.turn_until(25,-1*self.deltaR) #turn the robot

            self.angle = self.goalAngle

            return True


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

        self.distance = math.fabs(distance)

        self.radians = math.radians(rotation) #turns the gyro reading into radians

        self.gyro = rotation #keeps the gyro reading in degrees

        self.coordinates = XYCoordinates() #makes a new set of coordinates

        self.ninety = math.radians(90)

        if self.gyro == 0:
            self.coordinates.x = xCoord
            self.coordinates.y = yCoord + self.distance
        elif self.gyro == 180 or self.gyro == -180:
            self.coordinates.x = xCoord
            self.coordinates.y = yCoord - self.distance
        elif self.gyro == 90:
            self.coordinates.x = xCoord + self.distance
            self.coordinates.y = yCoord
        elif self.gyro == -90:
            self.coordinates.x = xCoord - self.distance
            self.coordinates.y = yCoord
        else:
            if self.gyro > 0 and self.gyro < 90: #first quadrant
                self.coordinates.x = xCoord + (math.sin(self.radians) * self.distance)
                self.coordinates.y = yCoord + (math.cos(self.radians) * self.distance)
            elif self.gyro > 0 and self.gyro > 90: #second quadrant
                self.coordinates.x = xCoord + (math.cos(self.radians - self.ninety) * self.distance)
                self.coordinates.y = yCoord - (math.sin(self.radians - self.ninety) * self.distance)
            elif self.gyro < 0 and self.gyro > -90: #fourth quadrant
                self.coordinates.x = xCoord - (math.sin(math.fabs(self.radians)) * self.distance)
                self.coordinates.y = yCoord + (math.cos(math.fabs(self.radians)) * self.distance)
            elif self.gyro < 0 and self.gyro < -90: #third quadrant
                self.coordinates.x = xCoord - (math.cos(math.fabs(self.radians + self.ninety)) * self.distance)
                self.coordinates.y = yCoord - (math.sin(math.fabs(self.radians + self.ninety)) * self.distance)

        self.coordinates.x = round(self.coordinates.x) #return values
        self.coordinates.y = round(self.coordinates.y)

        return self.coordinates

    # ---------------------------------------

    # DEV robot specific functions ----------

    def checkCone(self): # a simple test function to check to see if there is anything in front of the robot

        if self.distanceMiddle.distance() < 35:
            return True
        else:
            return False

# -------------------------------------------

# Hardware setup and port selection ---------

#region config
LeftDrive   = vexiq.Motor(1)
RightDrive  = vexiq.Motor(2, True) # Reverse Polarity
TouchLed    = vexiq.TouchLed(3)
MiddleUltra = vexiq.DistanceSensor(6, vexiq.UNIT_CM)
LeftColor   = vexiq.ColorSensor(7) # hue
RightColor  = vexiq.ColorSensor(8) # hue

import drivetrain
dt          = drivetrain.Drivetrain(LeftDrive, RightDrive, 200, 254.5)
#endregion config

# -------------------------------------------

# Pre-program object creation ---------------

robot = Robot(LeftDrive,RightDrive,dt,LeftColor,RightColor,MiddleUltra) #create robot class

# -------------------------------------------

# Main Program ------------------------------

while True:
    TouchLed.named_color(9) #blue

    vexiq.lcd_write("Angle: " + str(robot.angle),1)
    vexiq.lcd_write("X Coord: " + str(robot.x),2)
    vexiq.lcd_write("Y Coord: " + str(robot.y),3)


    if TouchLed.is_touch():

        TouchLed.named_color(3) #orange

        robot.moveBy(30)
        robot.rotateTo(90)

        robot.moveBy(30)
        robot.rotateTo(180)

        robot.moveBy(30)
        robot.rotateTo(-90)

        robot.moveBy(30)
        robot.rotateTo(0)

        sys.sleep(0.5)

# -------------------------------------------

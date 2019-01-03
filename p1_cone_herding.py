"""__CONFIG__
{"version":20,"widgetInfos":[{"hwid":"1","name":"LeftDrive","typeName":"motor","extraConfig":null,"bufferIndex":0},{"hwid":"2","name":"RightDrive","typeName":"motor_rp","extraConfig":null,"bufferIndex":1},{"hwid":"3","name":"ArmLeft","typeName":"motor","extraConfig":null,"bufferIndex":2},{"hwid":"4","name":"Claw","typeName":"motor","extraConfig":null,"bufferIndex":3},{"hwid":"5","name":"ArmRight","typeName":"motor_rp","extraConfig":null,"bufferIndex":4},{"hwid":"8","name":"LeftColour","typeName":"color_hue","extraConfig":null,"bufferIndex":5},{"hwid":"9","name":"RightColour","typeName":"color_hue","extraConfig":null,"bufferIndex":6},{"hwid":"10","name":"UltraLeft","typeName":"distance_cm","extraConfig":null,"bufferIndex":7},{"hwid":"11","name":"UltraRight","typeName":"distance_cm","extraConfig":null,"bufferIndex":8},{"hwid":"12","name":"TouchLed","typeName":"touch_led","extraConfig":null,"bufferIndex":9},{"hwid":"drivetrain","name":"dt","typeName":"drivetrain","extraConfig":{"leftMotorHwId":"1","rightMotorHwId":"2","wheelTravel":200,"trackWidth":212},"bufferIndex":10},{"hwid":"lcd","name":"lcd","typeName":"lcd","extraConfig":null,"bufferIndex":11},{"hwid":"sound","name":"sound","typeName":"sound","extraConfig":null,"bufferIndex":12},{"hwid":"btn_chk","name":"button_check","typeName":"face_button","extraConfig":null,"bufferIndex":13},{"hwid":"btn_up","name":"button_up","typeName":"face_button","extraConfig":null,"bufferIndex":14},{"hwid":"btn_down","name":"button_down","typeName":"face_button","extraConfig":null,"bufferIndex":15}]}"""

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

    colours = {"off":0,"red":1,"red_orange":2,"orange":3,"yellow_orange":4,"yellow":5,"yellow_green":6,"green":7,"blue_green":8,"blue":9,"blue_violet":10,"violet":11,"red_violet":12,"white":13}

    # ---------------------------------------

    # object instantiation ------------------

    def __init__(self,dt,armL,armR,claw,colorLeft,colorRight,distanceLeft,distanceRight,led):
        self.drivetrain = dt
        self.colorRight = colorRight
        self.colorLeft = colorLeft
        self.distanceLeft = distanceLeft
        self.distanceRight = distanceRight
        self.led = led
        self.armLeft = armL
        self.armRight = armR
        self.claw = claw

        self.armLeft.hold()
        self.armRight.hold()

        self.colorLeft.set_proximity_threshold(0)
        self.colorRight.set_proximity_threshold(0)

        #self.claw.run(-30)
        #sys.sleep(1)
        #self.claw.hold()
        #self.claw.reset_position()


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

    def isActivated(self): #will return true if the led is pressed
        if self.led.is_touch():
            return True
        else:
            return False

    def light(self,colour,blink = False): #control the led
        self.code = self.colours[colour]
        self.led.named_color(self.code)
        if blink == True:
            self.led.blink()
        return None

    def returnToHerdPoint(self,herdpoint): #return the robot to the herd point
        return None

    def moveToXYA(self,x,y,angle = None): #move the robot to an x coord, y coord and angle of rotation
        self.currentX = self.x
        self.currentY = self.y

        self.deltaX = math.fabs(x - self.currentX)
        self.deltaY = math.fabs(y - self.currentY)

        if self.deltaX == 0 and self.deltaY != 0:
            if self.currentY > y:
                self.rotateTo(180)
            else:
                self.rotateTo(0)
        elif self.deltaX != 0 and self.deltaY == 0:
            if self.currentX > x:
                self.rotateTo(-90)
            else:
                self.rotateTo(90)
        else:
            if x > self.currentX and y > self.currentY:
                self.rotation = math.degrees(math.atan(self.deltaX / self.deltaY))
                self.rotateTo(self.rotation)
            elif x > self.currentX and y < self.currentY:
                self.rotation = math.degrees(math.atan(self.deltaY / self.deltaX)) + 90
                self.rotateTo(self.rotation)
            elif x < self.currentX and y > self.currentY:
                self.rotation = math.degrees(math.atan(self.deltaX / self.deltaY)) * -1
                self.rotateTo(self.rotation)
            elif x < self.currentX and y < self.currentY:
                self.rotation = math.degrees(math.atan(self.deltaX / self.deltaY)) - 180
                self.rotateTo(self.rotation)

        self.distance = self.calculateDeltaD(x,y,self.currentX,self.currentY)
        self.motion = self.moveBy(self.distance)

        if self.motion == False:
            return False
        else:
            self.x = x
            self.y = y
            if angle != None:
                self.rotateTo(angle)
            return True

    def collectCone(self): #collect a cone
        #align to cone
        #calculate distance to cone
        #drive to cone
        #pick up cone
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

                vexiq.lcd_write("Hello!")
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

    def rotateBy(self,degrees): #turn the robot, positive for right, negative for left

        self.deltaA = round(degrees)
        self.drivetrain.turn_until(22,-1*self.deltaA) #turn the robot

        if self.deltaA > 0: #remove any excess 360 rotations
            while self.deltaA >= 360:
                self.deltaA = self.deltaA - 360
        elif self.deltaA < 0:
            while self.deltaA <= -360:
                self.deltaA = self.deltaA + 360

        self.newAngle = self.angle + self.deltaA #calculate final angle with v = u + d calculation

        if self.newAngle > 180: #trim difference at 180/-180 border
            self.newAngle = self.newAngle - 360
        if self.newAngle < -180:
            self.newAngle = self.newAngle + 360

        self.angle = round(self.newAngle) #set new angle

    def rotateTo(self,degrees): #rotate the robot to a certain angle

        if degrees < -180 or degrees > 180:
            return False #invalid goal angle
        else:
            self.currentAngle = self.angle #grabs the current angle of the robot
            self.goalAngle = round(degrees) #sets the goal degrees to a new variable

            if self.currentAngle < 0 and self.goalAngle == 180: #couteract 180/-180 clash
                self.goalAngle = -180
            elif self.currentAngle > 0 and self.goalAngle == -180:
                self.goalAngle = 180
            elif self.currentAngle == 180 and self.goalAngle < 0:
                self.currentAngle = -180
            elif self.currentAngle == -180 and self.goalAngle > 0:
                self.currentAngle = 180

            self.deltaR = self.goalAngle - self.currentAngle #calculates how far the robot needs to rotate

            #changes the deltaR value to the desired value (avoids 180/-180 cutoff)
            if self.deltaR > 180:
                self.deltaR = (self.deltaR - 180) * -1
            elif self.deltaR < -180:
                self.deltaR = (self.deltaR + 180) * -1

            self.drivetrain.turn_until(22,-1*self.deltaR) #turn the robot

            self.angle = self.goalAngle

            return True

    def liftArm(self):
        return None

    def lowerArm(self):
        return None

    def closeClaw(self):
        self.claw.run_until_position(70,60,True)
        return True

    def openClaw(self):
        self.claw.run_until_position(70,0,True)
        return True

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

    def calculateDeltaD(self,goalX,goalY,currentX,currentY): #function to return distance between two coordinates
        self.deltaX = math.fabs(currentX - goalX)
        self.deltaY = math.fabs(currentY - goalY)
        self.deltaD = math.fabs(math.sqrt((self.deltaX ** 2) + (self.deltaY ** 2)))
        return self.deltaD

    # ---------------------------------------

    # DEV robot specific functions ----------

    def checkCone(self): # a simple test function to check to see if there is anything in front of the robot

        if self.distanceLeft.distance() < 35 or self.distanceRight.distance() < 35:
            return True
        else:
            return False

    def alignToObject(self,repeat = False):

        ur = round(self.distanceRight.distance())
        ul = round(self.distanceLeft.distance())

        if ur > 50 and ul > 50:
            return None
        else:
            if repeat == True:
                while repeat:
                    ur = round(self.distanceRight.distance())
                    ul = round(self.distanceLeft.distance())
                    vexiq.lcd_write("LU: " + str(round(UltraLeft.distance())),4)
                    vexiq.lcd_write("RU: " + str(round(UltraRight.distance())),5)
                    if ur > ul:
                        self.drivetrain.turn(10)
                    elif ur < ul:
                        self.drivetrain.turn(-10)
                    else:
                        self.drivetrain.hold()

            elif repeat == False:
                while ur != ul:
                    if ur > ul:
                        self.drivetrain.turn(10)
                    elif ur < ul:
                        self.drivetrain.turn(-10)
                    else:
                        break
                    ur = round(self.distanceRight.distance())
                    ul = round(self.distanceLeft.distance())
                self.drivetrain.hold()

                sys.sleep(0.7)

                ur = round(self.distanceRight.distance())
                ul = round(self.distanceLeft.distance())

                if ur != ul:
                    self.alignToObject()

                return None

    def lookingAtCone(self):
        if (self.colorLeft.named_color() == 3 or self.colorLeft.named_color() == 4 or self.colorLeft.named_color() == 5) and (self.colorRight.named_color() == 3 or self.colorRight.named_color() == 4 or self.colorRight.named_color() == 5):
            return True
        else:
            return False


# -------------------------------------------

# Hardware setup and port selection ---------

#region config
LeftDrive   = vexiq.Motor(1)
RightDrive  = vexiq.Motor(2, True) # Reverse Polarity
ArmLeft     = vexiq.Motor(3)
Claw        = vexiq.Motor(4)
ArmRight    = vexiq.Motor(5, True) # Reverse Polarity
LeftColour  = vexiq.ColorSensor(8) # hue
RightColour = vexiq.ColorSensor(9) # hue
UltraLeft   = vexiq.DistanceSensor(10, vexiq.UNIT_CM)
UltraRight  = vexiq.DistanceSensor(11, vexiq.UNIT_CM)
TouchLed    = vexiq.TouchLed(12)

import drivetrain
dt          = drivetrain.Drivetrain(LeftDrive, RightDrive, 200, 212)
#endregion config

# DRIVETRAIN: 210 for foam tiles

# -------------------------------------------

# Pre-program object creation ---------------

robot = Robot(dt,ArmLeft,ArmRight,Claw,LeftColour,RightColour,UltraLeft,UltraRight,TouchLed) #create robot class

# -------------------------------------------

# Test Functions ----------------------------


# -------------------------------------------

# Main Program ------------------------------

while True:
    robot.light("blue")
    vexiq.lcd_write("Angle: " + str(robot.angle),1)
    vexiq.lcd_write("X Coord: " + str(robot.x),2)
    vexiq.lcd_write("Y Coord: " + str(robot.y),3)
    vexiq.lcd_write("LU: " + str(round(UltraLeft.distance())),4)
    vexiq.lcd_write("RU: " + str(round(UltraRight.distance())),5)

    if robot.isActivated():

        sys.sleep(0.2)
        robot.light("orange",True)

        # Motion call ---------------
        """
        while robot.isActivated() == False:
            if robot.lookingAtCone():
                vexiq.lcd_write("Found a cone :)")
            else:
                vexiq.lcd_write("No cone found :(")"""

        robot.openClaw()
        sys.sleep(2)
        robot.closeClaw()
        sys.sleep(5)
        robot.openClaw()

        # ---------------------------

        sys.sleep(0.5)

# -------------------------------------------

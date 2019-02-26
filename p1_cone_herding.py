# external library imports ------------------

import sys
import vexiq
import drivetrain
import math

# -------------------------------------------

# objects and classes -----------------------

# XYCoordinates class
class XYCoordinates:
    def __init__(self): #intiation function
        self.x = 0 #x coordinate
        self.y = 0 #y coordinate
        return None

# Cone class
class Cone:
    def __init__(self,x,y): #initiation function
        self.x = x # x coordinate
        self.y = y # y cordinate
        self.herded = False # shows whether the cone has been herded yet
        return None

# Readings class
class Readings:
    def __init__(self,left,right):
        self.left = left
        self.right = right

# Herd point class
class Herdpoint:
    def __init__(self,x,y,radius = 20):
        self.x = x
        self.y = y
        self.radius = radius

# Walls class
class Walls:
    def __init__(self, Top, Bottom, Left, Right): #specify all four walls
        self.top = Top
        self.bottom = Bottom
        self.left = Left
        self.right = Right

# Path class
class Path:
    def __init__(self, direction, x_line, y_upper, y_lower):

        self.direction = direction #sets all the variables needed
        self.xLine = x_line
        self.yUpper = y_upper
        self.yLower = y_lower
        self.completed = False
        self.xlastVisited = 0
        self.ylastVisited = 0
        self.simpleTraverse = False

        if self.direction == "up" and self.yUpper != None: #sets the goalY if they are able to be calculated
            self.simpleTraverse = True
            self.goalY = self.yUpper
        elif self.direction == "down" and self.yLower != None:
            self.simpleTraverse = True
            self.goalY = self.yLower

# Robot class
class Robot:

    # public properties----------------------

    x = 0 # x coordinate
    y = 0 # y coordinate
    angle = 0 # current angle of the robot
    robotRadius = 7 # distacne between center of displacement and ultrasonic sensors
    allCones = [] #holds all the cones
    carryingCone = False # shows whether the robot is carrying a cone or not
    robotWidth = 25 #needs to be measured

    #array storing all the colour codes for the led
    colours = {"off":0,"red":1,"red_orange":2,"orange":3,"yellow_orange":4,"yellow":5,"yellow_green":6,"green":7,"blue_green":8,"blue":9,"blue_violet":10,"violet":11,"red_violet":12,"white":13}

    # ---------------------------------------

    # object instantiation ------------------

    def __init__(self,dt,armL,armR,claw,colorLeft,colorRight,distanceLeft,distanceRight,led): #intiiation function

        self.drivetrain = dt #sets hardware variables
        self.colorRight = colorRight
        self.colorLeft = colorLeft
        self.distanceLeft = distanceLeft
        self.distanceRight = distanceRight
        self.led = led
        self.armLeft = armL
        self.armRight = armR
        self.claw = claw

        self.claw.off()
        self.armLeft.hold()
        self.armRight.hold()

        self.colorLeft.set_proximity_threshold(0) # sets the accuracy and distance on the colour sensors
        self.colorRight.set_proximity_threshold(0)

        self.herdpoint = Herdpoint(0,0) # sets the location of the herdpoint


    # ---------------------------------------

    # basic mathematical functions ----------

    def intify(self,number): #used to convert from float to int
        self.num = round(number)
        self.iterate = 0
        if number > 0: #for positive numbers
            while self.iterate != self.num:
                self.iterate+=1
        else: #for negative numbers
            while self.iterate != self.num:
                self.iterate-=1
        return self.iterate #returns intified number

    def meanOfValues(self,numbers): #used to get mean of a number of numbers
        self.mean = 0
        for x in numbers: #adds up all the values
            self.mean = self.mean + x

        self.mean = self.mean / len(numbers) #divides by number of values
        return self.mean

    def standardDeviation(self,numbers): #used to get standardDeviation of a number of numbers
        self.sd = 0
        self.mean = self.meanOfValues(numbers) #gets mean of all values
        for x in numbers:
            self.sd = self.sd + ((x - self.mean) ** 2)

        self.sd = self.sd / len(numbers) #calculates variance
        self.sd = math.sqrt(self.sd) #calculates standard deviation
        return round(self.sd)

    # ---------------------------------------

    # NON-DEV robot specific functions ------

    def isActivated(self): #will return true if the led is pressed
        if self.led.is_touch(): #checks the hardware
            return True
        else:
            return False

    def light(self,colour,blink = False): #control the led
        self.code = self.colours[colour] #finds the colour code in the colour dictionary
        self.led.named_color(self.code) #sets the colour of the led
        if blink == True: #if blink was specified, blink the led
            self.led.blink()
        return None

    def returnToHerdPoint(self,deliverCone = False): #return the robot to the herd point
        if self.herdpoint == None: #herdpoint doesn't exist
            return False
        else:
            self.hx = self.herdpoint.x #grab the herdpoint coordinates
            self.hy = self.herdpoint.y
            self.moveToXYA(self.hx,self.hy) #move the robot to the herdpoint

            self.intialDistance = self.resolveReadings(2) # grab the sighting distance

            if self.intialDistance < 40: #only if there is another cone there 
                self.deltaD = round(self.intialDistance - (35 - self.robotRadius)) #calculate change in displacement
                self.moveBy(self.deltaD,True) #move the robot

            if deliverCone == True: #deliver the cone
                self.deliverCone()
            
            return True

    def returnToPathPoint(self,path): #return the robot to a point on a path
        vexiq.lcd_write("Returning To Path")
        self.moveToXYA(path.xlastVisited,path.ylastVisited,None,True)
        self.debug()

    def moveToXYA(self,x,y,angle = None,ignoreCone = False): #move the robot to an x coord, y coord and angle of rotation
        self.currentX = self.x #grabs current x coordinate
        self.currentY = self.y #grabs current y coordinate

        self.deltaX = float(math.fabs(x - self.currentX)) #calulates distance between two x coordinates
        self.deltaY = float(math.fabs(y - self.currentY)) #calulates distance between two y coordinates

        if self.deltaX == 0 and self.deltaY != 0: #if the robot is only moving in the y axis
            if self.currentY > y: #if the movement is negative
                self.rotateTo(180) #rotate the robot to 180 degrees
            else: #if the movement is positive
                self.rotateTo(0) #rotate the robot to 0 degrees
        elif self.deltaX != 0 and self.deltaY == 0: #if the robot is only moving in the x axis
            if self.currentX > x: # if the movement is negative
                self.rotateTo(-90) #rotate the robot to -90 degrees
            else: #if the movmeent is postiive
                self.rotateTo(90) # rotate the robot to 90 degrees
        else: #if there is movmeent in the x and y axis'
            if x > self.currentX and y > self.currentY: #if the robot is moving into the first quadrant
                self.rotation = math.degrees(math.atan(self.deltaX / self.deltaY)) #calculate the amount of rotation relative to 0
                self.rotateTo(self.rotation) #rotate to this newly calculated angle
            elif x > self.currentX and y < self.currentY: #if the robot is moving into the second quadrant
                self.rotation = 180 - math.degrees(math.atan(self.deltaX / self.deltaY)) #calculate the amount of rotation relative to 0
                self.rotateTo(self.rotation) #rotate to this newly calculated angle
            elif x < self.currentX and y > self.currentY: #if the robot is moving into the fourth quadrant
                self.rotation = math.degrees(math.atan(self.deltaX / self.deltaY)) * -1 #calculate the amount of rotation relative to 0
                self.rotateTo(self.rotation) #rotate to this newly calculated angle
            elif x < self.currentX and y < self.currentY: #if the robot is moving into the third quadrant
                self.rotation = math.degrees(math.atan(self.deltaX / self.deltaY)) - 180 #calculate the amount of rotation relative to 0
                self.rotateTo(self.rotation) #rotate to this newly calculated angle

        self.distance = self.calculateDeltaD(x,y,self.currentX,self.currentY) # calculate the absolute value between the two coordinates
        self.motion = self.moveBy(self.distance,ignoreCone) #move by the newly found distance, and specifiying the ignoreCone parmater with a variable

        if self.motion == False: #if the robot was unable to complete its journey
            return False #return that journey was unsuccessful
        else: #otherwsie,
            self.x = x #set new x coordinate
            self.y = y #set new y coordinate
            if angle != None: # if a end rotation angle was specified
                self.rotateTo(angle) #rotate to the specified angle
            return True #return that journey was successful

    def collectCone(self): #collect a cone
        self.light("green",True)
        self.openClaw()
        sys.sleep(0.25)
        self.lowerArm()
        sys.sleep(0.25)

        self.moveBy(-8,True)
        self.moveBy(16,True)
        self.rotateBy(-10)
        self.rotateBy(20)
        self.rotateBy(-10)
        self.moveBy(8,True)
        self.moveBy(-2,True)

        self.closeClaw()
        self.openClaw()
        self.closeClaw()
        self.liftArm()
        self.light("orange",True)

    def deliverCone(self): #deliver a cone
        self.light("green",True)
        self.lowerArm()
        sys.sleep(0.25)
        self.openClaw()
        sys.sleep(0.25)
        self.liftArm()
        sys.sleep(0.25)
        self.openClaw() # second opening of claw as often claw sticks
        self.light("orange",True)
        return None

    def moveBy(self,distance,ignoreCone = False): #move the robot forwards by a certain distance

        self.currentAngle = self.angle #gets current gyro reading
        self.cm = self.intify(distance) #gets the distance to travel in cm (0 dp)
        self.numberOfIterations = self.intify(math.floor(self.cm / 30))
        self.remainder = self.intify(self.cm % 30)

        if self.numberOfIterations < 0: #going backwards!
            self.drivetrain.drive_until(30,self.cm*10) # no need to check for cones so just go for it

            self.resolveResult = self.resolveXY(self.x,self.y,self.cm,self.currentAngle) #update coordinates
            self.x , self.y = self.resolveResult.x , self.resolveResult.y

            return True
        else:
            if ignoreCone == False: #check for a cone
                if self.resolveReadings(1) == True: #if cone is in the way
                    return False #robot has been unable to reach the final destination

            if self.remainder == 0: #if distance is a multiple of 15

                for i in range(0,self.numberOfIterations):

                    if ignoreCone == False:
                        if self.resolveReadings(1) == True: #if cone is in the way
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

                    if ignoreCone == False:
                        if self.resolveReadings(1) == True: #if cone is in the way
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
                self.deltaR = (self.deltaR - 360)
            elif self.deltaR < -180:
                self.deltaR = (self.deltaR + 360)

            self.drivetrain.turn_until(22,-1*self.deltaR) #turn the robot

            self.angle = self.goalAngle #set new angle

            return True

    def liftArm(self): #lift the arm
        self.armLeft.run_to_position(100,0,True)
        self.armRight.run_to_position(100,0,True)
        #stops the function returning whilst still moving
        while self.armLeft.position() > 0 and self.armRight.position() > 0:
            continue
        self.armLeft.hold() #turns the motors off in their new position
        self.armRight.hold()
        return None

    def lowerArm(self): #lower the arm
        self.armLeft.run_to_position(30,260,True)
        self.armRight.run_to_position(30,260,True)
        self.giveUp = 0
        #stops the function returning whilst still moving
        while self.armLeft.position() < 260 and self.armRight.position() < 260:
            self.giveUp = self.giveUp + 1
            sys.sleep(0.25)
            if self.giveUp == 6:
                break
            else:
                continue
        self.armLeft.hold() #holds the motors in their new position
        self.armRight.hold()
        return None

    def closeClaw(self): #close the claw
        self.claw.run_until_position(70,170,True)
        self.claw.hold()
        return True

    def openClaw(self): #open the claw
        self.claw.run_until_position(70,0,True)
        self.claw.off()
        return True

    def resolveXY(self,xCoord,yCoord,distance,rotation): #function to update distaplacement of the robot by calculating new coordinates

        self.distance = distance

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

        self.coordinates.x = round(self.coordinates.x) #create new coordinates
        self.coordinates.y = round(self.coordinates.y)

        return self.coordinates #return values

    def calculateDeltaD(self,goalX,goalY,currentX,currentY): #function to return distance between two coordinates
        self.deltaX = math.fabs(currentX - goalX) #caclulates change in x axis
        self.deltaY = math.fabs(currentY - goalY) #caclulates change in y axis
        self.deltaD = math.fabs(math.sqrt((self.deltaX ** 2) + (self.deltaY ** 2))) # performs pythagoras on two values
        return self.deltaD #returns absolute value

    def recordNewCone(self,distance): #records the location of a new cone
        self.coneLocation = self.resolveXY(self.x,self.y,distance,self.angle) #gets the coordinates of the new cone
        self.duplicateCone = False

        for cone in self.allCones: #for all the cones currently in the field
            # finds the distance between new cone and cone in question
            self.distanceBetweenCones = self.calculateDeltaD(cone.x,cone.y,self.coneLocation.x,self.coneLocation.y)
            if self.distanceBetweenCones < 20: #20 being cone radius * 2 | detects a duplicate cone
                cone.x = (cone.x + self.coneLocation.x) / 2 #updates cone location if it was a duplicate
                cone.y = (cone.y + self.coneLocation.y) / 2
                self.duplicateCone = True
                break

        if self.duplicateCone == False: #if the cone was not a duplicate
            self.allCones.append(Cone(self.coneLocation.x,self.coneLocation.y)) #add a new cone object
            return True
        else:
            return False

    def averageReadings(self,delay = 0.1,repeat = 10,sd_multiplier = 2.5): #gets the readings of the ultrasonic, with many overloads
        self.leftNumbers = [] #variables
        self.rightNumbers = []
        self.leftReading = 0
        self.rightReading = 0

        for x in range(0,repeat): #grabs values
            self.leftNumbers.append(self.distanceLeft.distance())
            self.rightNumbers.append(self.distanceRight.distance())
            sys.sleep(delay)

        self.sdl = round(self.standardDeviation(self.leftNumbers) * sd_multiplier) #creates multiplied sd
        self.sdr = round(self.standardDeviation(self.rightNumbers) * sd_multiplier)
        self.lmean = self.meanOfValues(self.leftNumbers) #creates mean
        self.rmean = self.meanOfValues(self.rightNumbers)

        self.newLeftNumbers = [] #new arrays
        self.newRightNumbers = []

        for number in self.leftNumbers: #remove values outside the limits
            if number >= (self.lmean - self.sdl) or number <= (self.lmean + self.sdl):
                self.newLeftNumbers.append(number)
        for number in self.rightNumbers: #remove values outside the limits
            if number >= (self.rmean - self.sdr) or number <= (self.rmean + self.sdr):
                self.newRightNumbers.append(number)

        self.leftReading = round(self.meanOfValues(self.newLeftNumbers),2) #creates new averages
        self.rightReading = round(self.meanOfValues(self.newRightNumbers),2)
        self.newReadings = Readings(self.leftReading,self.rightReading) #creates a new readings class
        return self.newReadings #returns the readings

    def alignToCone(self): #will align the robot to a cone infront of it

        if self.resolveReadings(1) == False: # if not looking at cone and both distances sensors show nothing
            return False #return failed align
        else:

            # intial align - cone not in front of robot

            self.light("white",True) #intial align
            self.intialDistance = self.resolveReadings(2) # grab the sighting distance

            if self.intialDistance >= 20: #if the intial is large
                self.deltaD = round(self.intialDistance - (21 - self.robotRadius)) #calulate change in displacement
                self.moveBy(self.deltaD,True) #move the robot

            # line the robot up to be pointing at the cone

            self.light("yellow",True) #swing align

            self.maxSwingAmount = 5 #set the number of times the robot will swing from side to side
            self.intialSwingAmount = 5 #how many turns to do intially
            self.flag = False #shows if the robot has been successful in the sweeps
            self.turnAmount = 8 #how far to rotate the robot by

            self.readingsResult = self.averageReadings(0,5)
            if self.readingsResult.left > self.readingsResult.right: #set the intial swing direction
                self.directionOfSwing = 1 #turning right
            else:
                self.directionOfSwing = -1 #turning left

            #self.lightOn = False

            for x in range(0,self.intialSwingAmount): #intial swing
                if self.lookingAtCone():
                    self.flag = True
                    break
                self.rotateBy(self.directionOfSwing * self.turnAmount) #rotate the robot

            self.directionOfSwing = self.directionOfSwing * -1

            if self.flag == False: #if the cone was not found
                for y in range(((self.intialSwingAmount*2) + 1), (self.maxSwingAmount + ((self.intialSwingAmount*2) + 1))): #for number of swings
                    for z in range(0,y):
                        if self.lookingAtCone(): #check if looking at cone
                            self.flag = True
                            break
                        self.rotateBy(self.directionOfSwing * self.turnAmount) #rotate the robot
                    if self.lookingAtCone():
                        self.flag = True
                        break
                    self.directionOfSwing = self.directionOfSwing * -1 #change direction of swing

            # robot now aligned, final move to the robot

            if self.flag == False: #robot was unable to align
                return False
            else:
                self.light("violet",True) #ultra align

                self.uReadings = self.resolveReadings(3)

                self.deltaD = round(self.uReadings - 21) #creates distance to the cone
                self.moveBy(self.deltaD,True)

                robot.light("orange",True) #return to program

                return self.uReadings #returns distance to cone (to reduce wait time)

    def resolveReadings(self,option,distance = 1): #resolve the location of the cone using the ultrasonic sensors

        if option == 1: #BASIC DISTANCE CHECK, QUICK TO SEE IF ANYTHING IN THE WAY
            self.readingsResult = self.averageReadings(0,5) #grabs readings with 0 delay in the checks and half the number of readings
            if self.readingsResult.left < 40 or self.readingsResult.right < 40: #checks to see if the readings are below a certain distance
                return True #returns true
            else:
                return False #returns false

        elif option == 2: #UNALIGNED DISTANCE CHECK, WILL RETURN SHORTEST DISTANCE
            self.readingsResult = self.averageReadings() #grabs readings
            if self.readingsResult.left < self.readingsResult.right: #checks to see which reading is lower
                return self.readingsResult.left
            else:
                return self.readingsResult.right

        elif option == 3: #ALIGNED DISTANCE CHECK, WILL RETURN EXACT DISTANCE TO OBJECT
            self.readingsResult = self.resolveReadings(2) #grabs readings

            self.distance = math.sqrt((self.readingsResult ** 2) - (5**2)) #perform some cheeky pythagoras
            self.distance = round(self.distance + self.robotRadius) #add on robot radius (means the distance is relative to x,y coord)

            return self.distance

        elif option == 4:  #WILL TEST TO FIND THE DISTANCE BETWEEN THE TWO READINGS, SEE HOW EQUAL THEY ARE
            self.readingsResult = self.averageReadings(0) #grabs readings
            if self.readingsResult.left < 30 and self.readingsResult.right < 30:
                self.deltaU = math.fabs(self.readingsResult.left - self.readingsResult.right) #finds absolute difference
                if self.deltaU <= distance: #checks to see if difference is within range
                    return True
                else:
                    return False
            else:
                return False

        else: #IF NO VALID OPTION PICKED
            return None

    def lookingAtCone(self): #returns whether the robot is looking at a cone using colour sensors
        if (self.colorLeft.named_color() == 3 or self.colorLeft.named_color() == 4 or self.colorLeft.named_color() == 5 or self.colorLeft.named_color() == 6 or self.colorLeft.named_color() == 7) and (self.colorRight.named_color() == 3 or self.colorRight.named_color() == 4 or self.colorRight.named_color() == 5 or self.colorRight.named_color() == 6 or self.colorRight.named_color() == 7):
            return True
        else:
            return False

    # ---------------------------------------

    # DEV robot specific functions ----------

    def debug(self,updateScreen = True,debugDelay = 0,color = None): #used solely for debugging, allows time delay, screen update and led change
        if updateScreen == True:
            vexiq.lcd_write("X: " + str(robot.intify(robot.x)) + ", Y: "+ str(robot.intify(robot.y)) + ", A: " + str(robot.intify(robot.angle)),1) #updates screen

        if color != None:
            self.code = self.colours[color] #sets led colour
            self.led.named_color(self.code)

        sys.sleep(debugDelay) #delays the program
        return None

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

# DRIVETRAIN: 212 for foam tiles and table
# DRIVETRAIN: 215 for carpet

# -------------------------------------------

# Pre-program object creation ---------------

robot = Robot(dt,ArmLeft,ArmRight,Claw,LeftColour,RightColour,UltraLeft,UltraRight,TouchLed) #create robot class
allPaths = [] #stores all paths in use
coneWallDistance = 0 #the distance where they have reached the wall and cone cannot be in the way
zoneWalls = Walls(125,0,-50,50)
allConesHerded = False

# -------------------------------------------

# Search Functions --------------------------

def traversePathSimple(path):

    while robot.y != path.goalY: #while still not at the goalY (determines path completion)

        # realign to the path -----------------------------------------------------------------

        while robot.x != path.xLine: #keep aligning to the path line

            if robot.x != path.xLine:
                result = robot.moveToXYA(path.xLine,robot.y) #move back to the path
                
                if result == False:
                    result = robot.alignToCone()
                    if result != False:
                        if robot.carryingCone == True:

                            path.xLastPosition = robot.x #sets the xLastPosition and yLastPosiiton variables
                            path.yLastPosiiton = robot.y

                            robot.debug(False,0,"red")
                            robot.returnToHerdPoint(True) #take cone back
                            robot.carryingCone = False #set carrying cone to true
                            robot.debug(False,0,"red_orange")
                            vexiq.lcd_write("Returning to path")
                            robot.returnToPathPoint(path) #come back to path point

                        else:
                            robot.collectCone() #pickup the cone
                            robot.carryingCone = True #set carrying cone to true 
        
        #move the robot to the goalY ----------------------------------------------------------

        result = robot.moveToXYA(path.xLine,path.goalY) #move to the goalY

        #the move to xya has failed -----------------------------------------------------------

        if result == False:
            if robot.y < path.goalY + coneWallDistance and robot.y > path.goalY - coneWallDistance: #near enough to the wall
                break #complete the path
            else: #it is a cone (simple traverse knows where walls are)
                result = robot.alignToCone() #align to the cone
                if result != False:
                    if robot.carryingCone == True:

                        path.xLastPosition = robot.x #sets the xLastPosition and yLastPosiiton variables
                        path.yLastPosiiton = robot.y

                        robot.debug(False,0,"red")
                        robot.returnToHerdPoint(True) #take cone back
                        robot.carryingCone = False #set carrying cone to true
                        robot.debug(False,0,"red_orange")
                        vexiq.lcd_write("Returning to path")
                        robot.returnToPathPoint(path) #come back to path point

                    else:
                        robot.collectCone() #pickup the cone
                        robot.carryingCone = True #set carrying cone to true                        

        # set path as completed -------------------------------------------------------------------

    path.completed = True
    return path # path was completed

def createNewPath():
    pass

def initialUpDown():
    tempPath1 = Path("up",0,zoneWalls.top,zoneWalls.bottom) #traverse upwards
    traversePathSimple(tempPath1)

    tempPath2 = Path("down",0,zoneWalls.top,zoneWalls.bottom) #traverse downwards
    traversePathSimple(tempPath2)

    initialPath = Path("down",0,zoneWalls.top,zoneWalls.bottom) #creates the intial path and sends it to the array
    initialPath.completed = True #set path to completed                         
    allPaths.append(initialPath) #append path to array

def herdAllCones():
    initialUpDown()

# -------------------------------------------

# Main Program ------------------------------

while True:
    robot.light("blue")
    vexiq.lcd_write("X: " + str(robot.intify(robot.x)) + ", Y: "+ str(robot.intify(robot.y)) + ", A: " + str(robot.intify(robot.angle)),1) #standard screen output

    if robot.isActivated():

        sys.sleep(0.2)
        robot.light("orange",True)

        # Motion call ---------------

        herdAllCones()
        #robot.alignToCone()
        #robot.collectCone()

        # ---------------------------

        sys.sleep(0.2)

# -------------------------------------------

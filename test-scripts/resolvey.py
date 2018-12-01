# objects and classes -----------------------

import math

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

    def __init__(self):
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
            if self.gyro > 0 and self.gyro < 90:
                self.coordinates.x = xCoord + (math.sin(self.radians) * self.distance)
                self.coordinates.y = yCoord + (math.cos(self.radians) * self.distance)
            elif self.gyro > 0 and self.gyro > 90:
                self.coordinates.x = xCoord + (math.cos(self.radians - self.ninety) * self.distance)
                self.coordinates.y = yCoord - (math.sin(self.radians - self.ninety) * self.distance)
            elif self.gyro < 0 and self.gyro > -90:
                self.coordinates.x = xCoord - (math.sin(math.fabs(self.radians)) * self.distance)
                self.coordinates.y = yCoord + (math.cos(math.fabs(self.radians)) * self.distance)
            elif self.gyro < 0 and self.gyro < -90:
                self.coordinates.x = xCoord - (math.cos(math.fabs(self.radians + self.ninety)) * self.distance) 
                self.coordinates.y = yCoord - (math.sin(math.fabs(self.radians + self.ninety)) * self.distance)

        self.coordinates.x = round(self.coordinates.x)
        self.coordinates.y = round(self.coordinates.y)

        return self.coordinates

robot = Robot()

x = 0
y = 0

result = robot.resolveXY(x,y,10,45)

x = result.x
y = result.y

result = robot.resolveXY(x,y,10,180)

x = result.x
y = result.y

result = robot.resolveXY(x,y,10,-135)

print("X: " + str(result.x))
print("Y: " + str(result.y))

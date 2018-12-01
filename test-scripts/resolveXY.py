import math

class XYCoordinates:
    def __init__(self):
        self.x = 0
        self.y = 0


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

result = resolveXY(0,0,10,135)
print("X: " + str(result.x))
print("Y: " + str(result.y))

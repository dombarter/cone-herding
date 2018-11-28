import math

class XYCoordinates:
    def __init__(self):
        self.x = 0
        self.y = 0


def resolveXY(xCoord,yCoord,distance,gyro):

    gyro = math.radians(gyro) #turns the gyro reading into radians

    print(gyro)

    coordinates = XYCoordinates() #makes a new set of coordinates

    if gyro == 0:
        coordinates.y = yCoord + distance
    elif gyro == 180 or gyro == -180:
        coordinates.y = yCoord - distance
    elif gyro == 90:
        coordinates.x = xCoord + distance
    elif gyro == -90:
        coordinates.x = xCoord - distance
        print("hello")
    else:
        if gyro > 0 and gyro < 90:
            coordinates.x = xCoord + (math.sin(gyro) * distance)
            coordinates.y = yCoord + (math.cos(gyro) * distance)
        elif gyro > 0 and gyro > 90:
            coordinates.x = xCoord + (math.cos(gyro - 90) * distance)
            coordinates.y = yCoord - (math.sin(gyro - 90) * distance)
        elif gyro < 0 and gyro > -90:
            coordinates.x = xCoord - (math.sin(math.fabs(gyro)) * distance)
            coordinates.y = yCoord + (math.cos(math.fabs(gyro)) * distance)
        elif gyro < 0 and gyro < -90:
            coordinates.x = xCoord - (math.cos(math.fabs(gyro + 90)) * distance)
            coordinates.y = yCoord - (math.sin(math.fabs(gyro + 90)) * distance)

    coordinates.x = round(coordinates.x)
    coordinates.y = round(coordinates.y)

    return coordinates

result = resolveXY(0,0,10,-90)
print("X: " + str(result.x))
print("Y: " + str(result.y))

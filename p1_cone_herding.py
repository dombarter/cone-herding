"""__CONFIG__
{"version":20,"widgetInfos":[{"hwid":"1","name":"LeftDrive","typeName":"motor","extraConfig":null,"bufferIndex":0},{"hwid":"2","name":"RightDrive","typeName":"motor_rp","extraConfig":null,"bufferIndex":1},{"hwid":"3","name":"TouchLed","typeName":"touch_led","extraConfig":null,"bufferIndex":2},{"hwid":"4","name":"Gyro1","typeName":"gyro","extraConfig":null,"bufferIndex":3},{"hwid":"5","name":"Gyro2","typeName":"gyro","extraConfig":null,"bufferIndex":4},{"hwid":"6","name":"MiddleUltra","typeName":"distance_cm","extraConfig":null,"bufferIndex":5},{"hwid":"7","name":"LeftColor","typeName":"color_hue","extraConfig":null,"bufferIndex":6},{"hwid":"8","name":"RightColor","typeName":"color_hue","extraConfig":null,"bufferIndex":7},{"hwid":"drivetrain","name":"dt","typeName":"drivetrain","extraConfig":{"leftMotorHwId":"1","rightMotorHwId":"2","wheelTravel":200,"trackWidth":176},"bufferIndex":8},{"hwid":"lcd","name":"lcd","typeName":"lcd","extraConfig":null,"bufferIndex":9},{"hwid":"sound","name":"sound","typeName":"sound","extraConfig":null,"bufferIndex":10},{"hwid":"btn_chk","name":"button_check","typeName":"face_button","extraConfig":null,"bufferIndex":11},{"hwid":"btn_up","name":"button_up","typeName":"face_button","extraConfig":null,"bufferIndex":12},{"hwid":"btn_down","name":"button_down","typeName":"face_button","extraConfig":null,"bufferIndex":13}]}"""
import sys
import vexiq
import drivetrain
import math

class Robot():
    x = 0
    y = 0
    visitingHerdPoint = True
    carryingCone = False
    coneToCollect = False
    distanceTravelled = 0
    robotRadius = 15
    wheelCircumference = 20
    distanceSensorSpacing = 5
    def __init__(self,leftDrive,rightDrive,gyro1,gyro2,dt,colorLeft,colorRight,distanceMiddle): #object instantiation
        self.leftDrive = leftDrive
        self.rightDrive = rightDrive
        self.gyro1 = gyro1
        self.gyro2 = gyro2
        self.drivetrain = dt
        self.colorRight = colorRight
        self.colorLeft = colorLeft
        self.distanceMiddle = distanceMiddle

    def intify(self,number): #used to convert from float to int
        num = round(number)
        iterate = 0
        while iterate != num:
            iterate+=1
        return iterate

    def returnToHerdPoint(self,herdpoint):
        return None

    def moveToXYR(self,x,y,r):
        return None

    def collectCone(self):
        return None

    def moveBy(self,distance):

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

    def resolveXY(self,x,y,distance,rotation):
        return None

    def checkCone(self): # a simple test function to check to see if there is anything in front of the robot

        if self.distanceMiddle.distance() < 30:
            return True
        else:
            return False

#region config
LeftDrive   = vexiq.Motor(1)
RightDrive  = vexiq.Motor(2, True) # Reverse Polarity
TouchLed    = vexiq.TouchLed(3)
Gyro1       = vexiq.Gyro(4)
Gyro2       = vexiq.Gyro(5)
MiddleUltra = vexiq.DistanceSensor(6, vexiq.UNIT_CM)
LeftColor   = vexiq.ColorSensor(7) # hue
RightColor  = vexiq.ColorSensor(8) # hue

import drivetrain
dt          = drivetrain.Drivetrain(LeftDrive, RightDrive, 200, 176)
#endregion config

robot = Robot(LeftDrive,RightDrive,Gyro1,Gyro2,dt,LeftColor,RightColor,MiddleUltra)

while True:
    TouchLed.named_color(9)
    if TouchLed.is_touch():

        TouchLed.named_color(3) #orange

        result = robot.moveBy(30)

        if result == True:
            TouchLed.named_color(7) #green
        else:
            TouchLed.named_color(1) #red
        sys.sleep(1.5)

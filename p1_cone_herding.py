"""__CONFIG__
{"version":20,"widgetInfos":[{"hwid":"1","name":"LeftDrive","typeName":"motor","extraConfig":null,"bufferIndex":0},{"hwid":"2","name":"RightDrive","typeName":"motor_rp","extraConfig":null,"bufferIndex":1},{"hwid":"3","name":"TouchLed","typeName":"touch_led","extraConfig":null,"bufferIndex":2},{"hwid":"4","name":"Gyro1","typeName":"gyro","extraConfig":null,"bufferIndex":3},{"hwid":"5","name":"Gyro2","typeName":"gyro","extraConfig":null,"bufferIndex":4},{"hwid":"6","name":"DistanceMiddle","typeName":"distance_cm","extraConfig":null,"bufferIndex":5},{"hwid":"7","name":"ColorRight","typeName":"color_hue","extraConfig":null,"bufferIndex":6},{"hwid":"8","name":"ColorLeft","typeName":"color_hue","extraConfig":null,"bufferIndex":7},{"hwid":"drivetrain","name":"drivetrain","typeName":"drivetrain","extraConfig":{"leftMotorHwId":"1","rightMotorHwId":"2","wheelTravel":200,"trackWidth":176},"bufferIndex":8},{"hwid":"lcd","name":"lcd","typeName":"lcd","extraConfig":null,"bufferIndex":9},{"hwid":"sound","name":"sound","typeName":"sound","extraConfig":null,"bufferIndex":10},{"hwid":"btn_chk","name":"button_check","typeName":"face_button","extraConfig":null,"bufferIndex":11},{"hwid":"btn_up","name":"button_up","typeName":"face_button","extraConfig":null,"bufferIndex":12},{"hwid":"btn_down","name":"button_down","typeName":"face_button","extraConfig":null,"bufferIndex":13}]}"""
import sys
import vexiq
import drivetrain

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
    def __init__(self,leftDrive,rightDrive,gyro1,gyro2,distance,drivetrain): #object instantiation
        self.leftDrive = leftDrive
        self.rightDrive = rightDrive
        self.gyro1 = gyro1
        self.gyro2 = gyro2
        self.ultrasonic = distance
        self.drivetrain = drivetrain
        return None

    def intify(self,number): #used to convert from float to int
        num = round(number)
        iterate = 0
        while iterate != num:
            iterate+=1
        return iterate

    def move(self): #test function
        self.leftDrive.run(100)
        self.rightDrive.run(100)
        sys.sleep(1)
        self.leftDrive.off()
        self.rightDrive.off()

    def returnToHerdPoint(self,herdpoint):
        return None

    def moveToXYR(self,x,y,r):
        return None

    def collectCone(self):
        return None

    def moveBy(self,distance):
        self.flag = False
        self.degrees = (distance / self.wheelCircumference) * 360
        self.distancePerIteration = round(self.wheelCircumference * (18/360))
        self.numberOfIterations = round(self.degrees / 18)

        for i in range(0,self.intify(self.numberOfIterations)):
            self.drivetrain.drive_until(30,10)
            vexiq.lcd_write(i)

        self.leftDrive.off()
        self.rightDrive.off()

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

#region config
LeftDrive      = vexiq.Motor(1)
RightDrive     = vexiq.Motor(2, True) # Reverse Polarity
TouchLed       = vexiq.TouchLed(3)
Gyro1          = vexiq.Gyro(4)
Gyro2          = vexiq.Gyro(5)
DistanceMiddle = vexiq.DistanceSensor(6, vexiq.UNIT_CM)
ColorRight     = vexiq.ColorSensor(7) # hue
ColorLeft      = vexiq.ColorSensor(8) # hue

drivetrain     = drivetrain.Drivetrain(LeftDrive, RightDrive, 200, 176)
#endregion config

robot = Robot(LeftDrive,RightDrive,Gyro1,Gyro2,DistanceMiddle)

"""while True:
    if TouchLed.is_touch():
        robot.moveBy(20)
    vexiq.lcd_write(robot.ultrasonic.distance())
    LeftDrive.run(30,360,True)
    RightDrive.run(30,360,True)"""

LeftDrive.run(30,360,True)
RightDrive.run(30,360,True)

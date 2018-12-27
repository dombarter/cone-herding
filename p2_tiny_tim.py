"""__CONFIG__
{"version":20,"widgetInfos":[{"hwid":"1","name":"LeftDrive","typeName":"motor","extraConfig":null,"bufferIndex":0},{"hwid":"2","name":"RightDrive","typeName":"motor_rp","extraConfig":null,"bufferIndex":1},{"hwid":"3","name":"ArmLeft","typeName":"motor","extraConfig":null,"bufferIndex":2},{"hwid":"4","name":"Claw","typeName":"motor","extraConfig":null,"bufferIndex":3},{"hwid":"5","name":"ArmRight","typeName":"motor_rp","extraConfig":null,"bufferIndex":4},{"hwid":"drivetrain","name":"dt","typeName":"drivetrain","extraConfig":{"leftMotorHwId":"1","rightMotorHwId":"2","wheelTravel":200,"trackWidth":176},"bufferIndex":5},{"hwid":"lcd","name":"lcd","typeName":"lcd","extraConfig":null,"bufferIndex":6},{"hwid":"sound","name":"sound","typeName":"sound","extraConfig":null,"bufferIndex":7},{"hwid":"btn_chk","name":"button_check","typeName":"face_button","extraConfig":null,"bufferIndex":8},{"hwid":"btn_up","name":"button_up","typeName":"face_button","extraConfig":null,"bufferIndex":9},{"hwid":"btn_down","name":"button_down","typeName":"face_button","extraConfig":null,"bufferIndex":10},{"hwid":"joystick","name":"joystick","typeName":"joystick","extraConfig":null,"bufferIndex":11},{"hwid":"AxisA:y","name":"axisA","typeName":"joystick_axis","extraConfig":null,"bufferIndex":12},{"hwid":"AxisB:x","name":"axisB","typeName":"joystick_axis","extraConfig":null,"bufferIndex":13},{"hwid":"AxisC:x","name":"axisC","typeName":"joystick_axis","extraConfig":null,"bufferIndex":14},{"hwid":"AxisD:y","name":"axisD","typeName":"joystick_axis","extraConfig":null,"bufferIndex":15},{"hwid":"Eu","name":"bEup","typeName":"joystick_button","extraConfig":null,"bufferIndex":16},{"hwid":"Ed","name":"bEdown","typeName":"joystick_button","extraConfig":null,"bufferIndex":17},{"hwid":"Fu","name":"bFup","typeName":"joystick_button","extraConfig":null,"bufferIndex":18},{"hwid":"Fd","name":"bFdown","typeName":"joystick_button","extraConfig":null,"bufferIndex":19},{"hwid":"Lu","name":"bLup","typeName":"joystick_button","extraConfig":null,"bufferIndex":20},{"hwid":"Ld","name":"bLdown","typeName":"joystick_button","extraConfig":null,"bufferIndex":21},{"hwid":"Ru","name":"bRup","typeName":"joystick_button","extraConfig":null,"bufferIndex":22},{"hwid":"Rd","name":"bRdown","typeName":"joystick_button","extraConfig":null,"bufferIndex":23}]}"""
# VEX IQ Python-Project
import sys
import vexiq

#region config
LeftDrive  = vexiq.Motor(1)
RightDrive = vexiq.Motor(2, True) # Reverse Polarity
ArmLeft    = vexiq.Motor(3)
Claw       = vexiq.Motor(4)
ArmRight   = vexiq.Motor(5, True) # Reverse Polarity

import drivetrain
dt         = drivetrain.Drivetrain(LeftDrive, RightDrive, 200, 176)
joystick   = vexiq.Joystick()
#endregion config

clawOpen = False

while True:
    if joystick.axisD() > 15 or joystick.axisD() < -15:
        dt.drive(joystick.axisD())
    elif joystick.axisC() > 15 or joystick.axisC() < -15:
        dt.turn(joystick.axisC() * -1)
    elif joystick.axisA() > 15:
        ArmLeft.run(-100)
        ArmRight.run(-100)
    elif joystick.axisA() < -15:
        ArmLeft.run(40)
        ArmRight.run(40)
    elif joystick.bFup():
        if clawOpen:
            clawOpen = False
            Claw.run(100)
            sys.sleep(0.3)
        else:
            clawOpen = True
            Claw.run(-100)
            sys.sleep(0.3)
    else:
        dt.hold()
        ArmLeft.hold()
        ArmRight.hold()
        Claw.hold()
import math

class turn():
    def __init__(self,x,y):
        self.x = x
        self.y = y
        return None

    def rotate(self,x,y):
        self.currentX = self.x
        self.currentY = self.y
        self.deltaX = math.fabs(x - self.currentX)
        self.deltaY = math.fabs(y - self.currentY)

        if self.deltaX == 0 and self.deltaY != 0:
            if self.currentY > y:
                print(180)
            else:
                print(0)
        elif self.deltaX != 0 and self.deltaY == 0:
            if self.currentX > x:
                print(-90)
            else:
                print(90)
        else:
            if x > self.currentX and y > self.currentY:
                self.rotation = math.degrees(math.atan(self.deltaX / self.deltaY))
                print(self.rotation)
            elif x > self.currentX and y < self.currentY:
                self.rotation = math.degrees(math.atan(self.deltaY / self.deltaX)) + 90
                print(self.rotation)
            elif x < self.currentX and y > self.currentY:
                self.rotation = math.degrees(math.atan(self.deltaX / self.deltaY)) * -1
                print(self.rotation)
            elif x < self.currentX and y < self.currentY:
                self.rotation = math.degrees(math.atan(self.deltaX / self.deltaY)) - 180
                print(self.rotation)

turner = turn(0,0)

turner.rotate(0,0)

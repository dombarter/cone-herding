class coords:
    def __init__(self,x,y):
        self.x , self.y = x , y

def createTriangle(n):
    ynum = []
    for y in range(0,n):
        xnum = []
        for x in range(0,n):
            xnum.append(0.5*((x**2) + (x) + (y**2) + (3*y) + (2*x*y)))
        ynum.append(xnum)
    return ynum

def getCoords(n):

    flag , iterate = False , 5
    while not flag:
        triangle = createTriangle(iterate)
        for yLine in triangle:
            for number in yLine:
                if number == n:
                    flag = True
                    return(coords(yLine.index(number),triangle.index(yLine)))
        iterate+=1

res = getCoords(31)
print(str(res.x) + "," + str(res.y))

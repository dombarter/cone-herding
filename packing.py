import math

class coords:
    def __init__(self,x,y):
        self.x = x
        self.y = y

def triNumbers(n):

    number = n

    estimate = round(math.sqrt((2*number) + 1) - 1)
    y = 0.5 * ( (2 * number) - (estimate ** 2) - (estimate) )
    x = 0.5 * ( (estimate ** 2) + (3 * estimate) - (2 * number) )

    return coords(x,y)

result = triNumbers(26)
print(str(result.x) + ", " + str(result.y))
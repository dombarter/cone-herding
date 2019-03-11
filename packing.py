import math

number = 6

estimate = round(math.sqrt((2*number) + 1) - 1)
y = 0.5 * ( (2 * number) - (estimate ** 2) - (estimate) )
x = 0.5 * ( (estimate ** 2) + (3 * estimate) - (2 * number) )

print(str(x) + ", " + str(y))
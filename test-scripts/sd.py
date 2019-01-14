import math

def standardDeviation(numbers):
    sd = 0
    mean = 0
    for x in numbers:
        mean = mean + x
    mean = mean / len(numbers)
    for y in numbers:
        sd = sd + ((y - mean) ** 2)
    sd = sd / len(numbers)
    sd = math.sqrt(sd)
    return round(sd)

numbers = [11,12,13]

def meanOfValues(numbers):
    mean = 0
    for x in numbers:
        mean = mean + x

    mean = mean / len(numbers)
    return mean

print(meanOfValues(numbers))
        
        

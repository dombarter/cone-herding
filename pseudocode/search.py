import math

allPaths = [] #create a new array to store all paths
coneWallDistance = 15 #the distance where they have reached the wall and cone cannot be in the way
walls = Walls(100,-100,-100,100) #create our walls of known dimensions

# walls class
class Walls:
    def __init__(self, top, bottom, left, right): #specify all four walls
        self.top = top
        self.bottom = bottom
        self.left = left
        self.right = right
        return True

# path class
class Path:
    def __init__(self, direction, x_line, y_upper, y_lower):

        self.direction = direction #sets all the variables needed
        self.xLine = x_line
        self.yUpper = y_upper
        self.yLower = y_lower
        self.completed = False
        self.xlastVisited = 0
        self.ylastVisited = 0
        self.simpleTraverse = False

        if self.direction == "up" and self.yUpper != None: #sets the goalY if they are able to be calculated
            self.simpleTraverse = True
            self.goalY = self.yUpper
        elif self.direction == "down" and self.yLower != None:
            self.simpleTraverse = True
            self.goalY = self.yLower

# traverses a path and deals with any cone problems
def traversePath(path,robot):

    if path.simpleTraverse == True: #simple traverse ----------------------------------------------

        while robot.y != path.goalY: #while still not at the goalY (determines path completion)

            # realign to the path -----------------------------------------------------------------

            while robot.x != path.xLine: #keep aligning to the path line

                if robot.x != path.xLine:
                    result = robot.movetoXYA(path.xLine,robot.y) #move back to the path
                    
                    if result == False:
                        result = robot.alignToCone()
                        if robot.carryingCone == True:

                            path.xLastPosition = robot.x #sets the xLastPosition and yLastPosiiton variables
                            path.yLastPosiiton = robot.y

                            robot.returnToHerdPoint() #take cone back
                            robot.carryingCone = False #set carrying cone to true 
                            robot.returnToPathPoint() #come back to path point

                        else:
                            robot.collectCone() #pickup the cone
                            robot.carryingCone = True #set carrying cone to true
            
            #move the robot to the goalY ----------------------------------------------------------

            result = robot.movetoXYA(path.xLine,path.goalY) #move to the goalY

            #the move to xya has failed -----------------------------------------------------------
            if result == False:
                if robot.y < path.goalY + coneWallDistance and robot.y > path.goalY - coneWallDistance: #near enough to the wall
                    break #complete the path
                else: #it is a cone (simple traverse knows where walls are)
                    result = robot.alignToCone() #align to the cone
                    if robot.carryingCone == True:

                        path.xLastPosition = robot.x #sets the xLastPosition and yLastPosiiton variables
                        path.yLastPosiiton = robot.y

                        robot.returnToHerdPoint(True) #take cone back
                        robot.carryingCone = False #set carrying cone to true 
                        robot.returnToPathPoint() #come back to path point

                    else:
                        robot.collectCone() #pickup the cone
                        robot.carryingCone = True #set carrying cone to true                        

        # set path as completed -------------------------------------------------------------------

        path.completed = True
        return True # path was completed

    else: # non-simple traverse -------------------------------------------------------------------
        return None # yet to be designed 
        # this is where the robot will need to keep checking for boundaries etc..

#creates a new path given the currently created and visited paths
def createNewPath():

    furthestLeft = 0
    furthestRight = 0
    
    if math.fabs(walls.top - robot.y) < math.fabs(walls.bottom - robot.y): #closer to the top
        nextDirection = "down"
    else: #closer to the bottom
        nextDirection = "up"

    for path in allPaths: #finds the most outerly paths
        if path.xLine > furthestRight:
            furthestRight = path.xLine
        elif path.xLine < furthestLeft:
            furthestLeft = path.xLine
    
    if math.fabs(furthestLeft - robot.x) < math.fabs(furthestRight - robot.x): #closer to the left point
        preferredPath = "left"
    else: #closer to the right point
        preferredPath = "right"

    pathMade = False
    leftAttempted = False
    rightAttempted = False
    while pathMade == False:

        if preferredPath == "left" and leftAttempted == False:
            newPathX = furthestLeft - robot.robotWidth
            if newPathX < walls.left: #outisde the wall
                leftAttempted = True
                preferredPath = "right"
            elif maths.fabs(newPathX - walls.left) < robot.robotWidth:
                newPathX = walls.left + (robot.robotWidth - 1) #adjust the wall x- coordinate 
                pathMade = True
            else:
                pathMade = True
        elif preferredPath == "right" and rightAttempted == False:

        else:
            #no preffered path
            #at this point we should be returning none or something like that
    
#herd all cones function (basically the main function)
def herdAllCones():

    allConesHerded = False #set all cones herded to false

    tempPath = Path("up",0,walls.top,walls.bottom) #traverse upwards
    traversePath(tempPath,robot) 
    tempPath = Path("down",0,walls.top,walls.bottom) #traverse downwards
    traversePath(tempPath,robot)
    
    initialPath = Path("down",0,walls.top,walls.bottom) #creates the intial path and sends it to the array
    initialPath.completed = True #set path to completed                         
    allPaths.append(intialPath) #append path to array

    while allConesHerded == False: #while there are still cones in the field

        currentPath = createNewPath() #creates a new path
        if currentPath == None: #if there are no new cones in the zone
            allConesHerded = True #finish the program
        else:
            traversePath(currentPath,robot) #traverse the path
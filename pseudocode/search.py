allPaths = [] #stores all paths
robotWidth = 20 #needs to be measured
carryingCone = False #shows whether the robot has a cone currently or not
coneWallDistance = 15 #the distance where they have reached the wall and cone cannot be in the way

class Wall:
    def __init__(self, top = None, bottom = None, left = None, right = None):
        self.top = top
        self.bottom = bottom
        self.left = left
        self.right = right
        return True

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

def traversePath(self,path,robot):

    if path.simpleTraverse == True: #simple traverse

        while robot.y != path.goalY: #while still not at the goalY (determines path completion)

            # realign to the path
            if robot.x != path.xLine:
                result = robot.movetoXYA(path.xLine,robot.y) #move back to the path
                #deal with cone somehow
            else:

                #move the robot to the goalY
                result = robot.movetoXYA(path.xLine,path.goalY) #move to the goalY

                #the move to xya has failed
                if result == False:
                    if robot.y < path.goalY + coneWallDistance and robot.y > path.goalY - coneWallDistance: #near enough to the wall
                        break
                    else: #it is a cone (simple traverse knows where walls are)
                        result = robot.alignToCone()
                        if robot.carryingCone == True:
                            path.recordCurrentLocation() #sets the xLastPosition and yLastPosiiton variables
                            robot.returnToHerdPoint() #take cone back
                            robot.carryingCone = False #set carrying cone to true 
                            robot.returnToPathPoint() #come back to path point
                        else:
                            robot.collectCone() #pickup the cone
                            robot.carryingCone = True #set carrying cone to true                        

        path.completed = True
        return True # path was completed

    else:
        return None # yet to be designed 


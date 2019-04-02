def traversePathSimple(path):

    while robot.y != path.goalY: #while still not at the goalY (determines path completion)

        # realign to the path -----------------------------------------------------------------

        while robot.x != path.xLine: #keep aligning to the path line

            result = robot.moveToXYA(path.xLine,robot.y) #move back to the path

            if result == False:
                if robot.y < path.goalY + coneWallDistance and robot.y > path.goalY - coneWallDistance: #near enough to the wall
                    break #complete the path
                else: #it is a cone (simple traverse knows where walls are)
                    result = robot.alignToCone() #align to the cone
                    if result != False:
                        if robot.carryingCone == True:

                            path.xlastVisited = robot.x #sets the xLastPosition and yLastPosiiton variables
                            path.ylastVisited = robot.y
                            path.alastVisited = robot.angle_()

                            robot.debug(False,0,"red")
                            robot.returnToHerdPoint(True) #take cone back
                            robot.carryingCone = False #set carrying cone to true
                            robot.debug(False,0,"red_orange")
                            robot.returnToPathPoint(path) #come back to path point

                        else:
                            robot.collectCone() #pickup the cone
                            robot.carryingCone = True #set carrying cone to true    
        
        #move the robot to the goalY ----------------------------------------------------------

        result = robot.moveToXYA(path.xLine,path.goalY) #move to the goalY

        #the move to xya has failed -----------------------------------------------------------

        if result == False:
            if robot.y < path.goalY + coneWallDistance and robot.y > path.goalY - coneWallDistance: #near enough to the wall
                break #complete the path
            else: #it is a cone (simple traverse knows where walls are)
                result = robot.alignToCone() #align to the cone
                if result != False:
                    if robot.carryingCone == True:

                        path.xlastVisited = robot.x #sets the xLastPosition and yLastPosiiton variables
                        path.ylastVisited = robot.y
                        path.alastVisited = robot.angle_()

                        robot.debug(False,0,"red")
                        robot.returnToHerdPoint(True) #take cone back
                        robot.carryingCone = False #set carrying cone to true
                        robot.debug(False,0,"red_orange")
                        robot.returnToPathPoint(path) #come back to path point

                    else:
                        robot.collectCone() #pickup the cone
                        robot.carryingCone = True #set carrying cone to true                        

        # set path as completed -------------------------------------------------------------------

    path.completed = True
    return path # path was completed



            

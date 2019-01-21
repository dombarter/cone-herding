    def alignToCone(self):
        #will return false if evidently there is no cone there
        if self.lookingAtCone() == False and self.distanceLeft.distance() >= 45 and self.distanceRight.distance() >= 45:
            return False
        else:
            if self.checkDistance(True) > 25: #if there is a cone there but not closer than 25cm
                self.deltaD = round(self.calculateUltraDistance() - 25)
                self.moveBy(self.deltaD,True) #move to 25cm away
            self.maxSwingAmount = 5 #set the number of times to swing left and right
            if self.distanceLeft.distance() > self.distanceRight.distance(): #set the intial direction of swing
                self.directionOfSwing = 1
            else:
                self.directionOfSwing = -1
            for swing in range(4, self.maxSwingAmount + 4): #swing left and right
                for turn in range(0,swing):
                    if self.lookingAtCone(): #check for cone
                        break
                    self.rotateBy(self.directionOfSwing * 6) #turn the robot left or right
                    sys.sleep(0.6)
                if self.lookingAtCone(): #check for cone
                    break
                self.directionOfSwing = self.directionOfSwing * -1 #change the direction of swing
            self.deltaD = round(self.calculateUltraDistance() - 20) #move the robot to be 20cm away
            self.moveBy(self.deltaD,True)
            sys.sleep(1)
            self.deltaD = round(self.calculateUltraDistance() - 20) #move the robot to be 20cm away
            self.moveBy(self.deltaD,True)
            return True #return successful alignment



            

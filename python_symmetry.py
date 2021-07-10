from controller import Robot, Camera, Motor, Accelerometer,  GPS, TouchSensor, Motor

    

def clamp (value, min, max):
    if min > max:
        return value
    elif value < min:
        return min
    elif value > min:
        if value > max:
            return max
        else:
            return value
        

class Walk():
    def __init__(self):
    
        
        self.robot = Robot()  # Initialize the Robot class to control the robot 
        
        
        self.minMotorPositions = []
        self.maxMotorPositions = []
   
        
        self.mTimeStep = int(self.robot.getBasicTimeStep()) # Get the current simulation time mTimeStep of each simulation step
        
        self.HeadLed = self.robot.getLED('HeadLed')   # Get head LED light
        self.EyeLed = self.robot.getLED('EyeLed')   # Get head LED light
        self.HeadLed.set(0xff0000)  # Turn on the head LED light and set a color
        self.EyeLed.set(0xa0a0ff)  # Light up the eye LED lights and set a color
        
      
        self.positionSensors = [] # Initialize the joint angle sensor
        self.mMotors = []
        self.positionSensorNames = ('ShoulderR', 'ShoulderL', 'ArmUpperR', 'ArmUpperL',
                                    'ArmLowerR', 'ArmLowerL', 'PelvYR', 'PelvYL',
                                    'PelvR', 'PelvL', 'LegUpperR', 'LegUpperL',
                                    'LegLowerR', 'LegLowerL', 'AnkleR', 'AnkleL',
                                    'FootR', 'FootL', 'Neck', 'Head')  # Initialize each sensor name

        # Acquire and activate each sensor, update the value with mTimeStep as the cycle
        for i in range(0, len(self.positionSensorNames)):
            print(i)
            self.positionSensors.append(self.robot.getPositionSensor(self.positionSensorNames[i] + 'S'))
            self.positionSensors[i].enable(self.mTimeStep)
            self.mMotors.append(self.robot.getMotor(self.positionSensorNames[i]))
            self.minMotorPositions.append(self.mMotors[i].getMinPosition())
            self.maxMotorPositions.append(self.mMotors[i].getMaxPosition())
            

        #print(self.mMotors[2])
        
    def myStep(self):
        ret = self.robot.step(self.mTimeStep)
        if ret == -1:
            exit(0)
    
    def wait(self, ms):
        startTime = self.robot.getTime()
        s = ms / 1000.0
        while s + startTime >= self.robot.getTime():
            self.myStep()

    def run(self):
        
        print("-------Symmetry example of ROBOTIS OP2-------")
        print("The right arm is free while the left one mimics it.")
        print("In order to move the left arm, add a force to the right arm:")
        print("keep ALT/OPTION pressed and select the right arm.")
        print("Now you just have to move the mouse without releasing it.")
        print("This example illustrate also the selfCollision which is activated by default")
        
        position = [0, 0, 0]
        
        #print(position)
        #print(self.mMotors[2])

        self.mMotors[0].setAvailableTorque(0.0)
        self.mMotors[2].setAvailableTorque(0.0)
        self.mMotors[4].setAvailableTorque(0.0)
        
        
        self.myStep() # Simulate a step size and refresh the sensor reading
        

        while True:
            
            # Get position of right arm of the robot
            # invert (symmetry) and bound the positions
            position[0] = clamp(-self.positionSensors[0].getValue(), self.minMotorPositions[1], self.maxMotorPositions[1])
            position[1] = clamp(-self.positionSensors[2].getValue(), self.minMotorPositions[3], self.maxMotorPositions[3])
            position[2] = clamp(-self.positionSensors[4].getValue(), self.minMotorPositions[5], self.maxMotorPositions[5])
            print(position)
        
            #Set position of left arm of the robot
            self.mMotors[1].setPosition(position[0])
            self.mMotors[3].setPosition(position[1])
            self.mMotors[5].setPosition(position[2])
                
            # self.mGaitManager.step(self.mTimeStep)  # The gait generator generates a step-length movement
            self.myStep() # Simulate a step size
            

if __name__ == '__main__':
    walk = Walk()  
    walk.run()  
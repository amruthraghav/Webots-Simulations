from controller import Robot, Camera, Motor, Accelerometer,  GPS, TouchSensor
import os
import sys



try:
    pythonVersion = 'python%d%d' % (sys.version_info[0], sys.version_info[1])
    print(pythonVersion)
    libraryPath = os.path.join(os.environ.get("WEBOTS_HOME"), 'projects', 'robots', 'robotis', 'darwin-op', 'libraries',
                               pythonVersion)
    libraryPath = libraryPath.replace('/', os.sep)
    print(libraryPath)
    sys.path.append(libraryPath)
    from managers import RobotisOp2GaitManager, RobotisOp2MotionManager, RobotisOp2VisionManager

except ImportError:
    sys.stderr.write("Warning: 'managers' module not found.\n")
    sys.exit(0)

    

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
        
        self.camera = self.robot.getCamera('Camera')
        self.camera.enable(self.mTimeStep)
        
      
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
            

        self.mMotionManager = RobotisOp2MotionManager(self.robot) 
        self.mGaitManager = RobotisOp2GaitManager(self.robot, "config.ini")
        self.mVisionManager = RobotisOp2VisionManager(self.robot, self.camera.getWidth(), self.camera.getHeight() , 355, 15, 60, 15, 0, 30)  
    
        
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
    
        horizontal = 0.0
        vertical = 0.0
        width = self.camera.getWidth()
        height = self.camera.getHeight()
        
        print("---------------Visual Tracking---------------")
        print("This example illustrates the possibilities of Vision Manager.")
        print("Move the red ball by dragging the mouse while keeping both shift key and mouse button pressed.")
        self.myStep() # Simulate a step size and refresh the sensor reading
        

        while True:
            x,y = 0.0,0.0
            
            ballInFieldOfView = mVisionManager.getBallCenter(x,y, self.camera.getImage())
            
            if ballInFieldOfView:
                  self.EyeLed.set(0x00FF00)
            else:
                  self.EyeLed.set(0xFF0000)
            
            if ballInFieldOfView:
            
                  dh = 0.1 * ((x / width) - 0.5)
                  horizontal -= dh
                  dv = 0.1 * ((y / height) - 0.5)
                  vertical -= dv
                  horizontal = clamp(horizontal, minMotorPositions[18], maxMotorPositions[18])
                  horizontal = clamp(horizontal, minMotorPositions[19], maxMotorPositions[19])
                  mMotors[18].setPosition(horizontal)
                  mMotors[19].setPosition(vertical)
            self.myStep() # Simulate a step size
            

if __name__ == '__main__':
    walk = Walk()  
    walk.run()  
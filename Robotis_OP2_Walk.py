from controller import Robot, Camera, Motor, Accelerometer,  GPS, TouchSensor
import os
#import Config
#import original
import sys

try:
    pythonVersion = 'python%d%d' % (sys.version_info[0], sys.version_info[1])
    libraryPath = os.path.join(os.environ.get("WEBOTS_HOME"), 'projects', 'robots', 'robotis', 'darwin-op', 'libraries',
                               pythonVersion)
    libraryPath = libraryPath.replace('/', os.sep)
    sys.path.append(libraryPath)
    from managers import RobotisOp2GaitManager, RobotisOp2MotionManager
except ImportError:
    sys.stderr.write("Warning: 'managers' module not found.\n")
    sys.exit(0)

class Walk():
    def __init__(self):
        self.robot = Robot()  # Initialize the Robot class to control the robot 
        self.mTimeStep = int(self.robot.getBasicTimeStep()) # Get the current simulation time mTimeStep of each simulation step
        self.HeadLed = self.robot.getLED('HeadLed')   # Get head LED light
        self.EyeLed = self.robot.getLED('EyeLed')   # Get head LED light
        self.HeadLed.set(0xff0000)  # Turn on the head LED light and set a color
        self.EyeLed.set(0xa0a0ff)  # Light up the eye LED lights and set a color
        self.mAccelerometer = self.robot.getAccelerometer('Accelerometer')  # Get the accelerometer
        self.mAccelerometer.enable(self.mTimeStep)# Activate the sensor and update the value with mTimeStep as the cycle

        self.fup = 0
        self.fdown = 0 # Define two class variables for later judging whether the robot has fallen down
        
        self.mGyro = self.robot.getGyro('Gyro')  # Get the gyroscope
        self.mGyro.enable(self.mTimeStep)  # Activate the gyroscope and update the value with mTimeStep as the cycle

        self.camera = self.robot.getCamera('Camera') # Get the Camera
        self.camera.enable(self.mTimeStep)# Activate the Camera and update the value with mTimeStep as the cycle
        
        self.accelerometer = self.robot.getDevice('Accelerometer') # Get the Accelerometer
        self.accelerometer.enable(self.mTimeStep)# Activate the Accelerometer and update the value with mTimeStep as the cycle
        
        self.gps = self.robot.getGPS('gps')# Get the GPS
        self.gps.enable(self.mTimeStep)# Activate the GPS and update the value with mTimeStep as the cycle
        
        
        self.positionSensors = [] # Initialize the joint angle sensor
        self.positionSensorNames = ('ShoulderR', 'ShoulderL', 'ArmUpperR', 'ArmUpperL',
                                    'ArmLowerR', 'ArmLowerL', 'PelvYR', 'PelvYL',
                                    'PelvR', 'PelvL', 'LegUpperR', 'LegUpperL',
                                    'LegLowerR', 'LegLowerL', 'AnkleR', 'AnkleL',
                                    'FootR', 'FootL', 'Neck', 'Head')  # Initialize each sensor name

        # Acquire and activate each sensor, update the value with mTimeStep as the cycle
        for i in range(0, len(self.positionSensorNames)):
            self.positionSensors.append(self.robot.getPositionSensor(self.positionSensorNames[i] + 'S'))
            self.positionSensors[i].enable(self.mTimeStep)
            

        self.mKeyboard = self.robot.getKeyboard()  # Initialize keyboard reading class
        self.mKeyboard.enable(self.mTimeStep)   # Read from the keyboard with mTimeStep as the cycle

        self.mMotionManager = RobotisOp2MotionManager(self.robot) # Initialize the robot action group controller
        self.mGaitManager = RobotisOp2GaitManager(self.robot, "config.ini") # Initialize the robot gait controller
        
        
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
        
        print("-------Walk example of ROBOTIS OP2-------")
        print("This example illustrates Gait Manager")
        print("Press the space bar to start/stop walking")
        print("Use the arrow keys to move the robot while walking")
        
        self.myStep() # Simulate a step size and refresh the sensor reading

        self.mMotionManager.playPage(9)  #Perform the 9th action of the action group,initialize the standing posture, and prepare to walk
        self.wait(200)  # Wait 200ms

        self.isWalking = False # The robot does not enter the walking state at the beginning

        initialgps = self.gps.getValues() # store the starting GPS Location of the Robot in X,Y,Z Axis
        print("Starting Location:",initialgps) # printing the statement 

       

        while True:
            
            self.mGaitManager.setXAmplitude(0.0)  # Advance to 0
            self.mGaitManager.setAAmplitude(0.0) # Turn to 0
            key = 0  # The initial keyboard reading defaults to 0
            key = self.mKeyboard.getKey() # Read input from keyboard
            if key == 32:  # If a space is read, change the walking state
                if (self.isWalking):  # If the current robot is walking, stop the robot
                    self.mGaitManager.stop()
                    self.isWalking = False
                    self.wait(200)
                else:  # If the robot is currently stopped, it will start walking
                    self.mGaitManager.start()
                    self.isWalking = True
                    self.wait(200)
            elif key == 315: # 
                self.mGaitManager.setXAmplitude(1.0)
                print("GPS:",self.gps.getValues())
                print("Total Distance traveled in Terms of X Coordinate in metres:",abs(self.gps.getValues()[0]-initialgps[0]))
                #print("GPS Y:",self.gps.getValues()[1]-initialgps[1])
                print("Total Distance traveled in Terms of Z Coordinate in metres:",abs(self.gps.getValues()[2]-initialgps[2]))
                print("Speed:",self.gps.getSpeed())
               
            elif key == 317:  
                self.mGaitManager.setXAmplitude(-1.0)
                print("GPS:",self.gps.getValues())
                print("Total Distance traveled in Terms of X Coordinate in metres :",abs(self.gps.getValues()[0]-initialgps[0]))
                #print("GPS Y:",self.gps.getValues()[1]-initialgps[1])
                print("Total Distance traveled in Terms of Z Coordinate in metres :",abs(self.gps.getValues()[2]-initialgps[2]))
                print("Speed:",self.gps.getSpeed())
                
            elif key == 316: 
                self.mGaitManager.setAAmplitude(-0.5)
                print("GPS:",self.gps.getValues())
                print("Total Distance traveled in Terms of X Coordinate in metres:",abs(self.gps.getValues()[0]-initialgps[0]))
                #print("GPS Y:",self.gps.getValues()[1]-initialgps[1])
                print("Total Distance traveled in Terms of Z Coordinate in metres:",abs(self.gps.getValues()[2]-initialgps[2]))
                print("Speed:",self.gps.getSpeed())
                
            elif key == 314: 
                self.mGaitManager.setAAmplitude(0.5)
                print("GPS:",self.gps.getValues())
                print("Total Distance traveled in Terms of X Coordinatei n metres:",abs(self.gps.getValues()[0]-initialgps[0]))
                #print("GPS Y:",self.gps.getValues()[1]-initialgps[1])
                print("Total Distance traveled in Terms of Z Coordinate in metres:",abs(self.gps.getValues()[2]-initialgps[2]))
                print("Speed:",self.gps.getSpeed())
                
            self.mGaitManager.step(self.mTimeStep)  # The gait generator generates a step-length movement
            self.myStep() # Simulate a step size

    
    def auto_run(self):
        self.myStep()  # Simulate a step to refresh the sensor reading 
        self.mMotionManager.playPage(9)  # Perform action group 9, initialize standing position, and prepare to walk  
        self.wait(200)  # Wait 200ms  
        self.isWalking = False  # Initially, the robot did not enter the walking state  
        initialgps = self.gps.getValues() # store the starting GPS Location of the Robot in X,Y,Z Axis
        print("Starting Location:",initialgps) # printing the statement 
        while True:
            
            self.mGaitManager.start()  # Gait generator enters walking state  
            self.mGaitManager.setXAmplitude(1.0)  # Set robot forward  
            self.mGaitManager.step(self.mTimeStep)  # Gait generator generates a step action  
            self.myStep()  # Simulate one step  
            print("GPS:",self.gps.getValues())
            print("Total Distance traveled in Terms of X Coordinate:",self.gps.getValues()[0]-initialgps[0])
            #print("GPS Y:",self.gps.getValues()[1]-initialgps[1])
            print("Total Distance traveled in Terms of Z Coordinate::",self.gps.getValues()[2]-initialgps[2])
            print("Speed:",self.gps.getSpeed())


if __name__ == '__main__':
    walk = Walk()  
    walk.run()  
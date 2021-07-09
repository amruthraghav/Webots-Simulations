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
        self.robot = Robot()  
        self.mTimeStep = int(self.robot.getBasicTimeStep()) 
        self.HeadLed = self.robot.getLED('HeadLed')  
        self.EyeLed = self.robot.getLED('EyeLed')  
        self.HeadLed.set(0xff0000)  
        self.EyeLed.set(0xa0a0ff)  
        self.mAccelerometer = self.robot.getAccelerometer('Accelerometer')  
        self.mAccelerometer.enable(self.mTimeStep)

        self.fup = 0
        self.fdown = 0
        
        self.mGyro = self.robot.getGyro('Gyro')  
        self.mGyro.enable(self.mTimeStep)  

        self.camera = self.robot.getCamera('Camera')
        self.camera.enable(self.mTimeStep)
        
        self.accelerometer = self.robot.getDevice('Accelerometer')
        self.accelerometer.enable(self.mTimeStep)
        
        self.gps = self.robot.getGPS('gps')
        self.gps.enable(self.mTimeStep)
        initialgps = self.gps.getValues()
        
        self.left_touch = self.robot.getTouchSensor('left_touch')
        self.left_touch.enable(self.mTimeStep)
        
        self.right_touch = self.robot.getTouchSensor('right_touch')
        self.right_touch.enable(self.mTimeStep)
        

        self.positionSensors = []
        
        self.positionSensorNames = ('ShoulderR', 'ShoulderL', 'ArmUpperR', 'ArmUpperL',
                                    'ArmLowerR', 'ArmLowerL', 'PelvYR', 'PelvYL',
                                    'PelvR', 'PelvL', 'LegUpperR', 'LegUpperL',
                                    'LegLowerR', 'LegLowerL', 'AnkleR', 'AnkleL',
                                    'FootR', 'FootL', 'Neck', 'Head')  

        
        for i in range(0, len(self.positionSensorNames)):
            self.positionSensors.append(self.robot.getPositionSensor(self.positionSensorNames[i] + 'S'))
            self.positionSensors[i].enable(self.mTimeStep)
            #self.device[self.positionSensorNames[i]] = self.robot.getMotor(self.positionSensorNames[i])

        self.mKeyboard = self.robot.getKeyboard()  
        self.mKeyboard.enable(self.mTimeStep)  

        self.mMotionManager = RobotisOp2MotionManager(self.robot) 
        self.mGaitManager = RobotisOp2GaitManager(self.robot, "config.ini") 
        
        
        
    
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
        
        self.myStep()

        self.mMotionManager.playPage(9)  
        self.wait(200)  

        self.isWalking = False
        initialgps = self.gps.getValues()
        print("Starting Location:",initialgps)

        print("Left Touch:", self.left_touch.getValue())

        while True:
            self.checkIfFallen()
            self.mGaitManager.setXAmplitude(0.0)  
            self.mGaitManager.setAAmplitude(0.0) 
            key = 0  
            key = self.mKeyboard.getKey() 
            if key == 32:  
                if (self.isWalking):  
                    self.mGaitManager.stop()
                    self.isWalking = False
                    self.wait(200)
                else:  
                    self.mGaitManager.start()
                    self.isWalking = True
                    self.wait(200)
            elif key == 315:
                self.mGaitManager.setXAmplitude(1.0)
                #print("Accelerometer:",self.accelerometer.getValues())
                print("GPS:",self.gps.getValues())
                print("Total Distance traveled in Terms of X Coordinate:",self.gps.getValues()[0]-initialgps[0])
                #print("GPS Y:",self.gps.getValues()[1]-initialgps[1])
                print("Total Distance traveled in Terms of Z Coordinate::",self.gps.getValues()[2]-initialgps[2])
                print("Speed:",self.gps.getSpeed())
                #print("Left Touch:", self.left_touch.getValue())
            elif key == 317:  
                self.mGaitManager.setXAmplitude(-1.0)
                print("Accelerometer:",self.accelerometer.getValues())
                print("GPS:",self.gps.getValues())
                print("Total Distance traveled in Terms of X Coordinate:",self.gps.getValues()[0]-initialgps[0])
                #print("GPS Y:",self.gps.getValues()[1]-initialgps[1])
                print("Total Distance traveled in Terms of Z Coordinate::",self.gps.getValues()[2]-initialgps[2])
                print("Speed:",self.gps.getSpeed())
                
            elif key == 316: 
                self.mGaitManager.setAAmplitude(-0.5)
                print("Accelerometer:",self.accelerometer.getValues())
                print("GPS:",self.gps.getValues())
                print("Total Distance traveled in Terms of X Coordinate:",self.gps.getValues()[0]-initialgps[0])
                #print("GPS Y:",self.gps.getValues()[1]-initialgps[1])
                print("Total Distance traveled in Terms of Z Coordinate::",self.gps.getValues()[2]-initialgps[2])
                print("Speed:",self.gps.getSpeed())
                
            elif key == 314: 
                self.mGaitManager.setAAmplitude(0.5) 
                print("Accelerometer:",self.accelerometer.getValues())
                print("GPS:",self.gps.getValues())
                print("Total Distance traveled in Terms of X Coordinate:",self.gps.getValues()[0]-initialgps[0])
                #print("GPS Y:",self.gps.getValues()[1]-initialgps[1])
                print("Total Distance traveled in Terms of Z Coordinate::",self.gps.getValues()[2]-initialgps[2])
                print("Speed:",self.gps.getSpeed())
                
            self.mGaitManager.step(self.mTimeStep)  
            self.myStep()
            

    def checkIfFallen(self):
        acc = self.mAccelerometer.getValues()
        acc_tolerance = 80.0
        acc_step = 100
        # print(acc)
        # count how many steps the accelerometer
        # says that the robot is down
        if (acc[1] < 512.0 - acc_tolerance):
            self.fup = self.fup + 1
        else:
            self.fup = 0

        if (acc[1] > 512.0 + acc_tolerance):
            self.fdown = self.fdown + 1
        else:
            self.fdown = 0
            
        if (acc[0] < 512.0 - acc_tolerance):
            self.fright = self.fright + 1
        else:
            self.fright = 0

        if (acc[0] > 512.0 + acc_tolerance):
            self.fleft = self.fleft + 1
        else:
            self.fleft = 0

        # the robot face is down
        if (self.fup > acc_step):
            print("1")
            self.mMotionManager.playPage(10)  # f_up
            self.mMotionManager.playPage(9)   # init position
            self.fup = 0
  
        # the back face is down
        elif (self.fdown > acc_step) :
            print("2")
            self.mMotionManager.playPage(11)  # b_up
            self.mMotionManager.playPage(9)   # init position
            self.fdown = 0
            
        # the back face is down
        elif (self.fright > acc_step) :
            print("3")
            self.mMotionManager.playPage(13)
            self.fright = 0
            
        # the back face is down
        elif (self.fleft > acc_step) :
            print("4")
            self.mMotionManager.playPage(12)
            self.fleft  = 0
        #self.mMotionManager.playPage(10) 
        #self.mMotionManager.playPage(11) 
        #self.mMotionManager.playPage(9)
    
    def auto_run(self):
        self.myStep()  # Simulate a step to refresh the sensor reading 
        self.mMotionManager.playPage(9)  # Perform action group 9, initialize standing position, and prepare to walk  
        self.wait(200)  # Wait 200ms  
        self.isWalking = False  # Initially, the robot did not enter the walking state  
        while True:
            self.checkIfFallen()  # Determine if fall or not
            self.mGaitManager.start()  # Gait generator enters walking state  
            self.mGaitManager.setXAmplitude(1.0)  # Set robot forward  
            self.mGaitManager.step(self.mTimeStep)  # Gait generator generates a step action  
            self.myStep()  # Simulate one step  


if __name__ == '__main__':
    walk = Walk()  
    walk.run()  
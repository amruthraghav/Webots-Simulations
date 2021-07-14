from controller import Robot, Camera, Motor, Accelerometer,  GPS, TouchSensor, Motor
import os
import sys
import math

try:
    pythonVersion = 'python%d%d' % (sys.version_info[0], sys.version_info[1])
    libraryPath = os.path.join(os.environ.get("WEBOTS_HOME"), 'projects', 'robots', 'robotis', 'darwin-op', 'libraries',
                               pythonVersion)
    libraryPath = libraryPath.replace('/', os.sep)
    sys.path.append(libraryPath)
    from managers import RobotisOp2GaitManager, RobotisOp2MotionManager , RobotisOp2VisionManager
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
        self.BackLedRed = self.robot.getLED('BackLedRed')   # Get head LED light
        self.BackLedGreen = self.robot.getLED('BackLedGreen')   # Get head LED light
        self.BackLedBlue = self.robot.getLED('BackLedBlue')   # Get head LED light


        self.HeadLed.set(0xff0000)  # Turn on the head LED light and set a color
        # self.EyeLed.set(0xa0a0ff)  # Light up the eye LED lights and set a color
        
        self.mGyro = self.robot.getGyro('Gyro')  # Get the gyroscope
        self.mGyro.enable(self.mTimeStep)  # Activate the gyroscope and update the value with mTimeStep as the cycle

        self.camera = self.robot.getCamera('Camera') # Get the Camera
        self.camera.enable(self.mTimeStep)# Activate the Camera and update the value with mTimeStep as the cycle
        
        self.accelerometer = self.robot.getDevice('Accelerometer') # Get the Accelerometer
        self.accelerometer.enable(self.mTimeStep)# Activate the Accelerometer and update the value with mTimeStep as the cycle

        self.speaker = self.robot.getSpeaker("Speaker")
        self.speaker.setLanguage("en-US")

        self.positionSensors = [] # Initialize the joint angle sensor
        self.mMotors = []
        self.positionSensorNames = ('ShoulderR', 'ShoulderL', 'ArmUpperR', 'ArmUpperL',
                                    'ArmLowerR', 'ArmLowerL', 'PelvYR', 'PelvYL',
                                    'PelvR', 'PelvL', 'LegUpperR', 'LegUpperL',
                                    'LegLowerR', 'LegLowerL', 'AnkleR', 'AnkleL',
                                    'FootR', 'FootL', 'Neck', 'Head')  # Initialize each sensor name

        # Acquire and activate each sensor, update the value with mTimeStep as the cycle
        for i in range(0, len(self.positionSensorNames)):
            
            self.positionSensors.append(self.robot.getPositionSensor(self.positionSensorNames[i] + 'S'))
            self.positionSensors[i].enable(self.mTimeStep)
            self.mMotors.append(self.robot.getMotor(self.positionSensorNames[i]))
            self.minMotorPositions.append(self.mMotors[i].getMinPosition())
            self.maxMotorPositions.append(self.mMotors[i].getMaxPosition())
            

        self.mMotionManager = RobotisOp2MotionManager(self.robot) 
        self.mGaitManager = RobotisOp2GaitManager(self.robot, "config.ini")
        self.mVisionManager = RobotisOp2VisionManager(self.robot, self.camera.getWidth(), self.camera.getHeight() , 28, 20, 50, 45, 0, 30)

    def myStep(self):
        ret = self.robot.step(self.mTimeStep)
        if ret == -1:
            exit(0)

    def wait(self, ms):
        startTime = self.robot.getTime()
        s = ms / 1000.0
        while s + startTime >= self.robot.getTime():
            self.myStep()
    
    # Ball detection based on the ball color using the Vision Manager
    # - return: indicates if the algorithm found the ball
    # - args: return the position of the ball [-1.0, 1.0]

    def getBallCenter(x, y):
        width = self.camera.getWidth()
        height = self.camera.getHeight()
        im = self.camera.getImage()
        
        find = mVisionManager.getBallCenter(x, y, im)

        if not find:
            x = 0.0
            y = 0.0
            return False
        else:
            x = 2.0 * x / width - 1.0
            y = 2.0 * y / height - 1.0
            return True
    
    def run(self):
        print("---------------Demo of ROBOTIS OP2---------------")
        print("This demo illustrates all the possibilities available for the ROBOTIS OP2.")
        print("This includes motion playback, walking algorithm and image processing.")
    
        self.speaker.speak("Hi, my name is ROBOTIS OP2. I can walk, use my camera to find the ball, and perform complex motion like "
                "kicking the ball for example.", 1.0)
        
        self.myStep()

        self.EyeLed.set(0x00FF00)  # set eye led to green

        self.mMotionManager.playPage(1) # init position
        self.mMotionManager.playPage(24)  # hello
        self.mMotionManager.playPage(9)  # walkready position
        self.wait(200)  

        # play the motion preparing the robot to walk
        self.mGaitManager.start()
        self.mGaitManager.step(mTimeStep)

        # main loop
        px = 0.0
        py = 0.0
        fup = 0
        fdown = 0
        acc_tolerance = 80.0
        acc_step = 20

        while True:
            x, y, neckPosition, headPosition = 0,0,0,0
            ballInFieldOfView = getBallCenter(x,y, self.camera.getImage())
            acc = self.accelerometer.getValues()

            if acc[1] < 512.0 - acc_tolerance : 
                fup+=1
            else:
                fup = 0

            if acc[1] > 512.0 + acc_tolerance:
                fdown+=1
            else:
                fdown = 0

            # the robot face is down
            if fup > acc_step : 
                self.mMotionManager.playPage(1) # init position
                self.mMotionManager.playPage(10)  # hello
                self.mMotionManager.playPage(9)  # walkready position
                fup = 0
            
            #  the robot face is up
            elif fdown > acc_step : 
                self.mMotionManager.playPage(1) # init position
                self.mMotionManager.playPage(11)  # hello
                self.mMotionManager.playPage(9)  # walkready position
                fdown = 0
            
            # if the ball is in the field of view,
            # go in the direction of the ball and kick it
            elif ballInFieldOfView:
                # set eye led to blue
                self.EyeLed.set(0x0000FF)
                

                # compute the direction of the head
                # the head move at maximum by 0.015 [rad] at each time step
                x = 0.015 * x + px
                y = 0.015 * y + py
                px = x
                py = y
                neckPosition = clamp(-x, minMotorPositions[18], maxMotorPositions[18])
                headPosition = clamp(-y, minMotorPositions[19], maxMotorPositions[19])


                # go forwards and turn according to the head rotation
                if y < 0.1 : # ball far away, go quickly
                    self.mGaitManager.setXAmplitude(1.0)
                else: # ball close, go slowly
                    self.mGaitManager.setXAmplitude(0.5)
                self.mGaitManager.setAAmplitude(neckPosition)
                self.mGaitManager.step(mTimeStep)
                

                # Move head
                self.mMotors[18].setPosition(neckPosition)
                self.mMotors[19].setPosition(headPosition)

                #if the ball is close enough
                #kick the ball with the right foot
                if (y > 0.35) : 
                    self.mGaitManager.stop()
                    self.wait(500)
                    #set eye led to green
                    self.EyeLED.set(0x00FF00)
                    if x < 0.0:
                        self.mMotionManager.playPage(13)  # left kick
                    else:
                        self.mMotionManager.playPage(12)  # right kick
                    self.mMotionManager.playPage(9)     # walkready position
                    self.mGaitManager.start()
                    px = 0.0
                    py = 0.0
                
                else:
                    self.EyeLED.set(0xFF0000) # change the eye led to red

                    # turn round 
                    self.mGaitManager.setXAmplitude(0.0)
                    self.mGaitManager.setAAmplitude(-0.3)
                    self.mGaitManager.step(mTimeStep)

                    headPosition = clamp(0.7 * math.sin(2.0 * self.robot.getTime()), minMotorPositions[19], maxMotorPositions[19]);
                    self.mMotors[19].setPosition(headPosition)

            self.myStep()
                



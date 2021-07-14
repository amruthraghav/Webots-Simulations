from controller import Robot, Camera, Motor, Accelerometer,  GPS, TouchSensor, Speaker
import os
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

        
        
        self.speaker = self.robot.getSpeaker("Speaker")
        self.speaker.setLanguage("en-US")
        

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
    
        print("Welcome to the Smart News Bot. Listen to what the bot will be explaining soon.")
    
        self.speaker.speak("Hi! My Name is Robotis OP2. Welcome to the AI Vision and Robotics Course ."
                       "Congrats, on successfully finishing the first 3 modules . "
                       " In this Module, we will build a Smart Robot that can talk and answer questions . Lets get started."
                       "Hello, Using the text-to-speech of the Speaker device, I can speak 6 different languages: "
                        "English with US or UK accent, German, Spanish, French and Italian. "
                        "Using tags I can modulate my speech, like for example change <prosody pitch=\"+16.8st\">the pitch of my voice</prosody>, "
                        "<prosody pitch=\"-15st\">and speak with a very low pitch</prosody>. "
                        "<prosody rate=\"0.5\">And I can change the speed</prosody><prosody rate=\"1.5\">at which I speak</prosody>. "
                        "I can also <prosody volume=\"20\">adjust the volume of my voice</prosody>. "
                        "Last but not least, I can imitate animals: <audio src=\"sounds/cow.wav\">Meuh</audio>",1)
        
        self.wait(200)
        
        self.myStep()
        self.mMotionManager.playPage(1)
        self.mMotionManager.playPage(24)  
        self.mMotionManager.playPage(9)
        self.mMotionManager.playPage(38)    
        self.wait(30000)  

        self.isWalking = False
        
    
        #self.wait(15000)
        self.speaker.speak("Here is the list of topics I can provide news on! "
        " One - Football."
        " Two - Tennis."
        " Three - Business."
        " Four - Space."
        " Five - Technology."
        " Six - CoronaVirus."
        " Seven - Science."
        " Choose either of the topics by pressing on the robot and choose a number",1)
        
        print("Choose one of the numbers from the list, to get news on that topic")
        print(" One - Football.")
        print(" Two - Tennis.")
        print(" Three - Business.")
        print(" Four - Space.")
        print(" Five - Technology.")
        print(" Six - CoronaVirus.")
        print(" Seven - Science.")
        
#c27d49833b254c62b3024ffbe9df8056
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
                
            elif key == 317:  
                self.mGaitManager.setXAmplitude(-1.0)
                
            elif key == 316: 
                self.mGaitManager.setAAmplitude(-0.5)
                
            elif key == 314: 
                self.mGaitManager.setAAmplitude(0.5)
            
            elif key == 49:        
                self.speaker.speak("Regarding the football news, Italy defeated England to win the Euros 2020."
                                    "Messi wins his first international cup for Argentina in the Copa America.",1)
            elif key == 50:        
                self.speaker.speak("Novak Djokovic beats Matteo Berrettini in Wimbeldon Final 2021 to join Roger Federer and Rafael Nadal as twenty-time Grand Slam champion"
                                    ,1)
            elif key == 51:        
                self.speaker.speak("All over in the western world, local business are rallying to fully reopen. However, "
                                    "Slow vaccination rates in Asian Countries are causing the economy to be stagnant",1)
            elif key == 52:        
                self.speaker.speak("Sir Richard Branson's Virgin Galactic Spaceship flies to the edge of space making him the first billionaire to leave the Earth's atmosphere. Meanwhile, "
                "Amazon's cofounder Jeff Bezos is gearing up to fly to space real soon with his Blue Origin's spacecraft",1)
            
            elif key == 53:        
                self.speaker.speak("Google CEO Sundar Pichai says Artificial Intelligence will change the world and defend's Google's record on tax, privacy and data ."
                "Semiconductors,  are in high demand making it hard for computer and car manufacturers to make chips for their products",1)
            
            elif key == 54:        
                self.speaker.speak("The new Delta Variant of Coronavirus is much more stronger than any other variant previously known."
                " Vaccine companies, are rallying to verify if their vaccines are efficeint for this strain and how to neutralise this strain",1)
            
            elif key == 55:        
                self.speaker.speak("An atomic clock that could revolutionize space travel just passed its first test. "
                "Scientist spot, an electron capture supernova for the first time.",1)    
           
            self.mGaitManager.step(self.mTimeStep)  
            self.myStep()
            

    def checkIfFallen(self):
        acc = self.mAccelerometer.getValues()
        acc_tolerance = 80.0
        acc_step = 100
        #print(acc)
        #count how many steps the accelerometer
        #says that the robot is down
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

        #the robot face is down
        if (self.fup > acc_step):
            print("1")
            self.mMotionManager.playPage(10)  # f_up
            self.mMotionManager.playPage(9)   # init position
            self.fup = 0
  
        #the back face is down
        elif (self.fdown > acc_step) :
            print("2")
            self.mMotionManager.playPage(11)  # b_up
            self.mMotionManager.playPage(9)   # init position
            self.fdown = 0
            
        #the back face is down
        elif (self.fright > acc_step) :
            print("3")
            self.mMotionManager.playPage(13)
            self.fright = 0
            
        #the back face is down
        elif (self.fleft > acc_step) :
            print("4")
            self.mMotionManager.playPage(12)
            self.fleft  = 0
        #self.mMotionManager.playPage(10) 
        #self.mMotionManager.playPage(11) 
        #self.mMotionManager.playPage(9)
    


if __name__ == '__main__':
    walk = Walk()  
    walk.run()  
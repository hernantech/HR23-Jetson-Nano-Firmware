class CANclass():
    def __init__(self):
        print("CAN class initialized")
        self.canbus = "can0"
    def CANsend(address, message):
        print("sends can out on can0") #can0 is hardcoded
    def CANreceive(message):
        print("receives can message on can0 and parses it")
        #may need to offload this to a parsing function in C or cpp
        #may also need to use buffer and have pyCAN write a set of frames to a buffer which calls/triggers this function to parse it
        print("received message", message.ID())

class Application(CANclass, profile):
    def __init__(self):
        #self.array = array
        self.profile = profile #int value which sets the profile on startup
        #if profile == 1:
            #set some shit to slow/limp mode
        #if profile == 2:
            #party mode
            #do not check faults, run raw pedal:torque mapping (1-1)
        self.testvalue = 2
    def tick():
        checkFaults()
        updateValues()
    def calcAccel():
        #pull data from buffer
        #check for value jumps between ticks
        print("calculating acceleration")
    def calcTorque(self):
        #sends torque out via CAN
        print("calculating accel -> torque")
    def updateValues():
        print("updating buffer values")
        #checks faults



        
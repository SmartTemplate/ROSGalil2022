import rospy
from ros_igtl_bridge.msg import igtltransform, igtlstring
from ros_galil_2022.srv import Status, Config

# State Machine
NONE = 0    # robot not connected yet
CONNECT = 1 # robot connected to 3DSlicer
INIT = 2    # initializing
ZFRAME = 3  # define zFrame transform
IDLE = 4    # waiting
TARGET = 5  # define target
MOVE = 6    # move to target

class Interface:
    def __init__(self):

        # ROS Topics
        rospy.Subscriber('IGTL_STRING_IN', igtlstring, self.callbackString)
        rospy.Subscriber('IGTL_TRANSFORM_IN', igtltransform, self.callbackTransformation)

        # Variables
        #TODO: Discuss definition of flags and states
        self.flagInit = False
        self.flagZFrame = False
        self.flagTarget = False
        self.state = NONE

        # Initialize the node and name it.
        rospy.init_node('interface')
        rospy.loginfo('Interface Node')

        # Set timer for state machine loop
        self.rate = rospy.Rate(10) #10hz

#################################################################################################
#####    Callback Functions for subscribed topics     ###########################################
#################################################################################################

    # Received STRING from 3DSlicer OpenIGTLink Bridge
    #TODO: Finish all cases (MOVE, SERIAL)
    def callbackString(self, msg):
        rospy.loginfo(rospy.get_caller_id() + 'Received command %s', msg.name)

        #Command to initialize robot (Homing)
        if msg.name == 'INIT':
            initCondition = msg.data[4] + msg.data[5]
            rospy.loginfo(initCondition)
            rospy.wait_for_service('init_motors')
            try:
                init_motors = rospy.ServiceProxy('init_motors', Config)
                if (init_motors(initCondition)):
                    rospy.loginfo("Homing successful")
                    self.flagInit = True    
                else:
                    rospy.loginfo("Could not initialize motors")
            except rospy.ServiceException as e:
                rospy.loginfo("Controller service call failed: %s"%e)
        else:
            rospy.loginfo('Invalid message, returning to IDLE state')

    # Received TRANSFORM from 3DSlicer OpenIGTLink Bridge
    #TODO: Implement all cases (zFrameTransformation, targetTransformation and angleTransformation)
    def callbackTransformation(self, msg):
        rospy.loginfo(rospy.get_caller_id() + 'Received transformation ' + msg.name)

#TODO: Implement missing machine states
def main():
    try:
        #Instantiate object of node class
        interface = Interface()
    except rospy.ROSInterruptException: 
        rospy.loginfo('Could not initialize Interface Node')

    while not rospy.is_shutdown():
        #NONE State - Wait for OpenIGTLink Connection
        if (interface.state == NONE):
            try:
                rospy.wait_for_service('get_controller_connection_status', timeout=2) #2s
                #Service response
                if (rospy.ServiceProxy('get_controller_connection_status', Status) == True):
                    rospy.loginfo("Controller serial connection: ON")
                    interface.state = CONNECT
                else:
                    rospy.loginfo("Controller serial connection: OFF")
            except:
                rospy.loginfo("Controller node not responding")
        #CONNECT State - Do nothing
        elif (interface.state == CONNECT):
            rospy.loginfo('Waiting for robot homing')
        #Do nothing
        else:
            pass
        interface.rate.sleep()

if __name__ == '__main__':
    main()
import rospy

class Planner:

    def __init__(self):
        # ROS Topics:

        # ROS Services:

        # Init node
        rospy.init_node('planner', anonymous=True)

def main():
    try:
        #Initialize controller
        planner = Planner()
        rospy.loginfo('Planner Node\n')
    except rospy.ROSInterruptException: pass

    #spin() simply keeps python from exiting until this node is stopped
    rospy.spin()    


if __name__ == '__main__':
    main()
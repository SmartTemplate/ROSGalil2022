import rospy

class Planner:

    def __init__(self):
        # Init node
        rospy.init_node('planner', anonymous=True)
        rospy.loginfo('Planner Node')

        # ROS Topics:

        # ROS Services:


def main():
    try:
        #Initialize controller
        planner = Planner()
    except rospy.ROSInterruptException: 
        rospy.loginfo('Could not initialize Planner Node')

    #spin() simply keeps python from exiting until this node is stopped
    rospy.spin()    


if __name__ == '__main__':
    main()
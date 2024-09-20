import rospy
from geometry_msgs.msg import PoseStamped

def publisher():
    rospy.init_node('goal_publisher')
    pub = rospy.Publisher('multi_floor_planner/goal', PoseStamped, queue_size=10)
    rate = rospy.Rate(0.5)  
    while not rospy.is_shutdown():
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = -12.41 # 1.0*0.05
        pose.pose.position.y = 11.537 # 8.0*0.05
        pose.pose.position.z = 2.0 #1.0
        pose.pose.orientation.w = 1.0
        pub.publish(pose)
        rate.sleep() 

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

import rospy
from geometry_msgs.msg import PoseStamped

def publisher():
    rospy.init_node('pose_publisher')
    pub = rospy.Publisher('multi_floor_planner/initial_pose', PoseStamped, queue_size=10)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = 0 #-1.850 #0.0 
        pose.pose.position.y = 0 #1.500# 0.0
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

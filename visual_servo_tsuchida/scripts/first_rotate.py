import rospy
from geometry_msgs.msg import TwistStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

def twist_rotate(pub):
    sokudo = TwistStamped()
    sokudo.header.stamp = rospy.Time.now()
    sokudo.twist.linear.x = 0.0
    sokudo.twist.linear.y = 0
    sokudo.twist.linear.z = -0.1
    sokudo.twist.angular.x = 0
    sokudo.twist.angular.y = 0
    sokudo.twist.angular.z = 0
    pub.publish(sokudo)

if __name__ == '__main__':
    rospy.init_node('twist_init', anonymous=True)
    pub = rospy.Publisher('/jog_server/delta_jog_cmds', TwistStamped, queue_size=100)
    init_pub = rospy.Publisher('arm_controller/command', JointTrajectory, queue_size=10)
    init_msg = JointTrajectory()
    init_msg.header.stamp = rospy.Time.now()
    init_msg.joint_names = [ "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint" ]
    init_msg.points = [JointTrajectoryPoint() for i in range(1)]
    init_msg.points[0].positions = [-math.pi/2, -2.8 * math.pi/6, -math.pi/3, -1*math.pi/1.5, math.pi/2, -math.pi/2]
    init_msg.points[0].time_from_start = rospy.Time(1.0)
    init_pub.publish(init_msg)
    rospy.sleep(3)
    loop = rospy.Rate(5)
    while not rospy.is_shutdown():
        twist_rotate(pub)
        loop.sleep()
    
    

    
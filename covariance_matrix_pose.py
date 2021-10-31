import rospy
import std_msgs.msg
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

def new_pose(pose_position):

    # Published filtered pose
    pose_pub = rospy.Publisher('processed_pose', PoseWithCovarianceStamped, queue_size=10)
    rospy.sleep(1.)

    poseCovariance = PoseWithCovarianceStamped()
    # Header
    poseCovariance.header = std_msgs.msg.Header()
    poseCovariance.header.stamp = rospy.Time.now()
    poseCovariance.header.frame_id = 'map'
    # Create the Pose with Covariance
    poseCovariance.pose.pose.position.x = pose_position.pose.position.x  
    poseCovariance.pose.pose.position.y = pose_position.pose.position.y 
    poseCovariance.pose.pose.position.z = pose_position.pose.position.z 

    poseCovariance.pose.pose.orientation.x = pose_position.pose.orientation.x 
    poseCovariance.pose.pose.orientation.y = pose_position.pose.orientation.y 
    poseCovariance.pose.pose.orientation.z = pose_position.pose.orientation.z 
    poseCovariance.pose.pose.orientation.w = pose_position.pose.orientation.w

    poseCovariance.pose.covariance =     [0.0,0.0,0.0,0.0,0.0,0.0,
                                          0.0,0.0,0.0,0.0,0.0,0.0,
                                          0.0,0.0,0.0,0.0,0.0,0.0,
                                          0.0,0.0,0.0,0.0,0.0,0.0,
                                          0.0,0.0,0.0,0.0,0.0,0.0,
                                          0.0,0.0,0.0,0.0,0.0,0.0]
    # Publish
    rospy.sleep(0.01)
    while not rospy.is_shutdown():
        pose_pub.publish(poseCovariance)
        break

def pose_subscriber():

    pose_sub = rospy.Subscriber('/orb_slam2_mono/pose', PoseStamped, new_pose)
    rospy.spin()

def setup():
    print("Posting Pose with Covariance")
    rospy.init_node('pose_process', anonymous=True)

if __name__ == '__main__':
    setup()
    try:
        pose_subscriber()

    except rospy.ROSInterruptException:
        pass

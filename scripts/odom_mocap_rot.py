#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

class GlobalToMocapProvider:
    def __init__(self):
        rospy.init_node('global_to_mocap_node')

        # 1. Setup TF Buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # 2. Subs and Pubs
        # We listen to the Mocap Odometry
        self.sub = rospy.Subscriber('/natnet_ros/kingfisher/odom', Odometry, self.callback)
        # We publish the Mocap's position expressed in the OpenVINS 'global' frame
        self.pub = rospy.Publisher('/natnet_ros/mocap_in_global', Odometry, queue_size=10)

        rospy.loginfo("Computing Global -> Mocap Pose...")

    def callback(self, msg):
        try:
            # Look up the transform from Global to World
            # This uses the static world->global transform you just created
            # but reverses it (target: global, source: world)
            transform = self.tf_buffer.lookup_transform(
                "global", 
                "world", 
                rospy.Time(0), 
                rospy.Duration(0.1)
            )

            # --- TRANSFORM THE MOCAP POSE ---
            # We take the Mocap pose (which is in 'world') and move it to 'global'
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose = msg.pose.pose
            
            ps_transformed = tf2_geometry_msgs.do_transform_pose(ps, transform)

            # --- CONSTRUCT NEW MESSAGE ---
            out_msg = Odometry()
            out_msg.header = msg.header
            out_msg.header.frame_id = "global"
            out_msg.child_frame_id = "mocap_pose_in_global"

            out_msg.pose.pose = ps_transformed.pose
            
            # Print RPY of the Mocap relative to the Global origin
            q = ps_transformed.pose.orientation
            (r, p, y) = euler_from_quaternion([q.x, q.y, q.z, q.w])
            rospy.loginfo_throttle(1, "Mocap in Global - Pos: [%.2f, %.2f], Yaw: %.2f deg", 
                                   out_msg.pose.pose.position.x, 
                                   out_msg.pose.pose.position.y, 
                                   y * 57.2958)

            self.pub.publish(out_msg)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5, "Waiting for world->global transform: %s" % str(e))

if __name__ == '__main__':
    GlobalToMocapProvider()
    rospy.spin()

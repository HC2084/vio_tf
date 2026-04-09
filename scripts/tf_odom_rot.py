#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Vector3Stamped

class OdomFrameTransformer:
    def __init__(self):
        rospy.init_node('odom_frame_transformer')

        # 1. Setup TF Buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # 2. Subs and Pubs
        # Input: OpenVINS providing IMU pose in Global frame
        self.sub = rospy.Subscriber('/ov_msckf/odomimu', Odometry, self.callback)
        # Output: Robot Base pose in the Map (Mocap) frame
        self.pub = rospy.Publisher('/ov_msckf/base_link_corrected', Odometry, queue_size=10)
        
        rospy.loginfo("Transformer initialized. Mapping [world -> global -> imu -> base_link]")

    def callback(self, msg):
        try:
            # IMPORTANT: Look up the chain from the Ground Truth origin (map) 
            # all the way to the robot center (base_link)
            # This handles both the global yaw shift AND the D435i axis swap.
            transform = self.tf_buffer.lookup_transform(
                "global",                # Target: Where we want the pose (Ground Truth World)
                "base_link",          # Source: The frame we want to track (Robot Center)
                rospy.Time(0),     # Synchronize with the Odom timestamp
                rospy.Duration(0.1)
            )

            # --- TRANSFORM POSE ---
            # We treat the incoming message as a reference. 
            # Note: We use an identity pose because 'transform' already contains 
            # the full Map -> Base_link state.
            ps = PoseStamped()
            ps.header.frame_id = "base_link"
            ps.pose.orientation.w = 1.0 # Identity
            
            ps_transformed = tf2_geometry_msgs.do_transform_pose(ps, transform)

            # --- TRANSFORM VELOCITIES ---
            # Twist is usually expressed in the child_frame (base_link).
            # We just need to rotate the vectors from the IMU frame to the Base_link frame.
            # We look up the local extrinsic only for velocity.
            local_extrinsic = self.tf_buffer.lookup_transform("base_link", "imu", rospy.Time(0))

            v_lin = Vector3Stamped()
            v_lin.vector = msg.twist.twist.linear
            v_lin_transformed = tf2_geometry_msgs.do_transform_vector3(v_lin, local_extrinsic)

            v_ang = Vector3Stamped()
            v_ang.vector = msg.twist.twist.angular
            v_ang_transformed = tf2_geometry_msgs.do_transform_vector3(v_ang, local_extrinsic)

            # --- CONSTRUCT NEW MESSAGE ---
            out_msg = Odometry()
            out_msg.header.stamp = msg.header.stamp
            out_msg.header.frame_id = "global"
            out_msg.child_frame_id = "base_link"

            out_msg.pose.pose = ps_transformed.pose
            out_msg.twist.twist.linear = v_lin_transformed.vector
            out_msg.twist.twist.angular = v_ang_transformed.vector

            self.pub.publish(out_msg)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5, "Waiting for TF chain [map -> global -> imu -> base_link]: %s" % str(e))

if __name__ == '__main__':
    OdomFrameTransformer()
    rospy.spin()

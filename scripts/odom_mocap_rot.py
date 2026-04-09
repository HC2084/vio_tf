#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from tf.transformations import quaternion_matrix, inverse_matrix

class GlobalToMocapProvider:
    def __init__(self):
        rospy.init_node('global_to_mocap_node')

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.sub = rospy.Subscriber('/natnet_ros/kingfisher/odom', Odometry, self.callback)
        self.pub = rospy.Publisher('/natnet_ros/mocap_in_global', Odometry, queue_size=10)

    def callback(self, msg):
        try:
            # 1. Get the map-to-map alignment
            # This 'transform' contains the EXACT rotation between Mocap and VIO maps
            transform = self.tf_buffer.lookup_transform(
                "global", "world", rospy.Time(0), rospy.Duration(0.1))

            # 2. Transform the Pose
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose = msg.pose.pose
            ps_transformed = tf2_geometry_msgs.do_transform_pose(ps, transform)

            # 3. ROTATION LOGIC
            # R1: Room -> VIO Map
            q_map = [transform.transform.rotation.x, transform.transform.rotation.y, 
                     transform.transform.rotation.z, transform.transform.rotation.w]
            R_world_to_global = quaternion_matrix(q_map)

            # R2: VIO Map -> Drone Body
            q_body = [ps_transformed.pose.orientation.x, 
                      ps_transformed.pose.orientation.y, 
                      ps_transformed.pose.orientation.z, 
                      ps_transformed.pose.orientation.w]
            R_global_to_body = inverse_matrix(quaternion_matrix(q_body))

            # 4. Apply both rotations to the velocity vector
            v_world = np.array([msg.twist.twist.linear.x, 
                                msg.twist.twist.linear.y, 
                                msg.twist.twist.linear.z, 0])
            
            # This step converts 'Room Velocity' to 'Body Velocity' 
            # while accounting for the map rotation
            # v_global = np.dot(R_world_to_global, v_world)
            v_body = np.dot(R_global_to_body, v_world)

            # 5. Construct Message
            out_msg = Odometry()
            out_msg.header.stamp = msg.header.stamp
            out_msg.header.frame_id = "global"
            out_msg.child_frame_id = "mocap_body"
            
            out_msg.pose.pose = ps_transformed.pose
            out_msg.twist.twist.linear.x = v_body[0]
            out_msg.twist.twist.linear.y = v_body[1]
            out_msg.twist.twist.linear.z = v_body[2]

            self.pub.publish(out_msg)

        except Exception as e:
            rospy.logwarn_throttle(5, str(e))

if __name__ == '__main__':
    GlobalToMocapProvider()
    rospy.spin()
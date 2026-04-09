#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_matrix

class GlobalToMocapProvider:
    def __init__(self):
        rospy.init_node('global_to_mocap_node')

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.output_child_frame = rospy.get_param('~output_child_frame', 'kingfisher')

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
            # Known facts:
            # - incoming twist.linear is expressed in "world"
            # - tf global<-world includes the fixed ~90 deg yaw map alignment
            q_map = [transform.transform.rotation.x, transform.transform.rotation.y, 
                     transform.transform.rotation.z, transform.transform.rotation.w]
            R_world_to_global = quaternion_matrix(q_map)[:3, :3]

            q_source_in_global = [ps_transformed.pose.orientation.x,
                                  ps_transformed.pose.orientation.y,
                                  ps_transformed.pose.orientation.z,
                                  ps_transformed.pose.orientation.w]
            R_source_to_global = quaternion_matrix(q_source_in_global)[:3, :3]
            R_global_to_source = R_source_to_global.T

            # 4. Transform twist world -> global -> source body (same point)
            v_world = np.array([msg.twist.twist.linear.x, 
                                msg.twist.twist.linear.y, 
                                msg.twist.twist.linear.z])
            w_world = np.array([msg.twist.twist.angular.x,
                                msg.twist.twist.angular.y,
                                msg.twist.twist.angular.z])
            v_global = np.dot(R_world_to_global, v_world)
            w_global = np.dot(R_world_to_global, w_world)

            v_source = np.dot(R_global_to_source, v_global)
            w_source = np.dot(R_global_to_source, w_global)

            # 5. Optional source-child -> output-child adjoint transform.
            # For transform target<-source with (R, p):
            #   w_t = R w_s
            #   v_t = R v_s + p x (R w_s)
            source_child = "world"
            target_child = self.output_child_frame
            v_out = v_source
            w_out = w_source

            if target_child != source_child:
                try:
                    t_cs = self.tf_buffer.lookup_transform(
                        target_child, source_child, rospy.Time(0), rospy.Duration(0.05))
                    q_cs = [t_cs.transform.rotation.x,
                            t_cs.transform.rotation.y,
                            t_cs.transform.rotation.z,
                            t_cs.transform.rotation.w]
                    R_cs = quaternion_matrix(q_cs)[:3, :3]
                    p_cs = np.array([t_cs.transform.translation.x,
                                     t_cs.transform.translation.y,
                                     t_cs.transform.translation.z])
                    w_out = np.dot(R_cs, w_source)
                    v_out = np.dot(R_cs, v_source) + np.cross(p_cs, w_out)
                except Exception as tf_err:
                    rospy.logwarn_throttle(
                        5, "Twist child transform %s<- %s unavailable, using source frame twist: %s",
                        target_child, source_child, str(tf_err))
                    target_child = source_child

            # 6. Construct Message
            out_msg = Odometry()
            out_msg.header.stamp = msg.header.stamp
            out_msg.header.frame_id = "global"
            out_msg.child_frame_id = target_child
            
            out_msg.pose.pose = ps_transformed.pose
            out_msg.twist.twist.linear.x = v_out[0]
            out_msg.twist.twist.linear.y = v_out[1]
            out_msg.twist.twist.linear.z = v_out[2]
            out_msg.twist.twist.angular.x = w_out[0]
            out_msg.twist.twist.angular.y = w_out[1]
            out_msg.twist.twist.angular.z = w_out[2]

            self.pub.publish(out_msg)

        except Exception as e:
            rospy.logwarn_throttle(5, str(e))

if __name__ == '__main__':
    GlobalToMocapProvider()
    rospy.spin()
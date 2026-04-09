#!/usr/bin/env python3
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion

class WorldToGlobalPublisher:
    def __init__(self):
        rospy.init_node('world_to_global_publisher')

        # 1. TF Broadcaster
        self.static_br = tf2_ros.StaticTransformBroadcaster()
        
        # 2. State
        self.alignment_published = False
        
        # 3. Frames
        self.world_frame = "world"
        self.global_frame = "global"
        
        # 4. Subscriber to Mocap
        self.sub = rospy.Subscriber('/natnet_ros/kingfisher/odom', Odometry, self.callback)

        rospy.loginfo("Waiting for first Mocap message to anchor 'global' to 'world'...")

    def callback(self, msg):
        if self.alignment_published:
            return

        # 1. Extract Orientation for RPY conversion
        q = msg.pose.pose.orientation
        quat_list = [q.x, q.y, q.z, q.w]
        
        # Convert to Euler (Roll, Pitch, Yaw) in Radians
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)

        # 2. Create the TransformStamped
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.global_frame

        # Position
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Rotation (Quaternion)
        t.transform.rotation = q

        # 3. Broadcast as a static transform
        self.static_br.sendTransform(t)
        
        # 4. Logging output
        rospy.loginfo("--- World -> Global Alignment Published ---")
        rospy.loginfo("Translation: x=%.3f, y=%.3f, z=%.3f", 
                      t.transform.translation.x, 
                      t.transform.translation.y, 
                      t.transform.translation.z)
        
        # Print RPY in both Radians and Degrees for convenience
        rospy.loginfo("Rotation (rad): R=%.3f, P=%.3f, Y=%.3f", roll, pitch, yaw)
        rospy.loginfo("Rotation (deg): R=%.1f, P=%.1f, Y=%.1f", 
                      roll * 57.2958, pitch * 57.2958, yaw * 57.2958)
        
        self.alignment_published = True

if __name__ == '__main__':
    WorldToGlobalPublisher()
    rospy.spin()

#!/usr/bin/env python3
"""
Static TF: world -> global.

Default: Kabsch (Umeyama rigid) alignment from time-synchronized VIO + MoCap positions,
collected over a short window (default 0.5 s) so startup calibration can average more motion.

Run-to-run repeatability: use ~calibration_time_source:=message (default) so the window follows
sensor/header stamps (bags + /use_sim_time are consistent). Approximate sync still pairs
different messages if slop is large — tighten ~sync_slop if results drift.

Fallback: first MoCap odom only (legacy), matching global to that pose in world.
"""
import rospy
import numpy as np
import tf2_ros
import message_filters
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_matrix

RAD2DEG = 57.29577951308232


def _pair_stamp_nsec(vio_msg, mocap_msg):
    """Average of both header stamps in nanoseconds, or None if either stamp is unset."""
    tv = vio_msg.header.stamp
    tm = mocap_msg.header.stamp
    if tv.is_zero() or tm.is_zero():
        return None
    return (tv.to_nsec() + tm.to_nsec()) // 2


def _quat_msg_to_list(q):
    return [q.x, q.y, q.z, q.w]


def _rpy_report(prefix, quat_list, header_frame, child_frame):
    """Return formatted lines for logging orientation (rad + deg + quaternion)."""
    r, p, yaw = euler_from_quaternion(quat_list)
    return (
        "%s  frame_id=%s  child_frame_id=%s\n"
        "    RPY (rad): roll=%.5f  pitch=%.5f  yaw=%.5f\n"
        "    RPY (deg): roll=%.2f  pitch=%.2f  yaw=%.2f\n"
        "    quat xyzw: %.5f %.5f %.5f %.5f"
        % (
            prefix,
            header_frame,
            child_frame,
            r,
            p,
            yaw,
            r * RAD2DEG,
            p * RAD2DEG,
            yaw * RAD2DEG,
            quat_list[0],
            quat_list[1],
            quat_list[2],
            quat_list[3],
        )
    )


def kabsch_rigid(src_pts, dst_pts):
    """
    Least-squares rigid transform: dst ~= R @ src + t (column vectors).

    src_pts, dst_pts: (N, 3) matched rows.
    Returns R (3,3), t (3,).
    """
    if src_pts.shape != dst_pts.shape or src_pts.shape[1] != 3:
        raise ValueError("Expected (N,3) arrays of equal shape")
    n = src_pts.shape[0]
    if n < 3:
        rospy.logwarn("Kabsch: fewer than 3 samples; solution may be poorly constrained.")

    c_src = np.mean(src_pts, axis=0)
    c_dst = np.mean(dst_pts, axis=0)
    x = src_pts - c_src
    y = dst_pts - c_dst
    h = x.T @ y
    u, _, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1, :] *= -1.0
        r = vt.T @ u.T
    t = c_dst - r @ c_src
    return r, t


def world_to_global_static_transform(r_w2g, t_w2g):
    """
    Build geometry_msgs Transform for static TF parent=world, child=global.

    Point maps: p_global = R_w2g @ p_world + t_w2g.
    TF convention: p_world = R_tf @ p_global + trans (child -> parent).
    """
    r_tf = r_w2g.T
    trans = -r_tf @ t_w2g
    mat = np.eye(4)
    mat[:3, :3] = r_tf
    mat[:3, 3] = trans
    q = quaternion_from_matrix(mat)
    return trans, q


class WorldToGlobalPublisher:
    def __init__(self):
        rospy.init_node("world_to_global_publisher")

        self.static_br = tf2_ros.StaticTransformBroadcaster()
        self.alignment_published = False

        self.world_frame = rospy.get_param("~world_frame", "world")
        self.global_frame = rospy.get_param("~global_frame", "global")

        self.use_kabsch = rospy.get_param("~use_kabsch", True)
        # Short startup window: collect all synced pairs until duration elapses (or max_samples).
        self.calibration_duration = float(rospy.get_param("~calibration_duration", 0.9))
        self.min_samples = max(3, int(rospy.get_param("~min_samples", 3)))
        self.max_samples = max(1, int(rospy.get_param("~max_samples", 500)))
        # Back-compat: old ~num_samples meant "stop after N pairs" — use as max_samples cap.
        try:
            self.max_samples = max(self.max_samples, int(rospy.get_param("~num_samples")))
        except KeyError:
            pass
        self.sync_slop = float(rospy.get_param("~sync_slop", 0.02))
        self.queue_size = int(rospy.get_param("~sync_queue_size", 50))
        # message: window length from header stamps (repeatable with bags / sim time).
        # wall: rospy.Time.now() from first callback (varies with scheduling / load).
        self.calibration_time_source = rospy.get_param("~calibration_time_source", "message")

        self.vio_topic = rospy.get_param("~vio_odom_topic", "/ov_msckf/base_link_corrected")
        self.mocap_topic = rospy.get_param("~mocap_odom_topic", "/natnet_ros/kingfisher/odom")

        if self.use_kabsch:
            vio_sub = message_filters.Subscriber(self.vio_topic, Odometry)
            mocap_sub = message_filters.Subscriber(self.mocap_topic, Odometry)
            self._ts = message_filters.ApproximateTimeSynchronizer(
                [vio_sub, mocap_sub],
                queue_size=self.queue_size,
                slop=self.sync_slop,
            )
            self._ts.registerCallback(self._synced_callback)
            rospy.loginfo(
                "Kabsch mode: calibrate for up to %.4f s (min %d samples, cap %d) | '%s' + '%s' | "
                "slop=%.3fs | time=%s",
                self.calibration_duration,
                self.min_samples,
                self.max_samples,
                self.vio_topic,
                self.mocap_topic,
                self.sync_slop,
                self.calibration_time_source,
            )
        else:
            self.sub = rospy.Subscriber(self.mocap_topic, Odometry, self._legacy_mocap_callback)
            rospy.loginfo(
                "Legacy mode: waiting for first message on '%s' to anchor '%s' -> '%s'...",
                self.mocap_topic,
                self.world_frame,
                self.global_frame,
            )

    def _synced_callback(self, vio_msg, mocap_msg):
        if self.alignment_published:
            return

        if vio_msg.header.frame_id != self.global_frame:
            rospy.logwarn_throttle(
                5.0,
                "VIO odom frame_id is '%s', expected '%s'. Using positions anyway.",
                vio_msg.header.frame_id,
                self.global_frame,
            )
        if mocap_msg.header.frame_id != self.world_frame:
            rospy.logwarn_throttle(
                5.0,
                "MoCap odom frame_id is '%s', expected '%s'. Using positions anyway.",
                mocap_msg.header.frame_id,
                self.world_frame,
            )

        if not hasattr(self, "_world_pts"):
            self._world_pts = []
            self._global_pts = []
            self._mocap_quats = []
            self._vio_quats = []
            self._pair_stamp_nsec_list = []
            self._mocap_frame_ids = []
            self._vio_frame_ids = []
            self._calib_start_wall = None
            self._t0_stamp_nsec = None
            self._warned_stamp_fallback = False

        pair_nsec = _pair_stamp_nsec(vio_msg, mocap_msg)
        use_msg_time = self.calibration_time_source == "message" and pair_nsec is not None
        if not use_msg_time:
            if not self._warned_stamp_fallback and self.calibration_time_source == "message":
                rospy.logwarn(
                    "Kabsch: header stamps missing/zero; using wall clock for calibration window. "
                    "Fill stamp fields for repeatable runs."
                )
                self._warned_stamp_fallback = True
            if self._calib_start_wall is None:
                self._calib_start_wall = rospy.Time.now()

        pw = mocap_msg.pose.pose.position
        pg = vio_msg.pose.pose.position
        self._world_pts.append([pw.x, pw.y, pw.z])
        self._global_pts.append([pg.x, pg.y, pg.z])

        q_m = _quat_msg_to_list(mocap_msg.pose.pose.orientation)
        q_v = _quat_msg_to_list(vio_msg.pose.pose.orientation)
        self._mocap_quats.append(q_m)
        self._vio_quats.append(q_v)
        self._pair_stamp_nsec_list.append(pair_nsec)
        self._mocap_frame_ids.append(
            (mocap_msg.header.frame_id, mocap_msg.child_frame_id)
        )
        self._vio_frame_ids.append((vio_msg.header.frame_id, vio_msg.child_frame_id))

        n = len(self._world_pts)
        if use_msg_time:
            if self._t0_stamp_nsec is None:
                self._t0_stamp_nsec = pair_nsec
            elapsed = (pair_nsec - self._t0_stamp_nsec) * 1e-9
        else:
            elapsed = (rospy.Time.now() - self._calib_start_wall).to_sec()

        rospy.loginfo_throttle(
            0.1,
            "Kabsch: %d samples | elapsed %.4f / %.4f s (%s)\n%s\n%s",
            n,
            elapsed,
            self.calibration_duration,
            "stamp" if use_msg_time else "wall",
            _rpy_report(
                "  [MoCap] pose orientation (body in map frame):",
                q_m,
                mocap_msg.header.frame_id,
                mocap_msg.child_frame_id,
            ),
            _rpy_report(
                "  [VIO]   pose orientation (body in map frame):",
                q_v,
                vio_msg.header.frame_id,
                vio_msg.child_frame_id,
            ),
        )

        if elapsed < self.calibration_duration and n < self.max_samples:
            return

        self._finalize_kabsch(vio_msg, mocap_msg)

    def _finalize_kabsch(self, vio_msg, mocap_msg):
        """Run once after the calibration window (or max_samples)."""
        if self.alignment_published:
            return

        w = np.asarray(self._world_pts, dtype=np.float64)
        g = np.asarray(self._global_pts, dtype=np.float64)
        mq = list(self._mocap_quats)
        vq = list(self._vio_quats)
        n = w.shape[0]

        # Deterministic ordering: sort by pair time when stamps are available.
        if (
            len(self._pair_stamp_nsec_list) == n
            and all(t is not None for t in self._pair_stamp_nsec_list)
        ):
            order = np.argsort(np.asarray(self._pair_stamp_nsec_list, dtype=np.int64))
            w = w[order]
            g = g[order]
            mq = [mq[i] for i in order]
            vq = [vq[i] for i in order]
            mf = [self._mocap_frame_ids[i] for i in order]
            vf = [self._vio_frame_ids[i] for i in order]
            self._world_pts = w.tolist()
            self._global_pts = g.tolist()
            self._mocap_quats = mq
            self._vio_quats = vq
            self._mocap_frame_ids = mf
            self._vio_frame_ids = vf
            self._pair_stamp_nsec_list = [self._pair_stamp_nsec_list[i] for i in order]

        self._first_mocap_frames = self._mocap_frame_ids[0]
        self._first_vio_frames = self._vio_frame_ids[0]

        # Point-based Kabsch needs at least 3 pairs for a stable rotation (unless collinear).
        use_kabsch = n >= self.min_samples
        if not use_kabsch:
            rospy.logwarn(
                "Kabsch: only %d synced pair(s) after %.4f s (min_samples=%d). "
                "Publishing legacy alignment from first MoCap pose only.",
                n,
                self.calibration_duration,
                self.min_samples,
            )
            self._publish_legacy_from_first_mocap()
            return

        r_w2g, t_w2g = kabsch_rigid(w, g)
        trans, quat = world_to_global_static_transform(r_w2g, t_w2g)

        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.global_frame
        t.transform.translation.x = float(trans[0])
        t.transform.translation.y = float(trans[1])
        t.transform.translation.z = float(trans[2])
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])

        self.static_br.sendTransform(t)
        self.alignment_published = True

        roll, pitch, yaw = euler_from_quaternion(quat)
        rospy.loginfo("--- World -> Global (Kabsch) published ---")
        rospy.loginfo(
            "Reported body orientations (from odom poses; compare to your expectations):\n"
            "  First synced pair —\n%s\n%s\n"
            "  Last synced pair —\n%s\n%s",
            _rpy_report(
                "    [MoCap]",
                self._mocap_quats[0],
                self._first_mocap_frames[0],
                self._first_mocap_frames[1],
            ),
            _rpy_report(
                "    [VIO]",
                self._vio_quats[0],
                self._first_vio_frames[0],
                self._first_vio_frames[1],
            ),
            _rpy_report(
                "    [MoCap]",
                self._mocap_quats[-1],
                mocap_msg.header.frame_id,
                mocap_msg.child_frame_id,
            ),
            _rpy_report(
                "    [VIO]",
                self._vio_quats[-1],
                vio_msg.header.frame_id,
                vio_msg.child_frame_id,
            ),
        )
        rospy.loginfo(
            "Samples: %d | window: %.4f s | sync slop: %.3f s",
            n,
            self.calibration_duration,
            self.sync_slop,
        )
        rospy.loginfo(
            "Static TF %s -> %s (map alignment from Kabsch; NOT the same as body yaw above): "
            "t=(%.4f, %.4f, %.4f)",
            self.world_frame,
            self.global_frame,
            trans[0],
            trans[1],
            trans[2],
        )
        rospy.loginfo(
            "Static TF rotation RPY (rad): R=%.4f, P=%.4f, Y=%.4f",
            roll,
            pitch,
            yaw,
        )
        rospy.loginfo(
            "Static TF rotation RPY (deg): R=%.2f, P=%.2f, Y=%.2f",
            roll * RAD2DEG,
            pitch * RAD2DEG,
            yaw * RAD2DEG,
        )

    def _publish_legacy_from_first_mocap(self):
        """Use first collected MoCap pose (positions/quats lists) for static TF."""
        if not self._mocap_quats:
            rospy.logerr("Kabsch fallback: no MoCap data; cannot publish alignment.")
            self.alignment_published = True
            return

        qx, qy, qz, qw = self._mocap_quats[0]
        px, py, pz = self._world_pts[0]

        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.global_frame
        t.transform.translation.x = float(px)
        t.transform.translation.y = float(py)
        t.transform.translation.z = float(pz)
        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)

        self.static_br.sendTransform(t)
        self.alignment_published = True

        quat_list = [qx, qy, qz, qw]
        roll, pitch, yaw = euler_from_quaternion(quat_list)
        rospy.loginfo("--- World -> Global (legacy fallback after short window) ---")
        rospy.loginfo(
            "Translation: x=%.3f, y=%.3f, z=%.3f",
            px,
            py,
            pz,
        )
        rospy.loginfo("Rotation (rad): R=%.3f, P=%.3f, Y=%.3f", roll, pitch, yaw)
        rospy.loginfo(
            "Rotation (deg): R=%.1f, P=%.1f, Y=%.1f",
            roll * RAD2DEG,
            pitch * RAD2DEG,
            yaw * RAD2DEG,
        )

    def _legacy_mocap_callback(self, msg):
        if self.alignment_published:
            return

        q = msg.pose.pose.orientation
        quat_list = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = euler_from_quaternion(quat_list)

        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.global_frame

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = q

        self.static_br.sendTransform(t)
        self.alignment_published = True

        rospy.loginfo("--- World -> Global (legacy, first MoCap pose) ---")
        rospy.loginfo(
            "Translation: x=%.3f, y=%.3f, z=%.3f",
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z,
        )
        rospy.loginfo("Rotation (rad): R=%.3f, P=%.3f, Y=%.3f", roll, pitch, yaw)
        rospy.loginfo(
            "Rotation (deg): R=%.1f, P=%.1f, Y=%.1f",
            roll * 57.2958,
            pitch * 57.2958,
            yaw * 57.2958,
        )


if __name__ == "__main__":
    WorldToGlobalPublisher()
    rospy.spin()

import cv2
import numpy
import numpy as np
import rospy
import sensor_msgs.msg
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import tf2_ros
import tf_conversions
import geometry_msgs.msg
import tf.transformations

np.set_printoptions(linewidth=np.inf)


class Node(object):

    def __init__(self):
        self.bridge = CvBridge()
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.image_l = None
        self.image_r = None
        self.intrinsic_l = None
        self.intrinsic_r = None
        self.run_flag = False
        self._image_l_sub = rospy.Subscriber('/camera_l/image_raw', sensor_msgs.msg.Image, self._image_l_callback)
        self._image_r_sub = rospy.Subscriber('/camera_r/image_raw', sensor_msgs.msg.Image, self._image_r_callback)
        self._camera_info_l_sub = rospy.Subscriber('/camera_l/camera_info', sensor_msgs.msg.CameraInfo,
                                                   self._camera_info_l_callback)
        self._camera_info_r_sub = rospy.Subscriber('/camera_r/camera_info', sensor_msgs.msg.CameraInfo,
                                                   self._camera_info_r_callback)

        rospy.init_node('epipolar')

        while not rospy.is_shutdown():
            self.__spin_once()
            rospy.sleep(1)

    def __spin_once(self):
        retval_l, corners_l = cv2.findChessboardCorners(self.image_l, (8, 6))
        retval_r, corners_r = cv2.findChessboardCorners(self.image_r, (8, 6))
        cv2.drawChessboardCorners(self.image_l, (8, 6), corners_l, retval_l)
        cv2.drawChessboardCorners(self.image_r, (8, 6), corners_r, retval_r)

        # plt.figure(figsize=(10, 10))
        # plt.imshow(self.image_r, extent=[0, self.image_l.shape[1], 0, self.image_l.shape[0]])
        # plt.show()

        essential, m = cv2.findEssentialMat(corners_l, corners_r, self.intrinsic_l, cv2.LMEDS)
        ret, r, t, m = cv2.recoverPose(essential, corners_l, corners_r)

        a = None
        for i in range(len(corners_l)):
            ul = corners_l[i][0][0]
            vl = corners_l[i][0][1]
            ur = corners_r[i][0][0]
            vr = corners_r[i][0][1]
            a_l = np.array([[
                ur * ul,
                vr * ul,
                ul,
                ur * vl,
                vr * vl,
                vl,
                ur,
                vr,
                1.0,
            ]])
            if a is None:
                a = a_l
            else:
                a = np.append(a, a_l, axis=0)

        u, s, vt = np.linalg.svd(a)
        fundamental = vt[np.argmin(s)]
        fundamental /= fundamental[8]
        fundamental = np.reshape(fundamental, (3, 3))
        u, s, vt = np.linalg.svd(fundamental)
        s[2] = 0.0
        fundamental = np.linalg.multi_dot([u, np.diag(s), vt])

        essential = np.linalg.multi_dot([self.intrinsic_l.T, fundamental, self.intrinsic_r])

        u, s, vt = np.linalg.svd(essential)
        s = np.diag(s)
        w = np.array([[0, -1, 0],
                      [1, 0, 0],
                      [0, 0, 1]])
        w_inv = np.array([[0, -1, 0],
                          [1, 0, 0],
                          [0, 0, 1]])
        z = np.array([[0, 1, 0],
                      [-1, 0, 0],
                      [0, 0, 0]])

        txs = []
        rs = []

        txs.append(np.linalg.multi_dot([u, z, u.T]))
        rs.append(np.linalg.multi_dot([u, w_inv, vt]))

        tx = txs[0]
        r = rs[0]

        transform = np.array(
            [[r[0][0], r[0][1], r[0][2], 0],
             [r[1][0], r[1][1], r[1][2], 0],
             [r[2][0], r[2][1], r[2][2], 0],
             [0, 0, 0, 1]])

        print(np.mean(tf.transformations.quaternion_from_matrix(transform)))

        self.broadcaster.sendTransform(self._make_tf_quat('map',
                                                          '1',
                                                          tf.transformations.translation_from_matrix(transform),
                                                          tf.transformations.quaternion_from_matrix(transform)))

    def _image_l_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.image_l = image

    def _image_r_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.image_r = image

    def _camera_info_l_callback(self, msg):
        self.intrinsic_l = np.reshape(msg.K, (3, 3))

    def _camera_info_r_callback(self, msg):
        self.intrinsic_r = np.reshape(msg.K, (3, 3))

    def _make_tf(self, parent_id, child_id, translation, rotation):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent_id
        t.child_frame_id = child_id
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        q = tf_conversions.transformations.quaternion_from_euler(rotation[0], rotation[1], rotation[2])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        return t

    def _make_tf_quat(self, parent_id, child_id, translation, q):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent_id
        t.child_frame_id = child_id
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        return t

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

        # while not rospy.is_shutdown():
        #     self.__spin_once()
        #     rospy.sleep(0.1)
        rospy.sleep(0.1)
        self.__spin_once()

    def __spin_once(self):
        if not self.run_flag and self.image_l is not None and self.image_r is not None:
            self.run_flag = True
            retval_l, corners_l = cv2.findChessboardCorners(self.image_l, (8, 6))
            retval_r, corners_r = cv2.findChessboardCorners(self.image_r, (8, 6))
            cv2.drawChessboardCorners(self.image_l, (8, 6), corners_l, retval_l)
            cv2.drawChessboardCorners(self.image_r, (8, 6), corners_r, retval_r)

            # plt.figure(figsize=(10, 10))
            # plt.imshow(self.image_r, extent=[0, self.image_r.shape[1], 0, self.image_r.shape[0]])
            # plt.show()

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

            print(a)

            u, s, vt = np.linalg.svd(a)
            fundamental = vt[np.argmin(s)]
            fundamental /= fundamental[8]
            fundamental = np.reshape(fundamental, (3, 3))
            print(fundamental)

            f, mask = cv2.findFundamentalMat(corners_l, corners_r, cv2.FM_8POINT)
            print(f)

            f, mask = cv2.findFundamentalMat(corners_l, corners_r, cv2.FM_RANSAC)
            print(f)

            point_l = np.array([[corners_l[0][0][0], corners_l[0][0][1], 1]])
            point_r = np.array([[corners_r[0][0][0], corners_r[0][0][1], 1]])

            u, s, vt = np.linalg.svd(np.dot(point_l, fundamental))
            point_r_hat = vt[np.argmin(s)]
            point_r_hat /= point_r_hat[2]

            print('r_hat', point_r_hat)
            print('r', point_r)

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

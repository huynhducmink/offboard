#! /usr/bin/env python2

import cv2
from cv2 import aruco
import numpy as np
import rospy
import mavros
from mavros_msgs.msg import PositionTarget as PT
from std_msgs.msg import Float32
from mavros import setpoint as SP
import tf



class MarkerDetector():
    def __init__(self):
        rospy.init_node('marker_detector', anonymous=True)
        mavros.set_namespace('mavros')
        self.cap = cv2.VideoCapture(0)
        self.mtx = np.array([1.537822710187638449e+03, 0.0, 6.404980296570619203e+02, 0.0, 1.418298704599976645e+03, 2.406765310346528963e+02, 0.0, 0.0, 1.0]).reshape(3,3)
        self.dist = np.array([-3.479798240858279768e-02, 6.107234468717099851e-01, -1.615287344933384606e-02, -2.327112969541987163e-03,-2.305669941088070551e+00])
        # matrix from imu to camera
        self.imu_cam = np.zeros((4,4), dtype=np.float)
        self.dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.param = cv2.aruco.DetectorParameters_create()
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.aruco_marker_pos_pub = rospy.Publisher('/aruco_marker_pos', PT, queue_size=10)
        self.target_position = rospy.Publisher('/target_position', PT, queue_size=10)
        self.check_error_pos = rospy.Publisher('/check_error_pos', Float32, queue_size=10)
        # /mavros/local_position/pose
        local_position_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'),
            SP.PoseStamped, self._local_position_callback)
        # Initialize the parameters
        self.local_pos = [0.0] * 4
        # self.target_pos = [0.0] * 4
        # self.check_err = [0.0]

        self.rate = rospy.Rate(20)
    

    def _local_position_callback(self, topic):
        # Position data
        self.local_pos[0] = topic.pose.position.x
        self.local_pos[1] = topic.pose.position.y
        self.local_pos[2] = topic.pose.position.z
        
        # Orientation data
        (r, p, y) = tf.transformations.euler_from_quaternion([topic.pose.orientation.x, topic.pose.orientation.y, topic.pose.orientation.z, topic.pose.orientation.w])
        self.local_pos[3] = y

    def marker_pose(self):
        # rospy.init_node("pub_pos", anonymous=True)
        # rate = rospy.Rate(10)
        self.cap.set(3, 1280)
        self.cap.set(4, 720)
        # set dictionary size depending on the aruco marker selected
        self.param.adaptiveThreshConstant = 7
        # setup matrix from imu to cam
        self.imu_cam[0][1] = -1.0
        self.imu_cam[0][3] = 0.1
        self.imu_cam[1][0] = -1.0
        self.imu_cam[1][3] = -0.05
        self.imu_cam[2][2] = -1.0
        self.imu_cam[2][3] = -0.1
        self.imu_cam[3][3] = 1.0

        # create vector tvec1, tvec2
        tvec1 = np.zeros((4,1), dtype=np.float)
        tvec1[3][0] = 1.0
        tvec2 = np.zeros((4,1), dtype=np.float)
        # tvec2[3][0] = 1.0

        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            # operations on the frame
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

            # lists of ids and the corners belonging to each id
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.dict, parameters=self.param)

            if np.all(ids is not None):
                ret1 = aruco.estimatePoseSingleMarkers(corners=corners, markerLength=0.08,
                                                       cameraMatrix=self.mtx, distCoeffs=self.dist)
                rvec, tvec = ret1[0][0, 0, :], ret1[1][0, 0, :]
                # -- Draw the detected marker and put a reference frame over it
                aruco.drawDetectedMarkers(frame, corners, ids)
                aruco.drawAxis(frame, self.mtx, self.dist, rvec, tvec, 0.1)
                str_position0 = "Marker Position in Camera frame: x=%f  y=%f  z=%f" % (tvec[0], tvec[1], tvec[2])
                cv2.putText(frame, str_position0, (0, 50), self.font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                
                (rvec - tvec).any()  # get rid of that nasty numpy value array error
                
                tvec1[0][0] = tvec[0]
                tvec1[1][0] = tvec[1]
                tvec1[2][0] = tvec[2]

                if (self.local_pos[3] > -0.1 and self.local_pos[3] < 0.1):
                # self.pub.publish(marker_position[0], marker_position[1], marker_position[2])
                    check_err = np.linalg.norm([tvec[0], tvec[1]])
                    self.check_error_pos.publish(check_err)
                    # publish marker position in uav frame
                    marker_pos = PT()
                    tvec2 = np.matmul(self.imu_cam, tvec1)
                    # marker_pos.position.x = -tvec[1]
                    # marker_pos.position.y = -tvec[0]
                    # marker_pos.position.z = -tvec[2]
                    
                    marker_pos.position.x = tvec2[0][0]
                    marker_pos.position.y = tvec2[1][0]
                    marker_pos.position.z = tvec2[2][0]
                    self.aruco_marker_pos_pub.publish(marker_pos)

                    # publish target position in world frame
                    target_pos = PT()
                    # target_pos.position.x = self.local_pos[0] - tvec[1]
                    # target_pos.position.y = self.local_pos[1] - tvec[0]
                    target_pos.position.x = self.local_pos[0] + tvec2[0][0]
                    target_pos.position.y = self.local_pos[1] + tvec2[1][0]
                    # self.target_pos[0] = self.local_pos[0] + tvec[1]
                    # self.target_pos[1] = self.local_pos[1] + tvec[0]
                    self.target_position.publish(target_pos)
                    self.rate.sleep()
                else:
                    check_err = np.linalg.norm([tvec[0], tvec[1]])
                    self.check_error_pos.publish(check_err)

                    # setpose = np.zeros((3,1), dtype=np.float)
                    # setpose[0] = -tvec[1]
                    # setpose[1] = -tvec[0]
                    # setpose[2] = -tvec[2]
                    # # change form
                    rotMat = tr.euler_matrix(0, 0, self.local_pos[3])
                    rotMat = np.matmul(rotMat, self.imu_cam)
                    # rotMat = rotMat[0:3, 0:3]
                    tvec2 = np.matmul(rotMat, tvec1)
                    # publish marker position in uav frame
                    # marker_pos = PT()
                    # marker_pos.position.x = setpose[0]
                    # marker_pos.position.y = setpose[1]
                    # marker_pos.position.z = setpose[2]
                    marker_pos.position.x = tvec2[0][0]
                    marker_pos.position.y = tvec2[1][0]
                    marker_pos.position.z = tvec2[2][0]
                    self.aruco_marker_pos_pub.publish(marker_pos)

                    # publish target position in world frame
                    target_pos = PT()
                    target_pos.position.x = self.local_pos[0] + tvec2[0][0]
                    target_pos.position.y = self.local_pos[1] + tvec2[1][0]
                    # target_pos.position.x = self.local_pos[0] + setpose[0]
                    # target_pos.position.y = self.local_pos[1] + setpose[1]
                    # self.target_pos[0] = self.local_pos[0] + tvec[1]
                    # self.target_pos[1] = self.local_pos[1] + tvec[0]
                    self.target_position.publish(target_pos)
                    self.rate.sleep()

            cv2.imshow("frame", frame)
            cv2.waitKey(1)
            #self.rate.sleep()


if __name__ == '__main__':
    MD = MarkerDetector()
    try:
        MD.marker_pose()
    except rospy.ROSInterruptException:
        pass

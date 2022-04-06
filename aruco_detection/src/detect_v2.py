#!/usr/bin/env python

import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco
import numpy as np
class image_convert_pub:

  def __init__(self):
    self.image_pub = rospy.Publisher("/detected_markers",Image, queue_size=1)
    self.id_pub = rospy.Publisher("/aruco_ID", String, queue_size=1)
    self.bridge = CvBridge()
    print("In the callback first")
   # self.image_sub = rospy.Subscriber("/rgb/image_raw/", Image, self.callback)
    img = rospy.wait_for_message('/rgb/image_raw/',Image)
    camera2_info_msg = rospy.wait_for_message('/rgb/camera_info', CameraInfo)
    self.camera2_matrix_rgb = np.array(camera2_info_msg.K).reshape(3, 3)
    self.dist_coeffs_rgb = np.zeros(5)
    self.callback(img)
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    print("In the callback second")  	
    corners,markers_img, ids_list = self.detect_aruco(cv_image)
    corners_2,markers_img_2, ids_list_2 = self.detect_aruco_v2(cv_image)

    if ids_list is None and ids_list_2 is None:
      print("none")
      self.id_pub.publish(ids_list + ids_list_2)
    else:
      
      ids_str = ''.join(str(e) for e in ids_list)
      ids_str_2 = ''.join(str(e) for e in ids_list_2)
      print(ids_str + ids_str_2)
      self.id_pub.publish(ids_str + ids_str_2 )

    if np.all(ids_list is not None):  # If there are markers found by detector
            for i in range(0, len(ids_list)):  # Iterate in markers
                # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
                rvec, tvec = aruco.estimatePoseSingleMarkers(corners[i], 10,self.camera2_matrix_rgb,self.dist_coeffs_rgb)
                print(rvec)
                print(tvec)
                print(ids_list[i])
                print(".........")
                (rvec - tvec).any()  # get rid of that nasty numpy value array error
                aruco.drawDetectedMarkers(cv_image, corners)  # Draw A square around the markers
                aruco.drawAxis(cv_image, self.camera2_matrix_rgb, self.dist_coeffs_rgb, rvec, tvec, 0.01)  # Draw Axis
# Display the resulting frame

    if np.all(ids_list_2 is not None):  # If there are markers found by detector
            for i in range(0, len(ids_list_2)):  # Iterate in markers

                rvec_2, tvec_2 = aruco.estimatePoseSingleMarkers(corners_2[i],10,self.camera2_matrix_rgb,self.dist_coeffs_rgb)

                print(rvec_2)
                print(tvec_2)
                print(ids_list_2[i])
                print("....2nd.....")
                (rvec - tvec).any()  
    
    cv2.destroyAllWindows()
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(markers_img, "bgr8"))
    except CvBridgeError as e:
      print(e)

  def detect_aruco(self,img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
    parameters = aruco.DetectorParameters_create()
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
    output = aruco.drawDetectedMarkers(img, corners, ids)  # detect the sruco markers and display its aruco id.
    return corners,output, ids

  def detect_aruco_v2(self,img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
    output = aruco.drawDetectedMarkers(img, corners, ids)  # detect the sruco markers and display its aruco id.
    return corners,output, ids


def main():
  print("Initializing ROS-node")
  rospy.init_node('detect_markers', anonymous=True)
  print("Bring the aruco-ID in front of camera")
  ic = image_convert_pub()
  rospy.spin()

if __name__ == '__main__':
    main()

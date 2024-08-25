#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np

# -------------------CALIBRAR   Thresh BIN  ->  Jala bien

contador = 0

def Hminimo(val):
    pass

def Hmaximo(val):
    pass

def createTrackbars():
    cv2.namedWindow("Trackbars")
    cv2.createTrackbar("Thresh", "Trackbars", 0, 255, Hminimo)

def image_callback(data):
    global cam_w, cam_h, contador
    global frame, thresh, thresh_inv

    if contador == 0:
        createTrackbars()
        contador = 1

    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    # Convert ROS Image message to OpenCV image
    current_frame = br.imgmsg_to_cv2(data, desired_encoding="bgr8")

    # Process image
    frame = current_frame

    # Output debugging information to the terminal
    # rospy.loginfo("receiving video frame")

    thr = cv2.getTrackbarPos("Thresh", "Trackbars")

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, thr, 255, cv2.THRESH_BINARY)
    ret, thresh_inv = cv2.threshold(gray, thr, 255, cv2.THRESH_BINARY_INV)

    cv2.imshow("Thresh Calibration ", thresh)
    cv2.imshow("Thresh INV Calibration ", thresh_inv)

    cv2.waitKey(1)


def receive_message():
    # Tells rospy the name of the node.
    # Anonymous = True makes sure the node has a unique name. Random
    # numbers are added to the end of the name.
    #print ('receive msg')
    rospy.init_node('binCalib_sub_py', anonymous=True)

    # Node is subscribing to the video_frames topic
    rospy.Subscriber('/ardrone/bottom/image_raw',  Image, image_callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    # Close down the video stream when done
    cv2.destroyAllWindows()


if __name__ == '__main__':
    receive_message()



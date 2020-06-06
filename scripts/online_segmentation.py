#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide" # Hides the pygame version, welcome msg
from os.path import expanduser
import glob
import scipy.signal as signal
import sliding_window_approach

DBASW = sliding_window_approach.sliding_window()

class hilly_nav():

    def __init__(self):
        self.image = Image()
        self.roi_img = Image()
        self.final_img = []

        self.modifiedCenters_local = []
        self.crop_ratio = 0.3 # Ratio to crop the background parts in the image from top

        self.centerLine = []
        self.bridge = CvBridge()

        # subscribed Topic
        self.subscriber = rospy.Subscriber("/kinect2_camera/rgb/image_color_rect",Image, self.callback)

    def callback(self, ros_data):
       #### direct conversion to CV2 ####
       # np_arr = np.fromstring(ros_data.data, np.uint8)
       # self.image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
       try:
         self.image = self.bridge.imgmsg_to_cv2(ros_data, "bgr8")
       except CvBridgeError as e:
         print(e)

       self.run_lane_fit()

    def segment_image(self):

        # define range of blue color in HSV
        lower_plants = np.array([0,22,0])
        upper_plants = np.array([255,255,19])

        # Threshold the HSV image to get only blue colors
        self.roi_img = cv2.inRange(self.roi_img, lower_plants, upper_plants)

        coldata = np.sum(self.roi_img, axis=0)/255 # Sum the columns of warped image to determine peaks

        self.modifiedCenters_local = signal.find_peaks(coldata, height=100, distance=self.roi_img.shape[1]/3)

        # print self.modifiedCenters_local[0][0]

    def visualize_lane_fit(self, dst_size):

       self.roi_img, self.current_Pts = DBASW.sliding_window(self.roi_img, self.modifiedCenters_local)

       # Visualize the fitted polygonals (One on each lane and on average curve)
       self.roi_img, self.centerLine = DBASW.visualization_polyfit(self.roi_img, self.current_Pts)

       if len(self.modifiedCenters_local[0]):
           # for mc_in in range(len(self.modifiedCenters_local)):
               # print self.modifiedCenters_local[0][0], self.final_img.shape[0]-20
               cv2.circle(self.roi_img, (int(self.modifiedCenters_local[0][0]),int(self.roi_img.shape[0]-20)),
                                                                                0, (255,0,255), thickness=30, lineType=8, shift=0)

       self.final_img = self.image
       # self.final_img = cv2.cvtColor(self.image, cv2.COLOR_GRAY2RGB)
       rheight, rwidth = self.final_img.shape[:2]
       self.final_img[int(rheight*self.crop_ratio):rheight,0:rwidth] = cv2.addWeighted( self.roi_img, 0.6,self.final_img[int(rheight*self.crop_ratio):int(rheight),0:rwidth], 0.8, 0)

       cv2.imwrite('/home/saga/dummy.png', self.final_img)

    def run_lane_fit(self):
       # Setting the parameters for upscaling and warping-unwarping
       rheight, rwidth = self.image.shape[:2]
       self.roi_img = self.image[int(self.crop_ratio*rheight):rheight,0:rwidth]
       dst_size = self.roi_img.shape[:2]

       # Function to obtain the ground truth values in Map frame
       self.segment_image()

       # Overlay the inverse warped image on input image
       self.visualize_lane_fit(dst_size)

if __name__ == '__main__':

    # Initialize node
    rospy.init_node('online_segmentation')
    hilly_nav_obj = hilly_nav()

    try:
        rospy.spin()
    except KeyboardInterrupt:
       print("Shutting down")
    cv2.destroyAllWindows()

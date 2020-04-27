#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
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

    def segment_image(self):

        # define range of blue color in HSV
        lower_plants = np.array([0,22,0])
        upper_plants = np.array([255,255,19])

        # Threshold the HSV image to get only blue colors
        self.roi_img = cv2.inRange(self.roi_img, lower_plants, upper_plants)

        coldata = np.sum(self.roi_img, axis=0)/255 # Sum the columns of warped image to determine peaks

        self.modifiedCenters_local = signal.find_peaks(coldata, height=200, distance=self.roi_img.shape[1]/3)

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
    try:
        #Initialize node
        rospy.init_node('lateral_heading_offset')

        hilly_nav_obj = hilly_nav()
        input_dir = expanduser("~/Hilly_Nav_Paper/plant_images/")
        output_dir =expanduser("~/Hilly_Nav_Paper/fitted_images/")

        for label_file in sorted(glob.glob(os.path.join(input_dir, '*.png'))):
            print(label_file)
            with open(label_file) as f:
                base = os.path.splitext(os.path.basename(label_file))[0]
                hilly_nav_obj.image = cv2.imread(label_file)

                hilly_nav_obj.run_lane_fit()

                # Store the Cropped Images
                cv2.imwrite(output_dir+base+'.png', hilly_nav_obj.final_img)

    except rospy.ROSInterruptException:
         cv2.destroyAllWindows() # Closes all the frames
         pass

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

class hilly_nav():

    def __init__(self):
        self.image = Image()
        self.modifiedCenters_local = []
        self.final_img = []

    def segment_image(self, img):

        # define range of blue color in HSV
        lower_plants = np.array([0,22,0])
        upper_plants = np.array([255,255,19])

        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(img, lower_plants, upper_plants)


        coldata = np.sum(mask, axis=0)/255 # Sum the columns of warped image to determine peaks

        self.modifiedCenters_local = signal.find_peaks(coldata, height=100)

        self.final_img = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)

        if len(self.modifiedCenters_local[0]):
           # for mc_in in range(len(self.modifiedCenters_local)):
               print self.modifiedCenters_local[0][0], self.final_img.shape[0]-20

               cv2.circle(self.final_img, (int(self.modifiedCenters_local[0][0]),int(self.final_img.shape[0]-20)),
                                                                                0, (255,0,255), thickness=30, lineType=8, shift=0)

        return self.final_img

if __name__ == '__main__':
    try:
        #Initialize node
        rospy.init_node('lateral_heading_offset')

        hilly_nav_obj = hilly_nav()
        input_dir = expanduser("~/Hilly_Nav_Paper/plant_images/")
        output_dir =expanduser("~/Hilly_Nav_Paper/segmented_images/")

        for label_file in sorted(glob.glob(os.path.join(input_dir, '*.png'))):
            print(label_file)
            with open(label_file) as f:
                base = os.path.splitext(os.path.basename(label_file))[0]
                img = cv2.imread(label_file)

                # Function to obtain the ground truth values in Map frame
                mask = hilly_nav_obj.segment_image(img)

                # Store the Cropped Images
                cv2.imwrite(output_dir+base+'.png', mask)

    except rospy.ROSInterruptException:
         cv2.destroyAllWindows() # Closes all the frames
         pass

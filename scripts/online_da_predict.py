#!/usr/bin/env python
import rospy

import os
import glob
import argparse
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

import sys
sys.path.insert(1, '../image-segmentation-keras')
from keras_segmentation import predict
import time
start_time = time.time()
from sensor_msgs.msg import Image

class hilly_nav():

    def __init__(self):
        self.image = Image()
        self.pred_img = Image()
        self.overlay_img = Image()

        self.label_no = 1 # DA Class
        self.n_classes = 2

        self.bridge = CvBridge()
        self.img_received = False

        # Subscribers
        self.subscriber = rospy.Subscriber("/camera/color/image_raw",Image, self.callback)

        # Publishers
        self.seg_img_pub = rospy.Publisher('predicted_image', Image)
        self.overlay_img_pub = rospy.Publisher('overlay_image', Image)
        self.rate = rospy.Rate(1) # 10hz

    def callback(self, ros_data):
       #### direct conversion to CV2 ####
       try:
         self.image = self.bridge.imgmsg_to_cv2(ros_data, "bgr8")
         self.img_received = True
       except CvBridgeError as e:
         print(e)

    def visualize_segmentation(self, seg_arr, display = False):


        seg_img = predict.segmented_image_from_prediction(seg_arr, self.n_classes, input_shape = self.image.shape)

        # Reshaping the Lanes Class into binary array and Upscaling the image as input image
        self.overlay_img = cv2.addWeighted(self.image,0.7,seg_img,0.3,0)
        dummy_img = np.zeros(seg_arr.shape)
        dummy_img += ((seg_arr[:,: ] == self.label_no)*(255)).astype('uint8')
        original_h, original_w = self.overlay_img.shape[0:2]
        self.pred_img = cv2.resize(dummy_img, (original_w,original_h)).astype('uint8')
        upscaled_img_rgb = cv2.cvtColor(self.pred_img, cv2.COLOR_GRAY2RGB)

        # Stack input and segmentation in one video
        vis_img = np.vstack((
           np.hstack((self.image,
                      seg_img)),
           np.hstack((self.overlay_img,
                      upscaled_img_rgb)) # np.ones(overlay_img.shape,dtype=np.uint8)*128
        ))

        # return upscaled_img

    def predict_on_image(self, model, inp, visualize = None, display=False):

        # Run prediction (and optional, visualization)
        seg_arr, self.image = predict.predict_fast(model, inp)

        if visualize:
            self.visualize_segmentation(seg_arr, display=display)

            self.seg_img_pub.publish(self.bridge.cv2_to_imgmsg(self.pred_img, "mono8"))
            self.overlay_img_pub.publish(self.bridge.cv2_to_imgmsg(self.overlay_img, "rgb8"))
            # self.rate.sleep()

            # if display:
            #     cv2.imshow('Prediction', self.pred_img)
            # if not output_file is None:
            #     cv2.imwrite(output_file, self.pred_img)
        else:
            self.pred_img = None

        return seg_arr

if __name__ == '__main__':

    try:
        # Initialize node
        rospy.init_node('online_da_predict')
        nav_obj = hilly_nav()

        model_prefix = rospy.get_param("/model_prefix")
        epoch = rospy.get_param("/epoch")
        input_folder = rospy.get_param("/input_folder")
        output_folder = rospy.get_param("/output_folder")

        #Load model
        model = predict.model_from_checkpoint_path(model_prefix, epoch)

        while not rospy.is_shutdown():

            # print('Output_folder',output_folder)
            # im_files = glob.glob(os.path.join(input_folder,'*.jpg'))
            # print(os.path.join(input_folder+'*.png'))
            # for im in sorted(im_files):
            #     if output_folder:
            #         base = os.path.basename(im)
            #         output_file = os.path.join(output_folder,os.path.splitext(base)[0])+"_pred.png"
            #         print(output_file)
            #     else:
            #         output_file = None
            if nav_obj.img_received==True:
                seg_arr = nav_obj.predict_on_image(model,inp = nav_obj.image, visualize = True, display=True)

                print("--- %s seconds ---" % (time.time() - start_time))
                nav_obj.img_received = False

    except KeyboardInterrupt:
       print("Shutting down")
       cv2.destroyAllWindows()

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
        self.n_classes=2

        self.bridge = CvBridge()

        # Subscribers
        self.subscriber = rospy.Subscriber("/camera/color/image_raw",Image, self.callback)

        # Publishers
        self.seg_img_pub = rospy.Publisher('predicted_image', Image, queue_size=10)
        self.overlay_img_pub = rospy.Publisher('overlay_image', Image, queue_size=10)
        self.rate = rospy.Rate(1) # 10hz

    def callback(self, ros_data):
       #### direct conversion to CV2 ####
       try:
         self.image = self.bridge.imgmsg_to_cv2(ros_data, "bgr8")
       except CvBridgeError as e:
         print(e)

    def visualize_segmentation(self, seg_arr, display = False, output_file = None):


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

    def predict_on_image(self, model, inp, visualize = None, output_file = None, display=False):

        # Run prediction (and optional, visualization)
        seg_arr, self.image = predict.predict_fast(model, inp)

        if visualize:
            self.visualize_segmentation(seg_arr, display=display, output_file=output_file)

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

    # Initialize node
    rospy.init_node('online_da_predict')
    nav_obj = hilly_nav()

    parser = argparse.ArgumentParser(description="Example: Run prediction on an image folder. Example usage: python lane_predict.py --model_prefix=models/resnet_3class --epoch=25 --input_folder=Frogn_Dataset/images_prepped_test --output_folder=.")
    parser.add_argument("--model_prefix", default = '', help = "Prefix of model filename")
    parser.add_argument("--epoch", default = None, help = "Checkpoint epoch number")
    parser.add_argument("--input_folder",default = '', help = "(Relative) path to input image file")
    parser.add_argument("--output_folder", default = '', help = "(Relative) path to output image file. If empty, image is not written.")
    parser.add_argument("--display",default = False, help = "Whether to display video on screen (can be slow)")

    args = parser.parse_args()

    #Load model
    model = predict.model_from_checkpoint_path(args.model_prefix, args.epoch)

    while not rospy.is_shutdown():

        print('Output_folder',args.output_folder)
        im_files = glob.glob(os.path.join(args.input_folder,'*.jpg'))
        print(os.path.join(args.input_folder+'*.png'))
        for im in sorted(im_files):
            if args.output_folder:
                base = os.path.basename(im)
                output_file = os.path.join(args.output_folder,os.path.splitext(base)[0])+"_pred.png"
                print(output_file)
            else:
                output_file = None

            seg_arr = nav_obj.predict_on_image(model,inp = im, visualize = True, output_file = output_file, display=True)

            print("--- %s seconds ---" % (time.time() - start_time))

    try:
        rospy.spin()
    except KeyboardInterrupt:
       print("Shutting down")
       cv2.destroyAllWindows()

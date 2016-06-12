#! /usr/bin/python

import roslib
roslib.load_manifest('lasa_perception_module')
import rospy
import sys
import json
import cv2
import numpy as np
from sensor_msgs.msg import Image as rosImageMsg
import Image, ImageFilter
from cv_bridge import CvBridge
from lasa_perception_module.srv import *


def PCA(data):
	cov = np.cov(data)
	print 'Data Covariance:', cov
	eigenvalues, eigenvectors = np.linalg.eig(cov)
	return eigenvalues, eigenvectors	

class dough_detector(object):
  def __init__(self):      
 
      # Image processing visualization
      cv2.namedWindow("Black Table/Plate Mask", 0)
      cv2.moveWindow("Black Table/Plate Mask", 0, 0)
      cv2.namedWindow("Masked Scene", 0)
      cv2.moveWindow("Masked Scene", 385, 0)
      cv2.namedWindow("Gray-Blurred Image", 0)
      cv2.moveWindow("Gray-Blurred Image", 705, 0)
      cv2.namedWindow("Processed Image", 0)
      cv2.moveWindow("Processed Image", 1020, 0)
      cv2.namedWindow("Dough Detection", 0)
      cv2.moveWindow("Dough Detection", 1340, 0)
      
      self.bridge = CvBridge()      
      self.loop = 0    
      self.s = rospy.Service('dough_Pose', doughPose, self.callback)

  def callback(self,req):
	  #Extract Table Image from Service Request
	  cv_image = self.bridge.imgmsg_to_cv2(req.Table_image, desired_encoding="bgr8")
	  ##--------------------------------##
    	  ##---- FIND BLACK PLATE/TABLE ----##
    	  ##--------------------------------##
    	  
	  #Blur and Convert image to grayscale
	  b,g,r = cv2.split(cv_image)
	  ret,r_image = cv2.threshold(r,100,255,cv2.THRESH_BINARY_INV)
	  kernel = np.ones((5,5),np.uint8)
	  r_image = cv2.dilate(r_image,kernel,iterations = 1)

	  #Find Contours of Binary Image
	  table_cont = r_image
	  contours, hierarchy = cv2.findContours(table_cont,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	  # Finding contour with maximum area and store it as best_cnt
	  max_area = 0
	  for cnt in contours:
	    area = cv2.contourArea(cnt)
	    if area > max_area:
		max_area = area
		best_cnt = cnt
	  mask = np.zeros(r_image.shape,np.uint8)
	  cv2.drawContours(mask,[best_cnt],0,255,-1)
	  kernel = np.ones((5,5),np.uint8)
	  mask = cv2.dilate(mask,kernel,iterations = 1)
	  cv2.imshow("Black Table/Plate Mask", mask)
	  	  
	  masked_image = cv_image	  
	  masked_scene = cv2.bitwise_and(cv_image,masked_image,mask = mask)
	  cv2.imshow("Masked Scene", masked_scene)
	  
	  blur_image = cv2.medianBlur(masked_scene,15)
	  gray_image = cv2.cvtColor(blur_image, cv2.COLOR_BGR2GRAY)
	  height, width = gray_image.shape
	  cv2.imshow("Gray-Blurred Image", gray_image)
	  
	  ##--------------------------------##
    	  ##---- FIND BLACK PLATE/TABLE ----##
    	  ##--------------------------------##
	  
	  # Convert the image to black and white.
	  threshold = 150
	  ret,bw_image = cv2.threshold(gray_image,threshold,255,cv2.THRESH_BINARY)	
	  # Remove margins of BW Image
	  im_arr = bw_image
	  margin = 0.12
	  im_arr[0:int(height*margin), :] = 0
	  im_arr[int(height*(1-margin)):-1, :] = 0
	  im_arr[:, 0:int(width*margin)] = 0
	  im_arr[:, int(width*(1-margin)):-1] = 0
	  cv2.imshow("Processed Image", im_arr)
		  
	  #Find Contours of Binary Image
	  im_arr_cont = im_arr
	  contours, hierarchy = cv2.findContours(im_arr_cont,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	  # Finding contour with maximum area and store it as best_cnt
	  max_area = 0
	  for cnt in contours:
	    area = cv2.contourArea(cnt)
	    if area > max_area:
		max_area = area
		best_cnt = cnt
		
	  # finding centroid and principal direction of best_cnt
	  M = cv2.moments(best_cnt)
	  cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])	 
	  cv2.drawContours(cv_image, [best_cnt], 0, (0,0,255), 10) 
	  
	  mask = np.zeros(im_arr.shape,np.uint8)
	  cv2.drawContours(mask,[best_cnt],0,255,-1)
	  pixelpoints = np.nonzero(mask)
	  dough_cont = np.vstack([pixelpoints[0],pixelpoints[1]])
	  	  
	  # Compute PCA of dough contour
	  e = 'PCA Error'
	  try:
	    s, U = PCA(dough_cont)
	    #print 'Eigen Values: ', s
	    #print 'Eigen Vectors: ',U
	  except e:
	    print e
	  	  
	  if s[0] > s[1]:
	    first = 0
	    second = 1
	  else:
	    first = 1
	    second = 0
	    
	  centroid = np.matrix([cy,cx])
	  centroid = np.transpose(centroid)
	  #print 'Centroid: ',centroid
	  c = np.matrix([[cy,0], [0,cx]])
	  #print 'Centroid Mat: ', c	
	  v1 = np.transpose(np.matrix(U[:,first]))
	  v2 = np.transpose(np.matrix(U[:,second]))
	  #print 'First EigV: ', v1
	  #print 'Second EigV: ', v2
	  
	  dir1 = centroid + 0.2*c*v1 
	  dir2 = centroid + 0.2*c*v2 
	  
	  #asign values
	  dough_center = (cx, cy)
	  dough_dir1 = (int(dir1[1]),int(dir1[0]))
	  dough_dir2 = (int(dir2[1]),int(dir2[0]))
	  #print 'Dough Center: ', dough_center
	  #print 'Direction 1: ',dough_dir1
	  #print 'Direction 2: ',dough_dir2
	  
	  st1 = 'Dough Center: ' +  str(dough_center)
	  st2 = 'Direction 1: ' +  str(dough_dir1)
	  st3 = 'Direction 2: ' +  str(dough_dir2)
	  rospy.loginfo(st1)	  
	  rospy.loginfo(st2)
	  rospy.loginfo(st3)
	  
	  cv2.line(masked_scene, dough_center, dough_dir1, (0,0,255), 15)
	  cv2.line(masked_scene, dough_center, dough_dir2, (0,100,255), 5)
	  
	  # Compute desired shape from existing contour
	  (x,y),radius = cv2.minEnclosingCircle(best_cnt)
	  center = (int(x),int(y))
	  radius = int(radius)
	  cv2.circle(masked_scene,center,radius,(0,255,0),10)
	  
	  # Visualize Detection Result and Desired Shape
	  cv2.imshow("Dough Detection", masked_scene)	
	  cv2.waitKey(3)
	  self.loop = self.loop + 1	  
	  	  
	  #Return Dough Coordinates (Position and Direction)	  
	  return doughPoseResponse(dough_center,dough_dir1,dough_dir2)
	  
def main(args):
      dd = dough_detector()
      rospy.init_node('dough_Detector', anonymous=True)
      try:
	rospy.spin()
      except KeyboardInterrupt:
	print "Shutting down"
      cv2.destroyAllWindows() 
      
if __name__ == '__main__':
    main(sys.argv)
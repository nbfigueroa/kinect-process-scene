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
      cv2.namedWindow("Table/Plate Mask", 0)
      cv2.moveWindow("Table/Plate Mask", 0, 0)
      cv2.namedWindow("Masked Scene", 0)
      cv2.moveWindow("Masked Scene", 385, 0)
      cv2.namedWindow("Gray-Blurred Scene", 0)
      cv2.moveWindow("Gray-Blurred Scene", 705, 0)
      cv2.namedWindow("Processed Scene", 0)
      cv2.moveWindow("Processed Scene", 1020, 0)
      cv2.namedWindow("Dough Detection", 0)
      cv2.moveWindow("Dough Detection", 1340, 0)
      
      self.bridge = CvBridge()        
      self.s = rospy.Service('dough_Pose', doughPose, self.callback)
      #self.image_sub = rospy.Subscriber("/camera/rgb/image_rect_color",rosImageMsg,self.Imgcallback)
      #self.image_sub = rospy.Subscriber("/shoulder_kinect/rgb/image_rect_color",rosImageMsg,self.Imgcallback)
#      self.image_sub = rospy.Subscriber("/head_mount_kinect2/rgb_rect/image",rosImageMsg,self.Imgcallback)   
      #self.image_sub = rospy.Subscriber("/kinect2_head/rgb_lowres/image",rosImageMsg,self.Imgcallback)
      self.image_sub = rospy.Subscriber("/kinect2/rgb_lowres/image",rosImageMsg,self.Imgcallback)
  def Imgcallback(self, msg):
	 image =  self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
	 self.image = image
	 
  def callback(self,req):
	  image = self.image
	  #Extract Table Image from Service Request
	  cv_image = self.bridge.imgmsg_to_cv2(req.Table_image, desired_encoding="passthrough")
	  #cv2.imshow("Table/Plate Mask", cv_image)
	  r_image = cv_image
	  
	  ##---------------------------------------##
    	  ##------- CREATE PLATE/TABLE MASK -------##
    	  ##---------------------------------------##          
          
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
		  	   	  
	  # If using hull Minimize hull area
	  hull = cv2.convexHull(best_cnt)
	  M = cv2.moments(hull)
	  h_cx = int(M['m10']/M['m00'])
	  h_cy = int(M['m01']/M['m00'])
	  hull_ctr = np.zeros(hull.shape,np.uint8)
	  i = 0
	  for h in hull_ctr:
	    hull_ctr[i] = [h_cy,h_cx]
	    i=i+1
	  centered_hull =   hull - hull_ctr 	 
	  changed_hull = np.zeros(hull.shape,np.uint8)
	  changed_hull = centered_hull*0.90	
	  new_hull = changed_hull + hull_ctr
	  new_hull = np.ceil(new_hull)
	  nh = new_hull.astype(int)
	  
	  mask = np.zeros(r_image.shape,np.uint8)
	  #cv2.drawContours(mask,[nh],0,255,-1)	  
	  cv2.drawContours(mask,[best_cnt],0,255,-1)
	  kernel = np.ones((5,5),np.uint8)
	  mask = cv2.dilate(mask,kernel,iterations = 1)
	  cv2.imshow("Table/Plate Mask", mask)
	  
	  masked_image = image	  
	  masked_scene = cv2.bitwise_and(image,masked_image,mask = mask)
	  cv2.imshow("Masked Scene", masked_scene)
	  
	  blur_image = cv2.medianBlur(masked_scene,15)
	  gray_image = cv2.cvtColor(blur_image, cv2.COLOR_BGR2GRAY)
	  height, width = gray_image.shape
	  cv2.imshow("Gray-Blurred Scene", gray_image)
	  cv2.waitKey(3)
	  
	  # Convert the image to black and white.
	  bw_image = cv2.adaptiveThreshold(gray_image,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV,11,2)
	  
	  
	  ##-----------------------------------##
    	  ##---- FIND DOUGH ON PLATE/TABLE ----##
    	  ##-----------------------------------##	  
	  dough_found = 0
	  #Find Contours of Binary Image
	  im_arr_cont = bw_image
	  contours, hierarchy = cv2.findContours(im_arr_cont,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	  # Finding contour with maximum area and store it as best_cnt
	  max_area = 0
	  dough_found = 0
	  
	  for cnt in contours:	 
            print len(contours)
	    if dough_found==0:
	      (x, y), radius = cv2.minEnclosingCircle(cnt)
	      c_area= 3.14159*radius*radius	    
	      area = cv2.contourArea(cnt)	  	    
	      if area < 1:
		continue
	      rospy.loginfo("radius: %lf, area_ratio: %lf, area: %lf", radius, c_area/area, area)
	      #if area > 10000:
	      if area > 20000:
		rospy.logwarn("Contour too big")
		continue
	      if area < 800:
		rospy.logwarn("Contour too small")
		continue
	      #if radius > 60:
		#rospy.logwarn("Contour too small")
		#continue
	      (x, y),(width_e, height_e), angle  = cv2.fitEllipse(cnt)
	      radius_a = width_e/2
	      radius_b = height_e/2
	      e_area = 3.14159*radius_a*radius_b			
	      if abs(max(radius_a/radius_b,radius_b/radius_a) - 1) > 5:   
		rospy.logwarn("Contour is skewed due to radius")
		continue
	      elif area > max_area:
		  if e_area > 1.75*area:#was c_area
		    rospy.logwarn("Contour area skewed")
		    continue			
		  else:
		    circle_radius = radius
		    circle_area = c_area
		    ellipse_area = e_area
		    ellipse_rad_a = radius_a
		    ellipse_rad_b = radius_b
		    pixel_area = area
		    best_cnt = cnt
		    dough_found = 1
		  

	  if dough_found:
	    rospy.loginfo("Ellipse radius a: " + str(ellipse_rad_a))
	    rospy.loginfo("Ellipse radius b: " + str(ellipse_rad_b))
	    rospy.loginfo("Ellipse area: " + str(ellipse_area))
	    rospy.loginfo("Pixel area: " + str(pixel_area))
	   
	    # finding centroid and principal direction of best_cnt
	    M = cv2.moments(best_cnt)
	    cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])	 	    
	    mask = np.zeros(bw_image.shape,np.uint8)
	    cv2.drawContours(mask,[best_cnt],0,255,-1)
	    cv2.imshow("Processed Scene", mask)	
	    pixelpoints = np.nonzero(mask)
	    dough_cont = np.vstack([pixelpoints[0],pixelpoints[1]])
            print dough_cont.shape
		    
	    # Compute PCA of dough contour
	    e = 'PCA Error'
	    try:
	      s, U = PCA(dough_cont)
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
	    c = np.matrix([[cy,0], [0,cx]])
	    v1 = np.transpose(np.matrix(U[:,first]))
	    v2 = np.transpose(np.matrix(U[:,second]))
	    
	    iswhite=1
	    k=np.arange(0.01, 0.5, 0.01)
	    #i=0
	    #while iswhite:
	      #dir1 = centroid + k[i]*c*v1 
	      #iswhite = mask[int(dir1[0]),int(dir1[1])]	    
	      #i=i+1
	    
	    iswhite=1
	    i=0
	    while iswhite:
	      dir2_pos = centroid + k[i]*c*v2 
	      iswhite = mask[int(dir2_pos[0]),int(dir2_pos[1])]	    
	      i=i+1

	    iswhite=1
	    i=0
	    while iswhite:
	      dir2_neg = centroid - k[i]*c*v2 
	      iswhite = mask[int(dir2_neg[0]),int(dir2_neg[1])]	    
	      i=i+1
	    
	    #Some fixed position
	    #dir1 = centroid + 0.2*c*v1   
	    #dir2 = centroid + 0.2*c*v2 
	    
	    #assign values
	    dough_center = (cx, cy)
	    dough_dir2_pos = (int(dir2_pos[1]),int(dir2_pos[0]))
	    dough_dir2_neg = (int(dir2_neg[1]),int(dir2_neg[0]))

	    
	    
	    st1 = 'Dough Center: ' +  str(dough_center)
	    st2 = 'Positive Direction: ' +  str(dough_dir2_pos)
	    st3 = 'Negative Direction: ' +  str(dough_dir2_neg)
	    rospy.loginfo(st1)	  
	    rospy.loginfo(st2)
	    rospy.loginfo(st3)
		    
	    cv2.drawContours(image, [best_cnt], 0, (0,0,255), 3) 
	    cv2.line(image, dough_center, dough_dir2_pos, (255,0,0), 3)
	    cv2.line(image, dough_center, dough_dir2_neg, (0,255,0), 3)
	    
	    # Compute desired shape from existing contour
	    (x,y),radius = cv2.minEnclosingCircle(best_cnt)
	    center = (int(x),int(y))
	    radius = int(radius)
	    #cv2.circle(image,center,radius,(0,255,0),5)
	    ellipse = cv2.fitEllipse(best_cnt)
	    (x_e,y_e),(width_e, height_e),angle = ellipse
	    #cv2.ellipse(image,ellipse,(0,255,0),5)
	    
	    print "HHERREEE"
	    # Visualize Detection Result and Desired Shape
	    cv2.imshow("Dough Detection", image)	
	    cv2.waitKey(3)  

	    
	    #Return Dough Coordinates (Position and Direction)	  
	    return doughPoseResponse(dough_found, dough_center,dough_dir2_pos,dough_dir2_neg, pixel_area, ellipse_rad_a, ellipse_rad_b)	  
	  else:
	    return doughPoseResponse(dough_found, (0,0),(0,0),(0,0),0,0,0)	  
	 
	  
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

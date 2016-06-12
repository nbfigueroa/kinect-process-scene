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

def string2array(str):
	ret = [ord(c) for c in str]
	return np.array(ret, dtype=np.uint8)

def PCA(data):
	cov = np.cov(data)
	print 'Data Covariance:', cov
	eigenvalues, eigenvectors = np.linalg.eig(cov)
	return eigenvalues, eigenvectors	

def readCalibFile(filename):
	f = open(filename, 'r')
	# Loop over lines and extract variables of interest
	for line in f:
	    line = line.strip()
	    columns = line.split()	  
	    if columns:
	      name = columns[0]
	      if name=='o':
		o = np.matrix([float(columns[3]),  float(columns[4])])
	      if name=='Tc_ext':
		Tc_ext = np.matrix([float(columns[3]),  float(columns[4]), float(columns[5])])	
	      if name=='Rc_ext':
		Rc_ext = np.matrix([[float(columns[3]),  float(columns[4]), float(columns[5])],
				    [float(columns[6]),  float(columns[7]), float(columns[8])],
				    [float(columns[9]),  float(columns[10]), float(columns[11])]])		
	f.close()
	return o, Tc_ext, Rc_ext
	
class dough_detector:
  def __init__(self):      
 
      #self.image_pub = rospy.Publisher("rawDough",Image)
 
      # Image processing visualization
      cv2.namedWindow("Masked Scene", 0)
      cv2.moveWindow("Masked Scene", 385, 0)
      cv2.namedWindow("Gray-Blurred Image", 0)
      cv2.moveWindow("Gray-Blurred Image", 705, 0)
      cv2.namedWindow("Processed Image", 0)
      cv2.moveWindow("Processed Image", 1020, 0)
      cv2.namedWindow("Dough Detection", 0)
      cv2.moveWindow("Dough Detection", 1340, 0)
      
      self.bridge = CvBridge()
      self.image_sub = rospy.Subscriber("/camera/image",rosImageMsg,self.callback)
      
      calib_file = '/home/nbfigueroa/dev/ROSws/rosbuild/pizza_demo_year3/data/table_calibration/TableCalibrationResults.m'
      self.o, self.Tc_ext, self.Rc_ext = readCalibFile(calib_file)
      self.loop = 0

  def callback(self,msg):
	  cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
	  height, width = cv_image.shape[:2]
	  # Remove margins of Scene
	  mask = np.ones(cv_image.shape[:2],np.uint8)
	  margin = 0.18
	  mask[0:int(height*margin), :] = 0
	  mask[int(height*(1-margin)):-1, :] = 0
	  mask[:, 0:int(width*margin)] = 0
	  mask[:, int(width*(1-margin)):-1] = 0
	  masked_scene = cv2.bitwise_and(cv_image,cv_image,mask = mask)
	  cv2.imshow("Masked Scene", masked_scene)
	  
	  
	  #Blur and Convert image to grayscale	 
	  blur_image = cv2.medianBlur(masked_scene,15)	
	  gray_image = cv2.cvtColor(blur_image, cv2.COLOR_BGR2GRAY)
	  cv2.imshow("Gray-Blurred Image", gray_image)
	  cv2.waitKey(3)
	  
	  # Convert the image to black and white.
	  bw_image = cv2.adaptiveThreshold(gray_image,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV,11,2)
	  margin = 0.20
	  bw_image[0:int(height*margin), :] = 0
	  bw_image[int(height*(1-margin)):-1, :] = 0
	  bw_image[:, 0:int(width*margin)] = 0
	  bw_image[:, int(width*(1-margin)):-1] = 0	  
	  cv2.imshow("Processed Image", bw_image)
		  
	  #Find Contours of Binary Image
	  im_arr_cont = bw_image
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
	  ox,oy = int(self.o[0,1]), int(self.o[0,0])
	  cv2.circle(cv_image,(cx,cy),10,(0,0,255),-1)
	  cv2.circle(cv_image,(ox,oy),5,(255,0,0),-1)
	  cv2.drawContours(cv_image, [best_cnt], 0, (0,0,255), 10) 
	  
	  mask = np.zeros(bw_image.shape,np.uint8)
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
	  
	  dir1 = centroid + 0.5*c*v1 
	  dir2 = centroid + 0.5*c*v2 
	  
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
	  
	  cv2.line(cv_image, dough_center, dough_dir1, (0,0,255), 15)
	  cv2.line(cv_image, dough_center, dough_dir2, (0,100,255), 5)
	  
	  area = cv2.contourArea(best_cnt)
	  
	  # Write file with detected data
	  filename = '/home/nbfigueroa/dev/ROSws/rosbuild/pizza_demo_year3/data/captured/dough_' + str(self.loop) 
	  filename = filename + '.txt' 
	  #filename = '/home/nbfigueroa/dev/ROSws/rosbuild/pizza_demo_year3/data/captured/dough_meas.txt' 
	  data = {'ID': self.loop, 'Dough_Center': dough_center, 'Dough_D1': dough_dir1, 'Dough_D2': dough_dir2, 'Area': area};	  
	  rospy.loginfo(str(filename))
	  rospy.loginfo(str(data))
	  with open(filename, 'a') as outfile:
	    json.dump(data, outfile)
	  
	  # Compute desired shape from existing contour
	  (x,y),radius = cv2.minEnclosingCircle(best_cnt)
	  center = (int(x),int(y))
	  radius = int(radius)
	  cv2.circle(cv_image,center,radius,(0,255,0),10)
	  
	  # Visualize Detection Result and Desired Shape
	  cv2.imshow("Dough Detection", cv_image)	
	  cv2.waitKey(3)
	  self.loop = self.loop + 1	  
	  	  
	  #Publish to Task planninng topic
	  #msgOut = rosImageMsg()
	  #msgOut.width = width
	  #msgOut.height = height
	  #dataTmp = im_Arr.reshape((1,width*height))[0]
	  #msgOut.data = dataTmp.tolist()
	  #pub.publish(msgOut)
	  
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
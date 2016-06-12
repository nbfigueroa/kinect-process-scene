#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include "boost/filesystem.hpp"  
#include <algorithm>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
  
    ros::init(argc, argv, "image_capture");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/camera/image", 1);
    ros::Rate loop_rate(3);
    
    if (argc < 2){ 
	  //Capture frame from webcam in LASA setup
	  VideoCapture cap(1); 
	  cap.set(CV_CAP_PROP_FPS,15);	
	  if ( !cap.isOpened() )  // if not success, exit program
	  {
	    cout << "Cannot open the web cam" << endl;
	    return -1;
	  }
		
	int loop = 0;
	char fname[500];
	while (true)
	{
	    // retrieve the frame:
	    Mat frame;
	    if (!cap.read(frame)) {
		std::cout << "Unable to retrieve frame from video stream." << std::endl;
		continue;
	    }

	    // display it:
	    imshow("LiveStream", frame);

	    // stop if Esc has been pressed:
	    char c = NULL;
	    c = waitKey(1);
	    if (c == 27) {
	        ROS_INFO_STREAM("Pressed ESC");
		break;
	    }
	   
	    // Write frame to disk if "Space Bar" is pressed and send to topic	  
	    if(c == 32){
	      cout << "Pressed Space Bar"<< endl;
	      sprintf(fname, "/home/nbfigueroa/dev/ROSws/rosbuild/pizza_demo_year3/data/captured/frame%d.jpg", loop);
	      cv::imwrite(fname, frame); //save each frame locally
	      loop++;	
		if (nh.ok()) {
		  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
		  pub.publish(msg);
		  ros::spinOnce();
		  loop_rate.sleep();
		}
		else{
		  ROS_INFO_STREAM("Message could not be published correctly.");
		}
	   }
	}
	cap.release();
    }
    else{ //read the frames from directory
    
	string dir (argv[1]);
	boost::filesystem::directory_iterator iterator(dir);
	vector<string> img_names;
	vector<int> img_ids;
	for(; iterator != boost::filesystem::directory_iterator(); ++iterator){
	  img_names.push_back((iterator->path().filename()).string());
	}
	
	for(int i(0);i<img_names.size();i++){
	  string str = img_names[i];
	  string temp;
	  int number (0);
	  for (unsigned int j=0; j < str.size(); j++){
	    if (isdigit(str[j])){
		for (unsigned int a=j; a<str.size(); a++){
		    temp += str[a];
		}
		break;
	    }
	  }
	  std::istringstream stream(temp);
	  stream >> number; 
	  img_ids.push_back(number);
	}
	
	vector<string> sorted_img_names(img_names.size());
	for (int i = 0; i != img_ids.size(); ++i){
	  sorted_img_names[img_ids[i]] = dir + img_names[i]; 
	}
      
	int i = 0;    
	while (nh.ok() && i != img_ids.size()) {
		std:string img_filename = sorted_img_names[i];
		cv::Mat image = cv::imread(img_filename,CV_LOAD_IMAGE_COLOR);
		
		// display it:
		imshow("LiveStream", image);
		char c = NULL;
		c = waitKey();
		if (c == 27) {
		  ROS_INFO_STREAM("Pressed ESC");
		  break;
		}
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		i++;
	  }     
    }  
}
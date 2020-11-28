#include "ros/ros.h"
#include "ball_chaser_OOP/DriveToTarget.h"
#include <sensor_msgs/Image.h>

class ProcessImage{

	private:
		ros::NodeHandle* nh_; // public node handle
		ros::NodeHandle* pnh_; // private node handle (for accessing parameter values, not currently used)
		ros::ServiceClient client; // Define a global client that can request services
		ros::Subscriber sub; // Subscriber for image topic

	public:
		ProcessImage(ros::NodeHandle *nh, ros::NodeHandle *pnh); // Constructor	
		~ProcessImage(); //Destructor
		void drive_robot(float lin_x, float ang_z); // Calls the command_robot service to drive the robot in the specified direction
		void process_image_callback(const sensor_msgs::Image &img); // This callback continuously executes and reads the image data

}; // ProcessImage class


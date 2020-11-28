#include "ros/ros.h"
#include "ball_chaser_OOP/DriveToTarget.h"

class DriveBot{

	private:
		
		ros::NodeHandle* nh_; // public node handle
		ros::NodeHandle* pnh_; // private node handle (for accessing parameter values, not currently used)
		ros::ServiceServer service; // Service for receiving robot velocity commands
		ros::Publisher motor_command_publisher; // Publisher for motor commands

	public:

		// Constructor	
		DriveBot(ros::NodeHandle *nh, ros::NodeHandle *pnh);

		//Destructor
		~DriveBot();

		// Callback function that executes whenever a drive_bot service is requested
		bool handle_drive_request(ball_chaser_OOP::DriveToTarget::Request& req, ball_chaser_OOP::DriveToTarget::Response& res);

}; // DriveBot class


#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

enum class DriveDirection
{
	LEFT,
	FORWARD,
	RIGHT,
	STOP
};

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
	ball_chaser::DriveToTarget srv;
	srv.request.linear_x = lin_x;
	srv.request.angular_z = ang_z;

	if (!client.call(srv))
		ROS_ERROR("Failed to call DriveToTarget service");
}

void drive_towards(DriveDirection dir)
{
	switch (dir)
	{
	case DriveDirection::FORWARD:
		drive_robot(0.5, 0.0);
		break;
	case DriveDirection::LEFT:
		drive_robot(0.0, 0.5);
		break;
	case DriveDirection::RIGHT:
		drive_robot(0.0, -0.5);
		break;
	case DriveDirection::STOP:
		drive_robot(0.0, 0.0);
		break;
	}
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
	// if ball is not found, stop
	DriveDirection dir = DriveDirection::STOP;
	int third = img.width / 3;
	for (int i = 0; i < img.height * img.width; i++)
	{
		if (img.data[i * 3] + img.data[i * 3 + 1] + img.data[i * 3 + 2] == 255 * 3)
		{
			int xpos = i % img.width;
			if (xpos < third_limit)
			{
				// the ball is in the right third of the screen
				dir = DriveDirection::LEFT;
			}
			else if (xpos >= third && xpos <= third * 2)
			{
				// the ball is in the center of the screen
				dir = DriveDirection::FORWARD;
			}
			else
			{
				// the ball is on the right of the center
				dir = DriveDirection::RIGHT;
			}
			break;
		}
	}

	drive_towards(dir);
}

int main(int argc, char **argv)
{
	// Initialize the process_image node and create a handle to it
	ros::init(argc, argv, "process_image");
	ros::NodeHandle n;

	// Define a client service capable of requesting services from command_robot
	client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

	// Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
	ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

	// Handle ROS communication events
	ros::spin();

	return 0;
}

#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
if(lin_x==0.0 && ang_z==0.5)
{ROS_INFO_STREAM("drive_robot left ");
}

if(lin_x==0.0 && ang_z==-0.5)
{ROS_INFO_STREAM("drive_robot right ");
}

if(lin_x==0.5 && ang_z==0.0)
{ROS_INFO_STREAM("drive_robot straight ");
}

if(lin_x==0.0 && ang_z==0.0)
{//ROS_INFO_STREAM("drive_robot stop ");
}



	ball_chaser::DriveToTarget srv;
	srv.request.linear_x=lin_x;
	srv.request.angular_z=ang_z;

if(!client.call(srv))
ROS_ERROR("Failedto call service");

}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255,l=0,c=0,r=0,stop=1,i=0;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
	 for ( i = 0; i < img.height * img.step; i+=3)
		 {
//ROS_INFO_STREAM("the for loop");
//if(i==10)
//ROS_INFO("value for %d = %d",i,img.data[i]);

//ROS_INFO_STREAM("the master if");
        if (img.data[i] == white_pixel && img.data[i+1] == white_pixel && img.data[i+2] == white_pixel) {
	stop=0;
//ROS_INFO_STREAM("the first if");
	
            if(i%(img.step)<(img.step)/3)
			{
//ROS_INFO_STREAM("the left if");

				l++;}
			if(i%(img.step) >=(img.step)/3 && i/(img.step*3)<=2*(img.step)/3)
			{
//ROS_INFO_STREAM("the center if");

c++;}
            if(i%(img.step)>2*(img.step)/3)
			{
//ROS_INFO_STREAM("the right if");

r++;	}
    }
else
{
//ROS_INFO_STREAM("the stop else");
//drive_robot(0.0,0.0);
}
	
	}

//ROS_INFO("VV--before sending commands,r= %d ,c = %d, l=%d",r,c,l);	 
	if(r>c && r>l)//right
	{
//ROS_INFO("VV--right,r= %d ,c = %d, l=%d",r,c,l);
	drive_robot(1.0, -0.5);}
	if(l>c && l>r)//left
	{
//ROS_INFO("VV--left,r= %d ,c = %d, l=%d",r,c,l);
	drive_robot(1.0, 0.5);}
	if(c>l&&c>r)//straight
	{
//ROS_INFO("VV--center,r= %d ,c = %d, l=%d",r,c,l);	
drive_robot(1.0, 0.0);}
if(stop==1)		
{//ROS_INFO_STREAM("STOP");
drive_robot(0.0, 0.0);		}
}

int main(int argc, char** argv)
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

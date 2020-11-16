#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // ROS_INFO_STREAM("Driveing robot to the target");
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int target_color_r = 255;
    int target_color_g = 255;
    int target_color_b = 255;
    
    int target_pixels_left = 0;
    int target_pixels_center = 0;
    int target_pixels_right = 0;

    int img_pixel_pos = 0;

    // Define threshols 
    int img_threshold_left = (img.step * 10) / 24;
    int img_threshold_right = (img.step * 14) / 24;

    // Count white target pixels on left, center and right
    
    for (int i = 0; i < img.height; i++)
    {
        for (int j = 0; j < img.step-2; j=j+3)
        {
            img_pixel_pos = i * img.step + j;

            if (img.data[img_pixel_pos] == target_color_r &&
                img.data[img_pixel_pos+1] == target_color_g &&
                img.data[img_pixel_pos+2] == target_color_b)
            {
                if (j < img_threshold_left)
                {
                    target_pixels_left++;
                }
                else
                {
                    if (j > img_threshold_right)
                    {
                        target_pixels_right++;
                    }
                    else
                    {
                        target_pixels_center++;
                    }
                }
            }
        }
    }
    // ROS_INFO(">>>LEFT: %1.0f \tCENTER: %1.0f \tRIGHT: %1.0f", (float)target_pixels_left, (float)target_pixels_center,(float)target_pixels_right);

    // Compare pixel sums to define robot's next move 
    if (target_pixels_left == 0 && target_pixels_center == 0 && target_pixels_right == 0)
    {
        drive_robot(0, 0);
    }
    else
    {
        if (target_pixels_left > target_pixels_right)
        {
            if (target_pixels_left > target_pixels_center)
            {
                drive_robot(0, 0.2);
            }
            else
            {
                drive_robot(0.3, 0);
            }
        }
        else
        {
            if (target_pixels_right > target_pixels_center)
            {
                drive_robot(0, -0.2);
            }
            else
            {
                drive_robot(0.3, 0);
            }
        }
    }
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
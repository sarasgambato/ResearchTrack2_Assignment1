/**
* \file keyboard.cpp
*
* \brief Node to drive the robot manually
*
* \author Sara Sgambato
*
* \version 1.0
*
* \date 01/05/2022
*
* \details
*
* Subscribes to: <BR>
* /scan
* /cmd_key_vel
*
* Publishes to: <BR>
* /cmd_vel
*
* Description :
*
* This node prompts the user to choose between the assisted modality and the non-assisted modality.
* If the user choses the assisted modality, the node reads data from the /scan topic to avoid obstacles.
**/

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"

std_srvs::Empty rst; ///< Service to reset the simulation.

ros::Publisher pub; ///< Publisher on /cmd_vel.
geometry_msgs::Twist my_vel; ///< Message of type Twist.

bool assisted = false; ///< Variable to set the assisted modality.

float wall_th = 1; ///< Threshold from the wall.

float lin; ///< Variable to store the linear velocity.
float ang; ///< Variable to store the angular velocity.

std::string menu = R"(
***********************************************
You chose to drive the robot manually!
The default modality is the non-assisted keyboard.

Press:	a/A to drive with the assisted keyboard

Press:	b/B to go back to the main menu
***********************************************
)"; ///< Variable to print the main menu.

std::string assisted_menu = R"(
***********************************************
You are in the assisted modality!

Press:	n/N to go back to the default modality
***********************************************
)"; ///< Variable to print the assisted menu.

/**
* \brief Function to prompt the user.
*
* \return a char corresponding to the user input.
*
* This function asks the user whether they'd like to use the assisted modality. The choice is
* saved in a variable of type char.
*/
char chooseMod()
{
	char input;

    	system("clear");
    
    	// prompt the user
    	std::cout << menu;
    
   	// get the keybord input
  	std::cin >> input;
    
   	return input;
}

/**
* \brief Function to get the minimum value from an array.
*
* \param start defines the lower bound
* \param end defines the upper bound
* \param distances is the array which will be examinated
*
* \return a float corresponding to the minimum value of distances.
*
* This function checks the array given as input from the lower bound
* to the upper bound to get the minimum value of the array in this range.
*/
float getMinimum(int start, int end, float distances[])
{
    	float min = 50;
    	for(int i = start; i < end; i++) 
    	{
        	if (distances[i] < min)
            		min = distances[i];
    	}
    	
    	return min;
}

/**
* \brief Callback function for the /scan topic subscriber.
*
* \param msg is the message of type LaserScan
*
* This callback function becomes useful only when the user decides to use the assisted modality.
* In this case, the function gets the minimum value of 3 subsections of the array ranges, which contains the distances 
* from the obstacles in a range of 180 degrees: 
* - if there is a wall in front of the robot, the robot stops;
* - if there is a wall near the side of the robot, the robot turns until it is undetectable.
*/
void avoidCollision(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	if(assisted)
	{
	    	float distances[720];

	    	for(int i = 0; i < 720; i++)
			distances[i] = msg->ranges[i];

	    	float min_right = getMinimum(0, 69, distances);
	    	float min_front = getMinimum(300, 419, distances);
	    	float min_left = getMinimum(650, 719, distances);
	  
    		my_vel.linear.x = lin;
    		my_vel.angular.z = ang;

		// if the wall in front of the robot is too close, it cannot move forward
		if (min_front < wall_th)
		{
			my_vel.linear.x = 0;
    			system("clear");
			std::cout << "Detected wall in front of the robot. Turn left or right!\n";
		}
		
		// if the wall on the left of the robot is too close, the robot turns right
	    	else if (min_left < wall_th)
	    	{
	    		my_vel.angular.z = -1;
    			system("clear");
	    		std::cout << "Detected wall on the left of the robot! Turning right...\n";
	    	}
	    	
	    	// if the wall on the right of the robot is too close, the robot turns left
	    	else if (min_right < wall_th)
	    	{
	    		my_vel.angular.z = 1;
    			system("clear");
	    		std::cout << "Detected wall on the right of the robot! Turning left...\n";
	    	}
	    	
	    	else
	    	{
    			system("clear");
    			
	    		// prompt the user
    			std::cout << assisted_menu;
    		}
    		
	    	pub.publish(my_vel);
    	}
    	
    	return;
}

/**
* \brief Callback function for the /cmd_key_vel topic subscriber.
*
* \param msg is the message of type Twist
*
* This callback function saves the velocities got from msg in 2 gloabl variables, then it updates the
* velocities of the robot using the gloabl variables.
*/
void updateVel(const geometry_msgs::Twist::ConstPtr& msg)
{
	lin = msg -> linear.x;
	ang = msg -> angular.z;
	
	my_vel.linear.x = lin;
	my_vel.angular.z = ang;
	
	pub.publish(my_vel);
}

/**
* \brief Main function.
*/
int main(int argc, char **argv)
{
	// initialize the node, set up the NodeHandle for handling the communication with the ROS system
    	ros::init(argc, argv, "keyboard");
    	ros::NodeHandle nh;

    	pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    	
    	ros::Subscriber subC = nh.subscribe("/scan", 1, avoidCollision);
    	ros::Subscriber subV = nh.subscribe("/cmd_key_vel", 1, updateVel);
    	
    	ros::AsyncSpinner spinner(4);
  	spinner.start();
	
	while(1)
	{
		switch(chooseMod())
		{
			case 'a':
			case 'A':
				assisted = true;
				break;
				
			case 'n':
			case 'N':
				assisted = false;
				break;
				
			case 'b':
			case 'B':
				my_vel.linear.x = 0;
				my_vel.angular.z = 0;
				pub.publish(my_vel);
				return 0;
				break;
				
			default:
				std::cout << "Wrong key pressed, please try again.\n";
				sleep(2);
        			break;
		}
	}
	
	spinner.stop();
		
    	return 0;
}

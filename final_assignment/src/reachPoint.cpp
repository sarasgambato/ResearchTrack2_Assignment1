/**
* \file reachPoint.cpp
*
* \brief Node to reach a goal autonomously
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
* /move_base/feedback
* /move_base/status
*
* Publishes to: <BR>
* /move_base/goal
* /move_base/cancel
*
* Description :
*
* This node prompts the user to either set a new goal or cancel the current goal.
**/

#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "move_base_msgs/MoveBaseActionFeedback.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "time.h"

std_srvs::Empty rst; ///< Service to reset the simulation.

float xCoord; ///< Variables to store the x coordinate of the goal.
float yCoord; ///< Variables to store the y coordinate of the goal.

ros::Publisher pubG; ///< Publisher on /move_base/goal.
ros::Publisher pubC; ///< Publisher on /move_base/cancel.

ros::Subscriber subG; ///< Subscriber of /move_base/feedback.
ros::Subscriber subS; ///< Subscriber of /move_base/status.

move_base_msgs::MoveBaseActionGoal goal; ///< Message of type MoveBaseActionGoal to store the goal.

std::string goal_id; ///< Variable to store the goal id.

actionlib_msgs::GoalID id_cancel; ///< Message of type GoalID to store the goal to cancel.

int goal_status; ///< Variable to store the status of the goal.

bool flag = false; ///< Flag to control whether a goal is set or not.

std::string menu = R"(
***********************************************
You chose to let the robot drive on its own!
Press:	p/P to set the coordinates of the goal
Press:	c/C to cancel the current goal

Press:	b/B to go back to the main menu
***********************************************
)"; ///< Variable to print the main menu.

void getID(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg);
void getStatus(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);

/**
* \brief Function to prompt the user.
*
* \return a char corresponding to the user input.
*
* This function asks the user to set a new goal or to cancel the current one.
* The choice is saved in a variable of type char.
*/
char choice()
{
	char input;

    	system("clear");
    
    	// prompt the user
    	std::cout << menu;
    	
    	// show the current goal
    	if(flag)
    	{
    		std::cout << "Current goal: (" << xCoord << ", " << yCoord << ")\n";
    	}
    
   	// get the keybord input
  	std::cin >> input;
    
   	return input;
}

/**
* \brief Function to set the goal.
*
* This function asks the user to insert valid coordinates of a point to reach.
* After a valid input is detected, the goal parameters are set and the flag of the goal is set to true.
* The status is checked via the subscriber to the /move_base/status topic.
*/
void setGoal()
{	
	system("clear");
	
	std::cout << "Enter a point (x, y) that the robot will try to reach\n\n";
	
	std::cout << "x: ";
	std::cin >> xCoord;
	while(std::cin.fail())
    	{
		std::cin.clear();
		std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
		std::cout << "Invalid input, enter a number: ";
		std::cin >> xCoord;
    	}
    	
    	std::cout << "y: ";
	std::cin >> yCoord;
	while(std::cin.fail())
    	{
		std::cin.clear();
		std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
		std::cout << "Invalid input, enter a number: ";
		std::cin >> yCoord;
    	}
    	
    	// set the goal parameters
    	goal.goal.target_pose.pose.position.x = xCoord;
    	goal.goal.target_pose.pose.position.y = yCoord;
    	goal.goal.target_pose.header.frame_id = "map";
	goal.goal.target_pose.pose.orientation.w = 1;
	
	pubG.publish(goal);
	
	// I need to subscribe to these 2 topics to get the id of the goal and its status
	ros::NodeHandle nh;
	subG = nh.subscribe("/move_base/feedback", 1, getID);
    	subS = nh.subscribe("/move_base/status", 1, getStatus);
    	
	flag = true;
}


/**
* \brief Function to cancel the current goal.
*
* The function checks whether a goal has been set via the variable "flag": if not, a message is printed
* warning the user that no goal has been set yet, otherwise the function cancels the 
* current goal, sets the flag to false and informs the user that the operation succeded.
*/
void cancelGoal()
{	
	if(flag)
	{
		id_cancel.id = goal_id;
		subS.shutdown();
		subG.shutdown();
		pubC.publish(id_cancel);
		
		std::cout << "The goal has been correctly cancelled!\n";
		flag = false;
	}
	else
		std::cout << "No goal has been set yet!\n";
		
	sleep(2);
}

/**
* \brief Callback function for the /move_base/feedback topic subscriber.
*
* \param msg is the message of type MoveBaseActionFeedback.
*
* This callback function is used to get the id of the goal that has just been set.
*/
void getID(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg)
{
	goal_id = msg -> status.goal_id.id;
}

/**
* \brief Callback function for the /move_base/status topic subscriber.
*
* \param msg is the message of type GoalStatusArray.
*
* This callback function is used to get the status of the current goal: if the goal is either
* succeded, aborted or rejected, then the function informs the user of the status and cancels the goal.
*/
void getStatus(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
	if(goal_id == msg -> status_list[0].goal_id.id)
		goal_status = msg -> status_list[0].status;
		
	// goal SUCCEDED
	if(goal_status == 3)
	{
		std::cout << "The goal was successfully reached!\n";
		goal_status = -1;
		cancelGoal();
	}
	
	// goal ABORTED
	if(goal_status == 4)
	{
		std::cout << "The goal was aborted!";
		goal_status = -1;
		cancelGoal();
	}
	
	// goal REJECTED
	if(goal_status == 5)
	{
		std::cout << "The goal was rejected!";
		goal_status = -1;
		cancelGoal();
	}
	
}

/**
* \brief Main function.
*/
int main(int argc, char **argv)
{
	// initialize the node, set up the NodeHandle for handling the communication with the ROS system
    	ros::init(argc, argv, "reachPoint");
	ros::NodeHandle nh;

    	pubG = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1);
    	pubC = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
    	
    	ros::AsyncSpinner spinner(4);
  	spinner.start();
	
    	while(1)
    	{
    		switch(choice())
    		{
    			case 'p':
    			case 'P':
    				setGoal();
    				break;
    				
    			case 'c':
    			case 'C':
    				cancelGoal();
    				break;
        			
    			case 'b':
    			case 'B':
    				if(flag)
    					cancelGoal();
    				return 0;
    				break;
    				
    			default:
    				std::cout << "Wrong key pressed, please try again.";
        			sleep(2);
        			break;
    		}
    	}

	spinner.stop();
	
    	return 0;
}

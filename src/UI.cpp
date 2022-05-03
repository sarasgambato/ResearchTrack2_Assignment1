/**
* \file UI.cpp
*
* \brief User-Interface for the Robot Controller
*
* \author Sara Sgambato
*
* \version 1.0
*
* \date 01/05/2022
*
* \details
*
* Description :
*
* This node prompts the user to choose the modality with which they would like to
* control the robot behaviour.
**/

#include "ros/ros.h"
#include "std_srvs/Empty.h"

std_srvs::Empty rst; ///< Service to reset the simulation.

std::string menu = R"(
***********************************************
Choose how to control the robot:
Press:	1 to make the robot reach a point autonomously
Press:	2 to drive the robot with the keyboard
				
Press:	r/R to reset the simulation
Press:	e/E to exit the program
***********************************************
)"; ///< Variable to print the main menu.

/**
* \brief Function to prompt the user.
*
* \return a char corresponding to the user input.
*
* This function asks the user to choose a modality with which the robot will be controlled.
* The choice is saved in a variable of type char.
*/
char getInput()
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
* \brief Main function.
*/
int main(int argc, char **argv)
{
    	ros::init(argc, argv, "UI");

    	while(1) 
    	{
        	switch(getInput()) 
        	{
        		case '1':
        			system("rosrun final_assignment auto_controller");
        			break;
        			
        		case '2':
        			system("roslaunch final_assignment keyboard.launch");
        			system("clear");
        			break;
        			
        		case 'r':
        		case 'R':
        			ros::service::call("gazebo/reset_simulation", rst);
        			break;
        			
        		case 'e':
        		case 'E':
        			std::cout << "Exiting the program...";
        			sleep(2);
        			return 0;
        			break;
        			
        		default:
        			std::cout << "Wrong key pressed, please try again.\n";
        			sleep(2);
        			break;	
        	}
    	}

    	return 0;
}

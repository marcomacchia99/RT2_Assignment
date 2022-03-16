#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include <termios.h>

#define NC "\033[0m"
#define BOLD "\033[1m"

/**
* \file mainController.cpp
* \brief main controller for the robot
* \author Marco Macchia
* \version 1.0
* \date 16/03/2022
*
* \details
*
*
* Description : <BR>
* This node let the user choose what type of navigation the robot should use:
*
*    1 - Reach autonomousely a given position <BR>
*    2 - Drive the robot with the keyboard <BR>
*    3 - Drive the robot with the keyboard with automatic collision avoidance <BR>
*    4 - Reset simulation <BR>
*    0 - Exit from the program <BR>
*
*
**/

std_srvs::Empty reset; ///< variable used to reset the simulation

std::string mainMenu = R"(
        
Select one of the following behavior:

1 - Reach autonomousely a given position
2 - Drive the robot with the keyboard
3 - Drive the robot with the keyboard with automatic collision avoidance
4 - Reset simulation

0 - Exit from the program


)"; ///< instruction for the user

/**
 * @brief sets the read operation in non-blocking mode.
 * 
 * This function is used for non-blocking keyboard inputs. 
 * taken from teleop_twist_keyboard.cpp
 * at https://github.com/methylDragon/teleop_twist_keyboard_cpp/blob/master/src/teleop_twist_keyboard.cpp 
 * 
 * @return int - the pressed key 
 * 
 */
int getch(void)
{
    int ch;
    struct termios oldt;
    struct termios newt;

    // Store old settings, and copy to new settings
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // Make required changes and apply the settings
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_iflag |= IGNBRK;
    newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
    newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
    newt.c_cc[VMIN] = 1;
    newt.c_cc[VTIME] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &newt);

    // Get the current character
    ch = getchar();

    // Reapply old settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
}


/**
 * @brief This function displays the main menu and wait for a command
 * 
 */
int displayMainMenu()
{
    system("clear");

    std::cout << BOLD << "Welcome to the Mobile Robot Control System!" << NC;
    std::cout << mainMenu;

    int choice;

    choice = getch();

    return choice;
}

/**
 * @brief main function
 * 
 * The main function starts the current node and displays the current instruction.
 * Based on the user input it starts a specific node or perform a specific action
 * 
 * @param argc
 * @param argv
 * 
 * @return always 0
 * 
 */
int main(int argc, char **argv)
{

    ros::init(argc, argv, "mainController");

    while (true)
    {

        switch (displayMainMenu())
        {
        case '1':
            //launch reachPoint node
            system("rosrun RT2_Assignment reachPoint");
            break;
        case '2':
            //launch driveWithKeyboard node
            system("rosrun RT2_Assignment driveWithKeyboard");
            break;
        case '3':
            //launch driveWithKeyboardAssisted node
            system("rosrun RT2_Assignment driveWithKeyboardAssisted");
            break;
        case '4':
            //call gazebo/reset_simulation service and reset the simulation
            ros::service::call("/gazebo/reset_simulation", reset);
            break;
        case '0':
            //celar screen and exit from main
            system("clear");
            return 0;
            break;
        default:
            break;
        }
    }

    return 0;
}